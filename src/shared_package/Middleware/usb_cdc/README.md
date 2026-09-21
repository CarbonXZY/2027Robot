# USB CDC 通信封装

对 Linux 下 USB CDC ACM 虚拟串口设备 `/dev/ttyACMx` 的轻量 C++ 封装. 基于 POSIX 串口接口实现, 不依赖 ROS 与任何第三方库, 可直接用于嵌入式通信板, STM32 USB CDC 固件, 或其他会枚举成 ACM 设备的 USB 转串口模块.

## 文件组成

| 文件 | 说明 |
| --- | --- |
| `usb_cdc.hpp` | 类声明, 常量定义, 接口注释 |
| `usb_cdc.cpp` | 设备打开与配置, 收发线程实现 |

## 设计说明

类 `USB_CDC` 与单个设备路径一一绑定. 构造时打开设备并把串口配置为原始模式, 随后启动两个后台线程.

- 接收线程: 阻塞在 `read` 上等待设备数据. 收到数据后取出已注册的回调函数并立即在接收线程上下文中调用.
- 发送线程: 周期性检查发送队列 FIFO, 队列非空时取出一帧写入设备.

串口参数固定为 8N1, 无校验, 无硬件流控, 无软件流控, 关闭回显与信号处理. `VMIN` 为 1 且 `VTIME` 为 0, 即 `read` 阻塞直到至少有一个字节可读. 因此该封装不理解任何上层协议, 帧头, 帧尾与校验都由使用者在回调中或调用前自行处理.

资源管理遵循 RAII. 析构函数会自动停止两个线程并关闭文件描述符, 使用者不需要手动调用停止接口. 拷贝构造与拷贝赋值被显式删除, 因为该类持有线程与文件描述符等独占资源.

## 常量

| 名称 | 值 | 含义 |
| --- | --- | --- |
| `USB_CDC::kTxQueueSize` | 64 | 异步发送队列最大帧数, 队列满时 `TransmitAdd` 返回失败 |
| `USB_CDC::kTxPollIntervalMs` | 10 | 发送线程轮询周期毫秒数, 也是读取出错后的重试等待时间 |
| `USB_CDC::kMaxFrameSize` | 1024 | 单次 `read` 的最大字节数, 决定了接收回调中单包数据的上限 |
| `USB_CDC_DEFAULT_DEVICE` | `"/dev/ttyACM0"` | 默认设备路径常量, 位于全局命名空间 |

## 接口

| 接口 | 说明 |
| --- | --- |
| `USB_CDC(const std::string& device, uint32_t tx_period_ms = 10)` | 打开设备并启动收发线程. 打开失败不抛异常, 只向 `stderr` 打印错误, 需要用 `IsOpen` 检查 |
| `~USB_CDC()` | 停止线程, 等待线程退出, 关闭设备 |
| `void RegisterRxCallback(RxHandle callback)` | 注册接收回调, 线程安全, 可运行中替换, 传 `nullptr` 可取消回调 |
| `uint8_t TransmitNow(uint8_t* data, uint16_t length)` | 同步发送, 立即写设备. 返回 1 表示发送成功, 返回 0 表示失败 |
| `uint8_t TransmitAdd(uint8_t* data, uint16_t length)` | 异步发送, 拷贝数据入队. 返回 1 表示入队成功, 返回 0 表示入队失败 |
| `bool IsOpen() const` | 设备是否已成功打开 |
| `uint32_t GetTxPeriodMs() const` | 读取异步发送周期 |
| `void SetTxPeriodMs(uint32_t period_ms)` | 设置异步发送周期 |

回调类型定义为 `using RxHandle = void (*)(uint8_t* data, uint16_t length)`. 它是裸函数指针而非 `std::function`, 因此只能绑定静态函数或全局函数, 若需要绑定成员函数请用带上下文指针的适配层包一层.

## 返回值约定

两个发送接口都以 `uint8_t` 返回状态. `TransmitAdd` 返回 0 只表示队列满或参数非法, 不表示设备写入失败, 因为真正的写操作发生在后台线程. 需要确认数据确实写入设备的场景应使用 `TransmitNow`. 两个接口的成功返回值都是 1, 可以直接作为布尔值使用.

`TransmitNow` 在数据长度为零, 数据指针为空, 或设备未打开时直接返回 0.

## 线程与并发

- 接收回调在接收线程中被调用, 回调内部不能长时间阻塞, 否则会阻塞后续接收. 回调期间持有回调互斥锁, 所以不要在回调里再次调用 `RegisterRxCallback`, 否则会死锁.
- 两个发送接口可以跨线程同时调用. 写设备由一把写锁保护, 同步发送与异步发送线程不会交错写坏数据.
- `TransmitNow` 会等待写锁. 发送线程使用 `try_to_lock` 尝试获取写锁, 取不到时会把该帧放回队首并等待下一个周期.
- 发送队列中的每一帧都是深拷贝, 调用方在 `TransmitAdd` 返回后可以立即复用或释放自己的缓冲区.

## 已知行为

- 接收线程阻塞在 `read` 上. 若设备长期无数据, `read` 不会返回, 析构函数中的 `join` 会一直等待到下一次数据到达才结束. 对时序敏感的场景建议先关闭设备或被控端主动发送一帧心跳再析构.
- 设备被拔出后 `read` 会返回 0 并立即返回, 接收线程会进入忙循环. 当前实现只在返回负值且非 `EINTR` 时休眠, 所以拔出设备后 CPU 占用会升高.
- 发送线程在队列为空时执行 `continue` 而不睡眠, 同样是忙循环. 该线程真正等待的是队列中出现数据, 定时周期只影响有帧时的节流节奏.

## 使用示例

### 基础收发

```cpp
#include "usb_cdc.hpp"

#include <cstdio>
#include <thread>

namespace
{
// 接收回调, 由接收线程调用, 必须保持短小
void OnReceive(uint8_t* data, uint16_t length)
{
    std::printf("rx %u bytes\n", length);

    // received_frame 视为一帧完整数据, 这里自行解析上层协议
    for (uint16_t i = 0; i < length; ++i)
    {
        std::printf("%02X ", data[i]);
    }
    std::printf("\n");
}
}  // namespace

int main()
{
    // 每 20 毫秒尝试发送一帧队列中的数据
    USB_CDC cdc("/dev/ttyACM0", 20);

    if (!cdc.IsOpen())
    {
        std::fprintf(stderr, "open /dev/ttyACM0 failed\n");
        return 1;
    }

    cdc.RegisterRxCallback(OnReceive);

    // 同步发送, 立即写设备
    uint8_t hello[] = {0xAA, 0x01, 0x02, 0x55};
    if (cdc.TransmitNow(hello, sizeof(hello)) != 1)
    {
        std::fprintf(stderr, "transmit now failed\n");
    }

    // 异步发送, 入队后由后台线程按周期发出
    for (uint8_t i = 0; i < 8; ++i)
    {
        uint8_t frame[] = {0xAA, i, 0x55};
        if (cdc.TransmitAdd(frame, sizeof(frame)) != 1)
        {
            std::fprintf(stderr, "queue full, drop frame %u\n", i);
        }
    }

    std::this_thread::sleep_for(std::chrono::seconds(1));

    // 离开作用域时析构, 自动停止线程并关闭设备
    return 0;
}
```

### 用成员函数接收, 通过上下文指针适配

回调签名是裸函数指针, 需要访问对象状态时用 `void*` 上下文透传.

```cpp
#include "usb_cdc.hpp"

#include <cstdint>
#include <cstdio>

class Controller
{
public:
    explicit Controller(USB_CDC& cdc)
        : cdc_(cdc)
    {
        // 回调函数不携带上下文参数, 这里借助静态成员与单例指针完成绑定
        instance_ = this;
        cdc_.RegisterRxCallback(&Controller::RxTrampoline);
    }

    void RequestStop()
    {
        stop_requested_ = true;
        uint8_t cmd[] = {0xAA, 0x00, 0x55};
        cdc_.TransmitNow(cmd, sizeof(cmd));
    }

private:
    static void RxTrampoline(uint8_t* data, uint16_t length)
    {
        if (instance_ != nullptr)
        {
            instance_->OnFrame(data, length);
        }
    }

    void OnFrame(uint8_t* data, uint16_t length)
    {
        if (length < 2)
        {
            return;
        }

        if (data[0] == 0xAA && data[1] == 0x10)
        {
            stop_requested_ = true;
        }
    }

    USB_CDC& cdc_;
    bool     stop_requested_ = false;

    static Controller* instance_;
};

Controller* Controller::instance_ = nullptr;

int main()
{
    USB_CDC cdc(USB_CDC_DEFAULT_DEVICE);

    if (!cdc.IsOpen())
    {
        return 1;
    }

    Controller controller(cdc);
    controller.RequestStop();

    return 0;
}
```

### 运行中调整发送周期

```cpp
USB_CDC cdc("/dev/ttyACM0");

// 初始周期为 50 毫秒
cdc.SetTxPeriodMs(50);

// 高频模式下改为 5 毫秒
cdc.SetTxPeriodMs(5);
std::printf("tx period %u ms\n", cdc.GetTxPeriodMs());
```

## 构建集成

该模块只有两个源文件, 没有外部依赖. 若以 CMake 集成, 直接加入源文件列表即可.

```cmake
add_library(usb_cdc STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/usb_cdc/usb_cdc.cpp
)

target_include_directories(usb_cdc PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/usb_cdc
)

target_link_libraries(usb_cdc PUBLIC pthread)
```

## 部署注意

Linux 下普通用户默认无法直接访问 `/dev/ttyACM0`. 可以将当前用户加入 `dialout` 组, 或为设备添加 udev 规则. 加入用户组后需要重新登录才能生效.

```bash
sudo usermod -aG dialout $USER

# 或临时授权单次会话
sudo chmod 666 /dev/ttyACM0
```
