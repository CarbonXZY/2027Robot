# USB CDC 通信封装

对 Linux 下 USB CDC ACM 虚拟串口设备 `/dev/ttyACMx` 的轻量 C++ 封装. 基于 POSIX 串口接口实现, 不依赖 ROS 与任何第三方库, 可直接用于嵌入式通信板, STM32 USB CDC 固件, 或其他会枚举成 ACM 设备的 USB 转串口模块.

## 文件组成

| 文件 | 说明 |
| --- | --- |
| `usb_cdc.hpp` | 类声明, 常量定义, 接口注释 |
| `usb_cdc.cpp` | 设备打开与配置, 收发线程实现 |

## 设计说明

类 `USB_CDC` 与单个设备路径一一绑定. 构造时打开设备并把串口配置为原始模式, 随后启动两个后台线程.

- 接收线程: 用 `poll` 带超时等待设备可读, 可读时读出一段数据并在接收线程上下文中调用已注册的回调. 超时返回是为了让循环能定期检查运行标志, 从而保证析构时线程能真正退出, 不会挂在阻塞读上.
- 发送线程: 检查发送队列 FIFO, 队列非空时取出一帧写入设备, 队列为空时休眠一个周期再查. 空闲时不占用 CPU.

串口参数固定为 8N1, 无校验, 无硬件流控, 无软件流控, 关闭回显与信号处理. `VMIN` 为 1 且 `VTIME` 为 0, 但每次 `read` 之前都会先 `poll` 确认可读, 所以阻塞读不会长时间挂住线程. 因此该封装不理解任何上层协议, 帧头, 帧尾与校验都由使用者在回调中或调用前自行处理.

资源管理遵循 RAII. 析构函数会自动停止两个线程并关闭文件描述符, 使用者不需要手动调用停止接口. 拷贝构造与拷贝赋值被显式删除, 因为该类持有线程与文件描述符等独占资源.

设备被拔出或发生异常时, 接收线程会检测到并立即关闭设备, 复位运行标志, 再调用已注册的断连回调. 因此回调被调用时设备资源已经归还: 文件描述符已关闭, 发送线程正在退出, 接收线程不会再触碰设备. 回调里可以直接调用 `Connect` 重连, 不会因为 join 当前线程而死锁 —— `Disconnect` 对"在自己线程上被调用"做了保护, 这种情况改为 detach 该线程, 让它在回调返回后自行结束. 所以重连逻辑(含失败重试)可以直接写在回调里, 不必再置标志转交其他线程.

## 常量

| 名称 | 值 | 含义 |
| --- | --- | --- |
| `USB_CDC::kTxQueueSize` | 64 | 异步发送队列最大帧数, 队列满时 `TransmitAdd` 返回失败 |
| `USB_CDC::kRxPollTimeoutMs` | 10 | 接收线程 `poll` 的等待超时毫秒数, 也是析构等待接收线程退出的上界 |
| `USB_CDC::kTxPollIntervalMs` | 10 | 接收线程读取出错(非 `EINTR`)后的重试等待毫秒数 |
| `USB_CDC::kMaxFrameSize` | 1024 | 单次 `read` 的最大字节数, 决定了接收回调中单包数据的上限 |
| `STM32_CDC` | 正则 | 匹配 STM32 虚拟串口设备名的默认正则, 位于全局命名空间, 供 `FindDevice` 使用 |

## 接口

| 接口 | 说明 |
| --- | --- |
| `USB_CDC(const std::string& device, uint32_t tx_period_ms = 10)` | 打开设备并启动收发线程. 打开失败不抛异常, 只向 `stderr` 打印错误, 需要用 `IsOpen` 检查 |
| `~USB_CDC()` | 停止线程, 等待线程退出, 关闭设备 |
| `void RegisterRxCallback(RxHandle callback)` | 注册接收回调, 线程安全, 可运行中替换, 传 `nullptr` 可取消回调 |
| `void RegisterDisconnectCallback(DisconnectHandle callback)` | 注册断连回调, 设备拔出或异常断开时被调用; 用户主动调用 `Disconnect` 不触发. 回调内可直接调用 `Connect` 重连 |
| `uint8_t TransmitNow(uint8_t* data, uint16_t length)` | 同步发送, 立即写设备. 返回 1 表示整帧写完, 返回 0 表示失败 |
| `uint8_t TransmitAdd(uint8_t* data, uint16_t length)` | 异步发送, 拷贝数据入队. 返回 1 表示入队成功, 返回 0 表示入队失败 |
| `bool IsOpen() const` | 设备文件是否处于打开状态; 断连后变为 false |
| `bool IsRunning() const` | 收发线程是否在运行; 断连后变为 false |
| `bool Connect()` | 重新打开设备并启动收发线程. 幂等, 已连接时会先停下旧线程再重连, 失败返回 false. 可以在断连回调里直接调用 |
| `void Disconnect()` | 停止收发线程并关闭设备. 用户主动断开不触发断连回调. 在接收线程(即回调)中调用时会 detach 该线程而非 join 它, 避免自 join 死锁 |
| `uint32_t GetTxPeriodMs() const` | 读取异步发送周期 |
| `void SetTxPeriodMs(uint32_t period_ms)` | 设置异步发送周期 |
| `std::string FindDevice(const std::string& spec)` | 按设备路径或正则匹配设备节点, 见下 |

回调类型定义为 `using RxHandle = void (*)(uint8_t* data, uint16_t length)` 与 `using DisconnectHandle = void (*)()`. 它们都是裸函数指针而非 `std::function`, 因此只能绑定静态函数或全局函数, 若需要绑定成员函数请用带上下文指针的适配层包一层.

`FindDevice` 在 `spec` 以 `/` 开头时直接当作设备路径返回; 否则当作正则表达式, 先在 `/dev/serial/by-id` 中按设备名匹配(返回软链接本身, 不解析目标, 保证重连时节点号变了也能跟到同一物理设备), 再回退到扫描 `/dev/ttyACM*` 并按节点名或 sysfs 中的 product 字符串匹配. 找不到返回空串. 传 `STM32_CDC` 即可匹配常见的 STM32 虚拟串口.

## 返回值约定

两个发送接口都以 `uint8_t` 返回状态. `TransmitAdd` 返回 0 只表示队列满或参数非法, 不表示设备写入失败, 因为真正的写操作发生在后台线程. 需要确认数据确实写入设备的场景应使用 `TransmitNow`. 两个接口的成功返回值都是 1, 可以直接作为布尔值使用.

`TransmitNow` 会把整帧写完才返回: 遇到短写或被信号打断( `EINTR` )时会续写剩余部分. 只有在设备错误导致无法写满, 参数非法, 或设备未打开时才返回 0. 后台发送线程采用同样的写满策略, 写失败时丢弃该帧并向 `stderr` 打印错误, 不会重排队列.

## 线程与并发

- 接收回调在接收线程中被调用, 回调内部不能长时间阻塞, 否则会阻塞后续接收. 回调期间持有回调互斥锁, 所以不要在回调里再次调用 `RegisterRxCallback`, 否则会死锁.
- 断连回调同样在接收线程中被调用, 且调用时设备已经关闭( `IsOpen()` 为 false ). 可以在回调里直接调用 `Connect` 重连: `Disconnect` 会识别出"当前就在接收线程上", 改为 detach 而不是 join. 两条约束要遵守: 回调**必须能返回**(被 detach 的线程在回调返回后才结束, 永久阻塞会悬挂一个线程); 回调返回后 `RxThreadLoop` 不能再触碰任何成员变量, 否则会踩到 `Connect` 换过代的新线程与新文件描述符.
- 回调在重连前后是持久的. `Connect` 不会清掉已注册的接收回调与断连回调, 因此重连后不需要重新注册; 反过来, 如果改用"销毁对象再新建"的方式重连, 就必须重新注册, 否则重连后收不到数据.
- 两个发送接口可以跨线程同时调用. 写设备由一把写锁保护, 每帧要么整帧写完要么整帧不写, 同步发送与异步发送线程不会交错写坏数据.
- `TransmitNow` 会等待写锁. 发送线程使用 `try_to_lock` 尝试获取写锁, 取不到时会把该帧放回队首并休眠一个周期后重试.
- 发送队列中的每一帧都是深拷贝, 调用方在 `TransmitAdd` 返回后可以立即复用或释放自己的缓冲区.

## 已知行为

- 设备拔出或异常断开时, 接收线程会检测到 `POLLHUP` / `POLLERR`, 随即关闭设备, 复位运行标志并调用断连回调. 此后 `IsOpen()` 与 `IsRunning()` 均为 false.
- 断连后不会自动重连, 需要上层调用 `Connect`, 通常就写在断连回调里(可循环重试到成功). `Connect` 只会重新打开构造时绑定的那个路径. `FindDevice` 对 by-id 返回的是**软链接本身**(不解析目标), 所以构造时绑定的就是这条稳定软链接, 重新插入后节点号变了软链接会重新指向新节点, 重连自然跟得上; 只有直接以 `/dev/ttyACM0` 这类真实路径构造时, 节点号变化才会让重连失败.
- 如果在断连回调里重连, 那个接收线程会被 detach 出去. 它会在回调返回后结束, 不参与后续的 join, 析构也不会等它; 因此回调一旦返回就必须不再访问对象(例如不要在回调里死循环且永不退出). 若对象需要在回调仍在执行时被销毁, 必须由上层自行保证回调已经返回.
- 发送不做速率限制. 队列中有数据时会尽快连续发出, `tx_period_ms` 只影响队列空时的查询间隔, 抢锁失败后的重试间隔, 以及析构等待发送线程退出的上界.
- 异步发送从入队到发出的延迟最坏为一个 `tx_period_ms` (默认 10 毫秒), 平均约为其一半. 对延迟敏感的数据应使用 `TransmitNow`.
- 析构的等待时间上界约为 `max(kRxPollTimeoutMs, tx_period_ms)`, 设备空闲时也能正常返回, 不会挂在阻塞读上.
- 驱动层不保证帧一定完整到达对端. 若设备在一帧写到一半时被拔掉, 已经进入内核缓冲的字节会丢失, 对端会收到残缺帧. 上层协议应带帧头, 长度与校验, 由接收方丢弃残缺帧并按帧头重新同步.

## 使用示例

以下示例均假定已经 `using namespace Driver;`, 因此省略了 `Driver::` 前缀.

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
    // 队列为空时每 20 毫秒查询一次; 队列非空时立即连续发出
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
    const std::string device = FindDevice(STM32_CDC);

    if (device.empty())
    {
        std::fprintf(stderr, "usb cdc device not found\n");
        return 1;
    }

    USB_CDC cdc(device);

    if (!cdc.IsOpen())
    {
        return 1;
    }

    Controller controller(cdc);
    controller.RequestStop();

    return 0;
}
```

### 处理设备断开与重连

断连回调里可以直接重连, 循环重试到成功为止. 回调所在的接收线程会被 detach, 因此这段等待不会阻塞其他任何东西 —— 收发线程此时本来就已经停了.

需要一个"该放弃了吗"的出口, 否则进程退出时这个线程会一直重试下去. 下面的例子用回调注册时挂上的外部标志位(实际项目里可以接 `rclcpp::ok()` 之类的关闭信号).

注意 `USB_CDC` 在构造时就绑定了设备路径, `Connect` 只会重新打开同一个路径. 因此优先使用 `/dev/serial/by-id` 下的稳定名( `FindDevice` 默认就优先匹配这里 ), 这样设备重新插入后路径不变; 若是节点号会变的场景, 要在重试循环里重新 `FindDevice` 并把新路径交给新的 `USB_CDC` 实例.

```cpp
#include "usb_cdc.hpp"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <string>
#include <thread>

namespace
{
USB_CDC*              g_cdc = nullptr;                 // 回调里重连用
std::atomic<bool>     g_stop_requested{false};         // 进程要关闭了, 别再无脑重试

void OnDisconnect()
{
    // 本函数在接收线程中执行, 设备此时已关闭(IsOpen() 为 false).
    // 这些资源都已归还, 所以可以直接在这里 Connect, 不会自 join 死锁.
    std::printf("device lost, reconnecting...\n");

    while (!g_stop_requested.load())
    {
        if (g_cdc->Connect())
        {
            std::printf("reconnected\n");
            return;   // 回调必须能返回
        }

        // 设备还没插回来, 等一会儿再试. 注意这里不要用太长的间隔:
        // 关闭请求最多要等这么久的 delay 才能生效
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
}
}  // namespace

int main()
{
    const std::string path = FindDevice(STM32_CDC);
    if (path.empty())
    {
        std::fprintf(stderr, "usb cdc device not found\n");
        return 1;
    }

    USB_CDC cdc(path);
    g_cdc = &cdc;
    cdc.RegisterDisconnectCallback(OnDisconnect);

    // ... 主循环收发 ...

    // 退出前先让重连循环停止, 再销毁对象:
    // 被 detach 的接收线程不会被析构 join, 所以不能让它还在跑的时候对象就没了
    g_stop_requested = true;
    std::this_thread::sleep_for(std::chrono::milliseconds(600));
    return 0;
}
```

### 运行中调整发送周期

该周期决定发送线程在队列为空时的查询间隔, 以及抢锁失败后的重试间隔. 调小可以降低入队到发出的延迟, 代价是空闲时唤醒更频繁.

```cpp
USB_CDC cdc("/dev/ttyACM0");

// 空闲时每 50 毫秒查询一次队列
cdc.SetTxPeriodMs(50);

// 改用 5 毫秒, 降低发送延迟
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

## 测试

测试代码在 `test/` 目录, 使用伪终端 pty 充当假设备, 不需要真实硬件, 也不依赖第三方库. 构建产物放在 `test/build/`.

```bash
cd test
make run      # 编译并运行全部用例
make tsan     # 用 ThreadSanitizer 重编并运行, 检查数据竞争
```

| 分组 | 用例数 | 覆盖内容 |
| --- | --- | --- |
| 打开与生命周期 | 5 | 构造打开、打开失败、`Connect` 幂等、`Disconnect`、主动重连 |
| 同步发送 | 3 | 字节精确、非法参数、断开后发送失败 |
| 异步发送与队列 | 4 | 投递、FIFO 顺序、缓冲区深拷贝、队列容量 |
| 接收回调 | 5 | 字节精确、分片累计、替换回调、注销回调、原始模式无字节转换 |
| 断连检测与重连 | 3 | 拔出触发回调且仅一次、主动断开不触发、在回调里直接 `Connect` 重连并验证新链路可用 |
| 析构 | 3 | 空闲、队列有积压、拔出后均须及时返回 |
| 并发 | 1 | 同步与异步同时发送时的帧完整性 |
| 周期配置 | 1 | `GetTxPeriodMs` / `SetTxPeriodMs` |
| 设备查找 | 3 | 直连路径透传、空串与非法正则、无匹配 |
| 计时 | 2 | 空闲 CPU 占用、入队到发出的延迟上界 |

测试结果:

| 项目 | 结果 |
| --- | --- |
| 常规运行 | 30 用例通过, 0 失败; 178 次断言, 0 次失败 |
| 重复运行 | 连续 3 次运行, 逐用例结果一致 |
| ThreadSanitizer | 28 用例通过, 0 失败; 156 次断言, 0 次失败; 无数据竞争报告(2 个计时用例在该模式下自动跳过) |
| 变异测试 | 分别把接收线程的 `poll` 超时改坏、去掉发送线程的休眠、把一帧拆成两次 `write`、去掉 `Disconnect` 的自 join 保护, 对应用例均失败; 其余用例不受影响 |

## 部署注意

Linux 下普通用户默认无法直接访问 `/dev/ttyACM0`. 可以将当前用户加入 `dialout` 组, 或为设备添加 udev 规则. 加入用户组后需要重新登录才能生效.

```bash
sudo usermod -aG dialout $USER

# 或临时授权单次会话
sudo chmod 666 /dev/ttyACM0
```
