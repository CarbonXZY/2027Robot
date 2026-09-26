# Communication_Interface 通信中间件

算法与通信之间的中间层。上层的硬件接口（如 `chassis`）只绑定几个 C 结构体，
本层负责把这些结构体打包成字节流交给传输层，并在收到字节流时解包、校验、
分发回结构体。

本层**不知道 USB 的存在**，只认识一个裸函数指针和一个接收入口，
因此可以脱离 ROS 与真实设备单独编译和测试（见文末「仿真」）。

## 文件组成

| 文件 | 说明 |
| --- | --- |
| `Communication_Interface.hpp` | 类声明、线格式与常量定义、接口注释 |
| `Communication_Interface.cpp` | 打包、解包、校验、分发实现，以及全局实例定义 |

## 线格式

一帧的布局如下，**线上没有同步头，也没有长度字段**：

```
[ id : 1 字节 ][ data : tx_size/rx_size 字节 ][ crc16_lo ][ crc16_hi ]
```

- `id` 是 1 字节，由调用方在 `Register` 时指定，收发两侧必须一致。
- `data` 的长度**不在线上传输**，而是由收侧按 `id` 从注册表里查 `rx_size` 得到。
  这是本格式最关键的约定：**收发两侧对每个 id 的尺寸必须达成一致**。
- `crc16` 覆盖 `[id][data]`，算法为 CRC16/CCITT-FALSE
  （poly `0x1021`，init `0xFFFF`，输入输出均不反转，xorout 0），
  低字节在前。实现见 `Algorithm/alg_crc.h`。

一包 = 若干帧首尾相连。因为每帧自带 id 且长度可查，**任何一个由完整帧组成的
字节段都可以被独立解析**，所以发送侧可以在帧边界上任意切段，接收侧也不关心
一包里有几帧。

## 常量

| 名称 | 值 | 含义 |
| --- | --- | --- |
| `kMaxFrames` | 8 | 注册表容量，进程级硬上限 |
| `kBufferSize` | 128 | 发送打包缓冲大小，也是注册时 tx 总占用量的上限 |
| `kIdSize` | 1 | id 字段字节数 |
| `kCrcSize` | 2 | crc16 字段字节数 |
| `kFrameOverhead` | 3 | 每帧固定开销，等于 `kIdSize + kCrcSize` |
| `kMaxSendSize` | 64 | 单次交给 `send_function` 的字节上限，超了就切段发 |

`kMaxSendSize` 是留给传输层接收缓冲的余量。它与 `kBufferSize` 是两个独立的概念：
**注册总量可以到 128 字节，而每次实际发送的段不会超过 64 字节**，
中间的差额由 `Send()` 自己切段消化。

## 接口

| 接口 | 说明 |
| --- | --- |
| `void Init(Communication_Function send)` | 绑定发送函数，通常在 bringup 时调用一次 |
| `bool Register(uint8_t id, const void* tx, void* rx, uint8_t tx_size, uint8_t rx_size)` | 注册一帧，成功返回 `true` |
| `void Send()` | 打包所有已注册的 tx 帧并交给 `send_function` |
| `void Rx_RptlCallback(uint8_t* data, uint16_t length)` | 接收入口，由传输层的接收线程调用 |

发送回调类型为 `using Communication_Function = int64_t (*)(uint8_t* buf, size_t len)`，
返回值是实际接受的字节数，返回负值表示失败。它是**裸函数指针而非 `std::function`**，
所以只能绑定静态函数或无捕获 lambda。

全局实例 `Control_Frame::USB_Communication_Interface` 定义在 `.cpp` 末尾，
进程内共享同一个注册表和同一个 `send_function`。

### Register 的拒绝条件

以下任一情况返回 `false`，且不做任何修改：

- `tx` 与 `rx` 同时为 `nullptr`；
- `tx_size` 与 `rx_size` 同时为 `0`；
- 该 `id` 已经注册过（`Find` 是 first-match-wins，重复注册会让后来的永远被遮住）；
- 注册表已满（`count >= kMaxFrames`）；
- 该帧是 tx 帧，且加上它之后的 tx 总占用量会超过 `kBufferSize`。

`tx` 与 `rx` 可以单独为 `nullptr`，用来表达只发或只收。
注意**只有 tx 帧计入容量**，纯接收帧不占打包缓冲。

注册是**不可撤销**的，没有反注册接口，`Init` 也不重入（不清空 `count`/`used`）。
设计上假设绑定只在 bringup 时做一次。

## 发送

`Send()` 由控制线程调用，按注册顺序遍历所有 tx 非空的表项，逐帧写入打包缓冲：

1. 累加当前段；若**下一帧装进当前段会超过 `kMaxSendSize`**，先把当前段交给
   `send_function` 发出去，再从头开始新的一段。
2. 循环结束后把尾段发出。

因为只在帧边界上切，**每一段的开头必定是一个 id**，所以对端按包解析时天然对齐。
前面提到的 128 / 64 差额就是靠这一步消化的：注册总量超过 64 字节时，
`Send()` 会在帧边界上切成 n 段，每段单独调一次 `send_function`。
最坏情况是每帧独占一段，即 n 最多等于 tx 帧数（受 `kMaxFrames` 限制，至多 8）。

一个例外：**单帧本身就超过 `kMaxSendSize` 时无法切分**（从帧中间切开对端必丢），
只能让它独占一段、临时超限发出去。因此单个结构体不应超过 64 字节。

## 接收

`Rx_RptlCallback()` 由传输层的接收线程直接调用，就地解包、校验、分发，
**不跨调用保留任何状态**——没有重组缓冲，一包进来就处理完一包。

逐帧推进的流程：

1. 从偏移 `i` 处取 id，用 `Find` 查表项；查不到（或该表项只注册了 tx）
   就**停止解析**——id 无效，这一帧的长度无从得知，后面的字节无法定位。
2. 若 `i` 处的剩余长度不足一帧，说明这包不完整，**停止解析**。
3. 校验 CRC，通过就把 `data` 段 `memcpy` 进注册的 `rx` 结构体；不通过就**丢弃这一帧**，
   继续解析下一帧。

### 不同破坏位置的实际后果

以下行为由 `sim/` 下的仿真实测确认：

| 破坏位置 | 结果 |
| --- | --- |
| 数据字节 | 该帧 CRC 不过被丢弃，**同一包里后面的帧照常收到** |
| id 字节，改成未注册的值 | 从该帧起**整段尾部全部丢失**，直到下一个包才恢复 |
| id 字节，改成另一个已注册且 `rx_size` 相同的 id | 帧长仍然对得上，只有这一帧 CRC 不过被丢弃，后续帧不受影响 |
| 整帧被拆成两包发 | **整段丢失**，因为本层不重组（见「已知限制」） |

每帧一个 CRC 的意义就在这里：坏一帧只丢一帧，而不是丢掉整包。

## 线程与并发

- `Send()` 只应在控制线程调用。打包缓冲 `buffer` 与注册表在发送期间被读写。
- `Rx_RptlCallback()` 在接收线程中被调用，会写入各 `rx` 结构体。
- 因此 **`rx` 结构体是「接收线程写、控制线程读」**。两侧对同一结构体的访问是
  普通的内存读写，没有任何同步。这是刻意的设计取舍：控制器输入本身就有滞后，
  丢一个周期的采样对闭环稳定性没有影响，所以不做加锁也不做强同步。
- 注册表在 bringup 之后只读，两个线程同时读是安全的。

## 已知限制

- **`kMaxFrames = 8`** 是进程级上限。全局只有一个注册表、且只在 bringup 绑一次，
  所以一个进程里最多挂 8 个设备（底盘这种整帧绑定的只占 1 个）。
- **绑定只在 bringup 做一次**。`Init` 不可重入、`Register` 不可撤销，
  `on_cleanup` 之后重走 `on_configure` 会因为重复 id 被拒。USB 掉了就重启节点。
- **不做重组**。前提是下位机固件保证一次 `write` 对应一个 USB 传输，
  驱动程序按 URB 整块塞进 tty 缓冲，所以 `read()` 边界与 `write()` 边界对齐。
  若这个前提不成立，一包被拆开会丢掉整段（影响有限，不会持续失步）。
- **单帧超过 64 字节无法切分**，会临时超限独占一段发出。

## 使用示例

硬件接口侧只需声明收发结构体，在配置阶段绑定，之后读写结构体即可：

```cpp
#include <Communication_Interface.hpp>

struct Struct_Motor_Tx { float velocity[4]; };
struct Struct_Motor_Rx { Control_Frame::Struct_Motor_Base motor[4]; };

Struct_Motor_Tx Tx;
Struct_Motor_Rx Rx;

// 绑定：整个结构体一帧，id=1，下行 16 字节、上行 36 字节
Control_Frame::USB_Communication_Interface.Register(1, &Tx, &Rx, sizeof(Tx), sizeof(Rx));

// 控制线程：写结构体后打包发出
Tx.velocity[0] = 1.5f;
Control_Frame::USB_Communication_Interface.Send();

// 控制线程：读结构体（由接收线程在数据到达时填好）
const float v = Rx.motor[0].velocity;
```

传输层的接线方式见 `Task/tsk_config_and_callback.cpp`：在 `Task_Init()` 里打开
USB-CDC，用两个无捕获 lambda 分别接上收发，再调用 `Init()`。

## 仿真

`sim/` 下有一套脱离 ROS 的收发仿真，不依赖真实设备与 termios，
可直接用 g++ 编译运行：

```bash
g++ -std=c++17 -Wall -Wextra -Wpedantic \
  -Isrc/shared_package/Middleware -Isrc/shared_package/Device -Isrc/shared_package/Algorithm \
  sim/sim_slave.cpp sim/sim_host.cpp \
  src/shared_package/Middleware/Communication_Interface.cpp -o sim/sim_test.exe
./sim/sim_test.exe
```

- `sim_host.cpp` 用的是真的本中间件（全局实例），发送回调是一条「假传输」，
  把打包好的字节直接喂给从机，再把从机的回包喂回 `Rx_RptlCallback()`。
- `sim_slave.cpp` **独立重写了一遍线格式与 CRC**，且刻意不 include
  `motor_base.hpp`、自己按相同布局写了一遍结构体，用来交叉验证两侧对线格式和
  内存布局的理解确实一致。

覆盖的用例：重复 id 注册被拒、位精确往返（含 `-0.0`／大数／小数）、
position 按 id 落位、以及上表中的三种破坏注入。
