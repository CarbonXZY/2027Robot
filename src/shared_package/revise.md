# 修订记录：没有 USB 设备时 Ctrl+C 停不下来

日期：2026-10-12
范围：让"等设备"的忙等重试可以被 Ctrl+C 打断，其余行为不变

## 现象

设备没插（或没枚举出来）时启动 `./launch.sh`，按 Ctrl+C 停不下来，终端反复打印：

```
[WARNING] [launch]: user interrupted with ctrl-c (SIGINT) again, ignoring...
```

## 根因（gdb 抓到的调用栈）

```
main → ControllerManager::ControllerManager() → init_resource_manager()
     → ResourceManager::set_component_state() → Sensor::configure()
     → TelecontrolSystem::on_configure()            （chassis / leg 同理）
     → Task::TaskInit() → Utils::RetryUntil() → sleep_for(500ms)   ← 主线程卡在这
```

`TaskInit()` 里的 `RetryUntil` 是**无限重试**（`retry_times = 0`），而这段代码跑在
ros2_control_node 的**构造期**（`on_configure` 里），于是：

1. 主线程卡住 → 节点没 spin → `/controller_manager/*` 服务不 advertise；
2. 4 个 `spawner` 死等服务，也退不出来；
3. rclcpp 的 SIGINT 处理只设"shutdown 标志位"，打断不了卡在 `sleep_for` 里的线程，
   主线程永远不返回，进程不退出；
4. launch 已经在第一次 Ctrl+C 时进入 shutdown，之后的 Ctrl+C 只会打印 "again, ignoring"，
   最后靠 launch 自己的升级机制（SIGINT → 5s → SIGTERM → 10s → SIGKILL）才收场。

## 改动清单

| 文件 | 位置 | 改动 |
| --- | --- | --- |
| [Utils/utils.hpp](Utils/utils.hpp) | `RetryUntil` / 新增 `NeverCancel` | 增加可选的**取消谓词**：每轮 `action()` 之前问一次，返回 true 就放弃并返回 false；默认 `NeverCancel`（永不取消），即下游不传时行为与改前完全一致 |
| [Task/tsk_config_and_callback.hpp](Task/tsk_config_and_callback.hpp) | `TaskInit` | 增加可选参数 `shutdown_requested`（`std::function<bool()>`，默认空 = 不响应关闭请求），并带上 `<functional>` |
| [Task/tsk_config_and_callback.cpp](Task/tsk_config_and_callback.cpp) | `TaskInit` | 把 `shutdown_requested` 接到 `RetryUntil` 的取消谓词；被取消时直接退出进程（见下节） |
| [../chassis/src/chassis_system.cpp](../chassis/src/chassis_system.cpp) | `on_configure` | 调用点传入 `[]{ return !rclcpp::ok(); }` |
| [../leg/src/leg_system.cpp](../leg/src/leg_system.cpp) | `on_configure` | 同上 |
| [../Telecontrol/src/telecontrol_system.cpp](../Telecontrol/src/telecontrol_system.cpp) | `on_configure` | 同上 |

`rclcpp::ok()` 由调用点（ROS 侧的插件）提供，`shared_package` 仍然不依赖 ROS——
这是它 CMakeLists 里"纯工具库，无 ROS 运行时依赖"的约束。

## 为什么被取消时直接退出进程，而不是 `return false`

第一版实现在取消时 `return false`，让 `on_configure` 返回 `ERROR`，结果**每次 Ctrl+C 都变成一次崩溃**：

```
terminate called after throwing an instance of 'std::runtime_error'
  what():  Failed to set the initial state of the component : TelecontrolSystem to active
[ros2_control_node-2] Aborted (Signal sent by tkill())
```

即 Humble 的 `ControllerManager` 在构造期对 configure 失败是**抛异常**，异常没人接 →
`std::terminate` → `abort`。本机 `core_pattern` 走 apport，于是每按一次 Ctrl+C 就在
`/var/crash` 留一份 ~1.7MB 的 core dump（还有可能弹桌面崩溃提示）。

所以取消分支改为 `std::_Exit(EXIT_FAILURE)`：此时设备从未打开、本库也没起任何线程，
跳过析构最安全，退出也最快。

## 行为对比

| 场景 | 改前 | 改后 |
| --- | --- | --- |
| 设备在（正常启动） | 第一次尝试就打开，继续原有流程 | **完全一致**（取消谓词在第一轮返回 false，`action`/`judge`/节奏都没变） |
| 设备不在，不按 Ctrl+C | 无限重试，500ms 一轮，设备一出现就接上 | **完全一致**（仍是无限重试，设备出现即接上） |
| 设备不在，按 Ctrl+C | 卡住，第二次起只打印 "again, ignoring"，约 10s 后才被 SIGKILL 杀掉 | 最迟一轮（≤500ms）后跳出等待，打印一行 `TaskInit: 等待 USB-CDC 设备时收到关闭请求，退出`，进程直接退出 |

## 验证记录（无设备实测）

- 启动 8s 后仍在重试（无限等待的旧行为未被破坏）；
- 发 SIGINT（等价于 Ctrl+C）：**0.07s 内全部进程退出**，`ros2 launch` 退出码 0；
- 没有 abort、没有堆栈打印，`/var/crash` 无新增文件；
- 日志末尾：

```
[ros2_control_node-2] TaskInit: 等待 USB-CDC 设备时收到关闭请求，退出
[ERROR] [ros2_control_node-2]: process has died [pid 19059, exit code 1, ...]
```

（末尾那条 ERROR 是 launch 对非零退出码的常规告警；Ctrl+C 时其它节点也都是
`exit code -2` 的 ERROR，属于正常现象。）

---

# 修订记录：USB 掉线后自动重连

日期：2026-10-12
范围：下位机（STM32 USB-CDC）掉线后自动反复重连，可被 Ctrl+C 打断；启动路径行为不变

## 背景

原来只有"启动时等设备"(见上一节)。设备**运行中**被拔掉时，驱动会调用断连回调，
而当时的回调是个空函数 —— 拔了就是彻底失联，`IsOpen()`/`IsRunning()` 全为 false，
控制循环因为收不到下位机回包而停止发送，只能重启整套程序。

## 关键约束：回调里为什么原本不能直接 `Connect`

断连回调是在驱动的**接收线程**里被调用的，而 `Connect()` 的第一步是
`Disconnect()` → `rx_thread_.join()` —— join 的是当前线程自己：join 要等线程函数返回，
线程函数又在等 join 返回，必然死锁（实测会抛 `Resource deadlock avoided` → `abort`）。
所以上一版驱动文档里写的是"回调里只能置标志，重连转交其他线程"。

这层绕行是多余的：回调被调用时**设备资源其实已经全部归还**了 ——
驱动在调用回调之前就已经 `running_ = false`、`CloseDevice()`（fd 已关）。真正缺的只是
"别 join 自己"。

## 改动清单

| 文件 | 位置 | 改动 |
| --- | --- | --- |
| [Driver/usb_cdc/usb_cdc.cpp](Driver/usb_cdc/usb_cdc.cpp) | `Disconnect()` | 加**自 join 保护**：若当前就在接收线程上，`join()` 改为 `detach()`（该线程在回调返回后紧接着退出），否则照常 join |
| [Driver/usb_cdc/usb_cdc.hpp](Driver/usb_cdc/usb_cdc.hpp) | `RegisterDisconnectCallback` / `Disconnect` / `Connect` | 契约反转：从"不要在回调里 Connect"改为"可以，但回调必须能返回、返回后 `RxThreadLoop` 不得再触碰成员" |
| [Driver/usb_cdc/README.md](Driver/usb_cdc/README.md) | 设计说明、接口表、线程与并发、已知行为、重连示例 | 同步新契约；重连示例由"置标志交给主循环"改为"回调用循环重试" |
| [Driver/usb_cdc/test/driver_test.cpp](Driver/usb_cdc/test/driver_test.cpp) | 新增 `Reconnect_from_disconnect_callback` | 软链接当稳定名，验证回调里 `Connect` 不卡死、只回调一次、重连后数据真的能收到 |
| [Task/tsk_config_and_callback.cpp](Task/tsk_config_and_callback.cpp) | 新增 `TryOpenDevice` / `WaitForDeviceOpen` | 把 `TaskInit` 里那段 `RetryUntil` 抽成可复用函数，启动与重连共用 |
| [Task/tsk_config_and_callback.cpp](Task/tsk_config_and_callback.cpp) | `McuDisconnectCallback` | 回调里直接重连，循环重试到成功；关闭请求是唯一出口 |
| [Task/tsk_config_and_callback.cpp](Task/tsk_config_and_callback.cpp) | `TaskInit` | 把 `shutdown_requested` 存进静态量供回调用；启动改调 `WaitForDeviceOpen` |

### 抽出来的两个函数

- `TryOpenDevice()`：打开设备**一次**。`g_usb_device == nullptr`（首次启动）就
  `FindDevice` + 建对象；非空（重连）就 `g_usb_device->Connect()` —— **复用同一个对象**，
  因为收发回调是成员变量，`Connect` 不会清掉它们，所以不用重新注册。
- `WaitForDeviceOpen(shutdown_requested)`：`RetryUntil(TryOpenDevice, IsOpen, 500ms, 0, cancel)`。
  `TaskInit` 和 `McuDisconnectCallback` 都调它，因此重连的**节奏、判定、打断方式与刚启动完全一致**。

### 为什么重连不新建对象

1. `Connect` 不清回调，省掉重新注册（若新建对象就必须重新注册，否则重连后收不到数据）；
2. 在回调里销毁旧对象等于在它自己的线程里跑析构，能成立但依赖"回调返回后不碰成员"
   这条很脆的约定，能避则避。

## 行为对比

| 场景 | 改前 | 改后 |
| --- | --- | --- |
| 启动时有设备 | 打开，继续原有流程 | **完全一致**（同一段 `WaitForDeviceOpen`） |
| 启动时无设备 | 无限重试，Ctrl+C 打断后 `_Exit` | **完全一致** |
| 运行中拔线 | 永久失联，控制循环停发，只能重启程序 | 回调立刻进入重连循环（500ms 一轮），设备插回即自动恢复；恢复后收到第一包，存活标志位自动拉起，和刚启动一样 |
| 运行中拔线 + Ctrl+C | —— | 最迟一轮（≤500ms）跳出循环，打印一行后线程退出，进程正常结束 |

## 已知限制

- **节点号变了就重连不上。** `Connect` 只重开构造时绑定的那条路径，而
  `FindDevice` 对正则返回的是 `fs::canonical` 之后的**真实路径**（`/dev/ttyACM0`），
  不是 `/dev/serial/by-id` 里的稳定软链接。若重新枚举后节点号变了（例如 ACM0 被别的
  设备占走），重连会一直失败。这种情况会在 `stderr` 打一行告警（同一条路径只打一次），
  避免无声空转；要彻底解决得让 `FindDevice` 能返回 by-id 软链接本身而不是它的目标 —— 
  这属于驱动接口改动，本次未做。
- 只在驱动检测到 `POLLHUP`/`POLLERR` 时触发。下位机自己复位但 tty 没断（无 HUP）时
  回调不会触发，重连也就不会启动（此时 `g_mcu_alive` 会变 false，但不会重连）。

## 验证记录

**驱动层**（`Driver/usb_cdc/test`，伪终端假设备）：

- `make run`：30 用例通过、0 失败，178 次断言、0 失败；连续 3 次运行结果一致；
- `make tsan`：28 用例通过、0 失败，156 次断言、0 失败，无数据竞争报告；
- 变异测试：去掉 `Disconnect` 的自 join 保护后，新用例立刻 `abort`
  （`Resource deadlock avoided`），其余用例不受影响。

**Task 层 + 驱动**（端到端演练，脚本在 `/tmp/task_reconnect_check/`，
用伪终端当假设备、软链接当稳定名，编译时把设备选择串换成该软链接）：

```
[1] 假设备 A = /dev/pts/3, 稳定名 .../dev_link -> A
[2] TaskInit 返回, 设备已打开;  链路存活 = true
[3] 拔掉 A(关闭 master), 稳定名已指向新设备 B = /dev/pts/4
    Task: 下位机断开，开始重连
    Task: 下位机已重连                      <- 回调里的 Connect 没死锁
[4] 重连后链路存活 = true                   <- 新链路上收得到数据
[5] 拔掉 B 且不插回, 800ms 后置关闭请求(等价 Ctrl+C)
    USB_CDC: failed to open device ...      <- 重试循环在跑
    Task: 重连等待中收到关闭请求，放弃重连    <- 循环退出
[6] main 返回, 等 ASan 裁决 -> 退出码 0, 无报告
```

第 6 步是配合 AddressSanitizer 的判据：如果关闭请求没能让循环退出，`main` 返回时
`g_usb_device` 的静态析构就会和还在重试的线程撞车，被 ASan 抓成 use-after-free。

**真机**：当前 `/dev/ttyACM0` 在线，`TaskInit` 8ms 内打开成功，启动路径无回归。

**未覆盖**：真实拔插（本机无免密 sudo，无法 unbind USB 设备）；节点号变化时的告警分支
（该分支只在存在 `/dev/serial/by-id` 且正则命中时才会走到）。
