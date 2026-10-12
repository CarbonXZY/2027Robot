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
