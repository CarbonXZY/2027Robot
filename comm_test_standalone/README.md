# comm_test 独立线程测试（不依赖 ROS）

把 `comm_test_node` 从 ROS 里拿出来，用 `std::thread` 代替 `rclcpp::WallTimer`，
用普通 g++ 编译、脱离 colcon/ament，直接接真下位机测 `shared_package` 的
「中间层 Communication_Interface + 传输层 USB_CDC」。

## 文件

| 文件 | 说明 |
| --- | --- |
| `main.cpp` | 线程版测试主程序（发送 + 打印接收） |
| `shared_package/` | 测试依赖文件（复制自 `../src/shared_package`，见 sync.sh） |
| `build.sh` | g++ 编译 |
| `sync.sh` | 从 `../src/shared_package` 重新同步依赖文件 |

## 编译

```bash
./build.sh
```

## 运行

```bash
./comm_test_threaded              # 打开 /dev/ttyACM0
./comm_test_threaded /dev/ttyACM1 # 指定设备
```

启动后控制线程每 100ms 下发 100/200/300/400 四个轮速，并打印解析回来的
`w0..w3 velocity/position`。Ctrl-C 停止。

## 说明

- 接线与 `Task_Init()` 等价，但设备路径可通过参数指定。
- 结束时用 `std::_Exit(0)` 跳过 `~USB_CDC`（它 join 阻塞在 read() 的接收线程，
  与 `comm_test_node` 相同的规避）。
- `shared_package/` 是 `../src/shared_package` 的副本，改了源头后跑 `./sync.sh`。
