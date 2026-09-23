// main.cpp — 通信层独立线程测试（不依赖 ROS，接真下位机）
//
// 和 comm_test_node 等价：打开 USB-CDC、接上中间层，控制线程每 100ms
// 发 100/200/300/400，打印解析回来的 4 轮速度/位置。用 std::thread 代替
// rclcpp::WallTimer，普通 g++ 编译，脱离 colcon/ament。
//
// 用法：
//   ./comm_test_threaded [device]      # device 默认 /dev/ttyACM0

#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>

#include <Communication_Interface.hpp>
#include <motor_base.hpp>
#include <usb_cdc.hpp>

using Control_Frame::Struct_Motor_Base;
using Control_Frame::USB_Communication_Interface;

namespace {

constexpr size_t kWheels = 4;
constexpr uint8_t kId     = 1;

#pragma pack(push, 1)
struct Struct_Tx
{
    float velocity[kWheels];  // 下行：目标速度
};
struct Struct_Rx
{
    Struct_Motor_Base motor[kWheels];  // 上行：速度 + 位置
};
#pragma pack(pop)

Struct_Tx Tx;
Struct_Rx Rx;

volatile std::sig_atomic_t g_running = 1;
void OnSigint(int) { g_running = 0; }

// Init 的 send_function 是裸函数指针（不是 std::function），只能绑定无捕获函数/λ，
// 用全局指针引用 USB_CDC 实例（与 Task_Init 用 g_usb_device 一个思路）。
USB_CDC* g_cdc = nullptr;

int64_t SendCallback(uint8_t* data, size_t length)
{
    return g_cdc->TransmitNow(data, static_cast<uint16_t>(length)) == 1
        ? static_cast<int64_t>(length) : -1;
}

}  // namespace

int main(int argc, char** argv)
{
    const std::string device = (argc > 1) ? argv[1] : USB_CDC_DEFAULT_DEVICE;
    std::signal(SIGINT, OnSigint);

    USB_CDC cdc(device);  // 构造即打开设备、起 Rx/Tx 线程
    if (!cdc.IsOpen())
    {
        std::fprintf(stderr, "打开设备失败: %s\n", device.c_str());
        return 1;
    }

    // 接中间层（与 Task_Init 等价）
    g_cdc = &cdc;
    cdc.RegisterRxCallback([](uint8_t* data, uint16_t length) {
        USB_Communication_Interface.Rx_RptlCallback(data, length);
    });
    USB_Communication_Interface.Init(&SendCallback);

    if (!USB_Communication_Interface.Register(kId, &Tx, &Rx, sizeof(Tx), sizeof(Rx)))
    {
        std::fprintf(stderr, "Register 失败\n");
        return 1;
    }

    // 控制线程：100ms 一周期（替代 ROS wall_timer）
    std::thread ctrl([&] {
        while (g_running)
        {
            for (size_t k = 0; k < kWheels; ++k)
            {
                Tx.velocity[k] = static_cast<float>(k + 1) * 100.0f;
            }
            USB_Communication_Interface.Send();

            std::printf("发 100/200/300/400");
            for (size_t k = 0; k < kWheels; ++k)
            {
                std::printf(" w%zu %.3f/%.3f", k, Rx.motor[k].velocity, Rx.motor[k].position);
            }
            std::printf("\n");

            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    });

    ctrl.join();

    std::fflush(stdout);
    // 不走析构：~USB_CDC 会 join 阻塞在 read() 的接收线程（与 comm_test_node 同样的规避）
    std::_Exit(0);
}
