// comm_test_node.cpp
//
// 最小验证节点：Task_Init() 打开 USB-CDC 并把中间件接上，然后定时 Send()
// 并打印解析回来的轮子状态。不经过 ros2_control，跑出来不对就一定是
// Middleware + USB_CDC 的问题。
//
//   ros2 run comm_test comm_test_node

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <Communication_Interface.hpp>
#include <motor_base.hpp>
#include <tsk_config_and_callback.hpp>

namespace
{

constexpr size_t kWheels = 4;

#pragma pack(push, 1)
struct Struct_Tx
{
    float velocity[kWheels];                              // 下行：目标速度
};
struct Struct_Rx
{
    Control_Frame::Struct_Motor_Base motor[kWheels];      // 上行：速度 + 位置
};
#pragma pack(pop)

Struct_Tx Tx;
Struct_Rx Rx;

}  // namespace

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    if (!Control_Frame::Task_Init())
    {
        std::fprintf(stderr, "Task_Init 失败\n");
        return 1;
    }

    // 一个结构体一个 id：下行整包是 Struct_Tx（16 字节），上行整包是 Struct_Rx（32 字节）
    Control_Frame::USB_Communication_Interface.Register(1, &Tx, &Rx, sizeof(Tx), sizeof(Rx));

    auto node = std::make_shared<rclcpp::Node>("comm_test_node");
    auto timer = node->create_wall_timer(std::chrono::milliseconds(100), [] {
        // 100/200/300/400：数值按 id 区分，收到错位一眼看得出来
        for (size_t k = 0; k < kWheels; ++k)
        {
            Tx.velocity[k] = static_cast<float>(k + 1) * 100.0f;
        }
        Control_Frame::USB_Communication_Interface.Send();

        std::printf("发 100/200/300/400");
        for (size_t k = 0; k < kWheels; ++k)
        {
            std::printf(" w%zu %.3f/%.3f", k, Rx.motor[k].velocity, Rx.motor[k].position);
        }
        std::printf("\n");
    });

    rclcpp::spin(node);

    rclcpp::shutdown();
    std::fflush(stdout);
    std::_Exit(0);  // 不走析构：~USB_CDC 会 join 阻塞在 read() 的线程，正常退会卡死
}
