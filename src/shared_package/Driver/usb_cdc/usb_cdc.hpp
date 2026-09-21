#ifndef USB_CDC_HPP
#define USB_CDC_HPP

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

/**
 * @brief USB-CDC 设备封装类, 基于 POSIX 接口实现对 /dev/ttyACMx 的实时收发
 *
 * 该类绑定单个硬件接口(如 /dev/ttyACM0), 内部启动两个线程:
 * - 接收线程: 阻塞监听设备, 数据到达后写入环形缓冲区并触发用户注册的回调
 * - 发送线程: 周期性检测发送队列(FIFO), 队列非空时尝试发送
 *
 * 提供同步发送(TransmitNow)和异步发送(TransmitAdd)两种接口。
 * 析构时自动停止两个线程并关闭设备(RAII), 用户无需手动调用停止接口。
 */
class USB_CDC
{
public:
    // 发送队列(FIFO)最大容量, 单位条(帧)
    static constexpr uint32_t kTxQueueSize = 64;
    // 发送线程轮询周期, 单位毫秒
    static constexpr uint32_t kTxPollIntervalMs = 10;
    // 单帧最大长度, 单位字节
    static constexpr uint16_t kMaxFrameSize = 1024;

    // 接收回调函数指针类型
    using RxHandle = void (*)(uint8_t* data, uint16_t length);

    /**
     * @brief 构造函数, 打开并配置指定的 USB-CDC 设备, 启动收发线程
     * @param device 设备路径, 如 "/dev/ttyACM0"
     */
    explicit USB_CDC(const std::string& device, uint32_t tx_period_ms = 10);

    /**
     * @brief 析构函数, 自动停止收发线程并关闭设备(RAII)
     */
    ~USB_CDC();

    // 禁止拷贝, 该类管理线程和文件描述符等独占资源
    USB_CDC(const USB_CDC&) = delete;
    USB_CDC& operator=(const USB_CDC&) = delete;

    /**
     * @brief 注册接收回调函数, 接收线程收到数据后会调用该回调
     * @param callback 回调函数指针, 参数为数据指针和数据长度
     */
    void RegisterRxCallback(RxHandle callback);

    /**
     * @brief 同步发送函数, 立即调用系统接口发送数据
     * @param data 待发送数据指针
     * @param length 待发送数据长度
     * @return 1 表示发送成功, 0 表示发送失败
     */
    uint8_t TransmitNow(uint8_t* data, uint16_t length);

    /**
     * @brief 异步发送函数, 将数据加入发送队列(FIFO), 由后台线程定时发送
     * @param data 待发送数据指针
     * @param length 待发送数据长度
     * @return 1 表示成功加入队列, 0 表示队列已满, 加入失败
     */
    uint8_t TransmitAdd(uint8_t* data, uint16_t length);

    /**
     * @brief 判断设备是否已成功打开
     * @return true 表示设备已打开, false 表示打开失败
     */
    bool IsOpen() const { return fd_ >= 0; }

    /**
     * @brief 获取异步发送周期
     * @return 周期的毫秒数
     */
    uint32_t GetTxPeriodMs() const { return tx_period_ms_; }

    /**
     * @brief 设置异步发送周期
     * @param period_ms 周期的毫秒数
     */
    void SetTxPeriodMs(uint32_t period_ms) { tx_period_ms_ = period_ms; }

private:
    /**
     * @brief 打开设备文件并配置 termios 参数(原始模式, 无校验, 无流控)
     * @return true 表示成功, false 表示失败
     */
    bool OpenDevice();

    /**
     * @brief 接收线程主循环, 阻塞读取设备数据, 写入环形缓冲区并触发回调
     */
    void RxThreadLoop();

    /**
     * @brief 发送线程主循环, 周期性检测发送队列, 非空则尝试发送
     */
    void TxThreadLoop();

    // 设备路径
    std::string device_path_;
    // 设备文件描述符
    int fd_;

    // 接收线程
    std::thread rx_thread_;
    // 发送线程
    std::thread tx_thread_;
    // 发送周期毫秒数
    uint32_t tx_period_ms_ = 10;
    // 线程运行标志, 析构时置为 false 以停止线程
    std::atomic<bool> running_;

    // 注册的接收回调函数指针
    RxHandle rx_callback_;
    // 保护 rx_callback_ 的互斥锁
    std::mutex rx_callback_mutex_;

    // 发送帧结构
    struct TxFrame
    {
        std::vector<uint8_t> data;
    };

    // 发送队列(FIFO)
    std::vector<TxFrame> tx_queue_;
    // 保护发送队列的互斥锁
    std::mutex tx_queue_mutex_;

    // 保护设备写操作的互斥锁, 避免同步发送与异步发送线程并发写设备
    std::mutex tx_write_mutex_;
};

constexpr char USB_CDC_DEFAULT_DEVICE[] = "/dev/ttyACM0";

#endif // USB_CDC_HPP
