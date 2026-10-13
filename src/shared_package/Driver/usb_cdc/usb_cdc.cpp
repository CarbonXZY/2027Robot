#include "usb_cdc.hpp"

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <fcntl.h>
#include <poll.h>
#include <regex>
#include <termios.h>
#include <unistd.h>

namespace Driver
{

USB_CDC::USB_CDC(const std::string& device, uint32_t tx_period_ms)
    : device_path_(device), 
    fd_(-1), 
    tx_period_ms_(tx_period_ms),
    running_(false),
    rx_callback_(nullptr),
    disconnect_callback_(nullptr)
{
    Connect();
}

USB_CDC::~USB_CDC()
{
    Disconnect();
}

bool USB_CDC::Connect()
{
    // 幂等: 已连接时先收敛(未连接时 joinable() 为 false、fd_ < 0, 相当于空操作),
    // 避免对 joinable 的 std::thread 赋值触发 std::terminate, 同时防止 fd 泄漏
    Disconnect();

    if (!OpenDevice())
    {
        std::fprintf(stderr, "USB_CDC: failed to open device %s: %s\n",
                      device_path_.c_str(), std::strerror(errno));
        return false;
    }

    running_ = true;
    rx_thread_ = std::thread(&USB_CDC::RxThreadLoop, this);
    tx_thread_ = std::thread(&USB_CDC::TxThreadLoop, this);

    return true;
}

void USB_CDC::Disconnect()
{
    running_ = false;

    if (rx_thread_.joinable())
    {
        // 自 join 保护: 断连回调就在接收线程上执行, 上层可能在回调里直接调 Connect()
        // 重连, 此时本函数会跑在接收线程自己身上, join 自己必然死锁(join 要等线程
        // 函数返回, 而它在等 join 返回). 这种情况改为 detach: 该线程在回调返回后
        // 紧接着就会退出 RxThreadLoop, 不需要等待, 之后也能安全地给它赋新线程.
        // 由调用者保证 detach 后回调能返回, 见头文件里对断连回调的说明.
        if (rx_thread_.get_id() == std::this_thread::get_id())
        {
            rx_thread_.detach();
        }
        else
        {
            rx_thread_.join();
        }
    }

    if (tx_thread_.joinable())
    {
        tx_thread_.join();
    }

    CloseDevice();
}

void USB_CDC::CloseDevice()
{
    // 与设备写操作互斥, 避免关闭正被 write() 使用的 fd
    std::lock_guard<std::mutex> lock(tx_write_mutex_);

    int fd = fd_;
    if (fd >= 0)
    {
        close(fd);
        fd_ = -1;
    }
}

bool USB_CDC::OpenDevice()
{
    fd_ = open(device_path_.c_str(), O_RDWR | O_NOCTTY);
    if (fd_ < 0)
    {
        return false;
    }

    termios tty;
    std::memset(&tty, 0, sizeof(tty));

    if (tcgetattr(fd_, &tty) != 0)
    {
        close(fd_);
        fd_ = -1;
        return false;
    }

    // 原始模式, 无回显, 无信号处理
    cfmakeraw(&tty);

    // 8N1, 使能接收, 忽略调制解调器控制线
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;

    // read() 阻塞直到至少 1 字节可读, 无字符间超时
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd_, TCSANOW, &tty) != 0)
    {
        close(fd_);
        fd_ = -1;
        return false;
    }

    return true;
}

void USB_CDC::RegisterRxCallback(RxHandle callback)
{
    std::lock_guard<std::mutex> lock(rx_callback_mutex_);
    rx_callback_ = callback;
}

void USB_CDC::RegisterDisconnectCallback(DisconnectHandle callback)
{
    std::lock_guard<std::mutex> lock(disconnect_callback_mutex_);
    disconnect_callback_ = callback;
}

uint8_t USB_CDC::TransmitNow(uint8_t* data, uint16_t length)
{
    if (fd_ < 0 || data == nullptr || length == 0)
    {
        return 0;
    }

    std::lock_guard<std::mutex> lock(tx_write_mutex_);

    ssize_t written = write(fd_, data, length);
    if (written < 0 || static_cast<uint16_t>(written) != length)
    {
        return 0;
    }

    return 1;
}

uint8_t USB_CDC::TransmitAdd(uint8_t* data, uint16_t length)
{
    if (data == nullptr || length == 0)
    {
        return 0;
    }

    std::lock_guard<std::mutex> lock(tx_queue_mutex_);

    if (tx_queue_.size() >= kTxQueueSize)
    {
        return 0;
    }

    TxFrame frame;
    frame.data.assign(data, data + length);
    tx_queue_.push_back(std::move(frame));

    return 1;
}

void USB_CDC::RxThreadLoop()
{
    uint8_t read_buf[kMaxFrameSize];

    pollfd pfd{};
    pfd.fd = fd_;
    pfd.events = POLLIN;

    while (running_)
    {
        // 带超时等待可读: 有数据时立即返回, 无数据时超时返回,
        // 从而每轮都能检查 running_, 保证析构时线程能真正退出
        int ret = poll(&pfd, 1, static_cast<int>(kRxPollTimeoutMs));
        if (ret <= 0)
        {
            // 超时或被信号打断, 回到循环检查 running_
            continue;
        }

        if ((pfd.revents & (POLLHUP | POLLERR)) != 0)
        {
            // 设备拔出或异常: 复位运行标志, 使发送线程随之退出,
            // 并真正关闭设备, 避免 IsOpen() 继续误报
            running_ = false;
            std::fprintf(stderr, "USB_CDC: device %s disconnected\n",
                          device_path_.c_str());
            CloseDevice();

            // 通知上层(在接收线程中执行, 详见头文件说明)
            DisconnectHandle callback = nullptr;
            {
                std::lock_guard<std::mutex> lock(disconnect_callback_mutex_);
                callback = disconnect_callback_;
            }
            if (callback != nullptr)
            {
                callback();
            }

            break;
        }

        // poll 已确认可读, 此处阻塞 read 必定立即返回
        ssize_t n = read(fd_, read_buf, sizeof(read_buf));

        if (n > 0)
        {
            std::lock_guard<std::mutex> lock(rx_callback_mutex_);
            if (rx_callback_ != nullptr)
            {
                rx_callback_(read_buf, static_cast<uint16_t>(n));
            }
        }
        else if (n < 0 && errno != EINTR)
        {
            // 设备异常或断开, 短暂等待后重试
            std::this_thread::sleep_for(std::chrono::milliseconds(kTxPollIntervalMs));
        }
    }
}

void USB_CDC::TxThreadLoop()
{
    // 有帧时尽快发送, 无帧时睡眠一个周期再查, 避免空转占满 CPU
    while (running_)
    {
        TxFrame frame;
        bool has_frame = false;

        {
            std::lock_guard<std::mutex> lock(tx_queue_mutex_);
            if (!tx_queue_.empty())
            {
                frame = std::move(tx_queue_.front());
                tx_queue_.erase(tx_queue_.begin());
                has_frame = true;
            }
        }

        if (!has_frame)
        {
            // 队列为空: 睡一个周期再查, 该周期即入队到发出的最大延迟
            std::this_thread::sleep_for(std::chrono::milliseconds(tx_period_ms_));
            continue;
        }

        std::unique_lock<std::mutex> write_lock(tx_write_mutex_, std::try_to_lock);
        if (!write_lock.owns_lock())
        {
            // 当前无法发送(设备写操作被占用), 放回队首, 睡一个周期后重试
            std::lock_guard<std::mutex> lock(tx_queue_mutex_);
            tx_queue_.insert(tx_queue_.begin(), std::move(frame));
            std::this_thread::sleep_for(std::chrono::milliseconds(tx_period_ms_));
            continue;
        }

        // 全程持锁写满整帧: 短写或被信号打断时续写剩余部分,
        // 保证该帧在字节流中连续, 不会被其他写者(TransmitNow/CloseDevice)插入
        size_t offset = 0;
        while (offset < frame.data.size())
        {
            ssize_t n = write(fd_, frame.data.data() + offset, frame.data.size() - offset);

            if (n > 0)
            {
                offset += static_cast<size_t>(n);
                continue;
            }

            if (n < 0 && errno == EINTR)
            {
                // 被信号打断, 并非出错, 继续补写剩余部分
                continue;
            }

            // 设备错误(如拔出后的 EIO/ENODEV): 该帧已残缺且无法挽回,
            // 丢弃且不重排队列(避免队头阻塞), 交由接收线程/上层感知断连
            std::fprintf(stderr, "USB_CDC: write failed on %s: %s\n",
                          device_path_.c_str(), std::strerror(errno));
            break;
        }
    }
}

std::string FindDevice(const std::string & spec)
{
    namespace fs = std::filesystem;

    if (spec.empty())
    {
        return "";
    }

    // 直连：以 '/' 开头就是设备路径，原样返回
    if (spec.front() == '/')
    {
        return spec;
    }

    // 正则：按设备名匹配
    std::regex pattern;
    try
    {
        pattern = std::regex(spec);
    }
    catch (const std::regex_error &)
    {
        std::fprintf(stderr, "USB_CDC: invalid device regex: %s\n", spec.c_str());
        return "";
    }

    // 1) 优先匹配 /dev/serial/by-id 的稳定名（如 usb-STMicroelectronics_STM32_...-if00）
    std::error_code ec;
    const fs::path by_id_dir("/dev/serial/by-id");
    if (fs::is_directory(by_id_dir, ec))
    {
        for (const fs::directory_entry & entry : fs::directory_iterator(by_id_dir, ec))
        {
            if (!std::regex_search(entry.path().filename().string(), pattern))
            {
                continue;
            }
            ec.clear();
            const fs::path real = fs::canonical(entry.path(), ec);
            if (!ec)
            {
                return real.string();
            }
        }
    }

    // 2) 回退：直接扫 /dev/ttyACM*，匹配设备节点名或 sysfs 里的 product 字符串
    ec.clear();
    for (const fs::directory_entry & entry : fs::directory_iterator("/dev", ec))
    {
        const std::string name = entry.path().filename().string();
        if (name.rfind("ttyACM", 0) != 0)
        {
            continue;
        }

        if (std::regex_search(name, pattern))
        {
            return (fs::path("/dev") / name).string();
        }

        std::ifstream in(fs::path("/sys/class/tty") / name / "device" / "product");
        std::string product;
        if (std::getline(in, product) && std::regex_search(product, pattern))
        {
            return (fs::path("/dev") / name).string();
        }
    }

    return "";
}

} // namespace Driver
