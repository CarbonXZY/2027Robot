#include "usb_cdc.hpp"

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

USB_CDC::USB_CDC(const std::string& device, uint32_t tx_period_ms)
    : device_path_(device), 
    fd_(-1), 
    tx_period_ms_(tx_period_ms),
    running_(false), 
    rx_callback_(nullptr)
{
    if (!OpenDevice())
    {
        std::fprintf(stderr, "USB_CDC: failed to open device %s: %s\n",
                      device_path_.c_str(), std::strerror(errno));
        return;
    }

    running_ = true;
    rx_thread_ = std::thread(&USB_CDC::RxThreadLoop, this);
    tx_thread_ = std::thread(&USB_CDC::TxThreadLoop, this);
}

USB_CDC::~USB_CDC()
{
    running_ = false;

    if (rx_thread_.joinable())
    {
        rx_thread_.join();
    }

    if (tx_thread_.joinable())
    {
        tx_thread_.join();
    }

    if (fd_ >= 0)
    {
        close(fd_);
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

    while (running_)
    {
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
    // 创建定时器
    // 周期
    auto period = std::chrono::milliseconds(tx_period_ms_);
    auto next_wake_time = std::chrono::steady_clock::now() + period;

    // 定时发送
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
            continue;
        }

        std::unique_lock<std::mutex> write_lock(tx_write_mutex_, std::try_to_lock);
        if (!write_lock.owns_lock())
        {
            // 当前无法发送(设备写操作被占用), 放回队首, 下次定时重试
            std::lock_guard<std::mutex> lock(tx_queue_mutex_);
            tx_queue_.insert(tx_queue_.begin(), std::move(frame));
            continue;
        }

        write(fd_, frame.data.data(), frame.data.size());

        // 计算下一次回调时间
        period = std::chrono::milliseconds(tx_period_ms_);
        next_wake_time += period;
    }
}
