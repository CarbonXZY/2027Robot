#ifndef ALG_CIRCULAR_BUFFER_H
#define ALG_CIRCULAR_BUFFER_H

#include <atomic>
#include <stdint.h>

namespace Algorithm
{

/**
 * @brief 单生产者/单消费者无锁环形缓冲区，写满时丢弃新数据、保留并积压旧数据。
 *
 * 存储为编译期确定大小的成员数组，全程不做任何动态分配，
 * 对象可置于栈、.bss 或静态存储区。
 *
 * 写满时不做任何覆盖：Push() 返回 false，本次新数据被丢弃，缓冲区里已有的
 * 旧数据原样保留，继续按写入顺序等待 Pop()。因此缓冲区内容永远是一段连续、
 * 有序的历史，积压深度最多 N 个 —— 这是为结构体准备的：覆盖式写入会让生产者
 * 写到消费者正在读的那个槽，进而读到半新半旧的撕裂数据，这里从结构上排除了。
 *
 * 两端各自独占一个计数器：生产者只写 head_，消费者只写 tail_。
 *
 * @tparam T 元素类型。
 * @tparam N 容量，即最多可保存的元素个数。
 *
 * @note 典型用法：中断里 Push，主循环里 Pop。
 */
template <typename T, uint32_t N>
class Class_Circular_Buffer
{
public:
    Class_Circular_Buffer() = default;

    Class_Circular_Buffer(const Class_Circular_Buffer &) = delete;
    Class_Circular_Buffer &operator=(const Class_Circular_Buffer &) = delete;

    /**
     * @brief  写入一个元素。
     * @param  value 待写入的元素。
     * @return true 写入成功；false 缓冲区已满，本次数据被丢弃。
     */
    bool Push(const T &value);

    /**
     * @brief  取出最旧的元素。
     * @param  out 取出的元素，仅在返回 true 时有效。
     * @return true 取出成功；false 缓冲区为空。
     */
    bool Pop(T &out);

    /**
     * @brief 清空缓冲区。
     * @warning 仅在另一端处于静止状态时调用。这不是保守建议：若与 Push() 并发，
     *          生产者可能把元素写进旧的 head_ % N 槽、随后把 head_ 置成 1，
     *          消费者再 Pop 出来的就是槽 0 里的陈年数据。
     */
    void Clear();

private:
    T buffer_[N];

    std::atomic<uint32_t> head_{0};
    std::atomic<uint32_t> tail_{0};

    // 索引一旦不再是 lock-free 宽度，Push/Pop 就会在中断/控制环里隐式取锁。
    static_assert(std::atomic<uint32_t>::is_always_lock_free, "索引必须是 lock-free 宽度");
};

/**
 * @brief 写入元素：先落数据，再发布计数器，保证消费者不会读到半成品。
 */
template <typename T, uint32_t N>
bool Class_Circular_Buffer<T, N>::Push(const T &value)
{
    const uint32_t head = head_.load(std::memory_order_relaxed);

    // 满则丢弃。这里读到的 tail_ 只会偏旧（消费者之后只会前进），误差方向是
    // "更早丢弃"，绝不会写越界。也正因为满了就不写，head % N 永远不等于
    // tail % N 指向的、消费者可能正在读的那个槽。
    if (head - tail_.load(std::memory_order_relaxed) >= N)
    {
        return false;
    }

    buffer_[head % N] = value;

    // release：保证上面这笔载荷写对之后 acquire 到它的消费者可见。
    head_.store(head + 1, std::memory_order_release);

    return true;
}

/**
 * @brief 取出最旧元素。
 */
template <typename T, uint32_t N>
bool Class_Circular_Buffer<T, N>::Pop(T &out)
{
    const uint32_t tail = tail_.load(std::memory_order_relaxed);

    // acquire：与 Push 的 release 配对，保证下面读到的是发布完成的那个元素。
    if (head_.load(std::memory_order_acquire) == tail)
    {
        return false;
    }

    out = buffer_[tail % N];

    // relaxed 即可：消费者没有需要发布给生产者的写。生产者只拿 tail_ 的值做
    // 保守的满判断，读到旧值只会更早丢弃，误差方向安全。
    tail_.store(tail + 1, std::memory_order_relaxed);

    return true;
}

/**
 * @brief 清空缓冲区。
 */
template <typename T, uint32_t N>
void Class_Circular_Buffer<T, N>::Clear()
{
    head_ = 0;
    tail_ = 0;
}

} // namespace Algorithm

#endif // ALG_CIRCULAR_BUFFER_H
