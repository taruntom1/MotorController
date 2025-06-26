#pragma once
#include <vector>
#include <algorithm>
#include <type_traits>

template <typename T>
class RollingMeanAccumulator
{
    static_assert(std::is_arithmetic<T>::value, "RollingMeanAccumulator requires an arithmetic type.");

public:
    explicit RollingMeanAccumulator(size_t window_size)
        : window_size_(window_size), buffer_(window_size, T{}),
          sum_(T{}), index_(0), count_(0) {}

    void accumulate(T value)
    {
        if (count_ < window_size_)
        {
            ++count_;
        }
        else
        {
            sum_ -= buffer_[index_];
        }

        sum_ += value;
        buffer_[index_] = value;
        index_ = (index_ + 1) % window_size_;
    }

    T getRollingMean() const
    {
        return (count_ == 0) ? T{} : sum_ / static_cast<T>(count_);
    }

    void reset()
    {
        std::fill(buffer_.begin(), buffer_.end(), T{});
        sum_ = T{};
        index_ = 0;
        count_ = 0;
    }

private:
    size_t window_size_;
    std::vector<T> buffer_;
    T sum_;
    size_t index_;
    size_t count_;
};
