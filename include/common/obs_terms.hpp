#pragma once

#include <cassert>
#include <cstring>
#include <vector>

namespace cpp_control
{
namespace obs
{

/// Fixed-length observation history, mjlab CircularBuffer semantics:
///   - flattened output is chronological, oldest → newest
///   - after reset(), the first push() backfills every row with that value
/// (mjlab/utils/buffers/circular_buffer.py)
class HistoryTerm
{
public:
    HistoryTerm(int width, int history) : width_(width), history_(history)
    {
        data_.assign(static_cast<size_t>(width_) * history_, 0.0f);
    }

    void reset()
    {
        std::fill(data_.begin(), data_.end(), 0.0f);
        initialized_ = false;
        head_ = 0;
    }

    void push(const float* value)
    {
        if (!initialized_)
        {
            for (int h = 0; h < history_; ++h)
                std::memcpy(&data_[h * width_], value, width_ * sizeof(float));
            initialized_ = true;
            head_ = 0;
            return;
        }
        // head_ points at the OLDEST row; overwrite it and advance.
        std::memcpy(&data_[head_ * width_], value, width_ * sizeof(float));
        head_ = (head_ + 1) % history_;
    }

    /// Write the flattened history (oldest → newest) into dst[dim()].
    void write(float* dst) const
    {
        for (int h = 0; h < history_; ++h)
        {
            const int src_row = (head_ + h) % history_;
            std::memcpy(dst + h * width_, &data_[src_row * width_], width_ * sizeof(float));
        }
    }

    int width() const { return width_; }
    int dim() const { return width_ * history_; }

private:
    int width_;
    int history_;
    int head_ = 0;  ///< index of the oldest row
    bool initialized_ = false;
    std::vector<float> data_;
};

}  // namespace obs
}  // namespace cpp_control
