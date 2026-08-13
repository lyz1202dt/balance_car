#include <controller/lqr_gain_scheduler.hpp>

#include <algorithm>
#include <cmath>
#include <iterator>

namespace lqr_controller {

namespace {

constexpr size_t kLqrStateSize = 6;
constexpr size_t kLqrInputSize = 2;
constexpr size_t kKValuesPerEntry = kLqrStateSize * kLqrInputSize;

} // namespace

LqrGainScheduler::LqrGainScheduler() {
    ground_gain_.setZero();
    air_gain_.setZero();
}

bool LqrGainScheduler::load_table(
    const std::vector<double>& lengths, const std::vector<double>& values, std::string& error) {
    if (lengths.empty()) {
        error = "K.lengths must contain at least one leg length";
        return false;
    }
    if (values.size() != lengths.size() * kKValuesPerEntry) {
        error = "K.values must contain exactly 12 values for each K.lengths entry";
        return false;
    }

    table_.clear();
    table_.reserve(lengths.size());
    for (size_t entry_index = 0; entry_index < lengths.size(); ++entry_index) {
        const double length = lengths[entry_index];
        if (!std::isfinite(length)) {
            error = "K.lengths contains a non-finite value at index " + std::to_string(entry_index);
            return false;
        }

        LqrGainMatrix gain;
        for (size_t row = 0; row < kLqrInputSize; ++row) {
            for (size_t col = 0; col < kLqrStateSize; ++col) {
                const size_t value_index = entry_index * kKValuesPerEntry + row * kLqrStateSize + col;
                const double value       = values[value_index];
                if (!std::isfinite(value)) {
                    error = "K.values contains a non-finite value at index " + std::to_string(value_index);
                    return false;
                }
                gain(row, col) = value;
            }
        }
        table_.emplace_back(length, gain);
    }

    std::sort(table_.begin(), table_.end(), [](const TableEntry& lhs, const TableEntry& rhs) {
        return lhs.first < rhs.first;
    });

    for (size_t i = 1; i < table_.size(); ++i) {
        if (table_[i].first <= table_[i - 1].first) {
            error = "K.lengths entries must be unique";
            return false;
        }
    }

    return update(table_.front().first);
}

bool LqrGainScheduler::update(const double leg_length) {
    if (table_.empty() || !std::isfinite(leg_length)) {
        return false;
    }

    if (leg_length <= table_.front().first || table_.size() == 1) {
        ground_gain_ = table_.front().second;
    } else if (leg_length >= table_.back().first) {
        ground_gain_ = table_.back().second;
    } else {
        const auto upper = std::lower_bound(
            table_.begin(), table_.end(), leg_length,
            [](const TableEntry& entry, const double length) { return entry.first < length; });
        const auto lower = std::prev(upper);
        const double ratio = (leg_length - lower->first) / (upper->first - lower->first);
        ground_gain_       = (1.0 - ratio) * lower->second + ratio * upper->second;
    }

    air_gain_.setZero();
    air_gain_(1, 2) = ground_gain_(1, 2);
    air_gain_(1, 3) = ground_gain_(1, 3);
    return true;
}

bool LqrGainScheduler::empty() const { return table_.empty(); }

const LqrGainMatrix& LqrGainScheduler::ground_gain() const { return ground_gain_; }

const LqrGainMatrix& LqrGainScheduler::air_gain() const { return air_gain_; }

} // namespace lqr_controller
