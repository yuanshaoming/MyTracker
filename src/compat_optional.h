#pragma once

#include <utility>

namespace tracking {
namespace detail {

template <typename T>
class Optional {
public:
    Optional() = default;
    Optional(T value)
        : hasValue_(true), value_(std::move(value)) {}

    Optional& operator=(T value) {
        hasValue_ = true;
        value_ = std::move(value);
        return *this;
    }

    explicit operator bool() const noexcept {
        return hasValue_;
    }

    bool has_value() const noexcept {
        return hasValue_;
    }

    bool operator!() const noexcept {
        return !hasValue_;
    }

    T& operator*() noexcept {
        return value_;
    }

    const T& operator*() const noexcept {
        return value_;
    }

    T* operator->() noexcept {
        return &value_;
    }

    const T* operator->() const noexcept {
        return &value_;
    }

    T value_or(T defaultValue) const {
        return hasValue_ ? value_ : std::move(defaultValue);
    }

private:
    bool hasValue_ = false;
    T value_{};
};

}  // namespace detail
}  // namespace tracking
