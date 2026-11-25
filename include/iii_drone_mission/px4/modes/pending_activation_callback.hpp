#pragma once

#include <functional>
#include <utility>

namespace iii_drone::px4 {

class PendingActivationCallback final {
public:
    using Callback = std::function<void()>;

    void Set(Callback callback) {
        callback_ = std::move(callback);
    }

    void Cancel() noexcept {
        callback_ = nullptr;
    }

    void OnDeactivate(bool preserve) noexcept {
        if (!preserve) {
            Cancel();
        }
    }

    void Invoke() {
        auto callback = std::exchange(callback_, Callback{});
        if (callback) {
            callback();
        }
    }

private:
    Callback callback_;
};

} // namespace iii_drone::px4
