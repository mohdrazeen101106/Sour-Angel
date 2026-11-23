#pragma once

#include "sour_angel/lateral_controller.hpp"
#include "sour_angel/longitudinal_controller.hpp"
#include "sour_angel/telemetry.hpp"

#include <cstdint>

namespace sour_angel::control {

struct ControlStateEstimate {
    LongitudinalState longitudinal;
    LateralState lateral;
};

// Converts a complete Pico sensor snapshot to the state inputs used by the
// two LQR blocks. Encoder velocity is corrected for internal chassis pitch
// before shell speed and along-track position are estimated.
class ControlStateEstimator final {
public:
    static constexpr float kShellRadiusM = 0.3582F;
    static constexpr std::uint32_t kMaximumSampleGapUs = 20'000U;
    static constexpr std::uint32_t kMaximumStateAgeUs = 10'000U;

    void reset(float initial_position_m = 0.0F) noexcept;

    // Returns true when this snapshot advances the estimate.
    bool update(const TelemetryFrame& sample) noexcept;

    // Returns false when the estimate is uninitialized, discontinuous, or
    // older than kMaximumStateAgeUs.
    bool snapshot(std::uint32_t now_us,
                  ControlStateEstimate& estimate) const noexcept;

private:
    float position_m_{0.0F};
    LongitudinalState longitudinal_{};
    LateralState lateral_{};
    std::uint32_t last_sample_us_{0U};
    bool initialized_{false};
    bool state_valid_{false};
};

} // namespace sour_angel::control
