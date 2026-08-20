#pragma once

namespace mission {

enum class TargetApproachState {
    Align,
    TargetCenteredHold,
};

class TargetCenteredHold {
public:
    bool enter_if_ready(bool fresh_confirmed_target, bool centered,
                        double distance_m, double stop_distance_m) {
        if (state_ == TargetApproachState::TargetCenteredHold) return true;
        if (fresh_confirmed_target && centered && distance_m <= stop_distance_m) {
            state_ = TargetApproachState::TargetCenteredHold;
            return true;
        }
        return false;
    }

    bool holding() const { return state_ == TargetApproachState::TargetCenteredHold; }
    TargetApproachState state() const { return state_; }

    // A target loss invalidates the previous centered lock. The next fresh
    // observation must satisfy the normal center and distance checks again.
    void reset() { state_ = TargetApproachState::Align; }

private:
    TargetApproachState state_ = TargetApproachState::Align;
};

}  // namespace mission
