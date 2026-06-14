#pragma once

#include <string>

#include "vehicle_cmd.h"

namespace gd32_bridge {

// Publishes vehicle control commands (STOP, ADJUST, RESUME) to
// gnss_path_control via Unix domain datagram socket.
class VehicleCmdPublisher {
public:
    VehicleCmdPublisher();
    ~VehicleCmdPublisher();

    bool start();
    void stop();

    void publish(const VehicleCmdMessage &msg);

    // Convenience helpers
    void publishStop();
    void publishAdjust(float angle_delta_deg);
    void publishResume();
    void publishCrackStop();

private:
    int  sock_fd_ = -1;
    bool initSocket();
};

} // namespace gd32_bridge
