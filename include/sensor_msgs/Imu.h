#ifndef _SENSOR_MSGS_IMU_H
#define _SENSOR_MSGS_IMU_H_

#include <cstdint>
#include <array>
#include <string>
#include <memory>

namespace geometry_msgs {

struct Vector3 {
    double x;
    double y;
    double z;
};

struct Quaternion {
    double x;
    double y;
    double z;
    double w;
};

} // namespace geometry_msgs

namespace sensor_msgs {

struct Time {
    uint32_t sec;
    uint32_t nsec;

    double toSec() const {
        return double(sec) + 1e-9 * double(nsec);
    }

    uint64_t toNSec() const {
        return uint64_t(sec) * 1000000000ULL + uint64_t(nsec);
    }
};

struct Header {
    uint32_t seq;
    Time stamp;
    std::string frame_id;
};

struct Imu {
    Header header;

    geometry_msgs::Quaternion orientation;
    std::array<double, 9> orientation_covariance;

    geometry_msgs::Vector3 angular_velocity;
    std::array<double, 9> angular_velocity_covariance;

    geometry_msgs::Vector3 linear_acceleration;
    std::array<double, 9> linear_acceleration_covariance;
};

using ImuPtr      = std::shared_ptr<Imu>;
using ImuConstPtr = std::shared_ptr<const Imu>;

} // namespace sensor_msgs

#endif // _SENSOR_MSGS_IMU_H