#pragma once
#include <Eigen/Geometry>

namespace point_lio::util {

template <class T>
concept wxyz_trait = requires {
    T{}.w;
    T{}.x;
    T{}.y;
    T{}.z;
};

template <class T>
concept xyz_trait = requires {
    T{}.x;
    T{}.y;
    T{}.z;
};

template <typename Scale>
struct Quaternion : public Eigen::Quaternion<Scale> {
    using Eigen::Quaternion<Scale>::Quaternion;

    static auto from_wxyz(const wxyz_trait auto& t) { return Quaternion{t.w, t.x, t.y, t.z}; }

    auto sync_wxyz(wxyz_trait auto& t) const -> void {
        t.w = this->w();
        t.x = this->x();
        t.y = this->y();
        t.z = this->z();
    }

    static auto from_radius(double yaw, double pitch, double roll) {
        return Quaternion{
            Eigen::AngleAxis<Scale>(yaw, Eigen::Matrix<Scale, 3, 1>::UnitZ())
            * Eigen::AngleAxis<Scale>(pitch, Eigen::Matrix<Scale, 3, 1>::UnitY())
            * Eigen::AngleAxis<Scale>(roll, Eigen::Matrix<Scale, 3, 1>::UnitX())};
    }
    static auto from_radius(const Eigen::Vector3d& radius) {
        return from_radius(radius[0], radius[1], radius[2]);
    }

    static auto from_angle(double yaw, double pitch, double roll) {
        return from_radius(
            yaw * std::numbers::pi / 180.0, pitch * std::numbers::pi / 180.0,
            roll * std::numbers::pi / 180.0);
    }
    static auto from_angle(const Eigen::Vector3d& angle) -> Quaternion {
        return from_angle(angle[0], angle[1], angle[2]);
    }
};

template <typename Scale>
struct Vector3 : public Eigen::Vector3<Scale> {
    using Base = Eigen::Vector3<Scale>;
    using Base::Base;

    static auto from_xyz(const xyz_trait auto& t) { return Vector3{t.x, t.y, t.z}; }

    auto sync_xyz(xyz_trait auto& t) const -> void {
        t.x = (*this)[0];
        t.y = (*this)[1];
        t.z = (*this)[2];
    }
};

} // namespace point_lio::util
