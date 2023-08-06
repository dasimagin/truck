#include "geom/transform.h"

namespace truck::geom {

Transform::Transform(const Vec2& t, Angle a) : translation_(t), rotation_(a) {}

Transform::Transform(const Vec2& t, const Vec2& r) : translation_(t), rotation_(r) {}

namespace {

geom::Vec2 toVector(const tf2::Vector3& v) { return geom::Vec2(v.x(), v.y()); }

}  // namespace

geom::Angle toAngle(const tf2::Quaternion& q) {
    return geom::Angle::fromRadians(std::copysign(2 * std::acos(q.w()), q.z()));
}

Transform::Transform(const tf2::Transform& tf)
    : translation_(toVector(tf.getOrigin())), rotation_(toAngle(tf.getRotation())) {}

Vec2 Transform::apply(const Vec2& v) const { return translation_ + v.rotate(rotation_); }

Pose Transform::apply(const Pose& p) const { return Pose{apply(p.pos), p.dir.rotate(rotation_)}; }

Vec2 Transform::operator()(const Vec2& v) const { return apply(v); }

Pose Transform::operator()(const Pose& p) const { return apply(p); }

Transform Transform::inv() const {
    auto r_inv = rotation_.inv();
    return Transform(-translation_.rotate(r_inv), r_inv);
}

}  // namespace truck::geom
