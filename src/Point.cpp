#include "GeometryTools/Point.h"

namespace rd {

Angle::Angle() : r_(0.0) {}

Angle::Angle(double theta) : r_(theta) { center(); }

void Angle::center() { r_.angle() = r_.smallestAngle(); }

Angle &Angle::operator-=(const Angle &rhs) {
  r_ *= rhs.r_.inverse();
  center();
  return *this;
}

Angle Angle::operator-(const Angle &rhs) const {
  Angle a(*this);
  a -= rhs;
  return a;
}

Angle &Angle::operator-() {
  r_ = r_.inverse();
  return *this;
}

Angle Angle::operator-() const {
  Angle a(*this);
  return -a;
}

Angle &Angle::operator+=(const Angle &rhs) {
  r_ *= rhs.r_;
  center();
  return *this;
}

Angle Angle::operator+(const Angle &rhs) const {
  Angle a(*this);
  a += rhs;
  return a;
}

Angle &Angle::operator*=(const double s) {
  r_ = Eigen::Rotation2Dd(0.0).slerp(s, r_);
  return *this;
}

Angle Angle::operator*(const double s) const {
  Angle tmp(*this);
  tmp *= s;
  return tmp;
}

Point::Point(double x, double y) : p_(x, y) {}

Angle Point::angleTo(const Point &pt) const {
  const Eigen::Vector2d tmp = pt.p_ - p_;
  return Angle(std::atan2(tmp.y(), tmp.x()));
}

Angle Point::angleBetweenVectors(const Point &pt) const {
  Angle a(std::acos(this->dot(pt) / (this->norm() * pt.norm())));
  return a;
}

Point Point::closestPointBetween(const Point &a, const Point &b, double &t) const {
  Point ab = b - a;
  Point ap = *this - a;
  t = std::min(1., std::max(0., ap.dot(ab) / ab.squaredNorm()));
  Point pt = a + ab * t;
  return pt;
}

Point Point::transformIn(const PointOriented &frame) const {
  Point pt(*this);
  pt -= frame;
  pt.p_ = frame.theta().r_.inverse() * pt.p_;
  return pt;
}

Angle Point::polarAngle() const { return Angle(std::atan2(p_.y(), p_.x())); }

Point &Point::operator+=(const Point &rhs) {
  p_ += rhs.p_;
  return *this;
}

Point Point::operator+(const Point &rhs) const {
  Point tmp(*this);
  tmp += rhs;
  return tmp;
}

Point &Point::operator-=(const Point &rhs) {
  p_ -= rhs.p_;
  return *this;
}
Point Point::operator-(const Point &rhs) const {
  Point tmp(*this);
  tmp -= rhs;
  return tmp;
}

Point Point::operator-() const {
  Point tmp(*this);
  tmp.p_ = -tmp.p_;
  return tmp;
}

Point &Point::operator*=(const double s) {
  p_ *= s;
  return *this;
}

Point Point::operator*(const double s) const {
  Point tmp(*this);
  tmp *= s;
  return tmp;
}

PointOriented::PointOriented(double x, double y, double theta) : Point(x, y), a_(theta) {}

PointOriented::PointOriented(const Point& point, double theta) : Point(point), a_(theta) {}

PointOriented &PointOriented::operator+=(const PointOriented &rhs) {
  p_ += rhs.p_;
  a_ += rhs.a_;
  return *this;
}

PointOriented PointOriented::operator+(const PointOriented &rhs) const {
  PointOriented tmp(*this);
  tmp += rhs;
  return tmp;
}

PointOriented &PointOriented::operator-=(const PointOriented &rhs) {
  p_ -= rhs.p_;
  a_ -= rhs.a_;
  return *this;
}
PointOriented PointOriented::operator-(const PointOriented &rhs) const {
  PointOriented tmp(*this);
  tmp -= rhs;
  return tmp;
}

PointOriented &PointOriented::operator*=(const double s) {
  p_ *= s;
  a_ *= s;
  return *this;
}
PointOriented PointOriented::operator*(const double s) const {
  PointOriented tmp(*this);
  tmp *= s;
  return tmp;
}

}  // namespace rd