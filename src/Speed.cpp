#include "GeometryTools/Speed.h"

namespace rd {
Speed::Speed(double vx, double vy, double vtheta) : linearSpeed_(vx, vy), rotationalSpeed_(vtheta) {}

double Speed::linearSpeed() const { return linearSpeed_.norm(); }

Speed &Speed::operator*=(const double s) {
  linearSpeed_ *= s;
  rotationalSpeed_ *= s;
  return *this;
}

Speed Speed::operator*(const double s) {
  Speed tmp(*this);
  tmp *= s;
  return tmp;
}

Speed &Speed::operator+=(const Speed &rhs) {
  linearSpeed_ += rhs.linearSpeed_;
  rotationalSpeed_ += rhs.rotationalSpeed_;
  return *this;
}

Speed Speed::operator+(const Speed &rhs) const {
  Speed tmp(*this);
  tmp += rhs;
  return tmp;
}

Speed &Speed::operator-=(const Speed &rhs) {
  linearSpeed_ -= rhs.linearSpeed_;
  rotationalSpeed_ -= rhs.rotationalSpeed_;
  return *this;
}
Speed Speed::operator-(const Speed &rhs) const {
  Speed tmp(*this);
  tmp -= rhs;
  return tmp;
}

Speed Speed::operator-() const {
  Speed tmp(*this);
  tmp.linearSpeed_ = -tmp.linearSpeed_;
  tmp.rotationalSpeed_ = -tmp.rotationalSpeed_;
  return tmp;
}

}  // namespace rd
