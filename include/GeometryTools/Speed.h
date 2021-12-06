#ifndef SPEED_H
#define SPEED_H

#include <eigen3/Eigen/Geometry>

namespace rd {
class Speed {
 public:
  inline Speed() : linearSpeed_(0.0, 0.0), rotationalSpeed_(0.0){};
  Speed(double vx, double vy, double vtheta);

  double vx() const { return linearSpeed_.x(); };
  double vy() const { return linearSpeed_.y(); };
  double vtheta() const { return rotationalSpeed_; };

  double linearSpeed() const;

  inline friend std::ostream &operator<<(std::ostream &os, const Speed &speed) {
    os << speed.linearSpeed_.x() << ";" << speed.linearSpeed_.y() << ";" << speed.rotationalSpeed_;
    return os;
  };

  virtual Speed &operator*=(const double s);
  virtual Speed operator*(const double s);
  virtual Speed &operator+=(const Speed &rhs);
  virtual Speed operator+(const Speed &rhs) const;
  virtual Speed operator-() const;
  virtual Speed &operator-=(const Speed &rhs);
  virtual Speed operator-(const Speed &rhs) const;

 protected:
  Eigen::Vector2d linearSpeed_;
  double rotationalSpeed_;
};
}  // namespace rd

#endif /* SPEED_H */
