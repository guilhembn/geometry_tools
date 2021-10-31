#ifndef POINT_H
#define POINT_H

#include <eigen3/Eigen/Geometry>
#include <iostream>

namespace rd {

class PointOriented;

class Angle {
  friend class Point;

 public:
  Angle();
  Angle(double theta);
  Angle &operator-=(const Angle &rhs);
  Angle operator-(const Angle &rhs) const;
  Angle &operator-();
  Angle operator-() const;
  Angle &operator+=(const Angle &rhs);
  Angle operator+(const Angle &rhs) const;
  Angle &operator*=(const double s);
  Angle operator*(const double s) const;
  bool operator<(double angle) const { return *this < Angle(angle); }
  bool operator>(double angle) const { return *this > Angle(angle); }
  bool operator==(double angle) const { return *this == Angle(angle); }
  bool operator<(const Angle &rhs) const { return r_.smallestAngle() < rhs.r_.smallestAngle(); }
  bool operator>(const Angle &rhs) const { return r_.smallestAngle() > rhs.r_.smallestAngle(); }
  bool operator==(const Angle &rhs) const { return r_.smallestAngle() == rhs.r_.smallestAngle(); }

  double value() const { return r_.smallestAngle(); }
  double cos() const { return std::cos(r_.smallestAngle()); }
  double sin() const { return std::sin(r_.smallestAngle()); }

  inline friend std::ostream &operator<<(std::ostream &os, const Angle &pt) {
    os << pt.r_.smallestAngle() << "rad";
    return os;
  };

 protected:
  void center();
  Eigen::Rotation2Dd r_;
};

class Point {
 public:
  inline Point() : p_(0.0, 0.0){};
  Point(double x, double y);
  virtual Angle angleTo(const Point &pt) const;
  Angle angleBetweenVectors(const Point &pt) const;
  double x() const { return p_.x(); }
  double y() const { return p_.y(); }
  virtual double squaredDistanceTo(const Point &pt) const { return (pt.p_ - p_).squaredNorm(); }
  virtual double distanceTo(const Point &pt) const { return (pt.p_ - p_).norm(); }
  virtual double squaredNorm() const { return p_.squaredNorm(); }
  virtual double norm() const { return p_.norm(); }
  inline double dot(const Point &pt) const { return p_.dot(pt.p_); }
  /**
   * @brief Finds the closest point to *this on the ab segment.
   *
   * @param a First point of the segment
   * @param b Second point of the segment
   * @param t (out) the interpolation factor
   * @return Point The closest point from *this on the ab segment
   * (Point = a + t * (b - a))
   */
  virtual Point closestPointBetween(const Point &a, const Point &b, double &t) const;
  virtual Point transformIn(const PointOriented &frame) const;
  Angle polarAngle() const;
  virtual Point &operator+=(const Point &rhs);
  virtual Point operator+(const Point &rhs) const;
  virtual Point operator-() const;
  virtual Point &operator-=(const Point &rhs);
  virtual Point operator-(const Point &rhs) const;
  virtual Point &operator*=(const double s);
  virtual Point operator*(const double s);
  inline bool operator==(const Point &rhs) const { return p_ == rhs.p_; }

  inline friend std::ostream &operator<<(std::ostream &os, const Point &pt) {
    os << pt.p_.x() << ";" << pt.p_.y();
    return os;
  };

 protected:
  Eigen::Vector2d p_;
};

class PointOriented : public Point {
 public:
  PointOriented() = default;
  virtual ~PointOriented() = default;
  PointOriented(double x, double y, double theta);
  const Angle &theta() const { return a_; };

  inline friend std::ostream &operator<<(std::ostream &os, const PointOriented &pt) {
    os << pt.p_.x() << ";" << pt.p_.y() << ";" << pt.a_;
    return os;
  };
  virtual PointOriented &operator+=(const PointOriented &rhs);
  virtual PointOriented operator+(const PointOriented &rhs) const;
  virtual PointOriented &operator-=(const PointOriented &rhs);
  virtual PointOriented operator-(const PointOriented &rhs) const;
  virtual PointOriented &operator*=(const double s);
  virtual PointOriented operator*(const double rhs) const;

 private:
  Angle a_;
};

}  // namespace rd

#endif /* POINT_H */
