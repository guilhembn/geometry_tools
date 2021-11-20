#include "GeometryTools/Trajectory.h"

#include <limits>

namespace rd {

Path::Path() : points_({}) {}

Path::Path(const std::vector<PointOriented> &points) {
  for (PointOriented p : points) {
    points_.push_back(p);
  }
}

Path Path::lissajouPath(const PointOriented &robotPose, const size_t steps) {
  double theta = 0.;
  double dtheta = 2 * M_PI / steps;
  std::vector<PointOriented> pts;
  pts.reserve(steps);
  for (size_t i = 0; i < steps; i++) {
    theta = i * dtheta;
    pts.emplace_back(robotPose.x() + 500 * std::sin(theta), robotPose.y() + 500 * std::sin(2 * theta), 0.0);
  }
  return Path(pts);
}

Point Path::pointClosestTo(const Point &point, double &tOut, size_t &closestPrevIndex) const {
  double distMin = std::numeric_limits<double>::max();
  double tMin = 0.;
  size_t iMin = -1;
  Point pointMin;
  for (size_t i = 0; i < size() - 1; i++) {
    double t;
    Point pt = point.closestPointBetween(points_.at(i), points_.at(i + 1), t);
    double dist = pt.squaredDistanceTo(point);
    if (dist < distMin) {
      distMin = dist;
      tMin = t;
      iMin = i;
      pointMin = pt;
    }
  }
  tOut = tMin;
  closestPrevIndex = iMin;
  return pointMin;
}

Point Path::nextPointClosestTo(const Point &point, double &tOut, size_t &closestPrevIndex) const {
  double distMin = std::numeric_limits<double>::max();
  double tMin = 0.;
  size_t iMin = -1;
  Point pointMin;
  for (size_t i = 0; i < size() - 1; i++) {
    double t;
    Point pt = point.closestPointBetween(points_.at(i), points_.at(i + 1), t);
    double dist = pt.squaredDistanceTo(point);
    if (dist < distMin) {
      distMin = dist;
      tMin = t;
      iMin = i;
      pointMin = pt;
    }
    if (dist > distMin) {
      break;
    }
  }
  tOut = tMin;
  closestPrevIndex = iMin;
  return pointMin;
}

Point Path::pointAtDistanceFrom(const double distance, const Point &pointStart, size_t &previousClosestIndex) const {
  double t;
  size_t previousIndex;
  Point proj = pointClosestTo(pointStart, t, previousIndex);

  double distanceLeft = distance;
  double pathLen = points_.at(previousIndex + 1).distanceTo(proj);
  size_t browsingTraj = previousIndex;
  while (distanceLeft > pathLen) {  // while the distance left is greater than the length of a full segment
    t = 0.;
    browsingTraj += 1;
    distanceLeft -= pathLen;
    pathLen = (points_.at(browsingTraj + 1) - points_.at(browsingTraj)).norm();
  }
  Point ab = points_.at(browsingTraj + 1) - points_.at(browsingTraj);
  double tGoal = t + distanceLeft / ab.norm();
  Point goal = (Point)points_.at(browsingTraj) + ab * tGoal;
  previousClosestIndex = browsingTraj;
  return goal;
}

Point Path::pointAtBackwardDistanceFrom(const double distance, const Point &pointStart, size_t &previousClosestIndex) const {
  double t;
  size_t previousIndex;
  Point proj = pointClosestTo(pointStart, t, previousIndex);
  double distanceLeft = std::abs(distance);
  double pathLen = points_.at(previousIndex).distanceTo(proj);
  size_t browsingTraj = previousIndex;
  while (distanceLeft > pathLen) {  // while the distance left is greater than the length of a full segment
    t = 1.;
    browsingTraj -= 1;
    distanceLeft -= pathLen;
    pathLen = (points_.at(browsingTraj + 1) - points_.at(browsingTraj)).norm();
  }
  Point ab = points_.at(browsingTraj + 1) - points_.at(browsingTraj);
  double tGoal = t - distanceLeft / ab.norm();
  Point goal = (Point)points_.at(browsingTraj) + ab * tGoal;
  previousClosestIndex = browsingTraj;
  return goal;
}

double Path::distanceBetween(const Point &a, const Point &b) const {
  double t1, t2;
  size_t i1, i2;
  Point proj1 = pointClosestTo(a, t1, i1);
  Point proj2 = pointClosestTo(b, t2, i2);
  if (i1 == i2) {
    // Both projections are on the same segment
    return proj1.distanceTo(proj2);
  }
  if (i1 < i2) {
    double dist = proj1.distanceTo(points_.at(i1 + 1));
    for (size_t i = i1 + 1; i < i2; i++) {
      dist += points_.at(i).distanceTo(points_.at(i + 1));
    }
    dist += proj2.distanceTo(points_.at(i2));
    return dist;
  } else {
    double dist = proj2.distanceTo(points_.at(i2 + 1));
    for (size_t i = i2 + 1; i < i1; i++) {
      dist += points_.at(i).distanceTo(points_.at(i + 1));
    }
    dist += proj1.distanceTo(points_.at(i1));
    return dist;
  }
}

double Path::mengerCurvature(const size_t i, const double distance) const {
  // bad idea... Should use the angle instead: the curvature depends on the length between considered points. Does not work for polyline
  // Update: Actually, if we keep a specified distance to the point (before and after), it would return a curvature not depending on the distance between
  // consecutive points
  if (points_.size() < 2) {
    return 0.0;
  }
  if (i == 0 || i >= points_.size() - 1) {
    return 0.0;
  }
  const PointOriented &y = points_.at(i);
  size_t k;
  Point x = pointAtBackwardDistanceFrom(distance, y, k);
  Point z = pointAtDistanceFrom(distance, y, k);
  double xy = distanceBetween(x, y);
  double yz = distanceBetween(y, z);

  if (xy < distance - 0.001 && xy <= yz) {
    z = pointAtDistanceFrom(xy, y, k);
    yz = distanceBetween(y, z);
  } else if (yz < distance - 0.001 && yz <= xy) {
    x = pointAtBackwardDistanceFrom(yz, y, k);
    xy = distanceBetween(x, y);
  }

  if (x == y || y == z || x == z) {
    return 0.0;
  }
  const Angle xyzAngle = (x - y).angleBetweenVectors(z - y);
  return 2. * xyzAngle.sin() / (x - z).norm();
}

const PointOriented Path::at(size_t i) const { return points_.at(i); }

Trajectory Path::computeSpeeds(const double maxLinearSpeed, const double maxRotationalSpeed, const double maxCentripetalAcceleration, const double maxLinearAcceleration) const {
  std::vector<double> speeds(points_.size(), 0.);
  if (points_.size() < 2) {
    return Trajectory(*this, speeds);
  }
  speeds.front() = 0.;
  speeds.back() = 0.;
  for (size_t i = 1; i < points_.size() - 1; i++) {
    const double curvature = mengerCurvature(i, 20.);
    if (curvature < 0.01) {
      speeds.at(i) = maxLinearSpeed;  // Trajectory is straight, full speed
    } else {
      double centripetalAccMaxSpeed = std::sqrt(maxCentripetalAcceleration / curvature);  // Centripetal acceleration = v**2 * curvature
      double maxRotSpeed = maxRotationalSpeed / curvature;                        // maxRotSpeed / curvature = vx  (vtheta = vx * c)
      double minSpeed = std::min(maxRotSpeed, centripetalAccMaxSpeed);
      speeds.at(i) = std::min(maxLinearSpeed, std::max(0., minSpeed));
    }
  }

  for (int i = points_.size() - 2; i >= 0; i--) {
    // Browse the trajectory in reverse to find if the speed at a point dictates the speed at a previous one
    const PointOriented &next = points_.at(i + 1);
    const PointOriented &current = points_.at(i);
    const double dist = current.distanceTo(next);
    const double maxSpeedAtMaxDecel = std::sqrt(speeds.at(i + 1) * speeds.at(i + 1) + 2. * maxLinearAcceleration * dist);  // sqrt(v0**2 + 2 * maxAcc * distToTravel)
    speeds.at(i) = std::min(speeds.at(i), maxSpeedAtMaxDecel);
    // Speed is min between max speed due to traj angle and max possible speed to be at the next point at the right speed
  }
  return Trajectory(*this, speeds);
}

Trajectory::Trajectory() : Path(), speeds_({}) {}

Trajectory::Trajectory(const Path &path, const std::vector<double> &speeds) : Path(path) {
  for (const double speed : speeds) {
    speeds_.push_back(speed);
  }
  assert(points_.size() == speeds_.size());
}

const PointOrientedSpeed Trajectory::at(size_t i) const { return PointOrientedSpeed(points_.at(i), speeds_.at(i)); }

Point Trajectory::pointWithSpeedClosestTo(const Point &point, double &tOut, size_t &closestPrevIndex) const {
  double distMin = std::numeric_limits<double>::max();
  double tMin = 0.;
  size_t iMin = -1;
  Point pointMin;
  for (size_t i = 0; i < size() - 1; i++) {
    double t;
    Point pt = point.closestPointBetween(points_.at(i), points_.at(i + 1), t);
    double dist = pt.squaredDistanceTo(point);
    if (dist < distMin) {
      distMin = dist;
      tMin = t;
      iMin = i;
      pointMin = pt;
    }
    if (i != 0 && speeds_.at(i) == 0.0) {
      break;
    }
  }
  tOut = tMin;
  closestPrevIndex = iMin;
  return pointMin;
}

Point Trajectory::pointWithSpeedAtDistanceFrom(const double distance, const Point &pointStart, size_t &previousClosestIndex) const {
  double t;
  size_t previousIndex;
  Point proj = pointWithSpeedClosestTo(pointStart, t, previousIndex);

  double distanceLeft = distance;
  double pathLen = points_.at(previousIndex + 1).distanceTo(proj);
  size_t browsingTraj = previousIndex;
  while (distanceLeft > pathLen) {  // while the distance left is greater than the length of a full segment
    if (speeds_.at(browsingTraj + 1) == 0.0) {
      // If the robot must stop on this point, do not explore the rest, manage to get it there
      t = 0.;
      distanceLeft = (points_.at(browsingTraj + 1) - points_.at(browsingTraj)).norm();
      break;
    }
    t = 0.;
    browsingTraj += 1;
    distanceLeft -= pathLen;
    pathLen = (points_.at(browsingTraj + 1) - points_.at(browsingTraj)).norm();
  }
  Point ab = points_.at(browsingTraj + 1) - points_.at(browsingTraj);
  double tGoal = t + distanceLeft / ab.norm();
  Point goal = (Point)points_.at(browsingTraj) + ab * tGoal;
  previousClosestIndex = browsingTraj;
  return goal;
}

Point Trajectory::pointWithSpeedAtBackwardDistanceFrom(const double distance, const Point &pointStart, size_t &previousClosestIndex) const {
  double t;
  size_t previousIndex;
  Point proj = pointWithSpeedClosestTo(pointStart, t, previousIndex);
  double distanceLeft = std::abs(distance);
  double pathLen = points_.at(previousIndex).distanceTo(proj);
  size_t browsingTraj = previousIndex;
  while (distanceLeft > pathLen) {  // while the distance left is greater than the length of a full segment
    if (speeds_.at(browsingTraj) == 0.0) {
      // If the robot must stop on this point, do not explore the rest, manage to get it there
      t = 1.;
      distanceLeft = (points_.at(browsingTraj + 1) - points_.at(browsingTraj)).norm();
      break;
    }
    t = 1.;
    browsingTraj -= 1;
    distanceLeft -= pathLen;
    pathLen = (points_.at(browsingTraj + 1) - points_.at(browsingTraj)).norm();
  }
  Point ab = points_.at(browsingTraj + 1) - points_.at(browsingTraj);
  double tGoal = t - distanceLeft / ab.norm();
  Point goal = (Point)points_.at(browsingTraj) + ab * tGoal;
  previousClosestIndex = browsingTraj;
  return goal;
}
}  // namespace rd