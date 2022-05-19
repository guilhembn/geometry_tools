#include "GeometryTools/Trajectory.h"

#include <fstream>
#include <limits>
#include <sstream>
#include <unordered_set>

namespace rd
{

  Path::Path() : points_({}) {}

  Path::Path(const std::vector<PointOriented> &points)
  {
    for (PointOriented p : points)
    {
      points_.push_back(p);
    }
  }

  Path Path::lissajouPath(const PointOriented &robotPose, const size_t steps, const double sizeX, const double sizeY)
  {
    double theta = 0.;
    double dtheta = 2 * M_PI / steps;
    const double halfSizeX = sizeX / 2.;
    const double halfSizeY = sizeY / 2.;
    std::vector<PointOriented> pts;
    pts.reserve(steps);
    for (size_t i = 0; i < steps; i++)
    {
      theta = i * dtheta;
      pts.emplace_back(robotPose.x() + halfSizeX * std::sin(theta), robotPose.y() + halfSizeY * std::sin(2 * theta), 0.0);
    }
    return Path(pts);
  }

  Path Path::cubicBezier(const PointOriented &startPoint, const Point &controlPoint1, const Point &controlPoint2, const PointOriented &endPoint, const size_t steps)
  {
    double t = 0.;
    double dt = 1. / steps;
    std::vector<PointOriented> pts;
    pts.reserve(steps);
    const Point &sp = static_cast<Point>(startPoint);
    const Point &ep = static_cast<Point>(endPoint);
    for (size_t i = 0; i < steps - 1; i++)
    {
      t = dt * i;
      pts.emplace_back(sp * std::pow((1 - t), 3) + controlPoint1 * 3 * t * std::pow((1 - t), 2) + controlPoint2 * 3 * std::pow(t, 2) * (1 - t) + ep * std::pow(t, 3), 0.);
    }
    t = dt * steps;
    pts.emplace_back(sp * std::pow((1 - t), 3) + controlPoint1 * 3 * t * std::pow((1 - t), 2) + controlPoint2 * 3 * std::pow(t, 2) * (1 - t) + ep * std::pow(t, 3), endPoint.theta().value());
    return Path(pts);
  }

  Path Path::fromSVG(const std::string &svgPath, const size_t steps)
  {
    std::ifstream file;
    file.open(svgPath);
    std::string fullLine;
    Path toRet;
    if (!file.is_open())
    {
      throw std::runtime_error("Cannot open file");
    }
    std::string line;
    while (std::getline(file, line))
    {
      if (line.find("<path") != std::string::npos)
      {
        while (line.find("d=\"") == std::string::npos)
        {
          if (!std::getline(file, line))
          {
            throw std::runtime_error("Cannot find 'd' attribute in 'path' tag.");
          }
        }
        if (line.find("\"") != line.rfind("\""))
        {
          fullLine = line.substr(line.find("\"") + 1, line.rfind("\"") - line.find("\""));
        }
        else
        {
          fullLine = line.substr(line.find("\"") + 1);
          std::getline(file, line);
          while (line.find("\"") == std::string::npos)
          {
            fullLine += line;
            if (!std::getline(file, line))
            {
              throw std::runtime_error("Cannot find '\"' end delimiter to 'd' attribute");
            }
          }
          fullLine += line.substr(0, line.find("\""));
          break;
        }
      }
    }
    file.close();
    if (fullLine == "")
    {
      return Path();
    }
    std::vector<std::string> tokens;
    std::stringstream ss(fullLine); // Turn the string into a stream.
    std::string tok;
    while (ss >> tok)
    {
      tokens.push_back(tok);
    }
    std::string cmd = "";
    const std::unordered_set<std::string> cmds({"M", "m", "c", "C", "z"});
    std::vector<Point> beziersPoints;
    Point currentPt;
    for (const std::string &token : tokens)
    {
      if (token.size() == 1)
      {
        cmd = token;
        if (cmd == "z")
        {
          toRet += Path({toRet.at(0)});
        }
      }
      else
      {
        if (token.find(",") != token.size())
        {
          std::string xStr = token.substr(0, token.find(","));
          std::string yStr = token.substr(token.find(",") + 1);
          double x = atof(xStr.c_str());
          double y = atof(yStr.c_str());
          Point pt;
          if (cmd == "m" || cmd == "c")
          {
            pt = Point(x, y) + currentPt;
          }
          else
          {
            pt = Point(x, y);
          }
          if (cmd == "m" || cmd == "M")
          {
            currentPt = pt;
          }
          else if (cmd == "c" || cmd == "C")
          {
            if (beziersPoints.size() == 0)
            {
              beziersPoints.push_back(currentPt);
            }
            beziersPoints.push_back(pt);
            if (beziersPoints.size() == 4)
            {
              toRet += cubicBezier({beziersPoints.at(0), 0}, beziersPoints.at(1), beziersPoints.at(2), {beziersPoints.at(3), 0}, steps);
              beziersPoints.clear();
              currentPt = pt;
            }
          }
          else
          {
            std::cout << "Warning: Unknown svg path command '" << cmd << "'. Trying to continue." << std::endl;
          }
        }
      }
    }
    return toRet;
  }

  Point Path::pointClosestTo(const Point &point, double &tOut, size_t &closestPrevIndex) const
  {
    double distMin = std::numeric_limits<double>::max();
    double tMin = 0.;
    size_t iMin = -1;
    Point pointMin;
    for (size_t i = 0; i < size() - 1; i++)
    {
      double t;
      Point pt = point.closestPointBetween(points_.at(i), points_.at(i + 1), t);
      double dist = pt.squaredDistanceTo(point);
      if (dist < distMin)
      {
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

  Point Path::nextPointClosestTo(const Point &point, double &tOut, size_t &closestPrevIndex) const
  {
    double distMin = std::numeric_limits<double>::max();
    double tMin = 0.;
    size_t iMin = -1;
    Point pointMin;
    for (size_t i = 0; i < size() - 1; i++)
    {
      double t;
      Point pt = point.closestPointBetween(points_.at(i), points_.at(i + 1), t);
      double dist = pt.squaredDistanceTo(point);
      if (dist < distMin)
      {
        distMin = dist;
        tMin = t;
        iMin = i;
        pointMin = pt;
      }
      if (dist > distMin)
      {
        break;
      }
    }
    tOut = tMin;
    closestPrevIndex = iMin;
    return pointMin;
  }

  Point Path::pointAtDistanceFrom(const double distance, const Point &pointStart, size_t &previousClosestIndex) const
  {
    double t;
    size_t previousIndex;
    Point proj = pointClosestTo(pointStart, t, previousIndex);

    double distanceLeft = distance;
    double pathLen = points_.at(previousIndex + 1).distanceTo(proj);
    size_t browsingTraj = previousIndex;
    while (distanceLeft > pathLen)
    { // while the distance left is greater than the length of a full segment
      t = 0.;
      browsingTraj += 1;
      distanceLeft -= pathLen;
      if (browsingTraj == points_.size() - 2)
        break;
      pathLen = (points_.at(browsingTraj + 1) - points_.at(browsingTraj)).norm();
    }
    Point ab = points_.at(browsingTraj + 1) - points_.at(browsingTraj);
    double tGoal = t + distanceLeft / ab.norm();
    Point goal = (Point)points_.at(browsingTraj) + ab * tGoal;
    previousClosestIndex = browsingTraj;
    return goal;
  }

  Point Path::pointAtBackwardDistanceFrom(const double distance, const Point &pointStart, size_t &previousClosestIndex) const
  {
    double t;
    size_t previousIndex;
    Point proj = pointClosestTo(pointStart, t, previousIndex);
    double distanceLeft = std::abs(distance);
    double pathLen = points_.at(previousIndex).distanceTo(proj);
    size_t browsingTraj = previousIndex;
    while (distanceLeft > pathLen)
    { // while the distance left is greater than the length of a full segment
      t = 1.;
      distanceLeft -= pathLen;
      if (browsingTraj == 0)
        break;
      browsingTraj -= 1;
      pathLen = (points_.at(browsingTraj + 1) - points_.at(browsingTraj)).norm();
    }
    Point ab = points_.at(browsingTraj + 1) - points_.at(browsingTraj);
    double tGoal = t - distanceLeft / ab.norm();
    Point goal = (Point)points_.at(browsingTraj) + ab * tGoal;
    previousClosestIndex = browsingTraj;
    return goal;
  }

  double Path::distanceBetween(const Point &a, const Point &b) const
  {
    double t1, t2;
    size_t i1, i2;
    Point proj1 = pointClosestTo(a, t1, i1);
    Point proj2 = pointClosestTo(b, t2, i2);
    if (i1 == i2)
    {
      // Both projections are on the same segment
      return proj1.distanceTo(proj2);
    }
    if (i1 < i2)
    {
      double dist = proj1.distanceTo(points_.at(i1 + 1));
      for (size_t i = i1 + 1; i < i2; i++)
      {
        dist += points_.at(i).distanceTo(points_.at(i + 1));
      }
      dist += proj2.distanceTo(points_.at(i2));
      return dist;
    }
    else
    {
      double dist = proj2.distanceTo(points_.at(i2 + 1));
      for (size_t i = i2 + 1; i < i1; i++)
      {
        dist += points_.at(i).distanceTo(points_.at(i + 1));
      }
      dist += proj1.distanceTo(points_.at(i1));
      return dist;
    }
  }

  double Path::mengerCurvature(const size_t i, const double distance) const
  {
    // bad idea... Should use the angle instead: the curvature depends on the length between considered points. Does not work for polyline
    // Update: Actually, if we keep a specified distance to the point (before and after), it would return a curvature not depending on the distance between
    // consecutive points
    if (points_.size() < 2)
    {
      return 0.0;
    }
    if (i == 0 || i >= points_.size() - 1)
    {
      return 0.0;
    }
    const PointOriented &y = points_.at(i);
    size_t k;
    Point x = pointAtBackwardDistanceFrom(distance, y, k);
    Point z = pointAtDistanceFrom(distance, y, k);
    double xy = distanceBetween(x, y);
    double yz = distanceBetween(y, z);

    if (xy < distance - 0.001 && xy <= yz)
    {
      z = pointAtDistanceFrom(xy, y, k);
      yz = distanceBetween(y, z);
    }
    else if (yz < distance - 0.001 && yz <= xy)
    {
      x = pointAtBackwardDistanceFrom(yz, y, k);
      xy = distanceBetween(x, y);
    }

    if (x == y || y == z || x == z)
    {
      return 0.0;
    }
    const Angle xyzAngle = (x - y).angleBetweenVectors(z - y);
    return 2. * xyzAngle.sin() / (x - z).norm();
  }

  const PointOriented Path::at(size_t i) const { return points_.at(i); }

  Trajectory Path::computeSpeeds(const double maxLinearSpeed, const double, const double, const double) const
  {
    std::vector<double> speeds(points_.size(), maxLinearSpeed);
    if (points_.size() < 2)
    {
      return Trajectory(*this, speeds);
    }
    /*speeds.front() = 0.;
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
    }*/
    return Trajectory(*this, speeds);
  }

  Trajectory::Trajectory() : Path(), speeds_({}) {}

  Trajectory::Trajectory(const Path &path, const std::vector<double> &speeds) : Path(path)
  {
    for (const double speed : speeds)
    {
      speeds_.push_back(speed);
    }
    assert(points_.size() == speeds_.size());
  }

  const PointOrientedSpeed Trajectory::at(size_t i) const { return PointOrientedSpeed(points_.at(i), speeds_.at(i)); }

  Point Trajectory::pointWithSpeedClosestTo(const Point &point, double &tOut, size_t &closestPrevIndex, const size_t minIndex, const size_t maxIndex) const
  {
    double distMin = std::numeric_limits<double>::max();
    double tMin = 0.;
    size_t iMin = 0;
    Point pointMin;
    for (size_t i = minIndex; i < std::min(maxIndex, size() - 2); i++)
    {
      double t;
      Point pt = point.closestPointBetween(points_.at(i), points_.at(i + 1), t);
      double dist = pt.squaredDistanceTo(point);
      if (dist < distMin)
      {
        distMin = dist;
        tMin = t;
        iMin = i;
        pointMin = pt;
      }
      if (i != 0 && speeds_.at(i) == 0.0)
      {
        break;
      }
    }
    tOut = tMin;
    closestPrevIndex = iMin;
    return pointMin;
  }

  Point Trajectory::pointWithSpeedAtDistanceFrom(const double distance, const Point &pointStart, size_t &previousClosestIndex, const size_t minIndex, const size_t maxIndex) const
  {
    double t = 0.;
    size_t previousIndex = 0;
    Point proj = pointWithSpeedClosestTo(pointStart, t, previousIndex, minIndex, maxIndex);

    double distanceLeft = distance;
    double pathLen = points_.at(previousIndex + 1).distanceTo(proj);
    size_t browsingTraj = previousIndex;
    while (distanceLeft > pathLen)
    { // while the distance left is greater than the length of a full segment
      if (speeds_.at(browsingTraj + 1) == 0.0 || points_.size() == browsingTraj + 2)
      {
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

  Point Trajectory::pointWithSpeedAtBackwardDistanceFrom(const double distance, const Point &pointStart, size_t &previousClosestIndex) const
  {
    double t;
    size_t previousIndex;
    Point proj = pointWithSpeedClosestTo(pointStart, t, previousIndex);
    double distanceLeft = std::abs(distance);
    double pathLen = points_.at(previousIndex).distanceTo(proj);
    size_t browsingTraj = previousIndex;
    while (distanceLeft > pathLen)
    { // while the distance left is greater than the length of a full segment
      if (speeds_.at(browsingTraj) == 0.0)
      {
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

  Path &Path::operator+=(const Path &rhs)
  {
    points_.insert(points_.end(), rhs.points_.begin(), rhs.points_.end());
    return *this;
  }
  Path Path::operator+(const Path &rhs) const
  {
    Path tmp(*this);
    tmp += rhs;
    return tmp;
  }
} // namespace rd