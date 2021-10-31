#include "GeometryTools/Speed.h"

namespace rd {
Speed::Speed(double vx, double vy, double vtheta) : linearSpeed_(vx, vy), rotationalSpeed_(vtheta) {}

double Speed::linearSpeed() const { return linearSpeed_.norm(); }
}  // namespace rd
