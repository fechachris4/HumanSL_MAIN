// Test assertions must never no-op under a Release (NDEBUG) configure.
#undef NDEBUG

#include <cassert>
#include <cmath>
#include "WorldSdf.h"
int main() {
    const auto empty = MakeWorldSdf(std::nullopt);
    assert(empty.getSignedDistance(gtsam::Point3(0.4, 0.0, 0.4)) > 1.0);

    AxisAlignedBox box{{0.4, 0.0, 0.4}, {0.1, 0.1, 0.1}};
    const auto world = MakeWorldSdf(box);
    assert(world.getSignedDistance(gtsam::Point3(0.4, 0.0, 0.4)) < 0.0);   // inside
    const double near = world.getSignedDistance(gtsam::Point3(0.4, 0.0, 0.55));
    assert(near > 0.0 && near < 0.1);                                      // 5 cm off face
    assert(world.getSignedDistance(gtsam::Point3(-0.8, -0.8, 0.0)) > 0.5); // far
    return 0;
}
