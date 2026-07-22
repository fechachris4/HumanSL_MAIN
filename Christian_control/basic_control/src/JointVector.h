//
// JointVector: one value per joint, in Kortex actuator order: index 0 =
// joint 1 ... index 6 = joint 7 (same ordering as feedback.actuators(i) /
// command.actuators(i)). Fixed size, so "exactly 7 values" is enforced by
// the type at compile time.
//

#ifndef HUMANSL_MASTERS_PROJECT_2025_JOINTVECTOR_H
#define HUMANSL_MASTERS_PROJECT_2025_JOINTVECTOR_H

#include <array>

using JointVector = std::array<double, 7>;

#endif // HUMANSL_MASTERS_PROJECT_2025_JOINTVECTOR_H
