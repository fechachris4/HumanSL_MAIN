#include <cmath>
#include <cstdio>

#include "GoalSocket.h"

namespace {
int failures = 0;
void Check(bool condition, const char* what)
{
    if (!condition) {
        std::printf("FAIL: %s\n", what);
        ++failures;
    }
}
}

int main()
{
    GoalCommand point;
    Check(!ParseGoalCommandLine("POINT 0.4 0.2 0.3 INHERIT", 7, point),
          "point command parses");
    Check(point.command_id == 7 && point.kind == GoalKind::kPoint,
          "point command carries identity and kind");

    GoalCommand circle;
    Check(!ParseGoalCommandLine(
              "CIRCLE 0.4 0.2 0.3 0.1 1 0 0 5 RADIAL", 8, circle),
          "radial circle parses");
    Check(circle.orientation == GoalOrientation::kRadialInward,
          "circle orientation is typed");

    GoalCommand invalid;
    Check(ParseGoalCommandLine("POINT nan 0 0 INHERIT", 9, invalid).has_value(),
          "non-finite command rejected");

    GoalCommandSlot slot;
    point.command_id = 10;
    slot.Publish(point);
    circle.command_id = 11;
    slot.Publish(circle);
    GoalCommand latest;
    Check(slot.TakeLatest(latest) && latest.command_id == 11,
          "latest command supersedes unread older command");
    Check(!slot.TakeLatest(latest), "slot consumes latest command once");

    if (failures == 0)
        std::puts("test_goal_socket: all assertions passed");
    return failures == 0 ? 0 : 1;
}
