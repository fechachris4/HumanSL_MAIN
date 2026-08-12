#include "ViconInterface.h"

#include <cassert>

int main() {
    SegmentData empty_segment;
    assert(empty_segment.subject_name.empty());
    assert(empty_segment.segment_name.empty());
    assert(empty_segment.occluded);

    ViconInterface vicon;
    assert(vicon.getSegmentPoses().empty());
    return 0;
}
