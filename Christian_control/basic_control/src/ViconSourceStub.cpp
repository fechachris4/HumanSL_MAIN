//
// ViconSource, stub implementation — built when third_party/vicon_api is
// absent (or -DNO_VICON=ON), so the header's promise at the top of
// CMakeLists.txt stays true: the controller builds and runs with no Vicon
// SDK anywhere on the machine. StartViconSource returns nullptr; the
// caller prints why and the vicon_* log columns keep their documented
// absence values (NaN poses, sequence 0).
//

#include "ViconSource.h"

const char* const kViconSourceBuildMode = "absent";
const char* const kViconSourceHost = "192.168.128.206:801";

std::unique_ptr<ViconSource> StartViconSource(BasePoseSlot&)
{
    return nullptr;
}
