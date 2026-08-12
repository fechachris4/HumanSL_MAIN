#pragma once

#include "ViconInterface.h"
#include "ViconSnapshot.h"

#include <vector>

// Converts one frame's raw SDK reads (ViconInterface's millimetre,
// occluded-bool convention) into a validated ViconSnapshot: metres, and
// quaternions accepted only when finite and within tolerance of unit
// norm -- never silently normalised. Pure function: no SDK calls, no I/O,
// no state.
ViconSnapshot BuildSnapshot(unsigned int frame_number, double host_time_s,
                             double frame_rate_hz, double latency_total_s,
                             const std::vector<MarkerData>& markers,
                             const std::vector<SegmentData>& segments);
