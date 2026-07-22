//
// Measure: read sensor data from the arm (no motion commands).
//

#ifndef HUMANSL_MASTERS_PROJECT_2025_MEASURE_H
#define HUMANSL_MASTERS_PROJECT_2025_MEASURE_H

#include <BaseCyclicClientRpc.h>

namespace k_api = Kinova::Api;

// THE robot state reader: the program's single standalone RefreshFeedback;
// the cyclic loop instead reuses Refresh(command)'s reply —
// docs/decisions/single-loop-controller.md ("single reader").
k_api::BaseCyclic::Feedback read_feedback(k_api::BaseCyclic::BaseCyclicClient* base_cyclic);

#endif // HUMANSL_MASTERS_PROJECT_2025_MEASURE_H
