//
// Measure: read sensor data from the arm (no motion commands).
//

#include "hardware/Measure.h"

k_api::BaseCyclic::Feedback read_feedback(k_api::BaseCyclic::BaseCyclicClient* base_cyclic)
{
    // The ONLY RefreshFeedback call in the program (see Measure.h).
    return base_cyclic->RefreshFeedback();
}
