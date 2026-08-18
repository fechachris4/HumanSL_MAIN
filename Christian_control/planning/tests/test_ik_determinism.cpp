// The planner must produce the same plan for the same request.
//
// Before this was fixed, analytical_ik seeded from std::random_device, so
// identical circle requests traced to 2.406-2.409 mm every time but by
// DIFFERENT joint-space routes — closure drift swung 0.04-14 deg between
// runs of the same input. Cartesian repeatability is not enough: a
// controller experiment that cannot replay its own trajectory cannot
// attribute a difference in behaviour to the change it was testing.
//
// The tension this resolves is that the solver's exploration is genuinely
// random-restart based, and removing the randomness would weaken it. Streams
// keep both: different restarts within a run, identical restarts across runs.
#undef NDEBUG

#include <cassert>
#include <cstdio>
#include <set>

#include "analytical_ik.h"

int main() {
    Eigen::Matrix<double, 7, 1> preferred;
    preferred << 0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7;

    // --- same seed and stream => identical draws, every time.
    analytical_ik::IKSeeding seeding;
    seeding.seed = 12345;
    seeding.stream = 0;
    const auto first =
        analytical_ik::AnalyticalIKSolver::GenerateSeedsForTest(preferred, 6, seeding);
    const auto again =
        analytical_ik::AnalyticalIKSolver::GenerateSeedsForTest(preferred, 6, seeding);
    assert(first.size() == 6 && again.size() == 6);
    for (std::size_t i = 0; i < first.size(); ++i)
        assert((first[i] - again[i]).norm() < 1e-15 &&
               "the same seed and stream must draw the same restarts");

    // --- the RNG must NOT be a call-advancing static: an intervening call
    //     with a different stream must not change what this stream draws.
    analytical_ik::IKSeeding other = seeding;
    other.stream = 99;
    (void)analytical_ik::AnalyticalIKSolver::GenerateSeedsForTest(preferred, 6, other);
    const auto after_interference =
        analytical_ik::AnalyticalIKSolver::GenerateSeedsForTest(preferred, 6, seeding);
    for (std::size_t i = 0; i < first.size(); ++i)
        assert((first[i] - after_interference[i]).norm() < 1e-15 &&
               "draws must not depend on how many solves ran earlier — that is "
               "what a function-local static generator would do");

    // --- different streams must actually explore differently, or the
    //     multi-restart search collapses to one repeated attempt.
    analytical_ik::IKSeeding stream_b = seeding;
    stream_b.stream = 1;
    const auto b =
        analytical_ik::AnalyticalIKSolver::GenerateSeedsForTest(preferred, 6, stream_b);
    bool any_different = false;
    for (std::size_t i = 1; i < first.size(); ++i)  // index 0 is the preferred seed
        if ((first[i] - b[i]).norm() > 1e-9) any_different = true;
    assert(any_different &&
           "different streams must draw different restarts, or exploration is lost");

    // --- a different seed must also differ, so the seed is a real control.
    analytical_ik::IKSeeding other_seed = seeding;
    other_seed.seed = 999;
    const auto c = analytical_ik::AnalyticalIKSolver::GenerateSeedsForTest(
        preferred, 6, other_seed);
    any_different = false;
    for (std::size_t i = 1; i < first.size(); ++i)
        if ((first[i] - c[i]).norm() > 1e-9) any_different = true;
    assert(any_different && "changing the seed must change the restarts");

    // --- the first restart is always the caller's preferred configuration:
    //     the walk starts from where the arm actually is.
    assert((first[0] - preferred).norm() < 1e-15);

    // --- nearby (seed, stream) pairs must not correlate. Adjacent seeds are
    //     the common case (a sweep), and a weak mix would make a "different"
    //     run explore almost the same space.
    std::set<std::uint64_t> mixed;
    for (std::uint64_t s = 0; s < 64; ++s)
        for (std::uint64_t st = 0; st < 16; ++st)
            mixed.insert(analytical_ik::IKSeeding{s, st}.Mixed());
    assert(mixed.size() == 64 * 16 &&
           "every (seed, stream) pair must mix to a distinct value");

    std::printf("all IK determinism tests passed\n");
    return 0;
}
