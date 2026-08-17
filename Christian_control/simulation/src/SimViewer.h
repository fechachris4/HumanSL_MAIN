//
// SimViewer — optional passive MuJoCo viewer for humansl_sim, compiled
// only under -DHUMANSL_SIM_VIEWER=ON (vendored static GLFW + system
// X11/GL). Strictly read-only: it renders the plant state between
// control ticks on the same single thread and never writes mjData,
// advances time, or feeds anything back into the run. Headless remains
// the default and the only mode tests use.
//

#pragma once

#include <mujoco/mujoco.h>

struct GLFWwindow;

class SimViewer {
public:
    // Opens the window and builds the render scene/context. Throws
    // std::runtime_error if GLFW cannot initialise (e.g. no display).
    explicit SimViewer(const mjModel& model);
    ~SimViewer();

    SimViewer(const SimViewer&) = delete;
    SimViewer& operator=(const SimViewer&) = delete;

    // Draws one frame of the given state. Returns false once the user
    // has closed the window (the caller drops the viewer and the run
    // continues headless).
    bool Render(const mjModel& model, const mjData& data);

private:
    GLFWwindow* window_ = nullptr;
    mjvCamera camera_{};
    mjvOption option_{};
    mjvPerturb perturb_{};
    mjvScene scene_{};
    mjrContext context_{};
};
