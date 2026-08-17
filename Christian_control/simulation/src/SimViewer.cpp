#include "SimViewer.h"

#include <stdexcept>

#include <GLFW/glfw3.h>

SimViewer::SimViewer(const mjModel& model)
{
    if (!glfwInit())
        throw std::runtime_error("SimViewer: glfwInit failed (no display?)");
    window_ = glfwCreateWindow(1200, 900, "humansl_sim", nullptr, nullptr);
    if (!window_) {
        glfwTerminate();
        throw std::runtime_error("SimViewer: glfwCreateWindow failed");
    }
    glfwMakeContextCurrent(window_);
    glfwSwapInterval(1);

    mjv_defaultCamera(&camera_);
    mjv_defaultOption(&option_);
    mjv_defaultPerturb(&perturb_);
    mjv_defaultScene(&scene_);
    mjr_defaultContext(&context_);
    mjv_makeScene(&model, &scene_, 2000);
    mjr_makeContext(&model, &context_, mjFONTSCALE_150);

    // Frame the whole rig: look at the model's centre from a few metres.
    camera_.type = mjCAMERA_FREE;
    camera_.distance = 2.5;
    camera_.elevation = -20.0;
}

SimViewer::~SimViewer()
{
    mjr_freeContext(&context_);
    mjv_freeScene(&scene_);
    if (window_)
        glfwDestroyWindow(window_);
    glfwTerminate();
}

bool SimViewer::Render(const mjModel& model, const mjData& data)
{
    if (glfwWindowShouldClose(window_))
        return false;

    // mjv_updateScene takes a non-const mjData* but only reads the
    // physics state for rendering; the const_cast is confined to this
    // render-only boundary (the viewer never steps or edits the plant).
    mjData* mutable_data = const_cast<mjData*>(&data);

    mjrRect viewport{0, 0, 0, 0};
    glfwGetFramebufferSize(window_, &viewport.width, &viewport.height);
    mjv_updateScene(&model, mutable_data, &option_, &perturb_, &camera_,
                    mjCAT_ALL, &scene_);
    mjr_render(viewport, &scene_, &context_);
    glfwSwapBuffers(window_);
    glfwPollEvents();
    return true;
}
