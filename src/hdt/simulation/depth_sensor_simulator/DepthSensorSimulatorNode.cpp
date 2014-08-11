#include "DepthSensorSimulatorNode.h"

#include <memory>
#include <SDL.h>
#include <GL/glew.h>
//#include <GL/gl.h>
#include <hdt/common/utils/RunUponDestruction.h>

DepthSensorSimulatorNode::DepthSensorSimulatorNode() :
    nh_(),
    ph_("~")
{

}

DepthSensorSimulatorNode::~DepthSensorSimulatorNode()
{
    SDL_Quit();
}

int DepthSensorSimulatorNode::run(int argc, char* argv[])
{
    if (SDL_Init(SDL_INIT_EVERYTHING) < 0) {
        ROS_ERROR("Failed to initialize SDL");
        return FAILED_TO_INITIALIZE;
    }

    const int win_width = 1024;
    const int win_height = 786;
    std::unique_ptr<SDL_Window, std::function<void(SDL_Window*)>> window(
            SDL_CreateWindow(
                    "Depth Sensor Simulator",
                    SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
                    win_width, win_height,
                    SDL_WINDOW_OPENGL),
            [](SDL_Window* win) { if (win) SDL_DestroyWindow(win); });

    if (!window) {
        ROS_ERROR("Failed to create a window (%s)", SDL_GetError());
        SDL_ClearError();
        return FAILED_TO_INITIALIZE;
    }

    // NOTE: a little dirty in that we know here that SDL_GL_Context is represented by a void*
    std::unique_ptr<void, std::function<void(void*)>> rendering_context(SDL_GL_CreateContext(window.get()), [](void* context) { if (context) SDL_GL_DeleteContext(context); });
    if (!rendering_context) {
        ROS_ERROR("Failed to create rendering context (%s)", SDL_GetError());
        SDL_ClearError();
        return FAILED_TO_INITIALIZE;
    }

    if (glewInit() != GLEW_OK) {
        ROS_ERROR("Failed to initialize GLEW");
        return FAILED_TO_INITIALIZE;
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Init GL
    ////////////////////////////////////////////////////////////////////////////////

    glClearColor(0.5f, 0.5f, 0.5f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glViewport(0, 0, win_width, win_height);

    GLuint VertexArrayID;
    glGenVertexArrays(1, &VertexArrayID);
    glBindVertexArray(VertexArrayID);

    ////////////////////////////////////////////////////////////////////////////////
    // Executive loop
    ////////////////////////////////////////////////////////////////////////////////

    ros::Rate executive_rate(1.0);
    bool shutdown = false;
    bool skip_sleep = false;
    while (ros::ok() && !shutdown) {
        RunUponDestruction rod([&]() {
            if (skip_sleep) {
                return;
            }
            executive_rate.sleep();
            SDL_GL_SwapWindow(window.get());
        });

        SDL_Event event;
        while (SDL_PollEvent(&event))
        {
            switch (event.type)
            {
            case SDL_QUIT:
                shutdown = true;
                break;
            case SDL_WINDOWEVENT:
                switch (event.window.event) {
                case SDL_WINDOWEVENT_RESIZED:
                    ROS_INFO("Resized to %d x %d", event.window.data1, event.window.data2);
                    glViewport(0, 0, event.window.data1, event.window.data2);
                    break;
                }
                break;
            default:
                break;
            }
        }

        if (shutdown) {
            skip_sleep = true;
            break;
        }

        ROS_INFO("Spin");

        if (!last_planning_scene_world_) {
            continue;
        }

        last_planning_scene_world_->collision_objects;
    }

    return SUCCESS;
}

void DepthSensorSimulatorNode::planning_scene_world_cb(const moveit_msgs::PlanningSceneWorld::ConstPtr& msg)
{
    last_planning_scene_world_ = msg;

    for (const auto& collision_object : last_planning_scene_world_->collision_objects) {
//        collision_object.
    }
}
