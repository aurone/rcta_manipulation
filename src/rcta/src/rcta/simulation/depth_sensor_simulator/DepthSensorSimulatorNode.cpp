#include "DepthSensorSimulatorNode.h"

// standard includes
#include <cmath>
#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

// system includes
#include <Eigen/Dense>
#include <SDL.h>
#include <GL/glew.h>
#include <rospack/rospack.h>
#include <sbpl_geometry_utils/utils.h>
#include <spellbook/utils/RunUponDestruction.h>

#define GL_CALL(fun, ...) fun(__VA_ARGS__); if (glGetError() != GL_NO_ERROR) { ROS_ERROR("Call to " #fun " return an error"); }

static Eigen::Projective3f CreatePerspectiveMatrix(float fovy, float aspect, float near, float far)
{
    Eigen::Projective3f projection;
    float f = 1.0 / tan(0.5 * fovy);
    projection(0, 0) = f / aspect;
    projection(1, 0) = 0.0f;
    projection(2, 0) = 0.0f;
    projection(3, 0) = 0.0f;

    projection(0, 1) = 0.0f;
    projection(1, 1) = f;
    projection(2, 1) = 0.0f;
    projection(3, 1) = 0.0f;

    projection(0, 2) = 0.0f;
    projection(1, 2) = 0.0f;
    projection(2, 2) = (far + near) / (near - far);
    projection(3, 2) = -1.0f;

    projection(0, 3) = 0.0f;
    projection(1, 3) = 0.0f;
    projection(2, 3) = (2 * far * near) / (near - far);
    projection(3, 3) = 0.0f;

    return projection;
}

/// Code derived from tutorial at http://www.opengl-tutorial.org/beginners-tutorials/tutorial-2-the-first-triangle/
static GLuint LoadShaders(const char* vertex_file_path, const char* fragment_file_path)
{
    GLuint vertex_shader_id = glCreateShader(GL_VERTEX_SHADER);
    GLuint fragment_shader_id = glCreateShader(GL_FRAGMENT_SHADER);

    // Read the Vertex Shader code from the file
    std::string vertex_shader_code;
    std::ifstream vertex_shader_stream(vertex_file_path, std::ios::in);
    if (vertex_shader_stream.is_open()) {
        std::string line = "";
        while (std::getline(vertex_shader_stream, line)) {
            vertex_shader_code += "\n" + line;
        }
        vertex_shader_stream.close();
    }

    // Read the Fragment Shader code from the file
    std::string fragment_shader_code;
    std::ifstream fragment_shader_stream(fragment_file_path, std::ios::in);
    if (fragment_shader_stream.is_open()) {
        std::string line = "";
        while (std::getline(fragment_shader_stream, line)) {
            fragment_shader_code += "\n" + line;
        }
        fragment_shader_stream.close();
    }

    GLint result = GL_FALSE;
    int info_log_length;

    // compile vertex shader
    printf("Compiling shader: %s\n", vertex_file_path);
    const char* vertex_source_ptr = vertex_shader_code.c_str();
    glShaderSource(vertex_shader_id, 1, &vertex_source_ptr, NULL);
    glCompileShader(vertex_shader_id);

    // check vertex shader
    glGetShaderiv(vertex_shader_id, GL_COMPILE_STATUS, &result);
    glGetShaderiv(vertex_shader_id, GL_INFO_LOG_LENGTH, &info_log_length);

    // compile fragment shader
    printf("Compiling shader: %s\n", fragment_file_path);
    const char* fragment_source_ptr = fragment_shader_code.c_str();
    glShaderSource(fragment_shader_id, 1, &fragment_source_ptr, NULL);
    glCompileShader(fragment_shader_id);

    // check the fragment shader
    glGetShaderiv(fragment_shader_id, GL_COMPILE_STATUS, &result);
    glGetShaderiv(fragment_shader_id, GL_INFO_LOG_LENGTH, &info_log_length);
    std::vector<char> fragment_shader_error_msg(info_log_length);
    glGetShaderInfoLog(fragment_shader_id, info_log_length, NULL, &fragment_shader_error_msg[0]);
    fprintf(stdout, "%s\n", &fragment_shader_error_msg[0]);

    std::vector<char> vertex_shader_error_msg((int)info_log_length);
    glGetShaderInfoLog(vertex_shader_id, info_log_length, NULL, &vertex_shader_error_msg[0]);
    fprintf(stdout, "%s\n", &vertex_shader_error_msg[0]);

    // link the program
    fprintf(stdout, "Linking program\n");
    GLuint program_id = glCreateProgram();
    glAttachShader(program_id, vertex_shader_id);
    glAttachShader(program_id, fragment_shader_id);
    glLinkProgram(program_id);

    // check the program
    glGetProgramiv(program_id, GL_LINK_STATUS, &result);
    glGetProgramiv(program_id, GL_INFO_LOG_LENGTH, &info_log_length);
    std::vector<char> program_error_msg(std::max(info_log_length, (int)1));
    glGetProgramInfoLog(program_id, info_log_length, NULL, &program_error_msg[0]);
    fprintf(stdout, "%s\n", &program_error_msg[0]);

    glDeleteShader(vertex_shader_id);
    glDeleteShader(fragment_shader_id);
    return program_id;
}

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
    // find the hdt package
    rospack::Rospack rpack;
    std::vector<std::string> package_search_path;
    if (!rpack.getSearchPathFromEnv(package_search_path)) {
        ROS_ERROR("Failed to retrieve ROS package search path from the environment. Ensure that $ROS_ROOT is set");
        return FAILED_TO_INITIALIZE;
    }

    rpack.crawl(package_search_path, false);
    const std::string package_name = "hdt";
    std::string package_path;
    rpack.find(package_name, package_path);

    ROS_INFO("Located package '%s' at '%s'", package_name.c_str(), package_path.c_str());

    std::string resource_path = package_path + "/resource/shaders";
    ROS_INFO("Shader resources must be located at '%s'", resource_path.c_str());

    if (SDL_Init(SDL_INIT_EVERYTHING) < 0) {
        ROS_ERROR("Failed to initialize SDL");
        return FAILED_TO_INITIALIZE;
    }

    const int win_width = 800;
    const int win_height = 600;
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

    static const GLfloat g_vertex_buffer_data[] = {
        -1.0f, -1.0f, 0.0f,
         1.0f, -1.0f, 0.0f,
         0.0f,  1.0f, 0.0f
    };

    static const GLfloat g_cube_vertex_buffer_data[] = {
        -1.0f, -1.0f, -1.0f,
        -1.0f, -1.0f,  1.0f,
        -1.0f,  1.0f, -1.0f,
        -1.0f,  1.0f,  1.0f,
         1.0f, -1.0f, -1.0f,
         1.0f, -1.0f,  1.0f,
         1.0f,  1.0f, -1.0f,
         1.0f,  1.0f,  1.0f
    };

    static const GLfloat g_cube_unindexed_vertex_buffer_data[] = {
        -1.0f,-1.0f,-1.0f, // triangle 1 : begin
        -1.0f,-1.0f, 1.0f,
        -1.0f, 1.0f, 1.0f, // triangle 1 : end
         1.0f, 1.0f,-1.0f, // triangle 2 : begin
        -1.0f,-1.0f,-1.0f,
        -1.0f, 1.0f,-1.0f, // triangle 2 : end
         1.0f,-1.0f, 1.0f,
        -1.0f,-1.0f,-1.0f,
         1.0f,-1.0f,-1.0f,
         1.0f, 1.0f,-1.0f,
         1.0f,-1.0f,-1.0f,
        -1.0f,-1.0f,-1.0f,
        -1.0f,-1.0f,-1.0f,
        -1.0f, 1.0f, 1.0f,
        -1.0f, 1.0f,-1.0f,
         1.0f,-1.0f, 1.0f,
        -1.0f,-1.0f, 1.0f,
        -1.0f,-1.0f,-1.0f,
        -1.0f, 1.0f, 1.0f,
        -1.0f,-1.0f, 1.0f,
         1.0f,-1.0f, 1.0f,
         1.0f, 1.0f, 1.0f,
         1.0f,-1.0f,-1.0f,
         1.0f, 1.0f,-1.0f,
         1.0f,-1.0f,-1.0f,
         1.0f, 1.0f, 1.0f,
         1.0f,-1.0f, 1.0f,
         1.0f, 1.0f, 1.0f,
         1.0f, 1.0f,-1.0f,
        -1.0f, 1.0f,-1.0f,
         1.0f, 1.0f, 1.0f,
        -1.0f, 1.0f,-1.0f,
        -1.0f, 1.0f, 1.0f,
         1.0f, 1.0f, 1.0f,
        -1.0f, 1.0f, 1.0f,
         1.0f,-1.0f, 1.0f
    };

    GLuint vertex_buffer;
    glGenBuffers(1, &vertex_buffer);

    glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof(g_vertex_buffer_data), g_vertex_buffer_data, GL_STATIC_DRAW);

    GLuint program_id = LoadShaders((resource_path + "/simple.vert").c_str(), (resource_path + "/simple.frag").c_str());

//    Eigen::Affine3f model_matrix = Eigen::Affine3f::Identity();

    Eigen::Affine3f model_matrix(Eigen::Affine3f::Identity()); // world -> model
//    Eigen::Affine3f model_matrix(Eigen::AngleAxisf(M_PI / 4.0, Eigen::Vector3f(0.0f, 0.0f, 1.0f)));

//    Eigen::Affine3f view_matrix = Eigen::Affine3f::Identity();
    Eigen::Affine3f view_matrix(Eigen::Translation3f(0.0f, 0.0f, -5.0f)); // camera -> world (where the camera faces down the -z axis)

    Eigen::Projective3f projection_matrix = CreatePerspectiveMatrix(sbpl::utils::ToRadians(60.0), 1.0f, 0.01f, 100.0f); // clip -> camera
//    Eigen::Projective3f projection_matrix(Eigen::Projective3f::Identity());

    // clip -> model
    Eigen::Projective3f transformation_matrix(projection_matrix * view_matrix * model_matrix);

    GLuint matrix_id = glGetUniformLocation(program_id, "MVP");
    if (matrix_id == -1) {
        ROS_ERROR("Failed to get uniform 'MVP'");
    }

    GL_CALL(glUseProgram, program_id);
    std::cout << "Uniform MVP @ " << matrix_id << std::endl;
    GL_CALL(glUniformMatrix4fv, matrix_id, 1, GL_FALSE, transformation_matrix.data());

    ////////////////////////////////////////////////////////////////////////////////
    // Executive loop
    ////////////////////////////////////////////////////////////////////////////////

    ros::Rate executive_rate(30.0);
    bool shutdown = false;
    bool skip_sleep = false;
    std::chrono::time_point<std::chrono::high_resolution_clock> then = std::chrono::high_resolution_clock::now();
    float triangle_rotation = 0.0f;
    while (ros::ok() && !shutdown) {
        std::chrono::time_point<std::chrono::high_resolution_clock> now = std::chrono::high_resolution_clock::now();
        double dt = std::chrono::duration<double, std::ratio<1>>(now - then).count();

        RunUponDestruction rod([&]() {
            then = now;
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

        ////////////////////////////////////////////////////////////////////////////////
        // Rendering
        ////////////////////////////////////////////////////////////////////////////////

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glUseProgram(program_id);

        triangle_rotation += M_PI / 4 * dt;
        model_matrix = Eigen::AngleAxisf(triangle_rotation, Eigen::Vector3f(0.0f, 1.0f, 0.0f));
        Eigen::Projective3f transformation_matrix(projection_matrix * view_matrix * model_matrix);

        GL_CALL(glUniformMatrix4fv, matrix_id, 1, GL_FALSE, transformation_matrix.data());

        glEnableVertexAttribArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer);
        glVertexAttribPointer(
                0,              // attrib 0. no particular reason for attrib 0 but most match the value in the shader
                3,              // size
                GL_FLOAT,       // type
                GL_FALSE,       // normalized?
                0,              // stride
                (void*)0);      // array buffer offset

        glDrawArrays(GL_TRIANGLES, 0, 3); //// starting from vertex 0; 3 vertices total -> 1 triangle
        glDisableVertexAttribArray(0);

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
