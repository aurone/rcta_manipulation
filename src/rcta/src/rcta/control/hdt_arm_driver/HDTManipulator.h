// BEGIN_QNA_COPYRIGHT
// END_QNA_COPYRIGHT
// BEGIN_QNA_FILE_INFO

// HDTManipulatorTest.h: <file description>
// Created Oct 4, 2012 by mcsencsits
// Updated Oct 31, 2012 by mcsencsits

// END_QNA_FILE_INFO

#ifndef HDTMANIPULATORTEST_H_
#define HDTMANIPULATORTEST_H_

// standard includes
#include <cstdio>
#include <cstdlib>
#include <vector>
#include <string>
#include <thread>
#include <iostream>
#include <cmath>
#include <chrono>


// special includes
#include <ncurses.h>

// QNA library includes
#include "HDTManipulatorInterface.h"
#include "LinuxJoystick.h"

enum ColorPair{
   GO_TEXT = 1,
   NO_GO_TEXT,
   GO_DISPLAY,
   NO_GO_DISPLAY,
   NORMAL_TEXT,
   HIGHLIGHT_TEXT,
   WARNING_TEXT
};

enum DisplayContext{
   PRESET_CONTROL,
   EFFORT_CONTROL,
   MULTI_EFFORT_CONTROL,
   POSITION_CONTROL,
   EE_CONTROL
};


class HDTManipulatorTestApp{
public:
   HDTManipulatorTestApp();
   ~HDTManipulatorTestApp();

   void Run();
   bool Initialize();
   void Shutdown();
   bool SetTargetPositionDegrees(std::vector <double> initial_joint_angles_degrees);
   bool SetTargetPosition(std::vector <double> initial_joint_angles);
   bool SetPositionOffsets(std::vector <double> new_position_offsets);
   bool GetJointState(std::vector <double> & positions,
                                             std::vector <double> & velocities,
                                             std::vector <double> & torques,
                                             std::vector <double> & currents);
   bool Running() { return run_flag; };



protected:
   void Update();
   void ControlLoop();

   void PositionControlContext();
   void ImpedanceControlContext();



   bool InitializeJoy();
   bool InitializeManip();
   bool InitializeContexts();
   bool InitializeScreen();

   void DrawInstructions();

   bool switched_context;
   unsigned int prev_context;
   unsigned int curr_context;

   unsigned int prev_joint_ix;
   unsigned int curr_joint_ix;

   int n_rows;
   int n_cols;

   ManipulatorError err;
   ManipulatorParameters params;
   HDTManipulatorInterface manip;
   unsigned int num_joints;

   LinuxJoystick joy;
   float joy_db;
   std::string joy_name;
   unsigned int *btn_map;
   unsigned int *ax_map;
   std::string *btn_label;
   std::string *ax_label;
   unsigned int max_btn_label;
   unsigned int max_ax_label;

   std::vector<float> current_position;
   std::vector<float> current_position_raw;
   std::vector<float> position_offset;

   std::vector<float> current_velocity;
   std::vector<float> current_torque;
   std::vector<float> motor_current;

   std::vector<float> target_position;
   std::vector<float> target_velocity;
   std::vector<float> target_torque;
   std::vector<float> motor_current_limit;

   std::vector<float> inertia;
   std::vector<float> damping;
   std::vector<float> stiffness;

   std::vector<bool> temp_fault;
   std::vector<bool> volt_fault;
   std::vector<bool> current_fault;
   std::vector<bool> comms_fault;
   std::vector<bool> misc_fault;

   std::vector<RAD::SWState> sw_state;
   std::vector<RAD::BITStatus> bit_status;

   std::vector<DisplayContext> context_option;
   std::vector<std::string> context_name;

   std::vector<unsigned int> joy_ax_used;
   std::vector<std::string> joy_ax_instruction;
   std::vector<unsigned int> joy_btn_used;
   std::vector<std::string> joy_btn_instruction;

   bool run_flag;
   bool initialized_flag;
   bool initial_angles_flag;
   bool first_run_flag;
   std::thread t;
};

#endif /* HDTMANIPULATORTEST_H_ */
