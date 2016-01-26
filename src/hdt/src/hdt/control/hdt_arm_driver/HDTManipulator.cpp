// BEGIN_QNA_COPYRIGHT
// END_QNA_COPYRIGHT
// BEGIN_QNA_FILE_INFO

// HDTManipulatorTest.cpp: <file description>
// Created Oct 2, 2012 by mcsencsits
// Updated Oct 31, 2012 by mcsencsits

// END_QNA_FILE_INFO

#include "HDTManipulator.h"

#include <unistd.h>

static std::string param_file_name("HDTManipulatorParameters.txt");

/////////////////////////////////////////////////////////////////////
//                   Joystick Mappings                             //
/////////////////////////////////////////////////////////////////////
enum Buttons {
    BTN_QUIT = 0,
    BTN_INC_JOINT,
    BTN_DEC_JOINT,
    BTN_RESET_TARGET_POS_VEL,
    BTN_RESET_TARGET_TOR_CUR,
    BTN_SEND_MOTION_CMD,
    BTN_ACK_RESET,
    BTN_SEND_IMPEDANCE_CMD,
    BTN_IMPEDANCE_OFF,
    BTN_RESET_IMPEDANCE_PARAMS,
    BTN_COUNT
};

enum Axes {
    AX_POSITION = 0, AX_VELOCITY, AX_TORQUE, AX_MOTOR_CURRENT, AX_INERTIA, AX_DAMPING, AX_STIFFNESS, AX_COUNT
};

/// "Logitech Logitech Dual Action" mappings
unsigned int btn_map_logitech_dual_action[BTN_COUNT] = {
    8, // BTN_QUIT
    0, // BTN_INC_JOINT
    1, // BTN_DEC_JOINT
    3, // BTN_RESET_TARGET_POS_VEL
    2, // BTN_RESET_TARGET_TOR_CUR
    5, // BTN_SEND_MOTION_CMD
    9, // BTN_ACK_RESET
    5, // BTN_SEND_IMPEDANCE_CMD
    4, // BTN_IMPEDANCE_OFF
    3 }; // BTN_RESET_IMPEDANCE_PARAMS

std::string btn_labels_logitech_dual_action[12] = { "1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "L JS", "R JS" };

unsigned int ax_map_logitech_dual_action[AX_COUNT] = {
    1, // AX_POSITION
    0, // AX_VELOCITY
    3, // AX_TORQUE
    2, // AX_MOTOR_CURRENT
    1, // AX_INERTIA
    0, // AX_DAMPING
    2 }; // AX_STIFFNESS

std::string ax_labels_logitech_dual_action[6] = { "L Horz.", "L Vert.", "R Horz.", "R Vert.", "D Horz.", "D Vert." };

// "Xbox 360 Wireless Receiver" mappings (d-pad is composed of buttons)
std::string btn_labels_xbox_360_wireless[15] = {
    "A",
    "B",
    "X",
    "Y",
    "LB",
    "RB",
    "Start",
    "Guide",
    "L JS",
    "R JS", "Up",
    "Down",
    "Left",
    "Right",
    "Back" };

unsigned int btn_map_xbox_360_wireless[BTN_COUNT] = {
    14, // BTN_QUIT
    2, // BTN_INC_JOINT
    0, // BTN_DEC_JOINT
    3, // BTN_RESET_TARGET_POS_VEL
    1, // BTN_RESET_TARGET_TOR_CUR
    5, // BTN_SEND_MOTION_CMD
    6, // BTN_ACK_RESET
    5, // BTN_SEND_IMPEDANCE_CMD
    4, // BTN_IMPEDANCE_OFF
    3 }; // BTN_RESET_IMPEDANCE_PARAMS

std::string ax_labels_xbox_360_wireless[6] = { "L Horz.", "L Vert.", "LT", "R Horz.", "R Vert.", "RT" };

unsigned int ax_map_xbox_360_wireless[AX_COUNT] = {
    1, // AX_POSITION
    0, // AX_VELOCITY
    4, // AX_TORQUE
    3, // AX_MOTOR_CURRENT
    1, // AX_INERTIA
    0, // AX_DAMPING
    3 }; // AX_STIFFNESS

// "Microsoft X-Box 360 pad" mappings (d-pad is composed of axes)
std::string btn_labels_xbox_360_pad[11] = { "A", "B", "X", "Y", "LB", "RB", "Start", "Guide", "L JS", "R JS", "Back" };

unsigned int btn_map_xbox_360_pad[BTN_COUNT] = {
    10, // BTN_QUIT
    2, // BTN_INC_JOINT
    0, // BTN_DEC_JOINT
    3, // BTN_RESET_TARGET_POS_VEL
    1, // BTN_RESET_TARGET_TOR_CUR
    5, // BTN_SEND_MOTION_CMD
    6, // BTN_ACK_RESET
    5, // BTN_SEND_IMPEDANCE_CMD
    4, // BTN_IMPEDANCE_OFF
    3 }; // BTN_RESET_IMPEDANCE_PARAMS

std::string ax_labels_xbox_360_pad[8] = { "L Horz.", "L Vert.", "LT", "R Horz.", "R Vert.", "RT", "D Horz.", "D Vert." };

unsigned int ax_map_xbox_360_pad[AX_COUNT] = {
    1, // AX_POSITION
    0, // AX_VELOCITY
    4, // AX_TORQUE
    3, // AX_MOTOR_CURRENT
    1, // AX_INERTIA
    0, // AX_DAMPING
    3 }; // AX_STIFFNESS

/////////////////////////////////////////////////////////////////////
//                  Misc. Helper Functions                         //
/////////////////////////////////////////////////////////////////////

double degree_from_radian(double rad)
{
    static const double ratio = 180.0 / M_PI;
    return ratio * rad;
}

double norm_degree_from_radian(double rad)
{
    static const double ratio = 180.0 / M_PI;
    if (fabs(rad) > (M_PI))
        return ratio * atan2(sin(rad), cos(rad));
    else
        return ratio * rad;
}

static double get_current_time()
{
    typedef std::chrono::high_resolution_clock clk;
    static clk::time_point time;
    static clk::duration since_epoch;
    time = clk::now();
    since_epoch = time.time_since_epoch();

    return ((double) (since_epoch.count() * clk::period::num)) / clk::period::den;
}

void fill_line(char ch)
{
    int n_rows, n_cols, c_row, c_col;
    getmaxyx(stdscr, n_rows, n_cols);
    getyx(stdscr, c_row, c_col);
    while (c_col < n_cols) {
        addch(ch);
        c_col++;
    }
    return;
}

///////////////////////////////////////////////////////////////////
//                    Test Application                           //
///////////////////////////////////////////////////////////////////
HDTManipulatorTestApp::HDTManipulatorTestApp() :
        switched_context(true),
        prev_context(0),
        curr_context(0),
        prev_joint_ix(0),
        curr_joint_ix(0),
        n_rows(0),
        n_cols(0),
        err(ManipulatorError::NO_ERROR()),
        num_joints(0),
        joy_db(0.0),
        btn_map(NULL),
        ax_map(NULL),
        btn_label(NULL),
        ax_label(NULL),
        max_btn_label(0),
        max_ax_label(0),
        run_flag(false),
        initialized_flag(false),
        initial_angles_flag(false),
        first_run_flag(true)
{
}

HDTManipulatorTestApp::~HDTManipulatorTestApp()
{
    //t.join();
}

bool HDTManipulatorTestApp::SetPositionOffsets(std::vector<double> new_position_offsets)
{
    if (num_joints == new_position_offsets.size()) {
        printf("Setting position offsets\n");
        for (int i = 0; i < num_joints; i++) {
            position_offset[i] = new_position_offsets[i];
        }
        return true;
    }
    else {
        printf("Error: new_position_offsets size (%zd) is different from num_joints (%u)\n",
            new_position_offsets.size(), num_joints);
        return false;
    }
}

bool HDTManipulatorTestApp::SetTargetPosition(std::vector<double> initial_joint_angles)
{
    if (num_joints == initial_joint_angles.size()) {
        printf("Setting initial positions\n");
        for (int i = 0; i < num_joints; i++) {
            target_position[i] = initial_joint_angles[i];
        }
        initial_angles_flag = true;
        switched_context = true;
        return true;
    }
    else {
        printf("Error: initial_joint_angles size (%zd) is different from num_joints (%u)\n",
            initial_joint_angles.size(), num_joints);
        return false;
    }
}

bool HDTManipulatorTestApp::SetTargetPositionDegrees(std::vector<double> initial_joint_angles_degrees)
{
    if (num_joints == initial_joint_angles_degrees.size()) {
        printf("Setting initial positions in degrees\n");
        for (int i = 0; i < num_joints; i++) {
            target_position[i] = initial_joint_angles_degrees[i] * M_PI / 180.0;
        }
        initial_angles_flag = true;
        switched_context = true;
        return true;
    }
    else {
        printf("Error: initial_joint_angles size (%zd) is different from num_joints (%u)\n",
            initial_joint_angles_degrees.size(), num_joints);
        return false;
    }
}

bool HDTManipulatorTestApp::GetJointState(
    std::vector<double> & positions,
    std::vector<double> & velocities,
    std::vector<double> & torques,
    std::vector<double> & currents)
{
    if (initialized_flag) {
        positions.clear();
        velocities.clear();
        torques.clear();
        currents.clear();
        for (int i = 0; i < current_position.size(); i++) {
            positions.push_back(current_position[i]);
            velocities.push_back(current_velocity[i]);
            torques.push_back(current_torque[i]);
            currents.push_back(motor_current[i]);
        }

        /*
         manip.getPosition(current_position);
         manip.getVelocity(current_velocity);
         manip.getTorque(current_torque);
         manip.getMotorCurrent(motor_current);
         */
        return true;
    }
    else {
        printf("requesting positions of uninitialized manipulator");
        return false;
    }

}

bool HDTManipulatorTestApp::Initialize()
{
    printf("Initializing Joystick...");
    initialized_flag = InitializeJoy();
    printf("Done\n");

    if (initialized_flag) {
        printf("Initializing Manipulator...\n");

        printf("Hack to reinitialize CAN\n");
        printf("rmmod esd_usb2\n");
        system("rmmod esd_usb2");
        usleep(500000);
        printf("modprobe esd_usb2\n");
        system("modprobe esd_usb2");

        //initialized_flag = (/*InitializeJoy() &&*/ InitializeManip() && InitializeContexts() /*&& InitializeScreen()*/);
        initialized_flag = initialized_flag && InitializeManip();
        printf("Done\n");
    }

    if (initialized_flag) {

        printf("Initializing Contexts...");
        initialized_flag = initialized_flag && InitializeContexts();
        printf("Done\n");
    }

    usleep(1000000);

    if (initialized_flag) {

        printf("Initializing Screen...");
        initialized_flag = initialized_flag && InitializeScreen();
        printf("Done\n");
    }

    if (initialized_flag) {
        unsigned int num_joints = params.getNumJoints();

        // Initialize member vectors
        current_position.resize(num_joints, 0.0);
        current_position_raw.resize(num_joints, 0.0);
        current_velocity.resize(num_joints, 0.0);
        current_torque.resize(num_joints, 0.0);
        motor_current.resize(num_joints, 0.0);

        target_position.resize(num_joints, 0.0);
        position_offset.resize(num_joints, 0.0);

        target_velocity.resize(num_joints, 0.0);
        target_torque.resize(num_joints, 0.0);
        motor_current_limit.resize(num_joints, 10.0);

        inertia.resize(num_joints, 0.0);
        damping.resize(num_joints, 0.0);
        stiffness.resize(num_joints, 0.0);

        temp_fault.resize(num_joints, false);
        volt_fault.resize(num_joints, false);
        current_fault.resize(num_joints, false);
        comms_fault.resize(num_joints, false);
        misc_fault.resize(num_joints, false);

        sw_state.resize(num_joints, RAD::SW_INIT);
        bit_status.resize(num_joints, RAD::BIT_NO_ERROR);
    }

    return initialized_flag;
}

void HDTManipulatorTestApp::Shutdown()
{
    run_flag = false;
    usleep(500000);
    t.join();
}

bool HDTManipulatorTestApp::InitializeJoy()
{
    if (joy.Initialize(0)) {
        //joy.getName(joy_name);
        joy_name = joy.Name();
        printf("\n%s detected", joy_name.c_str());

        if (joy_name.compare("Logitech Logitech Dual Action") == 0) {
            btn_map = btn_map_logitech_dual_action;
            btn_label = btn_labels_logitech_dual_action;
            ax_map = ax_map_logitech_dual_action;
            ax_label = ax_labels_logitech_dual_action;
        }
        else if (joy_name.compare("Xbox 360 Wireless Receiver") == 0) {
            btn_map = btn_map_xbox_360_wireless;
            btn_label = btn_labels_xbox_360_wireless;
            ax_map = ax_map_xbox_360_wireless;
            ax_label = ax_labels_xbox_360_wireless;
            joy_db = 0.19;
        }
        else if (joy_name.compare("Microsoft X-Box 360 pad") == 0) {
            btn_map = btn_map_xbox_360_pad;
            btn_label = btn_labels_xbox_360_pad;
            ax_map = ax_map_xbox_360_pad;
            ax_label = ax_labels_xbox_360_pad;
            joy_db = 0.19;
        }
        else if (joy_name.find("Xbox") != std::string::npos) {
            btn_map = btn_map_xbox_360_wireless;
            btn_label = btn_labels_xbox_360_wireless;
            ax_map = ax_map_xbox_360_wireless;
            ax_label = ax_labels_xbox_360_wireless;
            joy_db = 0.19;
        }
        else if (joy_name.find("X-Box") != std::string::npos) {
            btn_map = btn_map_xbox_360_wireless;
            btn_label = btn_labels_xbox_360_wireless;
            ax_map = ax_map_xbox_360_wireless;
            ax_label = ax_labels_xbox_360_wireless;
            joy_db = 0.19;
        }
        else {
            printf("\nNo mapping available for joy stick: %s\n", joy_name.c_str());
            return false;
        }
    }
    else {
        // if no joystick let user know they can only view data
        printf("\nNo joy stick detected.");
        joy_name.assign("None detected");
        return false;
    }

    // calculate the max character count for button labels and axis labels
    for (int ix = 0; ix < BTN_COUNT; ix++)
        if (btn_label[btn_map[ix]].size() > max_btn_label)
            max_btn_label = btn_label[btn_map[ix]].size();
    for (int ix = 0; ix < AX_COUNT; ix++)
        if (ax_label[ax_map[ix]].size() > max_ax_label)
            max_ax_label = ax_label[ax_map[ix]].size();
    return true;
}

bool HDTManipulatorTestApp::InitializeManip()
{
    err = params.initialize(param_file_name);

    if (ManipulatorError::NO_ERROR() != err) {
        std::cout << std::endl << err << std::endl;
        return false;
    }

    err = manip.initialize(params);

    if (ManipulatorError::NO_ERROR() != err) {
        std::cout << std::endl << err << std::endl;
        return false;
    }

    num_joints = manip.getNumJoints();
    return true;
}

bool HDTManipulatorTestApp::InitializeContexts()
{
    context_option.clear();
    context_name.clear();
    //context_option.push_back(PRESET_CONTROL);       context_name.push_back("Preset Control");
    //context_option.push_back(EFFORT_CONTROL);       context_name.push_back("Joint Effort Control");
    //context_option.push_back(MULTI_EFFORT_CONTROL); context_name.push_back("Multi-Effort Control");
    context_option.push_back(POSITION_CONTROL);
    context_name.push_back("Joint Position Control");
    //options.push_back(EE_CONTROL); names.push_back("End Effector Control");
    return true;
}

bool HDTManipulatorTestApp::InitializeScreen()
{
    initscr();
    cbreak();
    noecho();
    start_color();
    init_pair(NORMAL_TEXT, COLOR_WHITE, COLOR_BLACK);   // normal display
    init_pair(HIGHLIGHT_TEXT, COLOR_BLUE, COLOR_WHITE);   // active joint display
    init_pair(GO_TEXT, COLOR_BLACK, COLOR_GREEN);       // status good/on
    init_pair(NO_GO_TEXT, COLOR_BLACK, COLOR_RED);      // status bad/off
    init_pair(GO_DISPLAY, COLOR_GREEN, COLOR_BLACK);    // status good/on
    init_pair(NO_GO_DISPLAY, COLOR_RED, COLOR_BLACK);   // status bad/off
    init_pair(WARNING_TEXT, COLOR_YELLOW, COLOR_BLACK);
    timeout(50); // getch() will wait for 50 millesconds or until a key is pressed
    keypad(stdscr, TRUE);
    //clearok(stdscr, TRUE);
    getmaxyx(stdscr, n_rows, n_cols);
    return true;
}

void HDTManipulatorTestApp::Run()
{
    if (initialized_flag) {
        run_flag = true;
        //std::thread t(&HDTManipulatorTestApp::ControlLoop, this);
        printf("Before ControlLoop\n");
        fflush(stdout);
        t = std::thread(&HDTManipulatorTestApp::ControlLoop, this);
        printf("After ControlLoop\n");
        fflush(stdout);

    }
}

void HDTManipulatorTestApp::ControlLoop()
{
    printf("Starting ControlLoop\n");

    while (run_flag) {
        Update();
        usleep(50000);
    }

    printf("Ending ControlLoop\n");
    endwin(); // end ncurses screen

}

//                                           Usage
// Axis                              Axis                              Axis                              |
// LS Horiz. - Target Position       RS Horiz. - Target Torque         LT - ???
//  LS Vert. - Target Velocity        RS Vert. - Current Limit         RT - ???
//
// Button                            Button                            Button
// RB - Send Command                  X - Increment Joint               Y - ???
// LB - Reset params                  A - Decrement Joint               Z - ???
//
//
//
void HDTManipulatorTestApp::DrawInstructions()
{
    int curr_row, curr_col;
    unsigned int display_width = 103;
    //unsigned int max_ax_instruction = 0;
    //unsigned int max_btn_instruction = 0;

    //for (int ix=0; ix < joy_ax_instruction.size(); ix++) if (joy_ax_instruction[ix].size() > max_ax_instruction) max_ax_instruction = joy_ax_instruction[ix].size();
    //for (int ix=0; ix < joy_btn_instruction.size(); ix++) if (joy_btn_instruction[ix].size() > max_btn_instruction) max_btn_instruction = joy_btn_instruction[ix].size();

    //display_width = 2 * (max_ax_label + 3 + max_ax_instruction) + 2;

    attrset(COLOR_PAIR(HIGHLIGHT_TEXT));
    mvprintw(n_rows - (4 + 12), 0, "                                                   Usage                                               ");
    //fill_line(' ');
    printw("\n");

    attrset(COLOR_PAIR(NORMAL_TEXT) | A_REVERSE); // Black text on white background for header
    attron(A_UNDERLINE);
    printw("Axis                              Axis                              Axis                               ");

    attrset(COLOR_PAIR(NORMAL_TEXT));
    getyx(stdscr, curr_row, curr_col);
    unsigned int row = curr_row + 1;
    for (int ix = 0; ix < joy_ax_used.size(); ix++) {
        unsigned int col = ix % 3;
        mvprintw(row, (34 * col) + (max_ax_label - ax_label[ax_map[joy_ax_used[ix]]].size()), "%s - %s", ax_label[ax_map[joy_ax_used[ix]]].c_str(), joy_ax_instruction[ix].c_str());
        if (col == 2)
            row++;
    }

    printw("\n");
    attrset(COLOR_PAIR(NORMAL_TEXT) | A_REVERSE); // Black text on white background for header
    attron(A_UNDERLINE);
    printw("\nButton                            Button                            Button                             ");

    attrset(COLOR_PAIR(NORMAL_TEXT));
    getyx(stdscr, curr_row, curr_col);
    row = curr_row + 1;
    for (int ix = 0; ix < joy_btn_used.size(); ix++) {
        unsigned int col = ix % 3;
        mvprintw(row, (34 * col), "%4s  - %s", btn_label[btn_map[joy_btn_used[ix]]].c_str(), joy_btn_instruction[ix].c_str());
        if (col == 2)
            row++;
    }
    move(row, 0);
}

void HDTManipulatorTestApp::Update()
{
    static short ch;

    ////////////////////////////
    // Update joystick state  //
    ////////////////////////////

    if (joy.NumAxes() > 0)
        joy.Update();

    ///////////////////////////////////////////////////////////////////////
    // check joy buttons and keyboard for desired context switch or quit //
    ///////////////////////////////////////////////////////////////////////

    ch = getch(); // there is a 50 ms timeout on this call set by timeout(), so this is a ~20 hz app
    if (joy.PressedButton(btn_map[BTN_QUIT]) || 'q' == ch || 'Q' == ch)
        run_flag = false;
    prev_context = curr_context;
    if ('1' <= ch && ch <= '4' && ((int) (ch - '1')) < context_option.size())
        curr_context = context_option[(int) (ch - '1')];

    //////////////////////////////////////////////////////////////////////////////
    // Use buttons labeled 1 & 2  and up/down arrow keys to select active joint //
    //////////////////////////////////////////////////////////////////////////////

    if (!joy.Button(btn_map[BTN_SEND_MOTION_CMD]) && (joy.PressedButton(btn_map[BTN_INC_JOINT]) || KEY_UP == ch))
        curr_joint_ix = (curr_joint_ix + 1) % num_joints;

    if (!joy.Button(btn_map[BTN_SEND_MOTION_CMD]) && (joy.PressedButton(btn_map[BTN_DEC_JOINT]) || KEY_DOWN == ch))
        curr_joint_ix = (curr_joint_ix + (num_joints - 1)) % num_joints;

    /////////////////////////////
    // Update manipulator data //
    /////////////////////////////

    manip.getVelocity(current_velocity);

    manip.getPosition(current_position_raw);
    for (size_t i = 0; i < current_position.size(); i++) {
        if (first_run_flag) {
            current_position[i] = current_position_raw[i] + position_offset[i];
        }
        else {
            double alpha = 0.75;
            if (fabs(current_velocity[i]) < 0.01) {
                alpha = 0.95;
            }
            current_position[i] = (current_position[i] - position_offset[i]) * alpha
                + current_position_raw[i] * (1 - alpha) + position_offset[i];
        }
    }

    manip.getTorque(current_torque);

    manip.getMotorCurrent(motor_current);

    manip.getSoftwareState(sw_state);
    manip.getBITStatus(bit_status);

    manip.getMotorTempFaultStatus(temp_fault);
    manip.getMotorVoltFaultStatus(volt_fault);
    manip.getMotorCurrentFaultStatus(current_fault);
    manip.getCommsFaultStatus(comms_fault);
    manip.getMiscFaultStatus(misc_fault);

    if (first_run_flag) {
        first_run_flag = false;
    }

    ////////////////////////////
    //   Run current context  //
    ////////////////////////////
    PositionControlContext();
    if (switched_context) {
        printf("Erasing previous screen\n");
        clear();
        InitializeScreen();
        joy_btn_used.push_back(BTN_INC_JOINT);
        joy_btn_used.push_back(BTN_DEC_JOINT);
        joy_btn_used.push_back(BTN_QUIT);

        joy_btn_instruction.push_back("Inc joint");
        joy_btn_instruction.push_back("Dec joint");
        joy_btn_instruction.push_back("Quit app");
    }

    switched_context = false;

    //////////////////////////////
    // Generate Usage Display   //
    //////////////////////////////

    DrawInstructions();

    //////////////////////////////
    //  Generate Common Display //
    //////////////////////////////

    if (manip.haveCommunication())
        attrset(COLOR_PAIR(GO_TEXT));
    else
        attrset(COLOR_PAIR(NO_GO_TEXT));
    mvprintw(n_rows - 4, 0, "Comms Status");

    if (manip.haveSupply())
        attrset(COLOR_PAIR(GO_TEXT));
    else
        attrset(COLOR_PAIR(NO_GO_TEXT));
    mvprintw(n_rows - 4, 15, "Power Status");

    if (manip.isControllable())
        attrset(COLOR_PAIR(GO_TEXT));
    else
        attrset(COLOR_PAIR(NO_GO_TEXT));
    mvprintw(n_rows - 4, 30, "Control Status");

    if (manip.resetOccurred())
        attrset(COLOR_PAIR(NO_GO_TEXT));
    else
        attrset(COLOR_PAIR(GO_TEXT));
    mvprintw(n_rows - 4, 45, "RESET");

    if (joy.NumAxes() > 0)
        joy.ClearButtons();
}

void HDTManipulatorTestApp::ImpedanceControlContext()
{
}

// Print out main joint data
// select joint using joystick buttons
// position commands to interface only
void HDTManipulatorTestApp::PositionControlContext()
{
    static int ix;
    static std::vector<float> zero(params.getNumJoints(), 0.0);

    if (switched_context) {
        for (ix = params.getNumJoints() - 1; ix >= 0; ix--) {
            if (initial_angles_flag) {
                target_velocity[ix] = 45 * M_PI / 180.0;
            }
            else {
                target_position[ix] = current_position[ix];
                target_velocity[ix] = 20 * M_PI / 180.0;
            }

        }
        erase(); // clear previous screen

        joy_btn_used.push_back(BTN_SEND_MOTION_CMD);
        joy_btn_instruction.push_back("Send motion command");

        joy_btn_used.push_back(BTN_ACK_RESET);
        joy_btn_instruction.push_back("Acknowledge Reset");

        joy_ax_used.push_back(AX_POSITION);
        joy_ax_instruction.push_back("Target Position");

        joy_ax_used.push_back(AX_VELOCITY);
        joy_ax_instruction.push_back("Target Velocity");

        joy_ax_used.push_back(AX_TORQUE);
        joy_ax_instruction.push_back("Target Torque");

        joy_ax_used.push_back(AX_MOTOR_CURRENT);
        joy_ax_instruction.push_back("Current Limit");
    }

    /////////////////
    // print title //
    /////////////////
    attrset(COLOR_PAIR(NORMAL_TEXT) | A_REVERSE);// Black text on white background for header
    mvprintw(0, 0, "Joint Position Control                                                                                  FAULT STS");
    fill_line(' ');
    mvprintw(1, 0, "                                                                                                        T V C C M");
    fill_line(' ');
    mvprintw(2, 0, "                                                                                                        E O R O I");
    fill_line(' ');
    mvprintw(3, 0, "      |      Position     |      Velocity     |       Torque      |    Motor Current  |  SW   |  BIT  | M L N M S");
    fill_line(' ');
    attron(A_UNDERLINE);
    mvprintw(4, 0, "Joint |  Target | Current |  Target | Current |  Limit  | Current |  Limit  | Current | State | Status| P T T M C");
    fill_line(' ');

    ////////////////////////////
    // process joystick input //
    ////////////////////////////

    target_position[curr_joint_ix] -= joy.Axis(ax_map[AX_POSITION], joy_db) * 0.523598776 / 20.0; // Allow target position to be changed at a rate of 30 deg/sec
    target_velocity[curr_joint_ix] -= joy.Axis(ax_map[AX_VELOCITY], joy_db) * 0.1;
    if (target_velocity[curr_joint_ix] < 0.0)
        target_velocity[curr_joint_ix] = 0.0;

    target_torque[curr_joint_ix] -= joy.Axis(ax_map[AX_TORQUE], joy_db) * 0.01;
    if (target_torque[curr_joint_ix] < 0.0)
        target_torque[curr_joint_ix] = 0.0;

    motor_current_limit[curr_joint_ix] -= joy.Axis(ax_map[AX_MOTOR_CURRENT], joy_db) * 0.01;
    if (motor_current_limit[curr_joint_ix] < 0.0)
        motor_current_limit[curr_joint_ix] = 0.0;

    attrset(COLOR_PAIR(NORMAL_TEXT));
    if (joy.Button(btn_map[BTN_SEND_MOTION_CMD])) {
        std::vector<float> target_position_offset;
        for (size_t i = 0; i < target_position.size(); i++) {
            target_position_offset.push_back(target_position[i] - position_offset[i]);
        }

        //err = manip.setTargetPositionVelocity(target_position, target_velocity);
        err = manip.setTargetPositionVelocity(target_position_offset, target_velocity);
        manip.setTargetTorque(target_torque);
        manip.setMotorCurrentLimit(motor_current_limit);
        mvprintw(13, 0, "Pressing button 5!");
        fill_line(' ');
        printw("%s", err.toString().c_str());
        fill_line(' ');
    }
    else {
        err = manip.setTargetPositionVelocity(target_position, zero);
        mvprintw(13, 0, "");
        fill_line(' ');
        printw("%s", err.toString().c_str());
        fill_line(' ');
    }

    // TODO: check return val of setTargetPositionVelocity, change display of target position/velocity when
    // position/velocity limit is violated by command

    ////////////////
    // print info //
    mvprintw(5, 0, "");
    ////////////////
    for (ix = params.getNumJoints() - 1; ix >= 0; ix--) {
        if (ix == curr_joint_ix)
            attrset(COLOR_PAIR(HIGHLIGHT_TEXT));
        else
            attrset(COLOR_PAIR(NORMAL_TEXT));
        printw("   % 2d : %+ 8.2f  %+ 8.2f  %+ 8.2f  %+ 8.2f  %+ 8.2f  %+ 8.2f  %+ 8.2f  %+ 8.2f ", ix,
            norm_degree_from_radian(target_position[ix]), norm_degree_from_radian(current_position[ix]),
            degree_from_radian(target_velocity[ix]), degree_from_radian(current_velocity[ix]), target_torque[ix],
            current_torque[ix], motor_current_limit[ix], motor_current[ix]);
        switch (sw_state[ix]) {
        case RAD::SW_INIT:
            attrset(COLOR_PAIR(NORMAL_TEXT));
            printw("  INIT ");
            break;
        case RAD::SW_FS:
            attrset(COLOR_PAIR(NO_GO_TEXT));
            printw("  FS   ");
            break;
        case RAD::SW_NOS:
        case RAD::SW_NOS_IDLE:
        case RAD::SW_NOS_SLEEP:
            attrset(COLOR_PAIR(GO_TEXT));
            printw("  NOS  ");
            break;
        case RAD::SW_RESET:
            attrset(COLOR_PAIR(HIGHLIGHT_TEXT));
            printw(" RESET ");
            break;
        default:
            attrset(COLOR_PAIR(HIGHLIGHT_TEXT));
            printw(" OTHER ");
            break;
        }

        attrset(COLOR_PAIR(NORMAL_TEXT));
        printw(" ");

        switch (bit_status[ix]) {
        case RAD::BIT_NO_ERROR:
            attrset(COLOR_PAIR(GO_TEXT));
            printw(" NO ERR");
            break;
        case RAD::BIT_NON_CRITICAL_ERROR:
            attrset(COLOR_PAIR(WARNING_TEXT));
            printw(" WARN  ");
            break;
        case RAD::BIT_CRITICAL_ERROR:
            attrset(COLOR_PAIR(NO_GO_TEXT));
            printw(" ERROR ");
            break;
        }

        attrset(COLOR_PAIR(NORMAL_TEXT));
        printw(" ");

        (temp_fault[ix]) ? attrset(COLOR_PAIR(NO_GO_DISPLAY)) : attrset(COLOR_PAIR(GO_DISPLAY));
        addch(' ');
        addch(ACS_DIAMOND);

        (volt_fault[ix]) ? attrset(COLOR_PAIR(NO_GO_DISPLAY)) : attrset(COLOR_PAIR(GO_DISPLAY));
        addch(' ');
        addch(ACS_DIAMOND);

        (current_fault[ix]) ? attrset(COLOR_PAIR(NO_GO_DISPLAY)) : attrset(COLOR_PAIR(GO_DISPLAY));
        addch(' ');
        addch(ACS_DIAMOND);

        (comms_fault[ix]) ? attrset(COLOR_PAIR(NO_GO_DISPLAY)) : attrset(COLOR_PAIR(GO_DISPLAY));
        addch(' ');
        addch(ACS_DIAMOND);

        (misc_fault[ix]) ? attrset(COLOR_PAIR(NO_GO_DISPLAY)) : attrset(COLOR_PAIR(GO_DISPLAY));
        addch(' ');
        addch(ACS_DIAMOND);
        printw("\n");
    }

    if (joy.PressedButton(btn_map[BTN_ACK_RESET]))
        manip.acknowledgeReset();
    return;
}
