#ifndef MoveArmCommandPanel_h
#define MoveArmCommandPanel_h

// standard includes
#include <map>
#include <memory>
#include <string>

// system includes
#include <QtGui>
#include <ros/ros.h>
#include <rviz/panel.h>

class MoveArmCommandModel;

class MoveArmCommandPanel : public rviz::Panel
{
    Q_OBJECT

public:

    MoveArmCommandPanel(QWidget* parent = 0);
    ~MoveArmCommandPanel();

    virtual void load(const rviz::Config& config);
    virtual void save(rviz::Config config) const;

public Q_SLOTS:

    /// \brief Load the robot with the parameter in the Robot Description line
    ///     edit box
    void loadRobot();

    /// \brief Update the GUI and visualizations
    void updateRobot();

    void syncRobot();

    /// \brief Update the robot visualization to reflect the current state of
    ///     the robot
    void updateRobotVisualization();

    void setJointVariableFromSpinBox(double value);

private:

    ros::NodeHandle m_nh;

    std::unique_ptr<MoveArmCommandModel> m_model;

    QLineEdit* m_robot_description_line_edit;
    QPushButton* m_load_robot_button;

    QComboBox* m_joint_groups_combo_box;
    QGroupBox* m_arm_commands_group;

    ros::Publisher m_marker_pub;

    // mapping from each qdoublespinbox to the index of the joint variable it
    // controls
    std::map<QDoubleSpinBox*, int> m_spinbox_to_vind;
    std::vector<QDoubleSpinBox*> m_vind_to_spinbox;

    std::string m_jmgoo;

    /// \brief Setup the baseline GUI for loading robots from URDF parameter
    void setupGUI();

    void setupRobotGUI();
    void syncSpinBoxes();

    bool isVariableAngle(int vind) const;
};

#endif
