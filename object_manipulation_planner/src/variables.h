#ifndef OBJECT_MANIPULATION_PLANNER_VARIABLES_H
#define OBJECT_MANIPULATION_PLANNER_VARIABLES_H

enum VariableIndex
{
    EE_PX = 0,
    EE_PY = 1,
    EE_PZ = 2,
    EE_QX = 3,
    EE_QY = 4,
    EE_QZ = 5,
    AR_FA = 6,
    TR_JP = 7,
    BD_TH = 8,
    BD_PX = 9,
    BD_PY = 10,
    OB_P = 11,
    VARIABLE_COUNT = 12
};

enum RobotVariableIndex
{
    WORLD_JOINT_X = 0,
    WORLD_JOINT_Y = 1,
    WORLD_JOINT_THETA = 2,
    TORSO_JOINT1 = 3,
    LIMB_JOINT1 = 4,
    LIMB_JOINT2 = 5,
    LIMB_JOINT3 = 6,
    LIMB_JOINT4 = 7,
    LIMB_JOINT5 = 8,
    LIMB_JOINT6 = 9,
    LIMB_JOINT7 = 10,
    HINGE = 11,
};

#endif

