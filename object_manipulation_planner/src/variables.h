#ifndef OBJECT_MANIPULATION_PLANNER_VARIABLES_H
#define OBJECT_MANIPULATION_PLANNER_VARIABLES_H

// FUCK! This order changes depending on which joint group is used for IK.
#define OMP_JOINT_GROUP_TORSO 0
#if OMP_JOINT_GROUP_TORSO

enum VariableIndex
{
    EE_PX           = 0,
    EE_PY           = 1,
    EE_PZ           = 2,
    EE_QX           = 3,
    EE_QY           = 4,
    EE_QZ           = 5,
    TR_JP           = 6,
    AR_FA           = 7,
    BD_TH           = 8,
    BD_PX           = 9,
    BD_PY           = 10,
    BD_PZ           = 11,
    OB_P            = 12,
    VARIABLE_COUNT  = 13
};

// ok, these are actually fine
enum RobotVariableIndex
{
    WORLD_JOINT_X       = 0,
    WORLD_JOINT_Y       = 1,
    WORLD_JOINT_THETA   = 2,
    TORSO_JOINT1        = 3,
    LIMB_JOINT1         = 4,
    LIMB_JOINT2         = 5,
    LIMB_JOINT3         = 6,
    LIMB_JOINT4         = 7,
    LIMB_JOINT5         = 8,
    LIMB_JOINT6         = 9,
    LIMB_JOINT7         = 10,
    WORLD_JOINT_Z       = 11,
    HINGE               = 12,
};

#else

enum VariableIndex
{
    EE_PX           = 0,
    EE_PY           = 1,
    EE_PZ           = 2,
    EE_QX           = 3,
    EE_QY           = 4,
    EE_QZ           = 5,
    AR_FA           = 6,
    BD_PZ           = 7,
    TR_JP           = 8,
    BD_TH           = 9,
    BD_PX           = 10,
    BD_PY           = 11,
    OB_P            = 12,
    VARIABLE_COUNT  = 13
};

enum RobotVariableIndex
{
    WORLD_JOINT_X       = 0,
    WORLD_JOINT_Y       = 1,
    WORLD_JOINT_THETA   = 2,
    WORLD_JOINT_Z       = 3,
    TORSO_JOINT1        = 4,
    LIMB_JOINT1         = 5,
    LIMB_JOINT2         = 6,
    LIMB_JOINT3         = 7,
    LIMB_JOINT4         = 8,
    LIMB_JOINT5         = 9,
    LIMB_JOINT6         = 10,
    LIMB_JOINT7         = 11,
    HINGE               = 12,
};

#endif

#endif

