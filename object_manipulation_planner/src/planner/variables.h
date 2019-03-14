#ifndef OBJECT_MANIPULATION_PLANNER_VARIABLES_H
#define OBJECT_MANIPULATION_PLANNER_VARIABLES_H

// standard includes
#include <utility>

enum VariableIndex
{
#if 0
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
#else
    EE_PX           = 0,
    EE_PY           = 1,
    EE_PZ           = 2,
    EE_QX           = 3,
    EE_QY           = 4,
    EE_QZ           = 5,
    BD_PX           = 6,
    BD_PY           = 7,
    BD_PZ           = 8,
    BD_QZ           = 9,
    BD_QY           = 10,
    BD_QX           = 11,
    TR_JP           = 12,
    AR_FA           = 13,
    OB_P            = 14,
    VARIABLE_COUNT
#endif
};

enum RobotVariableIndex
{
#if 0
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
#else
    WORLD_JOINT_X       = 0,
    WORLD_JOINT_Y       = 1,
    WORLD_JOINT_Z       = 2,
    WORLD_JOINT_YAW     = 3,
    WORLD_JOINT_PITCH   = 4,
    WORLD_JOINT_ROLL    = 5,
    TORSO_JOINT1        = 6,
    LIMB_JOINT1         = 7,
    LIMB_JOINT2         = 8,
    LIMB_JOINT3         = 9,
    LIMB_JOINT4         = 10,
    LIMB_JOINT5         = 11,
    LIMB_JOINT6         = 12,
    LIMB_JOINT7         = 13,
    HINGE               = 14,
#endif
};

enum PlanningModelVariable
{
    PMV_WORLD_JOINT_X = 0,
    PMV_WORLD_JOINT_Y,
    PMV_WORLD_JOINT_THETA,
    PMV_TORSO_JOINT1,
    PMV_LIMB_JOINT1,
    PMV_LIMB_JOINT2,
    PMV_LIMB_JOINT3,
    PMV_LIMB_JOINT4,
    PMV_LIMB_JOINT5,
    PMV_LIMB_JOINT6,
    PMV_LIMB_JOINT7,
    PMV_COUNT
};

constexpr std::pair<int, int> RobotToParentVariableMap[] =
{
    { WORLD_JOINT_X,        PMV_WORLD_JOINT_X       },
    { WORLD_JOINT_Y,        PMV_WORLD_JOINT_Y       },
    { WORLD_JOINT_Z,        -1                      },
    { WORLD_JOINT_YAW,      PMV_WORLD_JOINT_THETA   },
    { WORLD_JOINT_PITCH,    -1                      },
    { WORLD_JOINT_ROLL,     -1                      },
    { TORSO_JOINT1,         PMV_TORSO_JOINT1        },
    { LIMB_JOINT1,          PMV_LIMB_JOINT1         },
    { LIMB_JOINT2,          PMV_LIMB_JOINT2         },
    { LIMB_JOINT3,          PMV_LIMB_JOINT3         },
    { LIMB_JOINT4,          PMV_LIMB_JOINT4         },
    { LIMB_JOINT5,          PMV_LIMB_JOINT5         },
    { LIMB_JOINT6,          PMV_LIMB_JOINT6         },
    { LIMB_JOINT7,          PMV_LIMB_JOINT7         },
    { HINGE,                -1                      },
};

constexpr std::pair<int, int> RobotToParentVariablePairs[] =
{
    { WORLD_JOINT_X,    PMV_WORLD_JOINT_X       },
    { WORLD_JOINT_Y,    PMV_WORLD_JOINT_Y       },
    { WORLD_JOINT_YAW,  PMV_WORLD_JOINT_THETA   },
    { TORSO_JOINT1,     PMV_TORSO_JOINT1        },
    { LIMB_JOINT1,      PMV_LIMB_JOINT1         },
    { LIMB_JOINT2,      PMV_LIMB_JOINT2         },
    { LIMB_JOINT3,      PMV_LIMB_JOINT3         },
    { LIMB_JOINT4,      PMV_LIMB_JOINT4         },
    { LIMB_JOINT5,      PMV_LIMB_JOINT5         },
    { LIMB_JOINT6,      PMV_LIMB_JOINT6         },
    { LIMB_JOINT7,      PMV_LIMB_JOINT7         },
};

#endif

