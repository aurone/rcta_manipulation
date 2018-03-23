#ifndef hdt_kinematics_h
#define hdt_kinematics_h

#include <Eigen/Dense>

#define IKFAST_HAS_LIBRARY
#include "hdt_arm_transform6d.h"

namespace hdt
{

////////////////////////////////////////////////////////////////////////////////
// Forward raw ikfast functionality
////////////////////////////////////////////////////////////////////////////////

bool ComputeIk(const IkReal* eetrans,
               const IkReal* eerot,
               const IkReal* pfree,
               ikfast::IkSolutionListBase<IkReal>& solutions);

void ComputeFk(const IkReal* joints, IkReal* eetrans, IkReal* eerot);

int GetNumFreeParameters();

int* GetFreeParameters();

int GetNumJoints();

int GetIkRealSize();

const char* GetIkFastVersion();

int GetIkType();

const char* GetKinematicsHash();

} // namespace hdt

#endif

