#include <hdt_viz/hdt_kinematics/kinematics.h>

#include <array>
#include <iostream>

namespace hdt
{

bool ComputeIk(const IkReal* eetrans,
               const IkReal* eerot,
               const IkReal* pfree,
               ikfast::IkSolutionListBase<IkReal>& solutions)
{
    return ::ComputeIk(eetrans, eerot, pfree, solutions);
}

void ComputeFk(const IkReal* joints, IkReal* eetrans, IkReal* eerot)
{
    return ::ComputeFk(joints, eetrans, eerot);
}

int GetNumFreeParameters()
{
    return ::GetNumFreeParameters();
}

int* GetFreeParameters()
{
    return ::GetFreeParameters();
}

int GetNumJoints()
{
    return ::GetNumJoints();
}

int GetIkRealSize()
{
    return ::GetIkRealSize();
}

const char* GetIkFastVersion()
{
    return ::GetIkFastVersion();
}

int GetIkType()
{
    return ::GetIkType();
}

const char* GetKinematicsHash()
{
    return ::GetKinematicsHash();
}

} // namespace hdt

