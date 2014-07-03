#ifndef gripper_status_h
#define gripper_status_h

#include <string>

enum class Status
{
    Invalid,
    Reset,
    ActivationInProgress,
    ActivationComplete
};

enum class ObjectStatus
{
    Invalid,
    FingersInMotion,
    FingersStoppedDueToContactWhileOpening,
    FingersStoppedDueToContactWhileClosing,
    FingersAtRequestedPosition
};

enum class FaultStatus
{
    Invalid,
    NoFault,
    ActionDelayed,
    ActivationBitNotSet,
    CommunicationNotReady,
    AutomaticReleaseInProgress,
    OvercurrentProtectionTriggered,
    AutomaticReleaseCompleted
};

const bool no_fault(FaultStatus status);
const bool is_priority_fault(FaultStatus status);
const bool is_minor_fault(FaultStatus status);
const bool is_major_fault(FaultStatus status);

std::string to_string(Status s);
std::string to_string(ObjectStatus os);
std::string to_string(FaultStatus fs);

#endif
