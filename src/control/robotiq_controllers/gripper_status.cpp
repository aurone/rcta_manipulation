#include "gripper_status.h"

const bool no_fault(FaultStatus status)
{
    return status == FaultStatus::NoFault;
}

const bool is_priority_fault(FaultStatus status)
{
    return status == FaultStatus::ActionDelayed || status == FaultStatus::ActivationBitNotSet;
}

const bool is_minor_fault(FaultStatus status)
{
    return status == FaultStatus::CommunicationNotReady || status == FaultStatus::AutomaticReleaseInProgress;
}

const bool is_major_fault(FaultStatus status)
{
    return status == FaultStatus::OvercurrentProtectionTriggered || status == FaultStatus::AutomaticReleaseCompleted;
}

std::string to_string(Status s)
{
    switch (s)
    {
    case Status::Invalid:
        return "Invalid";
    case Status::Reset:
        return "Reset";
    case Status::ActivationInProgress:
        return "ActivationInProgress";
    case Status::ActivationComplete:
        return "ActivationComplete";
    }
}

std::string to_string(ObjectStatus os)
{
    switch (os)
    {
    case ObjectStatus::Invalid:
        return "Invalid";
    case ObjectStatus::FingersInMotion:
        return "FingersInMotion";
    case ObjectStatus::FingersStoppedDueToContactWhileOpening:
        return "FingersStoppedDueToContactWhileOpening";
    case ObjectStatus::FingersStoppedDueToContactWhileClosing:
        return "FingersStoppedDueToContactWhileClosing";
    case ObjectStatus::FingersAtRequestedPosition:
        return "FingersAtRequestedPosition";
    }
}

std::string to_string(FaultStatus fs)
{
    switch (fs)
    {
    case FaultStatus::Invalid:
        return "Invalid";
    case FaultStatus::NoFault:
        return "NoFault";
    case FaultStatus::ActionDelayed:
        return "ActionDelayed";
    case FaultStatus::ActivationBitNotSet:
        return "ActivationBitNotSet";
    case FaultStatus::CommunicationNotReady:
        return "CommunicationNotReady";
    case FaultStatus::AutomaticReleaseInProgress:
        return "AutomaticReleaseInProgress";
    case FaultStatus::OvercurrentProtectionTriggered:
        return "OvercurrentProtectionTriggered";
    case FaultStatus::AutomaticReleaseCompleted:
        return "AutomaticReleaseCompleted";
    }
}

