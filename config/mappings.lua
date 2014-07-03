--------------------------------------------------------------------------------
-- DO NOT MODIFY THESE COMMAND LISTS UNLESS YOU CHANGE THE TELEOP NODE AS WELL
--------------------------------------------------------------------------------
ButtonCommand = 
{
    NoCommand                       = 0,
    Quit                            = 1,
    IncreaseCurrentJoint            = 2,
    DecreaseCurrentJoint            = 3,
    ResetTargetPositionAndVelocity  = 4,
    ResetTargetTorqueAndCurrent     = 5,
    SendMotionCommand               = 6,
    AcknowledgeReset                = 7,
    SendImpedanceCommand            = 8,
    SendImpedanceOffCommand         = 9,
    ResetImpedanceParameters        = 10,
    DeadManSwitchOn                 = 11,
    DeadManSwitchOff                = 12
}

AxisCommand =
{
    NoCommand           = 0,
    Position            = 1,
    Velocity            = 2,
    Torque              = 3,
    MotorCurrent        = 4,
    Inertia             = 5,
    Damping             = 6,
    Stiffness           = 7,
    DecreasePosition    = 8,
    IncreasePosition    = 9,
    IncreaseVelocity    = 10,
    DecreaseVelocity    = 11,
    IncreaseTorque      = 12,
    DecreaseTorque      = 13,
    IncreaseMotorCurrent = 14,
    DecreaseMotorCurrent = 15
}

NumButtonCommands = #ButtonCommand
NumAxisCommands = #AxisCommand

--------------------------------------------------------------------------------
-- Freely modify mappings below this line
--------------------------------------------------------------------------------

function create_generic_xbox_mappings()
    return
    {
        Buttons =
        {
            [ "0"] = { Name = "A",                 OnPress = ButtonCommand.NoCommand,            OnRelease = ButtonCommand.NoCommand        },
            [ "1"] = { Name = "B",                 OnPress = ButtonCommand.NoCommand,            OnRelease = ButtonCommand.NoCommand        },
            [ "2"] = { Name = "X",                 OnPress = ButtonCommand.DeadManSwitchOn,      OnRelease = ButtonCommand.DeadManSwitchOff },
            [ "3"] = { Name = "Y",                 OnPress = ButtonCommand.NoCommand,            OnRelease = ButtonCommand.NoCommand        },
            [ "4"] = { Name = "LeftBumper",        OnPress = ButtonCommand.DecreaseCurrentJoint, OnRelease = ButtonCommand.NoCommand        },
            [ "5"] = { Name = "RightBumper",       OnPress = ButtonCommand.IncreaseCurrentJoint, OnRelease = ButtonCommand.NoCommand        },
            [ "6"] = { Name = "Back",              OnPress = ButtonCommand.AcknowledgeReset,     OnRelease = ButtonCommand.NoCommand        },
            [ "7"] = { Name = "Start",             OnPress = ButtonCommand.NoCommand,            OnRelease = ButtonCommand.NoCommand        },
            [ "8"] = { Name = "Xbox",              OnPress = ButtonCommand.Quit,                 OnRelease = ButtonCommand.NoCommand        },
            [ "9"] = { Name = "LeftThumbstick",    OnPress = ButtonCommand.NoCommand,            OnRelease = ButtonCommand.NoCommand        },
            ["10"] = { Name = "RightThumbstick",   OnPress = ButtonCommand.NoCommand,            OnRelease = ButtonCommand.NoCommand        },
            ["11"] = { Name = "DPadLeft",          OnPress = ButtonCommand.NoCommand,            OnRelease = ButtonCommand.NoCommand        },
            ["12"] = { Name = "DPadRight",         OnPress = ButtonCommand.NoCommand,            OnRelease = ButtonCommand.NoCommand        },
            ["13"] = { Name = "DPadUp",            OnPress = ButtonCommand.NoCommand,            OnRelease = ButtonCommand.NoCommand        },
            ["14"] = { Name = "DPadDown",          OnPress = ButtonCommand.NoCommand,            OnRelease = ButtonCommand.NoCommand        },
        },

        Axes = 
        {
            ["0"] = { Name = "LeftStickVertical",        OnValue = AxisCommand.Position         },
            ["1"] = { Name = "LeftStickHorizontal",      OnValue = AxisCommand.Velocity         },
            ["2"] = { Name = "RightStickVertical",       OnValue = AxisCommand.Torque           },
            ["3"] = { Name = "RightStickHorizontal",     OnValue = AxisCommand.MotorCurrent     },
            ["4"] = { Name = "LeftTrigger",              OnValue = AxisCommand.NoCommand        },
            ["5"] = { Name = "RightTrigger",             OnValue = AxisCommand.NoCommand        },
        }
    }
end

function create_xbox360_mappings()
    return
    {
        Buttons =
        {
            [ "0"] = { Name = "A",                 OnPress = ButtonCommand.NoCommand,            OnRelease = ButtonCommand.NoCommand        },
            [ "1"] = { Name = "B",                 OnPress = ButtonCommand.NoCommand,            OnRelease = ButtonCommand.NoCommand        },
            [ "2"] = { Name = "X",                 OnPress = ButtonCommand.DeadManSwitchOn,      OnRelease = ButtonCommand.DeadManSwitchOff },
            [ "3"] = { Name = "Y",                 OnPress = ButtonCommand.NoCommand,            OnRelease = ButtonCommand.NoCommand        },
            [ "4"] = { Name = "LeftBumper",        OnPress = ButtonCommand.DecreaseCurrentJoint, OnRelease = ButtonCommand.NoCommand        },
            [ "5"] = { Name = "RightBumper",       OnPress = ButtonCommand.IncreaseCurrentJoint, OnRelease = ButtonCommand.NoCommand        },
            [ "6"] = { Name = "Back",              OnPress = ButtonCommand.AcknowledgeReset,     OnRelease = ButtonCommand.NoCommand        },
            [ "7"] = { Name = "Start",             OnPress = ButtonCommand.NoCommand,            OnRelease = ButtonCommand.NoCommand        },
            [ "8"] = { Name = "Xbox",              OnPress = ButtonCommand.Quit,                 OnRelease = ButtonCommand.NoCommand        },
            [ "9"] = { Name = "LeftThumbstick",    OnPress = ButtonCommand.NoCommand,            OnRelease = ButtonCommand.NoCommand        },
            ["10"] = { Name = "RightThumbstick",   OnPress = ButtonCommand.NoCommand,            OnRelease = ButtonCommand.NoCommand        },
        },

        Axes = 
        {
            ["0"] = { Name = "LeftStickHorizontal",      OnValue = AxisCommand.Velocity         },
            ["1"] = { Name = "LeftStickVertical",        OnValue = AxisCommand.Position         },
            ["2"] = { Name = "LeftTrigger",              OnValue = AxisCommand.NoCommand        },
            ["3"] = { Name = "RightStickHorizontal",     OnValue = AxisCommand.MotorCurrent     },
            ["4"] = { Name = "RightStickVertical",       OnValue = AxisCommand.Torque           },
            ["5"] = { Name = "RightTrigger",             OnValue = AxisCommand.NoCommand        },
            ["6"] = { Name = "DPadHorizontal",           OnValue = AxisCommand.NoCommand        },
            ["7"] = { Name = "DPadVertical",             OnValue = AxisCommand.NoCommand        },
        }
    }
end

Mappings = 
{
    ["Generic X-Box pad"] = create_generic_xbox_mappings(),

    ["Logitech Logitech Dual Action"] =
    {
        Buttons =
        {
            [ "0"] = { Name =  "1",               OnPress = ButtonCommand.NoCommand, OnRelease = ButtonCommand.NoCommand },
            [ "1"] = { Name =  "2",               OnPress = ButtonCommand.NoCommand, OnRelease = ButtonCommand.NoCommand },
            [ "2"] = { Name =  "3",               OnPress = ButtonCommand.NoCommand, OnRelease = ButtonCommand.NoCommand },
            [ "3"] = { Name =  "4",               OnPress = ButtonCommand.NoCommand, OnRelease = ButtonCommand.NoCommand },
            [ "4"] = { Name =  "5",               OnPress = ButtonCommand.NoCommand, OnRelease = ButtonCommand.NoCommand },
            [ "5"] = { Name =  "6",               OnPress = ButtonCommand.NoCommand, OnRelease = ButtonCommand.NoCommand },
            [ "8"] = { Name =  "7",               OnPress = ButtonCommand.NoCommand, OnRelease = ButtonCommand.NoCommand },
            [ "7"] = { Name =  "8",               OnPress = ButtonCommand.NoCommand, OnRelease = ButtonCommand.NoCommand },
            [ "8"] = { Name =  "9",               OnPress = ButtonCommand.NoCommand, OnRelease = ButtonCommand.NoCommand },
            [ "9"] = { Name = "10",               OnPress = ButtonCommand.NoCommand, OnRelease = ButtonCommand.NoCommand },
            ["10"] = { Name = "LeftThumbstick",   OnPress = ButtonCommand.NoCommand, OnRelease = ButtonCommand.NoCommand },
            ["11"] = { Name = "RightThumbstick",  OnPress = ButtonCommand.NoCommand, OnRelease = ButtonCommand.NoCommand },
        },

        Axes =
        {
            ["0"] = { Name = "LeftStickVertical",         OnValue = AxisCommand.NoCommand },
            ["1"] = { Name = "LeftStickHorizontal",       OnValue = AxisCommand.NoCommand },
            ["2"] = { Name = "RightStickVertical",        OnValue = AxisCommand.NoCommand },
            ["3"] = { Name = "RightStickHorizontal",      OnValue = AxisCommand.NoCommand },
            ["4"] = { Name = "DPadVertical",              OnValue = AxisCommand.NoCommand },
            ["5"] = { Name = "DPadHorizontal",            OnValue = AxisCommand.NoCommand },
        },
    },

    ["Xbox 360 Wireless Receiver"] = create_generic_xbox_mappings(),

    ["Microsoft X-Box 360 pad"] = create_xbox360_mappings()
}
