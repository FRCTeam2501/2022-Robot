package frc.robot;

public final class Constants {
    // CAN IDs
    public static final int LEFT_FRONT_ID = 1,
            RIGHT_FRONT_ID = 2,
            LEFT_REAR_ID = 3,
            RIGHT_REAR_ID = 4,
            CLIMBER_ROTATION_ID = 5,
            CLIMBER_EXTENSION_ID = 6,
            INTAKE_ROTATION_ID = 13,
            INTAKE_MOTOR_ID = 16;

    // USB Joystick IDs
    public static final int DRIVE_STICK_ID = 0,
            CONTROL_STICK_ID = 1;

    // Driver Joystick Buttons
    public static final int RUN_INTAKE = 1,
            REVERSE_INTAKE = 2,
            TOGGLE_INTAKE = 3,
            INTAKE_MIDDLE = 4,
            ZERO_INTAKE_ENCODER = 8,
            REVERSE_DRIVETRAIN = 10,
            VISION_SWITCH_FEED = 12;
    // Controller Joystick buttons
    public static final int CLIMBER_EXTEND = 1,
            CLIMBER_RETRACT = 2,
            CLIMBER_IN = 5,
            CLIMBER_OUT = 6,
            CLIMBER_DISLODGE_WRENCH = 8;
}
