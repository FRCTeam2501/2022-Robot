package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {
    private final Drivetrain drivetrain;
    private final Joystick driveStick;

    public RobotContainer() {
        drivetrain = new Drivetrain();
        driveStick = new Joystick(Constants.DRIVE_STICK_ID);

        configureButtonBindings();

        drivetrain.setDefaultCommand(new RunCommand(
            () -> drivetrain.arcadeDrive(
                driveStick.getX(),
                driveStick.getY()
            ),
            drivetrain
        ));
    }

    private void configureButtonBindings() {
        JoystickButton temp = new JoystickButton(
            driveStick,
            Constants.DRIVE_STICK_REVERSE_DRIVETRAIN
        );
        temp.whenPressed(
            () -> drivetrain.setInverted(!drivetrain.isInverted()),
            drivetrain
        );
    }
}
