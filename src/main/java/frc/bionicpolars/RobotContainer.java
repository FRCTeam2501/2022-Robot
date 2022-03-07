package frc.bionicpolars;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.bionicpolars.subsystems.Drivetrain;
import frc.bionicpolars.subsystems.Vision;

public class RobotContainer {
    private final Drivetrain drivetrain = new Drivetrain();
    private final Vision vision = new Vision();
    private final Joystick driveStick = new Joystick(Constants.DRIVE_STICK_ID);

    public RobotContainer() {
        drivetrain.setDefaultCommand(new RunCommand(
            () -> {
                if(driveStick.getThrottle() >= 0.0)
                    drivetrain.arcadeDrive(
                        -1.0 * driveStick.getY(),
                        0.6 * driveStick.getX()
                    );
                else
                    drivetrain.arcadeDrive(
                        -0.6 * driveStick.getY(),
                        0.45 * driveStick.getX()
                    );
            },
            drivetrain
        ));

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        // Reverse drivetrain direction
        new JoystickButton(driveStick,
                Constants.REVERSE_DRIVETRAIN
        ).whenPressed(
            () -> drivetrain.setInverted(!drivetrain.isInverted()),
            drivetrain
        );

        // Switch camera feed
        new JoystickButton(driveStick,
                Constants.VISION_SWITCH_FEED
        ).whenPressed(
            () -> vision.switchFeed(),
            vision
        );
    }
}
