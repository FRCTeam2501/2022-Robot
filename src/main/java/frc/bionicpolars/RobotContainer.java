package frc.bionicpolars;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.bionicpolars.subsystems.Drivetrain;
import frc.bionicpolars.subsystems.Vision;
import frc.bionicpolars.subsystems.climber.Climber;
import frc.bionicpolars.subsystems.intake.Intake;

public class RobotContainer {
    private final Climber climber = new Climber();
    private final Drivetrain drivetrain = new Drivetrain();
    private final Intake intake = new Intake();
    private final Vision vision = new Vision();
    private final Joystick driveStick = new Joystick(Constants.DRIVE_STICK_ID),
            controlStick = new Joystick(Constants.CONTROL_STICK_ID);


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

        climber.setDefaultCommand(new RunCommand(
            () -> {
                if(Math.abs(controlStick.getY()) > 0.1) {
                    climber.set(
                        climber.getExtension(),
                        (1.0 * controlStick.getY())
                            + climber.getAngle()
                    );
                }
            },
            climber
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


        // Run the intake inwards
        new JoystickButton(driveStick,
                Constants.RUN_INTAKE
        ).whileHeld(new StartEndCommand(
            () -> intake.setMotor(0.8), 
            () -> intake.setMotor(0.0),
            intake
        ));

        // Reverse the intake
        new JoystickButton(driveStick,
                Constants.REVERSE_INTAKE
        ).whileHeld(new StartEndCommand(
            () -> intake.setMotor(-0.8),
            () -> intake.setMotor(0.0),
            intake
        ));

        // Toggle the intake
        new JoystickButton(driveStick,
                Constants.TOGGLE_INTAKE
        ).whenPressed(
            () -> {
                if(intake.getAngle() > -50.0)
                    intake.setAngle(-90.0);
                else
                    intake.setAngle(0.0);
            },
            intake
        );

        // Set the intake to the middle
        new JoystickButton(driveStick,
                Constants.INTAKE_MIDDLE
        ).whenPressed(
            () -> intake.setAngle(-30.0),
            intake
        );

        // Zero the intake encoder at min
        new JoystickButton(driveStick,
                Constants.ZERO_INTAKE_ENCODER_MIN
        ).whenPressed(
            () -> intake.zeroEncoder(-90.0),
            intake
        );

        // Zero the intake encoder at max
        new JoystickButton(driveStick,
                Constants.ZERO_INTAKE_ENCODER_MAX
        ).whenPressed(
            () -> intake.zeroEncoder(0.0),
            intake
        );


        // Run the climber upwards
        new JoystickButton(driveStick,
                Constants.CLIMBER_OUT
        ).whileHeld(new RunCommand(
            () -> {
                if(controlStick.getThrottle() >= 0.0)
                    climber.set(
                        climber.getExtension() + 2.5,
                        climber.getAngle()
                    );
                else
                    climber.set(
                        climber.getExtension() + 0.5,
                        climber.getAngle()
                );
            },
            climber
        ));


        // Switch camera feed
        new JoystickButton(driveStick,
                Constants.VISION_SWITCH_FEED
        ).whenPressed(
            () -> vision.switchFeed(),
            vision
        );
    }

    public Command getAutonomousCommand() {
        return null;
    }
}
