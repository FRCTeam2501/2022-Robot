package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import frc.robot.Constants;

public class Drivetrain {
    private final CANSparkMax leftFront, rightFront, leftRear, rightRear;
    private final MotorControllerGroup left, right;
    private final DifferentialDrive drive;
    private boolean isInverted = false;

    public Drivetrain() {
        leftFront = new CANSparkMax(Constants.LEFT_FRONT_ID, CANSparkMax.MotorType.kBrushless);
        rightFront = new CANSparkMax(Constants.RIGHT_FRONT_ID, CANSparkMax.MotorType.kBrushless);
        leftRear = new CANSparkMax(Constants.LEFT_REAR_ID, CANSparkMax.MotorType.kBrushless);
        rightRear = new CANSparkMax(Constants.RIGHT_REAR_ID, CANSparkMax.MotorType.kBrushless);

        left = new MotorControllerGroup(leftFront, leftRear);
        right = new MotorControllerGroup(rightFront, rightRear);

        drive = new DifferentialDrive(left, right);
    }

    public void arcadeDrive(double xSpeed, double zRotation) {
        if(isInverted)
            xSpeed *= -1.0;

        drive.arcadeDrive(
            Constants.FORWARD_SPEED * xSpeed,
            Constants.ROTATION_SPEED * zRotation
        );
    }

    public boolean isInverted() {
        return isInverted;
    }

    public void setInverted(boolean isInverted) {
        this.isInverted = isInverted;
    }
}
