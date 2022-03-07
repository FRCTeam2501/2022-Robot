package frc.bionicpolars.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.bionicpolars.Constants;

public class Drivetrain extends SubsystemBase {
    private final CANSparkMax
        leftFront = new CANSparkMax(Constants.LEFT_FRONT_ID,
                CANSparkMax.MotorType.kBrushless),
        rightFront = new CANSparkMax(Constants.RIGHT_FRONT_ID,
                CANSparkMax.MotorType.kBrushless),
        leftRear = new CANSparkMax(Constants.LEFT_REAR_ID,
                CANSparkMax.MotorType.kBrushless),
        rightRear = new CANSparkMax(Constants.RIGHT_REAR_ID,
                CANSparkMax.MotorType.kBrushless);
    private final DifferentialDrive
        drive = new DifferentialDrive(
                new MotorControllerGroup(leftFront, leftRear),
                new MotorControllerGroup(rightFront, rightRear));
    private boolean isInverted = false;


    public void arcadeDrive(double xSpeed, double zRotation) {
        if(isInverted)
            xSpeed *= -1.0;

        drive.arcadeDrive(xSpeed, zRotation);
    }

    public boolean isInverted() {
        return isInverted;
    }

    public void setInverted(boolean isInverted) {
        this.isInverted = isInverted;
    }
}
