package frc.bionicpolars.subsystems.intake;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.bionicpolars.Constants;

public class Intake extends SubsystemBase {
    CANSparkMax motor = new CANSparkMax(Constants.INTAKE_MOTOR_ID,
            CANSparkMax.MotorType.kBrushless);
    IntakeRotate rotation = new IntakeRotate();

    public double getAngle() {
        return rotation.Get();
    }

    public void setAngle(double angle) {
        rotation.Set(angle);
    }

    public double getMotor() {
        return motor.get();
    }

    public void setMotor(double speed) {
        motor.set(speed);
    }

    public void zeroEncoder(double angle) {
        rotation.Zero(angle);
    }
}
