package frc.bionicpolars.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import frc.bionicpolars.Constants;

public class IntakeRotate {
    public static final double GEARBOX_RATIO = 100.0,
            PULLEY_RATIO = 122.0 / 65.0,
            TURN_TO_DEGREE = 360 / (GEARBOX_RATIO * PULLEY_RATIO),
            P = 0.07,
            I = 0.0,
            D = 0.0;
    private CANSparkMax motor = new CANSparkMax(
            Constants.INTAKE_ROTATION_ID,
            CANSparkMax.MotorType.kBrushless);
    RelativeEncoder encoder = motor.getEncoder();
    SparkMaxPIDController pid = motor.getPIDController();
    double angle = 0;


    public IntakeRotate() {
        encoder.setPositionConversionFactor(TURN_TO_DEGREE);
        pid.setP(P);
        pid.setI(I);
        pid.setD(D);
        pid.setOutputRange(-1.0, 1.0);
    }

    public double Get() {
        return angle;
    }

    public void Set(double angle) {
        this.angle = angle;
        pid.setReference(angle, CANSparkMax.ControlType.kPosition);
    }

    public double GetActual() {
        return encoder.getPosition();
    }

    public void Zero() {
        encoder.setPosition(0);
    }
}
