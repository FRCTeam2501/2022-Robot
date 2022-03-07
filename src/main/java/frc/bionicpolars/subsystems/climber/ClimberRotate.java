package frc.bionicpolars.subsystems.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import frc.bionicpolars.Constants;

public class ClimberRotate {
    public static double GEARBOX_RATIO = 100.0,
            PULLEY_RATIO = 122.0 / 65.0,
            TURN_TO_DEGREE = 360 / (GEARBOX_RATIO * PULLEY_RATIO),
            P = 0.07,
            I = 0.0,
            D = 0.0;
    private CANSparkMax motor = new CANSparkMax(
            Constants.CLIMBER_ROTATION_ID,
            CANSparkMax.MotorType.kBrushless);
    private RelativeEncoder encoder = motor.getEncoder();
    private SparkMaxPIDController pid = motor.getPIDController();
    private double angle = 0.0;


    public ClimberRotate() {
        encoder.setPositionConversionFactor(TURN_TO_DEGREE);
        pid.setP(P);
        pid.setI(I);
        pid.setD(D);
        pid.setOutputRange(-1.0, 1.0);
    }

    public double get() {
        return angle;
    }

    public void set(double angle) {
        this.angle = angle;
        pid.setReference(angle, CANSparkMax.ControlType.kPosition);
    }

    public double getActual() {
        return encoder.getPosition();
    }

    public void zero() {
        encoder.setPosition(0.0);
    }
}
