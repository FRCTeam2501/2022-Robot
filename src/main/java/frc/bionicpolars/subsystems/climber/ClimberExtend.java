package frc.bionicpolars.subsystems.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import frc.bionicpolars.Constants;

public class ClimberExtend {
    private static final double GEARBOX_RATIO = 100.0,
            PULLEY_DIAMETER = 1.7887,
            PULLEY_CIRCUMFERENCE = Math.PI * PULLEY_DIAMETER,
            P = 2.0,
            I = 0.0,
            D = 0.0;
    private CANSparkMax motor = new CANSparkMax(
            Constants.CLIMBER_EXTENSION_ID,
            CANSparkMax.MotorType.kBrushless);
    private RelativeEncoder encoder = motor.getEncoder();
    private SparkMaxPIDController pid = motor.getPIDController();
    private double distance = 0.0;


    private double getTurns(double distance) {
        // Note: This only accounts for a single circumference,
        //  not the changing circumference of the pulley system
        return distance / PULLEY_CIRCUMFERENCE;
    }

    private double getDistance(double turns) {
        // Note: This only accounts for a single circumference,
        //  not the changing circumference of the pulley system
        return turns * PULLEY_CIRCUMFERENCE;
    }

    public ClimberExtend() {
        encoder.setPositionConversionFactor(GEARBOX_RATIO);
        pid.setP(P);
        pid.setI(I);
        pid.setD(D);
        pid.setOutputRange(-1.0, 1.0);
    }

    public double get() {
        return distance;
    }

    public void set(double distance) {
        this.distance = distance;
        pid.setReference(getTurns(distance), CANSparkMax.ControlType.kPosition);
    }

    public double getActual() {
        return getDistance(encoder.getPosition());
    }

    public void zero() {
        encoder.setPosition(0.0);
    }
}
