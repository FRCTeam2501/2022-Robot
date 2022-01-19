#include "subsystems/Drivetrain.h"
#include "Constants.h"


Drivetrain::Drivetrain() {
    // Create motor controllers
    leftFront = new rev::CANSparkMax(
        CONSTANTS::MOTORS::CAN::LEFT_FRONT_ID,
        rev::CANSparkMax::MotorType::kBrushless
    );
    rightFront = new rev::CANSparkMax(
        CONSTANTS::MOTORS::CAN::RIGHT_FRONT_ID,
        rev::CANSparkMax::MotorType::kBrushless
    );
    leftRear = new rev::CANSparkMax(
        CONSTANTS::MOTORS::CAN::LEFT_REAR_ID,
        rev::CANSparkMax::MotorType::kBrushless
    );
    rightRear = new rev::CANSparkMax(
        CONSTANTS::MOTORS::CAN::RIGHT_REAR_ID,
        rev::CANSparkMax::MotorType::kBrushless
    );


    // Setup current limits, idle modes, and encoder factors
    leftFront->SetSmartCurrentLimit(
        CONSTANTS::DRIVETRAIN::HARD_CURRENT_LIMIT.to<int>());
    leftFront->SetSecondaryCurrentLimit(
        CONSTANTS::DRIVETRAIN::SOFT_CURRENT_LIMIT.to<double>());
    leftFront->SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    leftFront->GetEncoder().SetPositionConversionFactor(
        CONSTANTS::DRIVETRAIN::TURN_TO_METER.to<double>());
    leftFront->GetEncoder().SetVelocityConversionFactor(
        CONSTANTS::DRIVETRAIN::TURN_TO_METER.to<double>());
    rightFront->SetSmartCurrentLimit(
        CONSTANTS::DRIVETRAIN::HARD_CURRENT_LIMIT.to<int>());
    rightFront->SetSecondaryCurrentLimit(
        CONSTANTS::DRIVETRAIN::SOFT_CURRENT_LIMIT.to<double>());
    rightFront->SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    rightFront->GetEncoder().SetPositionConversionFactor(
        CONSTANTS::DRIVETRAIN::TURN_TO_METER.to<double>());
    rightFront->GetEncoder().SetVelocityConversionFactor(
        CONSTANTS::DRIVETRAIN::TURN_TO_METER.to<double>());
    leftRear->SetSmartCurrentLimit(
        CONSTANTS::DRIVETRAIN::HARD_CURRENT_LIMIT.to<int>());
    leftRear->SetSecondaryCurrentLimit(
        CONSTANTS::DRIVETRAIN::SOFT_CURRENT_LIMIT.to<double>());
    leftRear->SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    leftRear->GetEncoder().SetPositionConversionFactor(
        CONSTANTS::DRIVETRAIN::TURN_TO_METER.to<double>());
    leftRear->GetEncoder().SetVelocityConversionFactor(
        CONSTANTS::DRIVETRAIN::TURN_TO_METER.to<double>());
    rightRear->SetSmartCurrentLimit(
        CONSTANTS::DRIVETRAIN::HARD_CURRENT_LIMIT.to<int>());
    rightRear->SetSecondaryCurrentLimit(
        CONSTANTS::DRIVETRAIN::SOFT_CURRENT_LIMIT.to<double>());
    rightRear->SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    rightRear->GetEncoder().SetPositionConversionFactor(
        CONSTANTS::DRIVETRAIN::TURN_TO_METER.to<double>());
    rightRear->GetEncoder().SetVelocityConversionFactor(
        CONSTANTS::DRIVETRAIN::TURN_TO_METER.to<double>());


    // Create motor controller groups
    left = new frc::MotorControllerGroup(*leftFront, *leftRear);
    right = new frc::MotorControllerGroup(*rightFront, *rightRear);

    // Create differential drivetrain object
    drive = new frc::DifferentialDrive(*left, *right);


    // Set default state of drivetrain to be not inverted
    isInverted = new bool(false);
}

Drivetrain::~Drivetrain() {
    delete leftFront;
    delete rightFront;
    delete leftRear;
    delete rightRear;

    delete left;
    delete right;

    delete drive;

    delete isInverted;
}

void Drivetrain::ArcadeDrive(double xSpeed, double zRotation) {
    if(*isInverted)
        xSpeed *= -1.0;

    drive->ArcadeDrive(
        CONSTANTS::DRIVETRAIN::FORWARD_SPEED * xSpeed,
        CONSTANTS::DRIVETRAIN::ROTATION_SPEED * zRotation
    );
}

bool Drivetrain::IsInverted() {
    return *isInverted;
}

void Drivetrain::SetInverted(bool inverted) {
    *isInverted = inverted;
}