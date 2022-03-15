#include "subsystems/Intake.h"

using namespace frc;

Intake::Intake()
{
//I realise that this is a pointer and should have a destructor but I just realized that coming home from duluth so I wont fix it
    rollerMotor = new rev::CANSparkMax(16, rev::CANSparkMax::MotorType::kBrushed);

    //  intakeLift.GetForwardLimitSwitch(rev::SparkMaxLimitSwitch::Type::kNormallyOpen).EnableLimitSwitch(true);
    intakeLift.SetSmartCurrentLimit(40);
    intakeLift.SetSecondaryCurrentLimit(50);
    //  intakeLift.GetReverseLimitSwitch(rev::SparkMaxLimitSwitch::Type::kNormallyOpen).EnableLimitSwitch(true);

//This is where we set the PID for the lift motor to control it accurately 
    intakeLiftPID.SetP(IntakeConstants::intakeLiftSetP);
    intakeLiftPID.SetI(IntakeConstants::intakeLiftSetI);
    intakeLiftPID.SetD(IntakeConstants::intakeLiftSetD);

    //Notice that the output range is not -1 to 1. This is to make it so that the motor only uses 10% on the way down. 
    intakeLiftPID.SetOutputRange(-0.1, 1.0);
    //SetPositionConversionFactor is one because I didn't want to figure out angles for it
    intakeLiftEncoder.SetPositionConversionFactor(1);
    intakeLift.SetInverted(false);

//Current limit stuff
    rollerMotor->SetSmartCurrentLimit(60);
    rollerMotor->SetSecondaryCurrentLimit(80);
}

//This was my solution to the problem of the lift slaming on the ground. I was going to change the P value to a lower value to 
//Make it less forceful downward and more power up. This is a bad Idea because it can overload teh can bus passing P values constantly so don't do it
void Intake::SetP(double pValue)
{
    intakeLiftPID.SetP(pValue);
    frc::SmartDashboard::PutNumber("Lift P valuel", pValue);
}

void Intake::RollerControl(double rollerSpeed)
{
//This was the function I used to control the roller on the intake.
//You pass it a value between -1 and 1 and it puts that to the motor power. 
//We dont use PID for the roller because its not that important to control its speed.
//By the way, You can also do Velocity PID as well as Position PID. We did this if we need a accurate flywheel like in 2020
    if (rollerSpeed > 1)
    {
        rollerSpeed = 1;
    }

    if (rollerSpeed < -1)
    {
        rollerSpeed = -1;
    }
//Sets the motor speed
    rollerMotor->Set(rollerSpeed);
}

void Intake::LiftControl(double liftAngle)
{
    //Sets the lift angle, the value that is passed is not an angle it is just turns of the NEO because I was too lazy to figure out degrees for it.
    //If you need the equation for figureing out conversion factor I think it is somewhere in the code, not this file though. 

    //This puts the target number for the lift to the dashboard
    frc::SmartDashboard::PutNumber("Lift target", liftAngle);
    //This sets the refernce for the PID controller to the value that we passed to this function
    intakeLiftPID.SetReference(liftAngle, rev::CANSparkMaxLowLevel::ControlType::kPosition);
}

void Intake::SetLiftEncoder(double intakeZero)
{
    //This function will take in a value that is passed to it and set the encoder on the lift motor to that value
    //This is how we zero the encoders at the start of every autonomous period. 
    intakeLiftEncoder.SetPosition(intakeZero);
}

void Intake::Periodic()
{
//Updates the lift encoder position every 20 ms
    frc::SmartDashboard::PutNumber("lift Actual", intakeLiftEncoder.GetPosition());
}