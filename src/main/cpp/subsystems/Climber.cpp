#include "subsystems/Climber.h"

using namespace frc;
using namespace std;

Climber::Climber()
{
    // pivotClimb.GetForwardLimitSwitch(rev::SparkMaxLimitSwitch::Type::kNormallyOpen).EnableLimitSwitch(true);
    pivotClimb.SetSmartCurrentLimit(ClimbConstants::pivotClimbSmartCurrentLimet);
    pivotClimb.SetSecondaryCurrentLimit(ClimbConstants::pivotClimbSeccondaryCurrentLimet);
    //  pivotClimb.GetReverseLimitSwitch(rev::SparkMaxLimitSwitch::Type::kNormallyOpen).EnableLimitSwitch(true);

    //  winchPin = new frc::Solenoid(frc::PneumaticsModuleType::CTREPCM,5);

    winch.SetInverted(true);
    pivotClimb.SetInverted(true);

    pivotPID.SetP(ClimbConstants::pivotClimbSetP);
    pivotPID.SetI(ClimbConstants::pivotClimbSetI);
    pivotPID.SetD(ClimbConstants::pivotClimbSetD);
    pivotPID.SetOutputRange(-1.0, 1.0);
    pivotEncoder.SetPositionConversionFactor(
        ((360.0 / (ClimbConstants::pivotConversionFactorOne * ClimbConstants::pivotConversionFactorTwo))));
    // Makes it so that one unit into the motor makes one degree of rotation of the climb arm

    winchPID.SetP(ClimbConstants::winchSetP);
    winchPID.SetI(ClimbConstants::winchSetI);
    winchPID.GetD(ClimbConstants::winchSetD);
    winchPID.SetOutputRange(-1.0, 1.0);

    winch.SetSmartCurrentLimit(ClimbConstants::winchSmartCurrentLimet);
    winch.SetSecondaryCurrentLimit(ClimbConstants::winchSeccondaryCurrentLimet);
    winchEncoder.SetPositionConversionFactor(1.0 / 100.0); // I think this is right
}



void Climber::DislodgeWrench(){
    //length = (winchEncoder.GetPosition() - 0.5);
    disclodgeTarget = (winchEncoder.GetPosition() - 0.5);
    winchPID.SetReference(disclodgeTarget, rev::CANSparkMaxLowLevel::ControlType::kPosition);
    dislodgingWrench = true;
    wrenchDislodged = true;
}

int Climber::ClimbControl(double angleAdjust, double lengthAdjust)
{
    if(wrenchDislodged == true){
    horizontalActivated = false;
    seccondaryMove = false;
    lengthChanged = false;
    swingActivated = false;
    thirdMove = false;
    cout << "ClimbControl start" << endl;
    cout<<"angleAdjust 1: "<<angleAdjust<<endl;
    cout<<"LengthAdjust 1: "<<lengthAdjust<<endl;
    // lengthAdjust is the new length that we want to set the arms to
    //  makes sure length is not outside of limet
    if (length > ClimbConstants::maxLength)
    {
        length = ClimbConstants::maxLength;
    }
    if (length < ClimbConstants::minLength)
    {
        length = ClimbConstants::minLength;
    }

    // makes shure we are not trying to set length to a value that is outside the length limet
    if (lengthAdjust > ClimbConstants::maxLength)
    {
        lengthAdjust = ClimbConstants::maxLength;
    }
    if (lengthAdjust < ClimbConstants::minLength)
    {
        lengthAdjust = ClimbConstants::minLength;
    }
    // checks if new length is legal and if not sets length to maximum it can and sets length adjust to zero

    // makes shure that angle is not outside of limet
    if (angle > ClimbConstants::maxAngle)
    {
        angle = ClimbConstants::maxAngle;
    }
    if (angle < ClimbConstants::minAngle)
    {
        angle = ClimbConstants::minAngle;
    }

    if (angleAdjust > ClimbConstants::maxAngle)
    {
        angleAdjust = (ClimbConstants::maxAngle);
    }
    if (angleAdjust < ClimbConstants::minAngle)
    {
        angleAdjust = ClimbConstants::minAngle;
    }
   // cout << "angleAdjust 2: " << angleAdjust << endl;
   // cout << "LengthAdjust 2: " << lengthAdjust << endl;
    if (angleAdjust > 1)
    {
        // checks new climber position to make shure that it is legal.
        if (lengthAdjust < (ClimbConstants::defaultClimbLength - (((ClimbConstants::pivotToFrameDist + ClimbConstants::maxDistFromFrame) - ClimbConstants::rotationOffset * std::cos((angleAdjust * ClimbConstants::pi / (180)))) / (std::sin((angleAdjust * ClimbConstants::pi / (180)))))))
        {
            // if the length is not legal, set it to the legal length.
            lengthAdjust = (ClimbConstants::defaultClimbLength - (((ClimbConstants::pivotToFrameDist + ClimbConstants::maxDistFromFrame) - ClimbConstants::rotationOffset * std::cos((angleAdjust * ClimbConstants::pi / (180)))) / (std::sin((angleAdjust * ClimbConstants::pi / (180))))));
            lengthChanged = true;
        }
    }
    //  cout<<"angleAdjust 3: "<<angleAdjust<<endl;
    //  cout<<"LengthAdjust 3: "<<lengthAdjust<<endl;
    // Checks if the sceleing is veing violated, if so, it will change the length to make it legal
    if (lengthAdjust > (((ClimbConstants::defaultScealing - ClimbConstants::rotationBigOffset * std::sin((angleAdjust * ClimbConstants::pi / (180)))) / std::cos((angleAdjust * ClimbConstants::pi / (180)))) - ClimbConstants::minExtension))
    {
        lengthAdjust = (((ClimbConstants::defaultScealing - ClimbConstants::rotationBigOffset * std::sin((angleAdjust * ClimbConstants::pi / (180)))) / std::cos((angleAdjust * ClimbConstants::pi / (180)))) - ClimbConstants::minExtension);
        lengthChanged = true;
    }
  //  cout << "angleAdjust 4: " << angleAdjust << endl;
   // cout << "LengthAdjust 4: " << lengthAdjust << endl;

    if ((angleAdjust <= ClimbConstants::batteryMaxAngle && angleAdjust >= ClimbConstants::batteryMinAngle)
            || (angleAdjust > ClimbConstants::batteryMaxAngle && angle <= ClimbConstants::batteryMaxAngle)
            || (angleAdjust < ClimbConstants::batteryMinAngle && angle >= ClimbConstants::batteryMinAngle))
    {
        cout<<"Battery Acitvated: ";
        if (angleAdjust <= ClimbConstants::batteryMaxAngle && angleAdjust >= ClimbConstants::batteryMinAngle)
        {
            cout<<"a";
            if (lengthAdjust < ClimbConstants::batteryMinLength)
            {
                lengthAdjust = ClimbConstants::batteryMinLength;
                cout<<"b";
            }
        }
        else
        {
            cout<<"c";
            if (lengthAdjust < ClimbConstants::batteryMinLength ||  length < ClimbConstants::batteryMinLength)
            {
                cout<<"d";
                targetLength = lengthAdjust;
                targetAngle = angleAdjust;
                lengthAdjust = ClimbConstants::batteryMinLength;
                lengthChanged = true;
                seccondaryMove = true;

                length = lengthAdjust;
                winchPID.SetReference(Climber::LengthToTurns(length), rev::CANSparkMaxLowLevel::ControlType::kPosition);
                pivotPID.SetReference(angle, rev::CANSparkMaxLowLevel::ControlType::kPosition);
            }
        }
        cout<<endl;
    }

    frc::SmartDashboard::PutNumber("Climb Seccondary move", seccondaryMove);
  //  cout << "seccondaryMove: " << seccondaryMove << endl;
    if (seccondaryMove == false)
    {
       // cout << "Actual set angleAdjust: " << angleAdjust << endl;
       // cout << "Actual set LengthAdjust: " << lengthAdjust << endl;
        length = lengthAdjust;
        angle = angleAdjust;

       // cout << "angle: " << angle << endl;
        //cout << "Length: " << length << endl;
        storeAngle = angle;
       // cout << "StoreAngle: " << storeAngle << endl;
        winchPID.SetReference(Climber::LengthToTurns(length), rev::CANSparkMaxLowLevel::ControlType::kPosition);
        pivotPID.SetReference(angle, rev::CANSparkMaxLowLevel::ControlType::kPosition);
       // cout << "fibbed length: " << Climber::LengthToTurns(length) << endl;
        
    }
    }
    return lengthChanged;
}

double Climber::LengthToTurns(double inchesToTurns)
{

    constexpr double pi = 3.141592653589793;

    constexpr double d = 0.05;
    constexpr double l = 28;

    constexpr double d0 = 1.7887;
    constexpr double c0 = d0 * pi;
    constexpr double c1 = (d0 + 1.0 * d) * pi;
    constexpr double c2 = (d0 + 2.0 * d) * pi;
    constexpr double c3 = (d0 + 3.0 * d) * pi;
    constexpr double c4 = (d0 + 4.0 * d) * pi;
    constexpr double c5 = (d0 + 5.0 * d) * pi;
    constexpr double c6 = (d0 + 6.0 * d) * pi;
    constexpr double l1 = (l - c0);
    constexpr double l2 = (l1 - c1);
    constexpr double l3 = (l2 - c2);
    constexpr double l4 = (l3 - c3);
    constexpr double l5 = (l4 - c4);
    constexpr double l6 = (l5 - c5);

    constexpr double f6 = ((1.0 / c5) * l5);
    constexpr double f5 = ((1.0 / c4) * (l4 - l5) + f6);
    constexpr double f4 = ((1.0 / c3) * (l3 - l4) + f5);
    constexpr double f3 = ((1.0 / c2) * (l2 - l3) + f4);
    constexpr double f2 = ((1.0 / c1) * (l1 - l2) + f3);
    constexpr double f1 = ((1.0 / c0) * (l - l1) + f2);

    double maxLength = 28;
    double turns;

    if (inchesToTurns <= l5)
    {
        turns = ((1.0 / c5) * inchesToTurns);
    }

    if (l5 < inchesToTurns && inchesToTurns <= l4)
    {
        turns = ((1.0 / c4) * (inchesToTurns - l5) + f6);
    }

    if (l4 < inchesToTurns && inchesToTurns <= l3)
    {
        turns = ((1.0 / c3) * (inchesToTurns - l4) + f5);
    }

    if (l3 < inchesToTurns && inchesToTurns <= l2)
    {
        turns = ((1.0 / c2) * (inchesToTurns - l3) + f4);
    }

    if (l2 < inchesToTurns && inchesToTurns <= l1)
    {
        turns = ((1.0 / c1) * (inchesToTurns - l2) + f3);
    }

    if (l1 < inchesToTurns && inchesToTurns <= l)
    {
        turns = ((1.0 / c0) * (inchesToTurns - l1) + f2);
    }

    return turns;
}

double Climber::GetAngle()
{
    return angle;
}

double Climber::GetLength()
{
    return length;
}

void Climber::ClimbPivotSetEncoder(double pivotSetEncoder)
{
    pivotEncoder.SetPosition(pivotSetEncoder);
}
void Climber::ClimbWinchSetEncoder(double winchSetEncoder)
{
    winchEncoder.SetPosition(winchSetEncoder);
}

void Climber::Periodic()
{
    frc::SmartDashboard::PutNumber("Climb Target Length", length);
    frc::SmartDashboard::PutNumber("Climb Fibbed Target Length", Climber::LengthToTurns(length));
    frc::SmartDashboard::PutNumber("Climb Target angle", angle);
    frc::SmartDashboard::PutNumber("Seccond Move: ", seccondaryMove);
    frc::SmartDashboard::PutNumber("Winch Encoder: ", winchEncoder.GetPosition());
    frc::SmartDashboard::PutNumber("Pivot Encoder: ", pivotEncoder.GetPosition());
    // This checks if we have a scedjuled seccond move once we have reached the angle we were going for
    if (seccondaryMove == true && thirdMove == false && (abs(winchEncoder.GetPosition() - 4) < 0.5))
    {
        angle = targetAngle;
        pivotPID.SetReference(angle, rev::CANSparkMaxLowLevel::ControlType::kPosition);
        thirdMove = true;
    }
    if (seccondaryMove == true && thirdMove == true && (abs(pivotEncoder.GetPosition() - targetAngle) < 1))
    {
        length = targetLength;
        winchPID.SetReference(LengthToTurns(length), rev::CANSparkMaxLowLevel::ControlType::kPosition);

        seccondaryMove = false;
        thirdMove = false;
    }
    if(dislodgingWrench == true && abs(winchEncoder.GetPosition() - dislodgeTarget)<0.25 ){
        //length = (dislodgeTarget + 0.5);
        winchPID.SetReference((dislodgeTarget + 0.5), rev::CANSparkMaxLowLevel::ControlType::kPosition);
        dislodgingWrench = false;
    }
}

void InitDefaultCommand()
{
}
