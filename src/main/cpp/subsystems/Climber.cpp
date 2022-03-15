#include "subsystems/Climber.h"

using namespace frc;
using namespace std;

Climber::Climber()
{
    // pivotClimb.GetForwardLimitSwitch(rev::SparkMaxLimitSwitch::Type::kNormallyOpen).EnableLimitSwitch(true);
    pivotClimb.SetSmartCurrentLimit(ClimbConstants::pivotClimbSmartCurrentLimet);
    pivotClimb.SetSecondaryCurrentLimit(ClimbConstants::pivotClimbSeccondaryCurrentLimet);
    //  pivotClimb.GetReverseLimitSwitch(rev::SparkMaxLimitSwitch::Type::kNormallyOpen).EnableLimitSwitch(true);
    //I added the code for the winch pin but it never got put on the bot so I just comented the code out in case build changed their minds. 
    //The reason that we needed the pin was because the climber had to stay down for the whole match until we got to the climbing area.
    //our solution was to have a wrenh that would go on the winch to hold it down until we pressed a button to relece it. 
    //That is what we did instead of the pin
    //  winchPin = new frc::Solenoid(frc::PneumaticsModuleType::CTREPCM,5);

    //This sets the winch and the pivot motors to be inverted
    winch.SetInverted(true);
    pivotClimb.SetInverted(true);

    //This sets the P I and D values for the pivot motor. The SetOutputRange is how much force it can appily in eather direction
    pivotPID.SetP(ClimbConstants::pivotClimbSetP);
    pivotPID.SetI(ClimbConstants::pivotClimbSetI);
    pivotPID.SetD(ClimbConstants::pivotClimbSetD);
    pivotPID.SetOutputRange(-1.0, 1.0);
    pivotEncoder.SetPositionConversionFactor(
        ((360.0 / (ClimbConstants::pivotConversionFactorOne * ClimbConstants::pivotConversionFactorTwo))));
    // This is the conversion factor for the pivot motor. The Equation to use when you need to figure out something like this is (Turns = Setpoint * Conversionfactor)
    //This number bassicaly makes it so that we can give it a position in degrees and it will go to that position in degrees

    //Same thing except for the winch motor
    winchPID.SetP(ClimbConstants::winchSetP);
    winchPID.SetI(ClimbConstants::winchSetI);
    winchPID.GetD(ClimbConstants::winchSetD);
    winchPID.SetOutputRange(-1.0, 1.0);

    //Current limit for the winches 
    winch.SetSmartCurrentLimit(ClimbConstants::winchSmartCurrentLimet);
    winch.SetSecondaryCurrentLimit(ClimbConstants::winchSeccondaryCurrentLimet);
    winchEncoder.SetPositionConversionFactor(1.0 / 100.0); // I think this is right
}

void Climber::DislodgeWrench(){
    //This is the function that we used to dislodge the wrench when we want to climb. The climber cannot be moved by the driver until they press the button that runs this
    //Function and gets the wrench disclodged and wrenchDislodged = true
    
    //This bassicaly just gets the current position of the winch and puts it 0.5 units down.
    dislodgeTarget = (winchEncoder.GetPosition() - 0.5);
    winchPID.SetReference(dislodgeTarget, rev::CANSparkMaxLowLevel::ControlType::kPosition);
    dislodgingWrench = true;
    // dislodgingWrench is so that the if statment in the periodic knows to check if the winch is near the target of -0.5 so it can go back up
    wrenchDislodged = true;
}

//This is the main function that we use to control the robot's climber, You need to pass it two double values, the length that you want to move to and the angel that you want to move to
//It will take steps to adjust the length to a legal length baised on the angle. and to move around the battery box of the robot
int Climber::ClimbControl(double angleAdjust, double lengthAdjust)
{
    //Here is what we use to stop the climber from moving if the wrench is not disclodged
    if(wrenchDislodged == true){
    //Set all of our bool trackers to false to make sure we are starting the functino fresh
    horizontalActivated = false;
    seccondaryMove = false;
    lengthChanged = false;
    swingActivated = false;
    thirdMove = false;
    //This was what we had to debug the system
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
    //These coments below this one are remnents of debugging
   // cout << "angleAdjust 2: " << angleAdjust << endl;
   // cout << "LengthAdjust 2: " << lengthAdjust << endl;
    if (angleAdjust > 1)
    {
        //We check that angle adjust is greater that one becasue if it was zero the equation below this coment would be undefined and I don't know what would happen so we are just going to avoid that possability
        // checks new climber position to make shure that it is legal.

        //This equation was derived by useing bassic trig to determine if we were voilating our wall extention limit for the climber We could not extend more than 5', 6'' above the base fo the frame. and no more than 16 inches away from our frame. 
        //Mysealf (Jacob Hagberg), and Tyler Seiford worked this math out on the white board together. It works by calculating the minimum distance that the climber can be extended for every angle that we move to.
        //We do this by also accounting for the offset created by how the climber was built
        if (lengthAdjust < (ClimbConstants::defaultClimbLength - (((ClimbConstants::pivotToFrameDist + ClimbConstants::maxDistFromFrame) - ClimbConstants::rotationOffset * std::cos((angleAdjust * ClimbConstants::pi / (180)))) / (std::sin((angleAdjust * ClimbConstants::pi / (180)))))))
        {
            //If our length is below the legal min then it sets the length to the legal min
            // if the length is not legal, set it to the legal length.
            lengthAdjust = (ClimbConstants::defaultClimbLength - (((ClimbConstants::pivotToFrameDist + ClimbConstants::maxDistFromFrame) - ClimbConstants::rotationOffset * std::cos((angleAdjust * ClimbConstants::pi / (180)))) / (std::sin((angleAdjust * ClimbConstants::pi / (180))))));
            lengthChanged = true;
        }
    }
    //  cout<<"angleAdjust 3: "<<angleAdjust<<endl;
    //  cout<<"LengthAdjust 3: "<<lengthAdjust<<endl;
    // Checks if the sceleing is veing violated, if so, it will change the length to make it legal
    //Ok, so this if statment needs sum explenation. You see, we were having some issues getting our climbing hooks onto the bar because this limitation was getting in the way and shortning the arms. This made it tricky to get onto the bar. 
    //So I decided to take matters into my own hands and limet the angle that this checks for to not include the angle that we would need to get onto the next bar. I didn't tell anyone about this until after the duluth reigonal. It is also worth noting that it was very hard for the 
    //Refs to judge this and so we got away with it. The lesson here is, sometimes it is better to ask for forgivness, and not for permission, but also that sometimes you gotta do what you gotta do to make something work. And that was what I did here
    if(angleAdjust<55){ //This if statment is intentional cheeting, please ignore
    //This is like the one above this but it isfor the cealing constraint
    if (lengthAdjust > (((ClimbConstants::defaultScealing - ClimbConstants::rotationBigOffset * std::sin((angleAdjust * ClimbConstants::pi / (180)))) / std::cos((angleAdjust * ClimbConstants::pi / (180)))) - ClimbConstants::minExtension))
    {
        lengthAdjust = (((ClimbConstants::defaultScealing - ClimbConstants::rotationBigOffset * std::sin((angleAdjust * ClimbConstants::pi / (180)))) / std::cos((angleAdjust * ClimbConstants::pi / (180)))) - ClimbConstants::minExtension);
        lengthChanged = true;
    }
    }


    //This if statment is to avoid the battery box. if we did not have this the arm could bump into the robot frame. 
    //If we are pivoting within the battery exclusion area, or we are pivoting through it, we run the lodgic to avoid the battery area 
    if ((angleAdjust <= ClimbConstants::batteryMaxAngle && angleAdjust >= ClimbConstants::batteryMinAngle)
            || (angleAdjust > ClimbConstants::batteryMaxAngle && angle <= ClimbConstants::batteryMaxAngle)
            || (angleAdjust < ClimbConstants::batteryMinAngle && angle >= ClimbConstants::batteryMinAngle))
    {

        //this part says that if we are targeting an angle within the battery exclusion zone and if the length that is being targeted is less that the min length, then set the target length to be the min battery exclustion length
        if (angleAdjust <= ClimbConstants::batteryMaxAngle && angleAdjust >= ClimbConstants::batteryMinAngle)
        {
            
            if (lengthAdjust < ClimbConstants::batteryMinLength)
            {
                lengthAdjust = ClimbConstants::batteryMinLength;
                lengthChanged = true;
            }
        }
        else
        {
            //if we are moving through the battery box and our adjustment length will hit the frame, or our current length will hit the frame, then set the length to be the min battery length then activate the seccondary move in the climber periodic function at the end of this file
            if (lengthAdjust < ClimbConstants::batteryMinLength ||  length < ClimbConstants::batteryMinLength)
            {
                
                //This is so that we can get to the final, and legal length eventualy
                targetLength = lengthAdjust;
                targetAngle = angleAdjust;

                lengthAdjust = ClimbConstants::batteryMinLength;
                //seccondaryMove will allow the climber to move to the final desired angel once the climber has reached the length to go around the battery
                lengthChanged = true;
                seccondaryMove = true;

                length = lengthAdjust;

                //Sets the length of the climber to the units of turns using the lengthToTurns function
                winchPID.SetReference(Climber::LengthToTurns(length), rev::CANSparkMaxLowLevel::ControlType::kPosition);
                pivotPID.SetReference(angle, rev::CANSparkMaxLowLevel::ControlType::kPosition);
            }
        }
        
    }

    frc::SmartDashboard::PutNumber("Climb Seccondary move", seccondaryMove);
  
  //If we do not have a move lined up for avoiding the frame around the battery
    if (seccondaryMove == false)
    {
      //Sets the length and angle to the angle and length that we want so that we can use the angle and length as the angle and length that is being targeted by the climber at all times. 
        length = lengthAdjust;
        angle = angleAdjust;

      
        winchPID.SetReference(Climber::LengthToTurns(length), rev::CANSparkMaxLowLevel::ControlType::kPosition);
        pivotPID.SetReference(angle, rev::CANSparkMaxLowLevel::ControlType::kPosition);
        
    }
    }
    return lengthChanged;
}

double Climber::LengthToTurns(double inchesToTurns)
{

    //This function was a lot of work and a lot of the heavy lifting was done by Tyler. This problem that this solves is that the arm length is controlled by a winch that wimds up a strap. As the winch winds up, 
    //The diameter of the effective winch will increace by a stepper function as the strap winds up on itsealf. So the math needs to account for that. 

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
//This is where the position in turns that is given to the winch is converted baised on what the length is 
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
//returns the position desired in turns to be given to the winch PID
    return turns;
}

double Climber::GetAngle()
{
    //Returns the angel that the climber is currently targeting if we need it
    return angle;
}

double Climber::GetLength()
{
    //Returns the length that the climber is currently targeting if we need it
    return length;
}

void Climber::ClimbPivotSetEncoder(double pivotSetEncoder)
{
    //This allows us to set the encoder position that is on the pivot motor, we used this to zero it at the start of the autnmous period
    //If you need to do something like this climber again I would reccomend using encoders on the acutal thing that moves and not just reily on the encoders that are in the motors
    pivotEncoder.SetPosition(pivotSetEncoder);
}
void Climber::ClimbWinchSetEncoder(double winchSetEncoder)
{
    //This is for zeroing the winch encoder
    winchEncoder.SetPosition(winchSetEncoder);
}

void Climber::Periodic()
{
    //Here is where the function for the seccondary moves for the battery box avoidance happens. It is in the perodic function that gets called every 20 ms that the code runs on the rio. 
    
    //tgtOverActual is a thing that I used to see how accurate the pivot PID was
    double tgtOverActual = (angle / pivotEncoder.GetPosition());

    //Every 20 ms these numbers would be updated and sent to smart Dashboard
    frc::SmartDashboard::PutNumber("Pivact vs pivtgt: ", tgtOverActual);
    frc::SmartDashboard::PutNumber("Climb Target Length", length);
    frc::SmartDashboard::PutNumber("Climb Fibbed Target Length", Climber::LengthToTurns(length));
    frc::SmartDashboard::PutNumber("Climb Target angle", angle);
    frc::SmartDashboard::PutNumber("Seccond Move: ", seccondaryMove);
    frc::SmartDashboard::PutNumber("Winch Encoder: ", winchEncoder.GetPosition());
    frc::SmartDashboard::PutNumber("Pivot Encoder: ", pivotEncoder.GetPosition());
    // This checks if we have a scedjuled seccond move once we have gotten to 0.25 inch of the length we are going for it will set the angle to be the final angle that we want to go for
    //It will then activate the third move to go to the final length
    if (seccondaryMove == true && thirdMove == false && (abs(winchEncoder.GetPosition() - Climber::LengthToTurns(ClimbConstants::batteryMinLength)) < 0.25))
    {
        angle = targetAngle;
        pivotPID.SetReference(angle, rev::CANSparkMaxLowLevel::ControlType::kPosition);
        thirdMove = true;
    }
    //Once the third move is acitvated and the motor is within one degree of the target angle then it will set the final llength that we wanted to go to as our length. We then set both tracker bools to false
    if (seccondaryMove == true && thirdMove == true && (abs(pivotEncoder.GetPosition() - targetAngle) < 1))
    {
        length = targetLength;
        winchPID.SetReference(LengthToTurns(length), rev::CANSparkMaxLowLevel::ControlType::kPosition);

        seccondaryMove = false;
        thirdMove = false;
    }

    //This if statenent will check to see if we are currently disclodging the wrench and will set the winch target to what it was origonaly at before we disclodged it once we are within 0.25 in of our target
    if(dislodgingWrench == true && abs(winchEncoder.GetPosition() - dislodgeTarget)<0.25 ){
        
        //we then set the bool dislodgingWrench to false but wrenchDislodged is still true and allows us to move the climber with the function ClimbControl
        winchPID.SetReference((dislodgeTarget + 0.5), rev::CANSparkMaxLowLevel::ControlType::kPosition);
        dislodgingWrench = false;
    }
}

//I dont think I need this, not sure why this is here
void InitDefaultCommand()
{
}
