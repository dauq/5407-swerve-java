package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Inputs;
import frc.robot.RampPower;

import java.util.Map;

import javax.lang.model.util.ElementScanner6;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.Constants.OIConstants;


public class ClimbSubsystem extends SubsystemBase {


    private static CANSparkMax mLift1 = new CANSparkMax(12, MotorType.kBrushless);
    private static CANSparkMax mLift2 = new CANSparkMax(13, MotorType.kBrushless);
    private static CANSparkMax mDart = new CANSparkMax(14, MotorType.kBrushless);
    private static RampPower rampDart = new RampPower( .8, .2, .25);

    private static DigitalInput mLift1IsDownProximity = new DigitalInput(7); 
    private static DigitalInput mLift1SeesRetroReflectTape = new DigitalInput(Constants.ClimberConstants.kLift1RRBannerDigitalPort); 

    private static RelativeEncoder mLift1Encoder = mLift1.getEncoder(); // part of NEO motor

    private static AnalogPotentiometer mLift2Potentiometer = new AnalogPotentiometer(new AnalogInput(2));
    
    private static double lastHandPotentiomenterValue = 0.0;
    private static int loopsOfHoldingHandPotentiometerValue = 0;
    private static boolean holdingHandPotentiometerValue = false;

    private static AnalogPotentiometer mDartPotentiometer = new AnalogPotentiometer(new AnalogInput(1));

    private static double lastDartPotentiomenterValue = 0.0;
    private static int loopsOfHoldingDartPotentiometerValue = 0;
    private static boolean holdingDartPotentiometerValue = false;

    private static Servo mLift1ReleaseServo = new Servo(0);  // release Lift1 at the end game

    private static Servo mLift1LockServo = new Servo(2);     // serevo to lock Lift1 so it will not drop. 


    public static double mLift1MotorRRPositOffset = 0.0;        // this offset is used to change the scale of the Lift1 encoder
                                                                // it uses retroreflective tape and a banner sensor. If the sensor
                                                                // see the tape this will set the offset to the current position
                                                                // of the encoder. Once it stops seeign the tape, we now can use
                                                                // it to accuratley know here L1 is above the tape. 
    public static double mLift1FullDownPosit = 0.0;
    public static boolean mLift1DownHasBeenReached = false;
    public static boolean mLift1Lock = false;                   // default starting position
                                                                // must reset robot to unlock

    double servoPosition = 1;
    boolean servoButtonReleased = true;
    

    public static boolean useSlowSpeed = false;

    public ClimbSubsystem() {

        mLift1.setIdleMode(CANSparkMax.IdleMode.kBrake);
        mLift2.setIdleMode(CANSparkMax.IdleMode.kBrake);

    }

    public double updateMap(Double input, Double left, Double right, Double mappedLeft, Double mappedRight){
        double output = mappedLeft+((input-left)*(mappedRight-mappedLeft)/(right-left));

        return output;
    }

    public static void reloadConfig(){


    }

    public static double checkLift1MotorRetroReflectivePosit(){

        /*******************************************************************************
         * we see the tape, we use NOT here because this is a digital channel
         * is in normally open mode. Open circuit is true, closed is false. 
         * if we see the tape the circuit is closed so it is false
         * ! or NOT chnages it to true. 
         * 
         * If true ( see the tape ) we take the current encoder setting and svae it 
         * as an offset. We keep doign this until we stop seeig the tape. 
         * 
         * Then the encoder position minus the offset gives us a true position. 
        ********************************************************************************/

        double posit = 0.0;

        if( !mLift1SeesRetroReflectTape.get() == true){                                 // we see the tape
            mLift1MotorRRPositOffset = mLift1Encoder.getPosition();                     // set encoder posit to offset. 
            posit = 0.0;                                                                // return 0 posit
        }else{
            posit = mLift1Encoder.getPosition() - mLift1MotorRRPositOffset;       // we do not see tape, now get a useful posit
                                                                                        // 1.0 - flips the numbers to be negative
                                                                                        // tower is going down. 
        }

        Constants.telemetry.putTrueBoolean("CLIMB Lift1 Retro Ref State", !mLift1SeesRetroReflectTape.get(), false);
        Constants.telemetry.putNumber("CLIMB Lift1 Retro Ref Posit", posit, false);

        return posit;

    }

    public void climb() {

		Constants.telemetry.putTrueBoolean("CLIMB Limit Switch", mLift1IsDownProximity.get(), false);

        
        if( !Inputs.masterEndgameArm ){
            mLift1Lock = false;
            setLift1Lock();
            return;
        }
    
        // Process Lift 1 stuff here. This lifts the robot to the first bar. 
        if( Inputs.climberLift1Release){        // sets the position of the L1 lock to prevernt the robot from dropping after power off.
            mLift1ReleaseServo.set(.3);
        }else{
            mLift1ReleaseServo.set(.7);
        }

        // this will release the Lift 1 to the full up position ready to climb. 
        if( Inputs.climberLift1Release == true){ // operator is releasing the Lift 1 stage, ready to climb

            mLift1ResetDownPosition();          // reset that we were in the down position
                                                // do this just in case we pulled down accidently before end game. 
                                                // we do not lock the climber, that is done in the Automation only

            mLift1Lock = false;                 // unlock the lift1 lock, make sure
        }

        checkLift1MotorRetroReflectivePosit();    // call it to process the offset so we can use it in the CLIMB Automation
                                                // later we will call it agin when in the climb

        setLift1Lock();                                 // set the position of the lift1 lock based upon mLift1Lock

        double mLift1speed  = Inputs.climberLift1PowerOverride != 0.0 
                   ?  Inputs.climberLift1PowerOverride : Constants.ClimberConstants.kLiftNormalSpeed;

        String status = "unk";

        if(Inputs.climberLift1Up) {                      
            if( mLift1Lock == true){                   
                status = "Locked!";
            }else{
                moveLift1(mLift1speed);
                status = "/\\ UP";
            }
        }else if(Inputs.climberLift1Down) {          
                moveLift1(-mLift1speed);
                status = "\\/ Down";
        } else {
            moveLift1(0.0);
            status = "Stop";
        }   

        Constants.telemetry.putString("CLIMB Lift1 Status", status, false);

        
        double mLift2speed  = Inputs.climberLift2PowerOverride != 0.0 
                            ?  Inputs.climberLift2PowerOverride : Constants.ClimberConstants.kLiftNormalSpeed;

        if(Inputs.climberLift2Up) {                     
            moveLift2(mLift2speed);
        } else if(Inputs.climberLift2Down) {          
            moveLift2(-mLift2speed);
        } else {
            moveLift2(0.0);
        }


        double mDartSpeed = Inputs.climberDartPowerOverride != 0.0 
                ?  Inputs.climberDartPowerOverride : Constants.ClimberConstants.kDartNormalSpeed;
        
        if(Inputs.climberDartUp) {                   
            moveDart(mDartSpeed);
        }else if(Inputs.climberDartDown) {             
            moveDart(-mDartSpeed);
        } else {
            moveDart(0.0);
        }



    }

    public static void setLift1Lock(){ 
        //This function turns the small motor from two positions, this determines whether the gear is stuck/unstuck.
        //If the motor is in the gear, the gear will only be able to go counter-clockwise, making the m1 lift unable to go up
        double power = 0.0;
        String status = "UNK";
        if(mLift1Lock == false){
            power = .9;
            status = "Unlock";
        } else{
            power = .1;
            status = "Lock";
        }
        Constants.telemetry.putNumber("CLIMB Lift1 Lock Power", power, false);
        Constants.telemetry.putString("CLIMB Lift1 Lock Status", status, false);
        mLift1LockServo.set(power);
    }

    public static double getLift2Position(){
        double temp = mLift2Potentiometer.get() + Constants.ClimberConstants.kLift2EncoderAdjust;
        Constants.telemetry.putNumber("CLIMB Hand (L2) Posit", temp);
        return temp;

    }

    public static double getDartPosition(){
        double temp = 1.0 - mDartPotentiometer.get() + Constants.ClimberConstants.kDartEncoderAdjust;
        
        Constants.telemetry.putNumber("CLIMB DART Posit", temp, false);
        return temp;

    }

    private double handleDeadband(double value){
        return Math.abs(value) >= 0.1? value : 0; //creating a deadzone for controls
    }

    private void moveLift1(double power){

        double deadbandPower = handleDeadband(power);
        boolean isDown = isLift1Down();

        if( deadbandPower < 0.0 && isDown == true){  // test if lift1 is hitting prox sensor. 
            deadbandPower = 0.0;							// stop
        }

        mLift1.set(-deadbandPower);   

        Constants.telemetry.putNumber("CLIMB Lift1 Power", deadbandPower, false);

    }

    public static void mLift1ResetDownPosition(){      // reset this do we can find it again while pulling up
        mLift1FullDownPosit = 0.0;
        mLift1DownHasBeenReached = false;
        mLift1Lock = false;
    }

   
    public static boolean isLift1Down(){
        boolean isDown = mLift1IsDownProximity.get();   // seeing if we are hitting the sensor
        double posit = mLift1Encoder.getPosition();

        if( isDown == true ){                          // we are full down, record this spot
            mLift1FullDownPosit = posit;
            mLift1DownHasBeenReached = true;
            mLift1Lock = true;
        }

        Constants.telemetry.putTrueBoolean("CLIMB Lift1 Is Down", isDown, false);
        Constants.telemetry.putTrueBoolean("CLIMB Lift1 Full Down Reached", mLift1DownHasBeenReached, false);
        Constants.telemetry.putNumber("CLIMB Lift1 Down Posit", mLift1FullDownPosit, false);

        return isDown;
    }

    private void moveLift2(double power){
        double inBound = Constants.ClimberConstants.kLift2FullInPosit;              // full in close to robot (up)
        double outBound = Constants.ClimberConstants.kLift2FullOutPosit ;           // full out at end of bar (down)

        double position = getLift2Position();

        double deadbandPower = handleDeadband(power);
        String status = "---";

        if( deadbandPower > 0 && position > inBound ) {            // test the boundries
            status = "[IN";
            deadbandPower = 0.0;
        } else if( deadbandPower < 0 && position < outBound ) {    // test the boundries
            deadbandPower = 0.0;
            status = "OUT]";
        } else if( deadbandPower > 0  )
            status = "<<< IN";
        else if( deadbandPower < 0 )
            status = ">>> Out";

        if (Math.abs(position - inBound) < 0.02 || Math.abs(position - outBound) < 0.02) {
            deadbandPower *= 0.5;
        }

        mLift2.set(deadbandPower);
        Constants.telemetry.putString("CLIMB Hand (L2) Status", status, false);
        Constants.telemetry.putNumber("CLIMB Hand (L2) Power", deadbandPower);

        SmartDashboard.putNumber("hand position", getLift2Position());

    }


    private void moveDart(double power){
        double lowerBound = Constants.ClimberConstants.kDartFullDownPosit;
        double upperBound = Constants.ClimberConstants.kDartFullUpPosit; 
        
        double position = getDartPosition();
        double deadbandPower = handleDeadband(power);


        if (   (position <= upperBound && deadbandPower > 0)  ||             // trying to go up, but above upper bound
               (position >= lowerBound && deadbandPower < 0)  ||             // trying to go down, but below lower bound
                                          deadbandPower == 0 )   {           // or in deadband
            deadbandPower = 0.0;                                             // out of bounds, kill the power
        }

        SmartDashboard.putNumber("dart position", getDartPosition());

        deadbandPower = rampDart.getRampedPower(deadbandPower);

        if (Math.abs(position - upperBound) <= 0.02 || Math.abs(position - lowerBound) <= 0.02) {
            deadbandPower *= 0.5;
        }

        mDart.set(deadbandPower);
        Constants.telemetry.putNumber("CLIMB DART Power", deadbandPower, false);

    }
}