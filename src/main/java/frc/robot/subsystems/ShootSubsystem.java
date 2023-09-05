package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Inputs;
import frc.robot.ProportionalResponse;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;

import frc.robot.Constants.ShooterConstants;


import edu.wpi.first.math.controller.ProfiledPIDController;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

import edu.wpi.first.networktables.NetworkTable;

import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj.Servo;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;

import com.revrobotics.ColorSensorV3;


import edu.wpi.first.wpilibj.util.Color;

import com.kauailabs.navx.frc.AHRS;

public class ShootSubsystem extends SubsystemBase {

    private AHRS m_navx = new AHRS(SPI.Port.kMXP);

    private final Joystick m_extraJoystick = new Joystick(3);
    private final TalonFX m_shooterMotor = new TalonFX(41);
    private final CANSparkMax m_backShooterMotor = new CANSparkMax(44, MotorType.kBrushless);


    private final CANSparkMax m_turret = new CANSparkMax(42, MotorType.kBrushless);
    private final AnalogPotentiometer turretTurnPotentiometer = new AnalogPotentiometer(new AnalogInput(3)); //was on port 0
    
    private static double lastTurretPotentiomenterValue = 0.0;
    private static int loopsOfHoldingTurretPotentiometerValue = 0;
    private static boolean holdingTurretPotentiometerValue = false;



    private final CANSparkMax m_gibRoller = new CANSparkMax(43, MotorType.kBrushless);



    private static final Servo m_hoodServo = new Servo(1);


    private final I2C.Port i2cPort = I2C.Port.kOnboard;
    private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);


    public static double limelightXOffset = 0.0;
    public static double limelightYOffset = 0.0;
    public static double limelightArea = 0.0;
    public static boolean limelightSeesATarget = false;

    public static double limelightDistanceToTarget = 0.0;

    public static double stationaryLimelightXOffset = 0.0;
    public static double stationaryLimelightDistanceToTarget = 0.0; 

    private static boolean turretIsOnTarget = false;
    private static boolean hoodIsOnTarget = false;
    private static boolean shooterIsAtSpeed = false;

    private static boolean shooterInFullAuto = false;
    private static boolean shooterWheelAuto = false;
    private static boolean shooterTargettingAuto = false;

    private static boolean redMode = false;
    private static double preRedModeAngle = 0.0;

    private static double hoodAngle = 0.5;
    private static double frontMotorPercentage = 0.0;
    private static double backMotorPercentage = 0.0;

    public static double shooterDesiredPIDVelocity = 0.0; // how fast lime light wants us to go
    
    public static double k = 0.0;

    public static double shooterMaxPIDVelocity = 
                    Constants.config.getDouble("SHOOTR_MaxPIDVelocity", 8000); // Max velocity  of shooter
    
    
    private static String robotColor = DriverStation.getAlliance().toString().toLowerCase(); // get from driver station




    private final ProfiledPIDController m_turretPIDController = new ProfiledPIDController(
        Constants.ShooterConstants.kP_TurretValue,        //P
        Constants.ShooterConstants.kI_TurretValue,        //I
        Constants.ShooterConstants.kD_TurretValue,        //D
        new TrapezoidProfile.Constraints(
            1.0,//1.0,
            0.0
        )
    );
    
    private final ProfiledPIDController m_turretPIDBoundController = new ProfiledPIDController(
        Constants.ShooterConstants.kP_TurretValue / 2,        //P
        Constants.ShooterConstants.kI_TurretValue,        //I
        Constants.ShooterConstants.kD_TurretValue,        //D
        new TrapezoidProfile.Constraints(
            1.0,//1.0,
            0.0
        )
    );

    double value = 0.0;

    double shooterPowerOffset = 0.0;

    public ShootSubsystem() {
        resetGyro();
        

        lastTurretPotentiomenterValue = turretTurnPotentiometer.get();

        m_shooterMotor.config_kF(0, Constants.ShooterConstants.kF_PidTuningValue, 30);
        m_shooterMotor.config_kP(0, Constants.ShooterConstants.kP_PidTuningValue, 30);
        m_shooterMotor.config_kI(0, Constants.ShooterConstants.kI_PidTuningValue, 30);
        m_shooterMotor.config_kD(0, Constants.ShooterConstants.kD_PidTuningValue, 30);
    }

    public double boundPower(double power, double lowerBound, double upperBound) {
        return Math.max(Math.min(power, upperBound), lowerBound);
    }

    public double returnTurretPowerToAngle(double angle){
        double turretPower = 0.0;
        
        double currentAngle = getActualTurretAngle();
        double powerToLeft = m_turretPIDBoundController.calculate(currentAngle, Constants.ShooterConstants.kTurretFullLeft);
        double powerToRight = m_turretPIDBoundController.calculate(currentAngle, Constants.ShooterConstants.kTurretExtendedRight);

        if (Inputs.masterEndgameArm) {
            powerToRight = m_turretPIDBoundController.calculate(currentAngle, Constants.ShooterConstants.kTurretFullRight);
        }

        powerToLeft = boundPower(powerToLeft, -1.0, 1.0);
        powerToRight = boundPower(powerToRight, -1.0, 1.0);

        turretPower = m_turretPIDController.calculate(currentAngle, angle);

        turretPower = boundPower(turretPower, powerToLeft, powerToRight);



        return turretPower;
    }

    



    public double updateMap(Double input, Double left, Double right, Double mappedLeft, Double mappedRight) {
        double output = mappedLeft + ((input - left) * (mappedRight - mappedLeft) / (right - left));

        return output;
    }

    public double getActualTurretAngle() {
        double temp = turretTurnPotentiometer.get();
        Constants.telemetry.putNumber("Turret Position", temp);
        return temp;

        
    }

    public double returnActualShooterSpeed() {   //Returns physical speed of shooter in RPM

        double temp = m_shooterMotor.getSelectedSensorVelocity(0);  // pid loop 0
        Constants.telemetry.putNumber("SHOOTR Velocity", temp);
        return temp;
    }

    public double returnIdealHoodPosition(double distanceToTarget) {

        
        distanceToTarget = Math.max(Math.min(distanceToTarget, 3.5), 0.0);

        hoodAngle = updateMap(distanceToTarget, 0.0, 3.5, 1.0, -1.0);
        hoodAngle = updateMap(hoodAngle, 1.0, -1.0, 0.3, 0.6);

        SmartDashboard.putNumber("hood angle", hoodAngle);

        if (Inputs.shooterWallShot) {
            return 0.28; //was 0.28 before 4/6/22, was 0.285 before lehigh
        }
        
        return hoodAngle;
    }

    public void resetGyro(){
        m_navx.setAngleAdjustment(0);
        m_navx.zeroYaw();
      }

    public void shoot() {
        readLimeLightValues();


        double driveXVelocityOffset = 5.0;
        double driveYVelocityOffset = 7.0;
        double turnAngleOffset = 0.362;

        double robotAngleOffsetFromTarget = 0.0;

        if( Inputs.driveResetGyro == true)
            resetGyro();

        double gyroAngle = m_navx.getAngle();


        double[] pointA = new double[2]; 
        double[] pointB = new double[2]; 
        double[] pointC = new double[2]; 
        double[] pointD = new double[2]; 

        double angle = -gyroAngle + 500 * (Constants.ShooterConstants.kTurretFullRight - getActualTurretAngle()) - stationaryLimelightXOffset;
        double radian = angle * Math.PI / 180;

        double appliedPowerX = Inputs.driverStrafe * Math.abs(Inputs.driverStrafe);
        double appliedPowerY = Inputs.driverPower * Math.abs(Inputs.driverPower);

        pointA[0] = 0.0;
        pointA[1] = 0.0;

        pointB[0] = 0.0;
        pointB[1] = stationaryLimelightDistanceToTarget;

        pointC[0] = pointB[0] - Math.cos(radian) * appliedPowerX * driveXVelocityOffset;
        pointC[1] = pointB[1] + Math.sin(radian) * appliedPowerX * driveYVelocityOffset;

        pointD[0] = pointC[0] - Math.sin(radian) * appliedPowerY * driveXVelocityOffset;
        pointD[1] = pointC[1] - Math.cos(radian) * appliedPowerY * driveYVelocityOffset;





        limelightDistanceToTarget = Math.sqrt(Math.pow(pointD[0], 2) + Math.pow(pointD[1], 2));
        
        double omega = Math.atan(pointD[0] / pointD[1]);

        limelightXOffset = stationaryLimelightXOffset + omega * Math.PI / 180;


       


        double limelightAngularWidth = 25.0;
        if (Math.abs(omega) > limelightAngularWidth) {
            limelightDistanceToTarget = stationaryLimelightDistanceToTarget;
            limelightXOffset = stationaryLimelightXOffset;
        } else {

        }





        //CODE TO GET RID OF DRIVING WHILE MOVING
        limelightDistanceToTarget = stationaryLimelightDistanceToTarget;
        limelightXOffset = stationaryLimelightXOffset;



        m_hoodServo.setPosition(returnIdealHoodPosition(limelightDistanceToTarget));

        /***********************************************************************
         * Switching auto modes Want to use seperate buttons to be sure operatro does it
         * right Toggling is nice but dangerous
         ************************************************************************/
        if (Inputs.shooterFullAutoModeOff) { // only switched when button pressed
            shooterInFullAuto = false;
        } else if (Inputs.shooterFullAutoModeOn) {
            shooterInFullAuto = true;
        } // only switched when button pressed



        Constants.telemetry.putTrueBoolean("SHOOTR Full Auto", shooterInFullAuto, true);
        Constants.telemetry.putTrueBoolean("SHOOTR Wheel Auto", shooterWheelAuto);
        Constants.telemetry.putTrueBoolean("SHOOTR Target Auto", shooterTargettingAuto);


        hoodIsOnTarget = false;
        shooterIsAtSpeed = false;
        
        // Reads all the limelight values.

        moveTurret("none");
        moveHood();


        SmartDashboard.putNumber("Shooter Power Offset", shooterPowerOffset);

        if (!Inputs.masterEndgameArm) {
            if (m_extraJoystick.getRawButtonPressed(8)) {
                shooterPowerOffset += 0.01;
            }
            if (m_extraJoystick.getRawButtonPressed(7)) {
                shooterPowerOffset -= 0.01;
            }   
        }

        if (!Inputs.masterEndgameArm) { // managed in inputs, extraControl.getRawButton(1) == false
            if (shooterInFullAuto ) {
                Constants.telemetry.putNumber("SHOOTR Input Power", Inputs.shooterManualPower, false);

                SmartDashboard.putNumber("distance", limelightDistanceToTarget);




                double frontV = 370.0 /*362.93*/ * (limelightDistanceToTarget * limelightDistanceToTarget) - 1624.7 * limelightDistanceToTarget + /*10459*/ 9600 + (frontMotorPercentage + shooterPowerOffset) * 22834;
                double backV = 0.0283 * limelightDistanceToTarget + .773 + (backMotorPercentage + shooterPowerOffset * 2.0);

                SmartDashboard.putNumber("front motor percentage", frontMotorPercentage);
                SmartDashboard.putNumber("front motor velocity", m_shooterMotor.getSelectedSensorVelocity(0));

                SmartDashboard.putNumber("back motor percentage", -backMotorPercentage);
                SmartDashboard.putNumber("back motor velocity", -m_backShooterMotor.get());

                if (Inputs.overrideShooterMotors) {
                    frontV = Inputs.overrideFrontShooterMotorPower;
                    backV = Inputs.overrideBackShooterMotorPower;
                }

                Constants.telemetry.putNumber("SHOOTR AUTO Desired Velocity", frontV, true);


                if (Inputs.shooterWallShot) {
                    double c = 9800; 

                    m_shooterMotor.set(ControlMode.Velocity, c);
                    m_backShooterMotor.set(-0.55);

                    frontV = c;
                } else if (Inputs.shooterWeakShot){
                    m_shooterMotor.set(ControlMode.PercentOutput, 0.1);
                    m_backShooterMotor.set(-0.2);
                } else {
                    m_shooterMotor.set(ControlMode.Velocity, frontV);
                    m_backShooterMotor.set(-backV);
                }

                

                shooterIsAtSpeed = (Math.abs(frontV - returnActualShooterSpeed()) <= 75);
                if (Inputs.shooterWallShot) {
                    shooterIsAtSpeed = (Math.abs(frontV - returnActualShooterSpeed()) <= 50);
                }


                Constants.telemetry.putNumber("SHOOTR Wanted Velocity", frontV);
                Constants.telemetry.putNumber("SHOOTR Actual Velocity", returnActualShooterSpeed());
                Constants.telemetry.putTrueBoolean("SHOOTR At Speed", shooterIsAtSpeed);


            } else {
                m_shooterMotor.set(ControlMode.PercentOutput, Inputs.shooterManualPower);
                double c = 2.0;
                m_backShooterMotor.set(-Inputs.shooterManualPower * c);
            }
        } else {
            m_shooterMotor.set(ControlMode.PercentOutput, 0);
            Constants.telemetry.putNumber("SHOOTR Input Power", Inputs.shooterManualPower, false);
            m_backShooterMotor.set(0.0);
        }

        Constants.telemetry.putNumber("SHOOTR Current Velocity", m_shooterMotor.getSelectedSensorVelocity(), true);
        Constants.telemetry.putBoolean("B: Is operator trigger pressed?", Inputs.operatorTrigger, true);
        Constants.telemetry.putBoolean("B: Is there a target?", limelightSeesATarget, true);
        Constants.telemetry.putBoolean("B: Is turret on target?", turretIsOnTarget, true);
        Constants.telemetry.putBoolean("B: Is the shooter at the right speed?", shooterIsAtSpeed, true);

        if (Inputs.operatorTrigger && Inputs.operatorThumbSafety) { 
            m_gibRoller.set(1.0);

        } else if (Inputs.operatorTrigger && !Inputs.masterAutoEnabled && ((limelightSeesATarget && turretIsOnTarget) || Inputs.shooterWallShot) && shooterIsAtSpeed && !Inputs.robotOperatingModeEndGame) { // for now just check 4, still need to add hood at correct angle
            m_gibRoller.set(1.0);       // fire and all targets are good

  

        } else if (Inputs.operatorTrigger && Inputs.masterAutoEnabled && turretIsOnTarget && shooterIsAtSpeed && !Inputs.robotOperatingModeEndGame) {
            m_gibRoller.set(1.0);       // fire and all targets are good
        } else if (Inputs.shooterReverseGibRoller){
            m_gibRoller.set(-1.0);
            // will set the tony roller in reverse at the same time. 
        } else if (Inputs.shooterWeakShot){
            m_gibRoller.set(1.0);
        } else {
            m_gibRoller.set(0.0);
        }

 

    }

    public String returnIsAllianceColor(String ballColor) {
        String colorIsAllianceColor = "neither";
        if (robotColor.startsWith(ballColor)) {
                colorIsAllianceColor = "yes";
        } else if (ballColor != "none") {
            colorIsAllianceColor = "no";
        }

        Constants.telemetry.putString("SHOOTR BALL Is Alliance Color", colorIsAllianceColor, false);

        return colorIsAllianceColor;

    }

    public double handleDeadband(double value) {
        double deadband = 0.2;
        return Math.abs(value) >= deadband ? value : 0;
    }

    private double determineRobotTurn(){
        double rot = Inputs.driverTurn;

        if (Math.abs(rot) <= 0.075)
            rot = 0;  
    
        rot *= Math.abs(rot);

        if (overrideDriveSub() != 0){
            rot = overrideDriveSub();
        }

        return rot;
    }

    public double overrideDriveSub() {
        double idealTurretAngle = getActualTurretAngle() + ((limelightSeesATarget ? limelightXOffset : 0)  / 360);
        double idealTurretPower = 0.0;


        if (limelightSeesATarget) {
            idealTurretPower = returnTurretPowerToAngle(idealTurretAngle);
        } else {
            idealTurretPower = handleDeadband(Inputs.turretAdjust);
        }

        if (idealTurretPower < 0.0 && getActualTurretAngle() < Constants.ShooterConstants.kTurretFullLeft) {
            idealTurretPower = 0.0; // kill power as we are hard left stop
        } else if (idealTurretPower > 0.0 && getActualTurretAngle() > Constants.ShooterConstants.kTurretExtendedRight){
            idealTurretPower = 0.0; // kill power as we are hard right stop
        } else if (Inputs.masterEndgameArm && idealTurretPower > 0.0 && getActualTurretAngle() > Constants.ShooterConstants.kTurretFullRight) {
            idealTurretPower = 0.0; // kill power as we are hard right stop
        }

        double powerToLeft = m_turretPIDController.calculate(getActualTurretAngle(), Constants.ShooterConstants.kTurretFullLeft);
        double powerToRight = m_turretPIDController.calculate(getActualTurretAngle(), Constants.ShooterConstants.kTurretExtendedRight);
        
        if (Inputs.masterEndgameArm){
            m_turretPIDController.calculate(getActualTurretAngle(), Constants.ShooterConstants.kTurretFullRight);
        }

        idealTurretPower = boundPower(idealTurretPower, powerToLeft, powerToRight);

        double c = 0.5; 
        double p = 0.25;

        if (Math.abs(powerToLeft) < c && handleDeadband(Inputs.turretAdjust) < 0){
            return handleDeadband(Inputs.turretAdjust) * p;
        } else if (Math.abs(powerToRight) < c && handleDeadband(Inputs.turretAdjust) > 0){
            return handleDeadband(Inputs.turretAdjust) * p;
        } else {
            return 0;
        }

        
    }



    public void moveTurret(String ballColor) {

        double idealTurretAngle = getActualTurretAngle() + ((limelightSeesATarget ? limelightXOffset : 0)  / 360);
        double idealTurretPower = 0.0;

        if (Inputs.overrideTurret) {
            idealTurretAngle = Inputs.overrideTurretAngle;
        }

        


        if (limelightSeesATarget) {
            idealTurretPower = returnTurretPowerToAngle(idealTurretAngle);
        } else {
            idealTurretPower = handleDeadband(Inputs.turretAdjust);
        }

        if (idealTurretPower < 0.0 && getActualTurretAngle() < Constants.ShooterConstants.kTurretFullLeft) {
            idealTurretPower = 0.0; // kill power as we are hard left stop
        } else if (Inputs.masterEndgameArm && idealTurretPower > 0.0 && getActualTurretAngle() > Constants.ShooterConstants.kTurretFullRight){
            idealTurretPower = 0.0; // kill power as we are hard right stop
        } else if (idealTurretPower > 0.0 && getActualTurretAngle() > Constants.ShooterConstants.kTurretExtendedRight){
            idealTurretPower = 0.0; // kill power as we are hard right stop
        }

        double powerToLeft = m_turretPIDController.calculate(getActualTurretAngle(), Constants.ShooterConstants.kTurretFullLeft);
        double powerToRight = m_turretPIDController.calculate(getActualTurretAngle(), Constants.ShooterConstants.kTurretExtendedRight);

        if (Inputs.masterEndgameArm){
            powerToRight = m_turretPIDController.calculate(getActualTurretAngle(), Constants.ShooterConstants.kTurretFullRight);
        }

        if (Inputs.masterEndgameArm || Inputs.shooterWallShot) { //If we are either in endgame or we are using the short shot, make the turret straight
            idealTurretPower = powerToRight;
        } else {
            idealTurretPower = boundPower(idealTurretPower, powerToLeft, powerToRight);
        }

        Constants.telemetry.putNumber("turret position", getActualTurretAngle(), true);

        if (limelightSeesATarget) {
            idealTurretPower += determineRobotTurn() * (-4.0 + k);
        }
        


        idealTurretPower = boundPower(idealTurretPower, powerToLeft, powerToRight);
        Constants.telemetry.putNumber("turret power", idealTurretPower, true);



        m_turret.set(-idealTurretPower);

        SmartDashboard.putNumber("limelightXOffset", limelightXOffset);

        if (limelightSeesATarget && Math.abs(idealTurretAngle - getActualTurretAngle()) < 0.01) {
            turretIsOnTarget = true;
        } else {
            turretIsOnTarget = false;
        }
        


    }

    public void moveHood() {

        String status = "STOP"; // tell us what is happening for temetry
        hoodIsOnTarget = false;

        double angle = 0;
        double power = Inputs.hoodAdjust;



        power = Inputs.applyDeadBand(power, Constants.ShooterConstants.kHoodPowerDeadBand); // only really needed fro
                                                                                            // auto

        /*
         * test we are not going past the limits of the turret if(power > 0.0 && angle >
         * Constants.ShooterConstants.kHoodFullUp){ power = 0.0; // kill power as we are
         * hard left stop status = "TOP"; }else if(power < 0.0 && angle <
         * Constants.ShooterConstants.kHoodFullDown){ power = 0.0; // kill power as we
         * are hard right stop status = "BOTM";
         */

        if (power < 0.0)
            status = "<< DOWN";
        else if (power > 0.0)
            status = ">> UP";
        else if (hoodIsOnTarget)
            status = "[**]";


        Constants.telemetry.putNumber("HOOD Power", power);
        Constants.telemetry.putString("HOOD Status", status);
    }

    public void readLimeLightValues() {
        double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0); // valid target
        double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0); // dX
        double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0); // dY
        double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0); // target area
                                                                                                         // 0%-100%
        double ts = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ts").getDouble(0); // skew (-90deg
                                                                                                         // - 0 deg?)

        
                                                                                                         
        
        limelightYOffset = ty;
        limelightArea = ta;
        limelightSeesATarget = !(tv < 1);

        Constants.telemetry.putNumber("CAMERA X", limelightXOffset, false);
        Constants.telemetry.putNumber("CAMERA Y", limelightYOffset, false);
        Constants.telemetry.putNumber("CAMERA Area", limelightArea, false);
        Constants.telemetry.putTrueBoolean("CAMERA Sees Target", limelightSeesATarget, false);


        
        stationaryLimelightXOffset = tx - 2.5;
        stationaryLimelightDistanceToTarget = calculateDistanceToBasket(Math.toRadians(ty));
    }

    private double calculateDistanceToBasket(double radians) {
        double distanceMeters = (ShooterConstants.kDeltaHeightMeters
                / (Math.tan(ShooterConstants.kLimelightTiltAngleRadians + radians)));

        Constants.telemetry.putNumber("distance to target", distanceMeters, false);

        return distanceMeters;
    }

    public double autoTurretTarget(String ballColor){       // figure out turret power to be used based upon limelight

        double turretOnTargetWindow = .47;
        double maxPower = 0.3; 
        double power = 0.0;
        double slowZone = 8.0;
        String status = "Unk";
        double leftSlowZone = 0.5 + slowZone;
        double rightSlowZone = 0.5 - slowZone; 

        String colorIsAllianceColor = returnIsAllianceColor(ballColor);

        if (!redMode) {
            preRedModeAngle = getActualTurretAngle();
        }

       

        double redModeOffset = 0;


        double offsetToUse = 0.0;

        if (redMode){//colorIsAllianceColor == "no"){ //if ball isnt the allance color it misses
            if (Math.abs(getActualTurretAngle() - Constants.ShooterConstants.kTurretFullLeft) < Math.abs(getActualTurretAngle() - Constants.ShooterConstants.kTurretExtendedRight)) {
                redModeOffset = 0.1;
            } else {
                redModeOffset = -0.1;
            }

            offsetToUse = (preRedModeAngle + redModeOffset) - getActualTurretAngle();
        }  

        Constants.telemetry.putBoolean("in red mode", redMode, false);

        if (!redMode){
            if( Math.abs(limelightXOffset) < turretOnTargetWindow ){       // are we in the target zone?
                power = 0.0; 
                turretIsOnTarget = true;
                status = "[**]";
            } else if(limelightXOffset < 0.0 ){     // target is on the left of the camera
                power = -maxPower;
                status = "<< LEFT";
                turretIsOnTarget = false;
            } else if(limelightXOffset > 0.0 ){     // target is on the right of the camera
                power = maxPower;
                status = ">> RIGHT";
                turretIsOnTarget = false;
            }
            
            if((limelightXOffset < leftSlowZone) && 
            (limelightXOffset > rightSlowZone)){
                power *=0.1;
            }
        } else {
            if(Math.abs(offsetToUse) < 0.01){       // am I in the target zone?
                power = 0.0; 
                turretIsOnTarget = true;
                status = "[**]";
            } else if(offsetToUse < 0.0 ){     // target is on the left of the camera
                power = -maxPower;
                status = "<< LEFT";
                turretIsOnTarget = false;
            } else if(offsetToUse > 0.0 ){     // target is on the right of the camera
                power = maxPower;
                status = ">> RIGHT";
                turretIsOnTarget = false;
            }
            
            if(Math.abs(offsetToUse) <  0.05){
                power *=0.1;
            }
        }
        
        Constants.telemetry.putString("TURRET AUTO Status", status);
        Constants.telemetry.putNumber("TURRET AUTO Power", power);

        return power;

    }

    public void pointBlankShoot(){
        m_hoodServo.setPosition(Constants.ShooterConstants.kPointBlankHood);
        m_shooterMotor.set(ControlMode.Velocity, 13585);
        m_gibRoller.set(1.0);
        m_turret.set(returnTurretPowerToAngle(Constants.ShooterConstants.kTurretFullLeft));
    }
}