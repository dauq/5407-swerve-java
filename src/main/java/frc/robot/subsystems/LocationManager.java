package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Inputs;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.NetworkTableInstance;
/**This class deals with the location/positions of many variables within the robot. 
 * From where the robot is at relative to the field, to which pixle the target is at,
 * the location manager handles the complicated formulas used to calculate the location
 * and change certain variables as needed.
*/
public class LocationManager extends SubsystemBase{
    double x = 0.0;
    double y = 0.0;

    private AHRS m_navx = new AHRS(SPI.Port.kMXP);
    double gyroAngle = 0.0;

    boolean foundPosition = false;
    
    double stationaryLimelightXOffset = 0.0;
    double stationaryLimelightDistanceToTarget = 0.0;
    boolean limelightSeesATarget = false;

    double l = -48.0;

    double driveTime = 0.0;
    double drivePower = 0.0;
    int loopCount = 0;
    boolean begunDriveTest = false;

    boolean enableLimelight = true;

    boolean shooterInFullAuto = false;
    double shooterPowerOffset = 0.0;


    public ShootSubsystem m_robotShoot;
    private static Joystick m_extraControl    = new Joystick(3);

    public LocationManager(ShootSubsystem subsystem) {
        resetGyro();

        this.m_robotShoot = subsystem;
        foundPosition = false;
    }

    public void driveTest(){
        if (Inputs.masterAutoEnabled && loopCount < driveTime * 50 && begunDriveTest) {
            Inputs.driverPower = drivePower;
            loopCount += 1;
        } else if (Inputs.masterAutoEnabled && !begunDriveTest) {
            loopCount = 0;
            begunDriveTest = true;
        } else {
            Inputs.driverPower = 0.0;
        }

        if (!Inputs.masterAutoEnabled){
            begunDriveTest = false;
        }

        if (m_extraControl.getRawButtonReleased(10)){
            driveTime += 0.1;
        } else if (m_extraControl.getRawButtonReleased(9)){
            driveTime -= 0.1;
        }

        if (m_extraControl.getRawButtonReleased(12)){
            drivePower += 0.05;
        } else if (m_extraControl.getRawButtonReleased(11)){
            drivePower -= 0.05;
        }

        SmartDashboard.putNumber("POSTMAN: drive time", driveTime);
        SmartDashboard.putNumber("POSTMAN: drive power", drivePower);
        SmartDashboard.putNumber("POSTMAN: loop count", loopCount);
        SmartDashboard.putBoolean("POSTMAN: MASTER DRIVE TEST", Inputs.masterAutoEnabled);
        SmartDashboard.putBoolean("POSTMAN DRIVE TEST BEGUN", begunDriveTest);
    }

    public void loop(){


        SmartDashboard.putNumber("POSTMAN l", l);

        manageGyro();
        readLimeLightValues();

        manageLocation();

        double[] pointF = returnMovingTarget();

        if (foundPosition) {

            turnTurretToTarget(pointF[0], pointF[1]);
            setShooterPowerToTarget(pointF[0], pointF[1]);
            setHoodAngleToTarget(pointF[0], pointF[1]);
        }
    }

    public void manageGyro(){
        if( Inputs.driveResetGyro == true)
            resetGyro();

        gyroAngle = m_navx.getAngle();

        
    }

    public void resetGyro(){
        m_navx.setAngleAdjustment(0);
        m_navx.zeroYaw();
    }

    public void manageLimelight(){
        enableLimelight = Inputs.masterAutoEnabled;
        SmartDashboard.putBoolean("POSTMAN: Limelight Enabled", enableLimelight);

        if (enableLimelight) {
            readLimeLightValues();
        } else {
            limelightSeesATarget = false;
        }
    }

    public void readLimeLightValues() {

        double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0); // valid target
        double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0); // dX
        double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0); // dY                                                       

        limelightSeesATarget = !(tv < 1);

        stationaryLimelightXOffset = tx - 2.5;
        stationaryLimelightDistanceToTarget = calculateDistanceToBasket(Math.toRadians(ty));

        if (limelightSeesATarget) {
            foundPosition = true;
        }
    }

    private double calculateDistanceToBasket(double radians) {
        double distanceMeters = (Constants.ShooterConstants.kDeltaHeightMeters
                / (Math.tan(Constants.ShooterConstants.kLimelightTiltAngleRadians + radians)));

        return distanceMeters;
    }

    public void manageLocation(){
        if (limelightSeesATarget){
            double angle = -gyroAngle + (500 + l) * (Constants.ShooterConstants.kTurretFullRight - m_robotShoot.getActualTurretAngle()) - stationaryLimelightXOffset;
            double radian = angle * Math.PI / 180;

            x = stationaryLimelightDistanceToTarget * Math.sin(radian);
            y = stationaryLimelightDistanceToTarget * Math.cos(radian);
        } else {
            if (Inputs.masterAutoEnabled) {
                double xI = convertPowerToDistance(Inputs.driverStrafe);
                double yI = convertPowerToDistance(Inputs.driverPower);

                x -= xI;
                y -= yI;
            }
        }

        Constants.telemetry.putNumber("POSTMAN: robot x", x, true);
        Constants.telemetry.putNumber("POSTMAN: robot y", y, true);
        Constants.telemetry.putBoolean("POSTMAN: robot targeting", limelightSeesATarget, true);
    }

    public double[] returnMovingTarget() {
        double driveXVelocityOffset = 3;
        double driveYVelocityOffset = 5;

        double turnAngleOffset = 0.362;

        double[] pointA = new double[2]; 
        double[] pointB = new double[2]; 
        double[] pointF = new double[2]; 

        double appliedPowerX = Inputs.driverStrafe * Math.abs(Inputs.driverStrafe);
        double appliedPowerY = Inputs.driverPower * Math.abs(Inputs.driverPower);

        pointA[0] = 0.0;
        pointA[1] = 0.0;

        pointB[0] = x;
        pointB[1] = y;

        pointF[0] = pointB[0] - appliedPowerX * driveXVelocityOffset;
        pointF[1] = pointB[1] + appliedPowerY * driveYVelocityOffset;

        Constants.telemetry.putNumber("POSTMAN: point F x", pointF[0], true);
        Constants.telemetry.putNumber("POSTMAN: point F y", pointF[1], true);

        if (Inputs.masterAutoEnabled) {
            return pointF;
        } else {
            return pointB;
        }
        
    }

    public double returnAngleToTarget(double robotX, double robotY) {
        
        return Math.atan(robotX / robotY);
    }

    public void turnRobotToTarget(double robotX, double robotY) {

        double radian = returnAngleToTarget(robotX, robotY);
        double angleDifference = radian * 180 / Math.PI + gyroAngle;

        double rightAngleBound = (500 + l) * (Constants.ShooterConstants.kTurretFullRight - Constants.ShooterConstants.kTurretExtendedRight);
        double leftAngleBound = (500 + l) * (Constants.ShooterConstants.kTurretFullRight - Constants.ShooterConstants.kTurretFullLeft);

        Inputs.overrideTurretAngle = Constants.ShooterConstants.kTurretFullRight - (angleDifference / (500 + l));

        double map = m_robotShoot.updateMap(angleDifference, rightAngleBound, leftAngleBound, -1.0, 1.0);
        map = Math.min(Math.max(map, -1.0), 1.0);

        if (map > 0.5) {
            Inputs.rotationOverride = !Inputs.leftDriverTrigger;
            Inputs.rotationInputOverride = m_robotShoot.updateMap(map, 0.5, 1.0, 0.0, -0.1);
        } else if (map < -0.5) {
            Inputs.rotationOverride = !Inputs.leftDriverTrigger;
            Inputs.rotationInputOverride = m_robotShoot.updateMap(map, -0.5, -1.0, 0.0, 0.1);
        } else {
            Inputs.rotationOverride = false;
        }

        if (!Inputs.masterAutoEnabled){
            Inputs.rotationOverride = false;
        }

        
    }
    
    public void turnTurretToTarget(double robotX, double robotY) {
        double radian = returnAngleToTarget(robotX, robotY);
        double angleDifference = radian * 180 / Math.PI + gyroAngle;

        SmartDashboard.putNumber("POSTMAN: Turret Angle Difference", angleDifference);

        double rightAngleBound = (500 + l) * (Constants.ShooterConstants.kTurretFullRight - Constants.ShooterConstants.kTurretExtendedRight);
        double leftAngleBound = (500 + l) * (Constants.ShooterConstants.kTurretFullRight - Constants.ShooterConstants.kTurretFullLeft);

        Inputs.overrideTurretAngle = Constants.ShooterConstants.kTurretFullRight - (angleDifference / (500 + l));

        if (angleDifference > rightAngleBound && angleDifference < leftAngleBound){
            Inputs.overrideTurret = true;
        } else {
            Inputs.overrideTurret = false;
        }

        if (!Inputs.masterAutoEnabled){
            Inputs.overrideTurret = false;
        }
    }

    public void setShooterPowerToTarget(double robotX, double robotY) {
        double distance = Math.sqrt(robotX * robotX + robotY * robotY);

        if (!Inputs.masterEndgameArm) {
            if (m_extraControl.getRawButtonPressed(8)) {
                shooterPowerOffset += 0.01;
            }
            if (m_extraControl.getRawButtonPressed(7)) {
                shooterPowerOffset -= 0.01;
            }   
        }

        if (Inputs.shooterFullAutoModeOff) { // only switched when button pressed
            shooterInFullAuto = false;
        } else if (Inputs.shooterFullAutoModeOn) {
            shooterInFullAuto = true;
        }

        double frontV = 370.0 /*362.93*/ * (distance * distance) - 1624.7 * distance + /*10459*/ 9600 + (shooterPowerOffset) * 22834;
        double backV = 0.0283 * distance + .773 + (shooterPowerOffset * 2.0);

        if (!Inputs.masterEndgameArm && shooterInFullAuto) { // managed in inputs, extraControl.getRawButton(1) == false
            if (!Inputs.shooterWallShot && !Inputs.shooterWeakShot) {
                if (Inputs.masterAutoEnabled) {
                    Inputs.overrideShooterMotors = true;
                    Inputs.overrideFrontShooterMotorPower = frontV;
                    Inputs.overrideBackShooterMotorPower = backV;
                } else {
                    Inputs.overrideShooterMotors = false;
                }
            } else {
                Inputs.overrideShooterMotors = false;
            }
        } else {
            Inputs.overrideShooterMotors = false;
        }

        if (!Inputs.masterAutoEnabled){
            Inputs.overrideShooterMotors = false;
        }
        
        SmartDashboard.putNumber("POSTMAN: front motor power", Inputs.overrideFrontShooterMotorPower);
        SmartDashboard.putNumber("POSTMAN: back motor power", Inputs.overrideBackShooterMotorPower);
        SmartDashboard.putBoolean("POSTMAN: override shooter power", Inputs.overrideShooterMotors);
    }

    public void setHoodAngleToTarget(double robotX, double robotY) {
        double distance = Math.sqrt(robotX * robotX + robotY * robotY);
        double idealHoodAngle = m_robotShoot.returnIdealHoodPosition(distance);

        if (!Inputs.masterEndgameArm && shooterInFullAuto) {
            if (Inputs.shooterWallShot) {
                Inputs.overrideHood = false;
            } else {
                Inputs.overrideHoodAngle = idealHoodAngle;
                Inputs.overrideHood = true;
            }            
        } else {
            Inputs.overrideHoodAngle = 0.5;
            Inputs.overrideHood = true;
        }
    }

    private double convertPowerToDistance(double powerApplied) {
        
        //constant equals distance the robot drives in one playbak (0.02 seconds), might not be linear

        double distanceInches = 314.81 * (powerApplied * powerApplied) + 29.385 * Math.abs(powerApplied) - 23.968;

        if (Math.abs(powerApplied) < 0.25) {
            distanceInches = 0.0;
        }

        if (powerApplied < 0) {
            distanceInches *= -1;
        }

        double distanceMeters = distanceInches * 0.0254;

        double distancePerFrame = distanceMeters / 100;

        distancePerFrame *= 0.9;


        return distancePerFrame;
    }
}
