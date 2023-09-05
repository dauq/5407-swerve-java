// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class DriveSubsystem extends SubsystemBase {
  // Robot swerve modules, stating them.
  private final SwerveModule m_frontLeft =
      new SwerveModule(
          DriveConstants.kFrontLeftDriveMotorPort,
          DriveConstants.kFrontLeftTurningMotorPort,
          DriveConstants.kFrontLeftTurningEncoderPorts,
          DriveConstants.kFrontLeftDriveEncoderReversed,
          DriveConstants.kFrontLeftTurningEncoderReversed,
          214.1); // cancoder offset

  private final SwerveModule m_rearLeft =
      new SwerveModule(
          DriveConstants.kRearLeftDriveMotorPort,
          DriveConstants.kRearLeftTurningMotorPort,
          DriveConstants.kRearLeftTurningEncoderPorts,
          DriveConstants.kRearLeftDriveEncoderReversed,
          DriveConstants.kRearLeftTurningEncoderReversed,
          -113.0); // cancoder offset

  private final SwerveModule m_frontRight =
      new SwerveModule(
          DriveConstants.kFrontRightDriveMotorPort,
          DriveConstants.kFrontRightTurningMotorPort,
          DriveConstants.kFrontRightTurningEncoderPorts,
          DriveConstants.kFrontRightDriveEncoderReversed,
          DriveConstants.kFrontRightTurningEncoderReversed,
          18.5);// cancoder offset

  private final SwerveModule m_rearRight =
      new SwerveModule(
          DriveConstants.kRearRightDriveMotorPort,
          DriveConstants.kRearRightTurningMotorPort,
          DriveConstants.kRearRightTurningEncoderPorts,
          DriveConstants.kRearRightDriveEncoderReversed,
          DriveConstants.kRearRightTurningEncoderReversed,
          -17.5); // cancoder offset

  // The gyro sensor
  public static AHRS m_gyro = new AHRS();

  private final XboxController m_driveController = new XboxController(0);

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(DriveConstants.kDriveKinematics, m_gyro.getRotation2d());

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {}

  double defaultXSpeed = 0;
  double defaultYSpeed = 0;
  double defaultRot = 0;
  double startingPointX = 0;
  double startingPointY = 0;
  double destinationPointX = 0;
  double destinationPointY = 0;
  boolean autonMode = false;

  @Override
  public void periodic() {
    // Update the odometry in the periodic block

      m_odometry.update(
        m_gyro.getRotation2d(),
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState() );
        
        SmartDashboard.putNumber("periodic X", m_odometry.getPoseMeters().getX() );
        SmartDashboard.putNumber("periodic Y", m_odometry.getPoseMeters().getY() );
        SmartDashboard.putNumber("periodic rot", m_odometry.getPoseMeters().getRotation().getDegrees() );
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void zeroGyro(){
    m_gyro.zeroYaw();
}

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }

  public static double getCubePower(double dInPower){
    return dInPower * Math.abs(dInPower * dInPower);   // yes this is not needed to perserve the sign, just go with it. 
}

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {

    Pose2d currentPose = m_odometry.getPoseMeters();
    double currentX = currentPose.getX();
    double currentY = currentPose.getY();

   /****************************************************/
    SmartDashboard.putString("Gyro YAW", Double.toString((double)m_gyro.getYaw()));
    SmartDashboard.putBoolean("Cool Auton Mode", autonMode);
    SmartDashboard.putNumber("currentX", currentX);
    SmartDashboard.putNumber("currentY", currentY);



    var swerveModuleStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(-xSpeed, ySpeed, -rot, Rotation2d.fromDegrees((m_gyro.getYaw()))) //xspeed was positive
                : new ChassisSpeeds(-xSpeed, ySpeed, rot)); //xspeed was positive

    m_frontLeft.setDesiredState(swerveModuleStates[1]); //Adjusted the ports for each motor to change which motor got which position for turning
    m_frontRight.setDesiredState(swerveModuleStates[0]); 
    m_rearLeft.setDesiredState(swerveModuleStates[3]);
    m_rearRight.setDesiredState(swerveModuleStates[2]);

    
    if (m_driveController.getAButtonReleased()) {
      zeroGyro();
    }

    if (m_driveController.getBButtonReleased()) {
      m_frontLeft.changeBrakeMode();
      m_frontRight.changeBrakeMode();
      m_rearLeft.changeBrakeMode();
      m_rearRight.changeBrakeMode();
    }

    if (m_driveController.getYButtonReleased()) {
      if (autonMode == true) {
        autonMode = false;
      }
      else {
        autonMode = true;
        Pose2d ourPose = m_odometry.getPoseMeters();
        startingPointX = ourPose.getX();
        startingPointY = ourPose.getY();
        destinationPointX = startingPointX + 5000;
        destinationPointY = startingPointY + 5000;
      }
          
    }
  } 
  
  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[1]); //Adjusted the ports for each motor to change which motor got which position for turning
    m_frontRight.setDesiredState(desiredStates[0]);
    m_rearLeft.setDesiredState(desiredStates[3]);
    m_rearRight.setDesiredState(desiredStates[2]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  
}
