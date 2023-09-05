package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.Inputs;
import frc.robot.MyTimedPower;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

public class IntakeSubsystem extends SubsystemBase {
    private final TalonSRX m_intakeDeployMotor = new TalonSRX(16);  
    private final TalonSRX m_intakeProcessMotors = new TalonSRX(17);


    DigitalInput m_intakeLimitSwitch = new DigitalInput(4);

    private static final DigitalInput photogateShooter = new DigitalInput(9);
    private static final DigitalInput photogateIntake = new DigitalInput(8);

    private MyTimedPower intakeDeployPowerManager;

    private double intakeStartingPosition = 0;



    public IntakeSubsystem() {

        //before Worlds QM4 had 0.75 power down and -0.4 power up
        intakeDeployPowerManager = new MyTimedPower( 2.0, 0.6, -.6 ); // time, deploy power, retract power
        m_intakeDeployMotor.setNeutralMode(NeutralMode.Brake);

        m_intakeProcessMotors.setInverted(true);
                    
        m_intakeProcessMotors.configPeakCurrentLimit(30);
        m_intakeProcessMotors.configPeakCurrentDuration(100);
        m_intakeProcessMotors.configContinuousCurrentLimit(10);
        m_intakeProcessMotors.enableCurrentLimit(true);

    }

    public static boolean ball1IsReady(){   // uses sensor values to determine if ball 1 is ready

        if (photogateShooter.get()){        
            return false;                   // ball is not ready
        } else{                             
            return true;                    // ball is ready
        }

    }

    public static boolean ball2IsReady(){ //check if ball 2 ready (works same as ball 1 check)

        if (photogateIntake.get()){
            return false;
        } else{
            return true;
        }

    }

    public boolean intakeNeedsBalls(){              
        if (photogateShooter.get()){                //checks if there is ball in the robot; if photogateShooter returns false means ball in photogate (is inverted)
            Inputs.shootPhotogateSeesBall = false;  
        } else{
            Inputs.shootPhotogateSeesBall = true;   
        }
        Constants.telemetry.putTrueBoolean("INTAKE Ball 1 Ready", !photogateShooter.get());
        Constants.telemetry.putTrueBoolean("INTAKE Ball 2 Ready", !photogateIntake.get());

        if (photogateIntake.get() || photogateShooter.get()){ //if there is <=1 balls in the robot, the robot needs balls
            return true;
        } else {
            return false;
        }
    }


    public void intake(ShootSubsystem m_robotShoot){
        SmartDashboard.putBoolean("intake limit switch", m_intakeLimitSwitch.get());


        double tonyRollerPower = 0.0;

        double intakeDeployRetractPower = 0.0;

        if (Inputs.masterEndgameArm){ // managed in inputs, extraControl.getRawButton(1) == true
            intakeDeployRetractPower = intakeDeployPowerManager.getPower( false ); // force a retract
            m_intakeDeployMotor.set(ControlMode.PercentOutput,intakeDeployRetractPower);
            m_intakeProcessMotors.set(ControlMode.PercentOutput, 0);
            return;                                                                 // we are in endgame, get out of these
        }

        /********************************************************
         * service the intake device, false is up, true is down
         *********************************************************/
        if( Inputs.intakeDeploy == true && intakeNeedsBalls() == false  )  // if down and have enough falls
            Inputs.intakeDeploy = false;                                   

        if( Inputs.intakeDeploy == false &&                 // want to retract, switch says we are down. 
                    m_intakeLimitSwitch.get() == true )     // digital switch set to Normally Closed. True is Deployed. 
            intakeDeployPowerManager.resetTimer();          // will cause the power to start the retract process again

        intakeDeployRetractPower = intakeDeployPowerManager.getPower( Inputs.intakeDeploy ); // driver will hold intake deploy button when needed
        
        
        

        // service the intake rollers
        double intakeRollerPower = 0.0;

        if (Inputs.intakeDeploy == true) { // if the intake hopper is activated then drop it and start roller motors
            m_intakeDeployMotor.setNeutralMode(NeutralMode.Coast);
            intakeRollerPower = 1.0;
        } else {                                // pull up the intake and stop the roller 
            m_intakeDeployMotor.setNeutralMode(NeutralMode.Brake);
            intakeRollerPower = 0.0;
        }

        if (m_intakeLimitSwitch.get() && intakeDeployRetractPower > 0.0) {
            intakeDeployRetractPower = 0.0;
        }
        
        m_intakeDeployMotor.set(ControlMode.PercentOutput,intakeDeployRetractPower);

        Constants.telemetry.putNumber("INTAKE Roller Power", intakeRollerPower, false);

        /********************************************************
         * service the ball movement system
         *********************************************************/


        if( Inputs.intakeDeploy == false && intakeDeployPowerManager.wasThisEverStarted() == false){ // never deployed 
            tonyRollerPower = 0.0;
        } else if(Inputs.shooterReverseGibRoller){ // driver trying to fix shooter, back up gib and back up tony roller
            tonyRollerPower = -Constants.ShooterConstants.kTonyRollerPower;
        } else if (Inputs.driverTrigger || Inputs.operatorTrigger) {                                 // driver wants to fire shooter
            tonyRollerPower = Constants.ShooterConstants.kTonyRollerPower;
        } else if (Inputs.shooterWeakShot) {
            tonyRollerPower = Constants.ShooterConstants.kTonyRollerPower;
        } else if( Inputs.intakeDeploy == false && intakeDeployPowerManager.getTime() > 4.0  ){
                tonyRollerPower = 0.0;
        }else if (intakeNeedsBalls() == true) {                               // intake sees 1 ball in hopper, if it sees 2 it will turn false
            tonyRollerPower = Constants.ShooterConstants.kTonyRollerPower;
        } else {
            tonyRollerPower = 0.0;
        }

        m_intakeProcessMotors.set(ControlMode.PercentOutput, tonyRollerPower);


        Constants.telemetry.putNumber("INTAKE Tony Roller Power", tonyRollerPower, false);

    }

}
