package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import com.kauailabs.navx.frc.AHRS;

/* Auton Class
	This is custom version of autonomous operation. As you can see it extends the StateMachine class so it will can take
    on all the StateMachine's charactistics. Usign this you can design up to 10 autonomous state machines to do run
    during the autonomous period for the robot.

 */


public class Auton extends MyStateMachine {

    // Auton class constructor. We are passing several other classes that we need access to get use their FBW values.
    // We are passing references for these classes.

    String status = "";

	private Timer timShootingLimit = new Timer();	// used to decide if we should keep shooting
	
    private TimedRampPower trpDrivePower;

    public  Auton() {               // constructor
        reset();
        timStepTimer.start();       // required as MyStateMachine cannot do this.
        timShootingLimit.start();   // locally declared one

        // create a Timed Ramp Power for the drive. Will redefine later as needed 
        trpDrivePower = new TimedRampPower( 2.5,   // totaltime
                                            .70,    // max power
                                            .10,    // min power
                                            .10,    // ramp up percent
                                            .10     // ramp down percent
                                         );
    }

    public void auton1() { // This overrides the auton2 method defined in the state machine class.
        
        String sAuton = "Auton1 - 2 Ball Right ";
        Constants.telemetry.putString("Auton ", sAuton );

        Constants.telemetry.saveDouble("Auton Step Timer", timStepTimer.get() );  // capture timer on ever step

        if( bStepFirstPass)                                         // no need to do this in every                                                          
            timStepTimer.reset();                                   // reset the timer on a step change

        Inputs.fieldCentric = false;        // do this in call cases

        switch (iStep) {                            // switch statement.
            // it will look at iStep and find the case where it should run code.
            // if iStep not found, it will go to default section at bottom.

            case 0:                                    // case 0 is where we can set up a bunch of stuff for the state machine.    

                if (Inputs.autonDelay == 0) {
                    iStep += 1;
                } else if (timStepTimer.get() >= Inputs.autonDelay) {
                    iStep += 1;
                }

                break;

            case 1: // drive back to wall with intake down
                if (bStepFirstPass) {

                    Constants.telemetry.saveString("Auton Step Desc", status );           
                    trpDrivePower.reconfigure(Constants.Auton2BallRightConstants.kInitial_Time, 
                                              Constants.Auton2BallRightConstants.kInitial_Power, 
                                              -.1,  // min drive power
                                              .10,  // ramp up percent, .20 as we have to reset gyro here
                                              .20);

                }
                status = "Step " + String.valueOf(iStep) + ": ";
                status += "Started, rolling back to ball, intake delpoyed.";


                Inputs.intakeDeploy = true;                 // drop the intake
                Inputs.shooterFullAutoModeOn = true;       // turn on targetting

                Inputs.driverPower = trpDrivePower.getRampedPower(timStepTimer.get());
                
                if( trpDrivePower.isDone())             // we are far enought
                    iStep = 3; //++;

                Constants.telemetry.saveString("Auton Step Desc", status );           
                break;

            case 2: // drive back forward to get a good shot.
                if (bStepFirstPass) {
                    status = "Step " + String.valueOf(iStep) + ": First Pass...";
                    Constants.telemetry.saveString("Auton Step Desc", status );           
                    trpDrivePower.reconfigure(Constants.Auton2BallRightConstants.kToShoot_Time, 
                                            Constants.Auton2BallRightConstants.kToShoot_Power, 
                                             .1,  // min drive power
                                            .10,  // ramp up percent, .20 as we have to reset gyro here
                                            .20);

                    break;
                }
                status = "Step " + String.valueOf(iStep) + ": ";
                status += "Started, rolling back to start Pos, intake retracted.";


                Inputs.intakeDeploy = false;            //pick up the intake
                Inputs.shooterFullAutoModeOn = true;    // turn on targetting

                Inputs.driverPower = trpDrivePower.getRampedPower(timStepTimer.get());
                
                if( trpDrivePower.isDone())              
                    iStep++;

                Constants.telemetry.saveString("Auton Step Desc", status );           
                break;


            case 3: // Stop and shoot
                if (bStepFirstPass) {
                    timShootingLimit.reset();						// reset this timer
                }

                status = "Step " + String.valueOf(iStep) + ": ";
                status += "Shooting";

                Inputs.intakeDeploy = false;                    // drop the intake
                Inputs.shooterFullAutoModeOn = true;            // turn on targetting

                Inputs.operatorTrigger = true;

                if( IntakeSubsystem.ball1IsReady() ||          // if there is a ball in the shooter
                    IntakeSubsystem.ball2IsReady() )
                    timShootingLimit.reset();						// reset this timer
    
                if( IntakeSubsystem.ball1IsReady() == false &&          // no ball in shooter
                    IntakeSubsystem.ball2IsReady() == false &&
                    timShootingLimit.get() > Constants.Auton2BallRightConstants.kShoot_Time  )   // 			// we waited long enough
                    iStep++;                                            // next step, 
    
                Constants.telemetry.saveString("Auton Step Desc", status );           
                break;
            
            case 4: // drive back out of tarmac area
                if (bStepFirstPass) {

                    Constants.telemetry.saveString("Auton Step Desc", status );           
                    trpDrivePower.reconfigure(Constants.Auton2BallRightConstants.kBackOut_Time, 
                                              Constants.Auton2BallRightConstants.kBackOut_Power, 
                                              -.1,  // min drive power
                                              .10,  // ramp up percent, .20 as we have to reset gyro here
                                              .20);

                }
                status = "Step " + String.valueOf(iStep) + ": ";
                status += "Started, rolling back to ball, intake delpoyed.";

                Inputs.driveDesiredHeading = Constants.Auton2BallRightConstants.kBackOut_Heading;       
                Inputs.driveGyroCorrected = true;       // force robot to turn an maintain this heading

                Inputs.intakeDeploy = false;             // drop the intake
                Inputs.shooterFullAutoModeOff = true;    // turn on targetting

                Inputs.driverPower = trpDrivePower.getRampedPower(timStepTimer.get());
                
                if( trpDrivePower.isDone())             // Once its done moving, shift to next step
                    iStep++;

                Constants.telemetry.saveString("Auton Step Desc", status );           
                break;


            default:
                bIsDone = true;
                Inputs.shooterFullAutoModeOff = true;            // turn on targetting
                Constants.telemetry.saveString("Auton Step Desc", "Done, shooter off!" );           


        }  // end of switch statement                                                        
    }



    public void auton2() { // Basic backup, get a ball, drive up and shoot on the Left tarmac
        DriveSubsystem.m_gyro.zeroYaw();
        String sAuton = "Auton2 - 2 Ball ";
        Constants.telemetry.putString("Auton ", sAuton );

        Constants.telemetry.saveDouble("Auton Step Timer", timStepTimer.get() );  // capture timer on every step

        if( bStepFirstPass){                                         // no need to do this in every                                                          
            timStepTimer.reset();                                   // reset the timer on a step change
        }
        
        switch (iStep) {                            // switch statement.
            // it will look at iStep and find the case where it should run code.
            // if iStep not found, it will go to default section at bottom.

            case 0:                                    // case 0 is where we can set up a bunch of stuff for the state machine.    
                if (bStepFirstPass) {
                }

                if (Inputs.autonDelay == 0) {
                    iStep += 1;
                } else if (timStepTimer.get() >= Inputs.autonDelay) {
                    iStep += 1;
                }

                break;

            case 1: // drive back to a certain spot for 4.0 seconds
                if (bStepFirstPass) {

                    Constants.telemetry.saveString("Auton Step Desc", status );           
                    trpDrivePower.reconfigure(Constants.Auton2BallConstants.kInitial_Time, 
                                              -Constants.Auton2BallConstants.kInitial_Power, 
                                              -.1,  // min drive power
                                              .10,  // ramp up percent, .20 as we have to reset gyro here
                                              .20);

                }
                status = "Step " + String.valueOf(iStep) + ": ";
                status += "Started, rolling back to ball, intake delpoyed.";

                Inputs.intakeDeploy = true;             // drop the intake
                Inputs.shooterFullAutoModeOn = false;    // turn on targetting
                Inputs.shooterFullAutoModeOff = true;    // turn on targetting

                Inputs.driverPower = trpDrivePower.getRampedPower(timStepTimer.get());
                
                if( trpDrivePower.isDone())             
                    iStep = 3;

                Constants.telemetry.saveString("Auton Step Desc", status );           
                break;

            case 2: // drive back to a certain spot for 4.0 seconds
                if (bStepFirstPass) {
                    status = "Step " + String.valueOf(iStep) + ": First Pass...";
                    Constants.telemetry.saveString("Auton Step Desc", status );           
                    trpDrivePower.reconfigure(Constants.Auton2BallConstants.kToShoot_Time, 
                                            Constants.Auton2BallConstants.kToShoot_Power, 
                                             -.1,  // min drive power
                                            .10,  // ramp up percent, .20 as we have to reset gyro here
                                            .20);

                    break;
                }
                status = "Step " + String.valueOf(iStep) + ": ";
                status += "Started, rolling back to start Pos, intake retracted.";

                Inputs.driveDesiredHeading = Constants.Auton2BallConstants.kToShoot_Heading;       
                
                Inputs.driveGyroCorrected = false; 

                Inputs.intakeDeploy = false;             // drop the intake
                Inputs.shooterFullAutoModeOn = true;    // turn on targetting
                Inputs.shooterFullAutoModeOff = false;    // turn on targetting

                Inputs.driverPower = trpDrivePower.getRampedPower(timStepTimer.get());
                
                if( trpDrivePower.isDone())             
                    iStep++;

                Constants.telemetry.saveString("Auton Step Desc", status );           
                break;


            case 3: // Stop and shoot
            if (bStepFirstPass) {
            }
            status = "Step " + String.valueOf(iStep) + ": ";
            status += "Shooting";

                Inputs.intakeDeploy = false;                    // drop the intake
                Inputs.shooterFullAutoModeOn = true;            // turn on targetting
                Inputs.shooterFullAutoModeOff = false;          

                Inputs.operatorTrigger = true;

                if( timStepTimer.get() > 5.0 )
                    iStep++;

                Constants.telemetry.saveString("Auton Step Desc", status );           
                break;
            
            default:
                bIsDone = true;
                Inputs.shooterFullAutoModeOff = true;            // turn on targetting
                Constants.telemetry.saveString("Auton Step Desc", "Done, shooter off!" );           


        }  // end of switch statement                                                        
    }

    public void auton5() { // This overrides the auton1 method defined in the state machine class.
        String sAuton = "Auton5 - Play Auton 3 ball";
        Constants.telemetry.putString("Auton ", sAuton );

        if(Robot.pbAuton3ball.bMomentsLoaded==true && Robot.pbAuton3ball.isPlaybackDone()==false) 
            Robot.pbAuton3ball.playNextMoment();

    }


    public void auton3(){ //This auton will attempt to shoot the ball placed inside it, the one behind it, and the third ball that set at the player termianl
        String sAuton = "Auton3 - 3 Ball Right ";
        Constants.telemetry.putString("Auton ", sAuton );

        Constants.telemetry.saveDouble("Auton Step Timer", timStepTimer.get() );  // capture timer on ever step

        if( bStepFirstPass)                                                                                               
            timStepTimer.reset();                                   // reset the timer only once

        Inputs.fieldCentric = false;        // do this in call cases

        switch (iStep) {                            // switch statement.
            // it will look at iStep and find the case where it should run code.
            // if iStep not found, it will go to default section at bottom.

            case 0:                                    // case 0 is where we can set up a bunch of stuff for the state machine.    
                if (bStepFirstPass) {
                }

                iStep++;
                break;

            case 1: // drive back to wall where ball is
                if (bStepFirstPass) {
                    trpDrivePower.reconfigure(Constants.Auton3BallRightConstants.kInitial_Time, 
                                              Constants.Auton3BallRightConstants.kInitial_Power, 
                                              -.1,  // min drive power
                                              .20,  // ramp up percent, .20 as we have to reset gyro here
                                              .20);
                }
                status = "Step " + String.valueOf(iStep) + ": ";
                status += "Started, rolling back to ball, intake delpoyed.";

                Inputs.intakeDeploy = true;             // drop the intake
                Inputs.shooterFullAutoModeOn = true;    // turn on targetting

                Inputs.driverPower = trpDrivePower.getRampedPower(timStepTimer.get());
                
                if( trpDrivePower.isDone())             
                    iStep++;

                Constants.telemetry.saveString("Auton Step Desc", status );           
    
                break;

            case 2: // drive back to spot I started at
                if (bStepFirstPass) {
                    trpDrivePower.reconfigure(Constants.Auton3BallRightConstants.kToShoot_Time, 
                                              Constants.Auton3BallRightConstants.kToShoot_Power, 
                                              -.1,  // min drive power
                                              .20,  // ramp up percent, .20 as we have to reset gyro here
                                              .20);
                }
                status = "Step " + String.valueOf(iStep) + ": ";
                status += "Started, rolling back to ball, intake delpoyed.";
                
                Inputs.intakeDeploy = false;             // drop the intake
                Inputs.shooterFullAutoModeOn = true;    // turn on targetting

                Inputs.driverPower = trpDrivePower.getRampedPower(timStepTimer.get());
                
                if( trpDrivePower.isDone())             
                    iStep++;

                Constants.telemetry.saveString("Auton Step Desc", status );           
    
                break;

            case 3: // Shoot
                if (bStepFirstPass) {
					  timShootingLimit.reset();	// reset this timer
                }
                status = "Step " + String.valueOf(iStep) + ": ";
                status += "Shoot!";

                Inputs.intakeDeploy = false;             // pick up intake
                Inputs.shooterFullAutoModeOn = true;    // turn on targetting

                Inputs.operatorTrigger = true;

                if( IntakeSubsystem.ball1IsReady() ||          // if there is a ball in the shooter
                      IntakeSubsystem.ball2IsReady() )
					  timShootingLimit.reset();						// reset this timer

                if( IntakeSubsystem.ball1IsReady() == false &&          // no ball in shooter
                      IntakeSubsystem.ball2IsReady() == false &&
                        timShootingLimit.get() > Constants.Auton3BallRightConstants.kShoot_Time )   // 			// we waited long enough
                    iStep++;                                            // next step, 
                                                                        
                Constants.telemetry.saveString("Auton Step Desc", status );           

                break;


            case 4:                                         // drive to a the terminal at 20 degrees
                if (bStepFirstPass) {

                    trpDrivePower.reconfigure(Constants.Auton3BallRightConstants.kToBall3_Time, 
                                                Constants.Auton3BallRightConstants.kToBall3_Power,
                                               .1,  // this direction will be adjusted to the max power direction 
                                              .10, 
                                              .30);  // .30 is ramp down 30 percent before the end
                }
                status = "Step " + String.valueOf(iStep) + ": ";
                status += "Going to Ball 3, slewing turret left.";
    
                Inputs.driveDesiredHeading = Constants.Auton3BallRightConstants.kToBall3_Heading; 
                Inputs.driveGyroCorrected = true; 
                Inputs.driverPower = trpDrivePower.getRampedPower(timStepTimer.get());
                Inputs.shooterFullAutoModeOn = false;    // turn on targetting
                Inputs.intakeDeploy = true;

                if( trpDrivePower.isDone())
                    iStep++;

                Constants.telemetry.saveString("Auton Step Desc", status );           
                break; 
				

            case 5: // Taking the third ball shot
                if (bStepFirstPass) {

                }
                status = "Step " + String.valueOf(iStep) + ": ";
                status += "Shooting ball 3.";
        
                Inputs.operatorTrigger = true;                
                Inputs.shooterFullAutoModeOn = true;    // turn on targetting
                Inputs.intakeDeploy = false;

                Constants.telemetry.saveString("Auton Step Desc", status );           
                break;
                
            default: 
                Inputs.driverPower = 0.0;
                Inputs.shooterFullAutoModeOff = true;    // turn on targetting

            }   // end of switch/case statement
        
    } // end of auton2 method


    public void auton4(){ //Similarly with auton3, but with additional steps that uitilize additional sensors to shoot 3-4 balls
        String sAuton = "Auton4 - 4 Ball ";
        Constants.telemetry.putString("Auton ", sAuton );

        Constants.telemetry.saveDouble("Auton Step Timer", timStepTimer.get() );  // capture timer on ever step

        if( bStepFirstPass)                                         // no need to do this in every                                                          
            timStepTimer.reset();                                   // reset the timer on a step change

        Inputs.fieldCentric = false;        // do this in call cases

        switch (iStep) {                            // switch statement.
            // it will look at iStep and find the case where it should run code.
            // if iStep not found, it will go to default section at bottom.

            case 0:                                    // case 0 is where we can set up a bunch of stuff for the state machine.    
                if (bStepFirstPass) {
                }

                iStep++;
                break;

            case 1: // drive back to a certain spot for 4.0 seconds
                if (bStepFirstPass) {
                    trpDrivePower.reconfigure(Constants.Auton4BallConstants.kInitial_Time, 
                                              Constants.Auton4BallConstants.kInitial_Power, 
                                              .10,  // min drive power, max power sets direction
                                              .10,  // ramp up percent, .20 as we have to reset gyro here
                                              .10);
                }
                status = "Step " + String.valueOf(iStep) + ": ";
                status += "Started, rolling back to ball, intake delpoyed.";
                
                Inputs.fieldCentric = false;
                Inputs.driveDesiredHeading = 0.0;       
                Inputs.driveGyroCorrected = true;      // don't use at this point // force robot to turn an maintain this heading

                Inputs.intakeDeploy = true;             // drop the intake
                Inputs.shooterFullAutoModeOn = true;    // turn on targetting

                Inputs.driverPower = trpDrivePower.getRampedPower(timStepTimer.get());
                
                if( trpDrivePower.isDone())            
                    iStep++;

                Constants.telemetry.saveString("Auton Step Desc", status );           
    
                break;


            case 2: // shoot
                if (bStepFirstPass) {
					  timShootingLimit.reset();	// reste this itmer
                }
                status = "Step " + String.valueOf(iStep) + ": ";
                status += "Shoot!";

                Inputs.intakeDeploy = false;             // pick up intake
                Inputs.shooterFullAutoModeOn = true;    // turn on targetting

                Inputs.operatorTrigger = true;

                if( IntakeSubsystem.ball1IsReady() ||          // ball in shooter
                      IntakeSubsystem.ball2IsReady() )
					  timShootingLimit.reset();						// reset this timer

                if( IntakeSubsystem.ball1IsReady() == false &&          // no ball in shooters
                      IntakeSubsystem.ball2IsReady() == false &&
                        timShootingLimit.get() > Constants.Auton4BallConstants.kInitial_ShootTime )   // 			// we waited long enough
                    iStep++;                                            // next step, 
                                                                        
                Constants.telemetry.saveString("Auton Step Desc", status );           

                break;

           
            case 3:   // Traverse to Terminal and third ball                                     
                if (bStepFirstPass) {

                    trpDrivePower.reconfigure(Constants.Auton4BallConstants.kToTerm_Time, 
                                                Constants.Auton4BallConstants.kToTerm_Power,
                                               .1,  // this direction will be adjusted to the max power direction 
                                              .10, 
                                              .30);  // .50 is ramp down 50 percent before the end
                }
                status = "Step " + String.valueOf(iStep) + ": ";
                status += "To Terminal";
    
                Inputs.driveDesiredHeading = Constants.Auton4BallConstants.kToTerm_Heading; 
                Inputs.driveGyroCorrected = true; 

                // Here we are traversing
                Inputs.driverStrafe = trpDrivePower.getRampedPower(timStepTimer.get());
                Inputs.shooterFullAutoModeOff = true;    // turn off targetting
                Inputs.intakeDeploy = false;

                if( trpDrivePower.isDone())
                    iStep++;   // stop here

                Constants.telemetry.saveString("Auton Step Desc", status );           
                break; 

            case 4:   // drive back to the terminal, collect first ball
                if (bStepFirstPass) {
                    trpDrivePower.reconfigure(Constants.Auton4BallConstants.kAtTerm_Time, 
                                                Constants.Auton4BallConstants.kAtTerm_Power,
                                                -.1, 
                                                .10, 
                                                Constants.Auton4BallConstants.kAtTerm_RampDownProp);  // .50 is ramp down 50 percent before the end
                }
                status = "Step " + String.valueOf(iStep) + ": ";
                status += "Drive up to terminal";
    
                Inputs.driveDesiredHeading = Constants.Auton4BallConstants.kAtTerm_Heading; 
                Inputs.driveGyroCorrected = true; 
                Inputs.driverPower = trpDrivePower.getRampedPower(timStepTimer.get());
                Inputs.shooterFullAutoModeOff = true;    // turn on targetting
                Inputs.intakeDeploy = true;
                
                if( trpDrivePower.isDone()){
                    iStep++;
                }

                break;

            case 5:   // Should have the ball, raise the intake
                if (bStepFirstPass) {
                }
                status = "Step " + String.valueOf(iStep) + ": ";
                status += "At Teminal, raise intake";
    
                Inputs.driverPower = .3;                 // turn off targetting
                Inputs.shooterFullAutoModeOff = true;    // turn off targetting
                Inputs.intakeDeploy = false;
                
                if( timStepTimer.get() > Constants.Auton4BallConstants.kAtTerm_IntakeUpTime){
                    iStep++;
                }

                break;


            case 6:   // Human player rolls ball under, drop the intake
                if (bStepFirstPass) {
                }
                status = "Step " + String.valueOf(iStep) + ": ";
                status += "At Teminal, lower intake";
    
                Inputs.shooterFullAutoModeOn = true;    // turn on targetting
                Inputs.intakeDeploy = true;
                
                if( timStepTimer.get() > Constants.Auton4BallConstants.kAtTerm_IntakeDownTime){
                    iStep++;
                }

                break;



            case 7: // drive To shoot
                if (bStepFirstPass) {
                    trpDrivePower.reconfigure(Constants.Auton4BallConstants.kShot2_DriveTime, 
                                        Constants.Auton4BallConstants.kShot2_Power,
                                        -.1, 
                                        .10, 
                                        .40);  // .50 is ramp down 50 percent before the end

                }
                status = "Step " + String.valueOf(iStep) + ": ";
                status += "Shoot!";
        
                Inputs.driveDesiredHeading = Constants.Auton4BallConstants.kShot2_Heading; 
                Inputs.driveGyroCorrected = true; 
                Inputs.shooterFullAutoModeOn = true;    // turn on targetting
                Inputs.intakeDeploy = false;
                
                Inputs.driverPower = trpDrivePower.getRampedPower(timStepTimer.get());
 
                if( trpDrivePower.isDone() )
                    iStep++;

                Constants.telemetry.saveString("Auton Step Desc", status );           
                break; 
    

            case 8: // Taking the final shot
                if (bStepFirstPass) {

                }
                status = "Step " + String.valueOf(iStep) + ": ";
                Inputs.shooterFullAutoModeOn = true;    // turn on targetting

                if(timStepTimer.get() < Constants.Auton4BallConstants.kShot2_DelayShotTime){
                    status += "Delaying the Shot to allow robot to stop forward momentum!";
                } else {
                    Inputs.operatorTrigger = true;                
                    status += "Shooting!!!";
                }


                Constants.telemetry.saveString("Auton Step Desc", status );           
                // no need to move to a new step as shooting is the last thing we do in auton
                break; 
                
            default: 
                status = "Step " + String.valueOf(iStep) + ": ";
                status += "Done";
                Inputs.driverPower = 0.0;
                Constants.telemetry.saveString("Auton Step Desc", status );           

            }   // end of switch/case statement
        
    } // end of auton4 method


} // end of auton class
