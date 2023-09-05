package frc.robot;

//import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.ClimbSubsystem;

import javax.lang.model.util.ElementScanner6;

//import frc.robot.MyGotoPosition;


public class ClimberAutomation extends MyStateMachine {

    // define a goto for this lift.
    public static MyGotoPosition gotoLift2 = new MyGotoPosition( "gotoLift2", 
                                     Constants.ClimberConstants.kLift2FullOutPosit, 
                                            Constants.ClimberConstants.kLift2FullInPosit); 


    ClimberAutomation(){
        bStandAloneMachine = true;        // Set MyStatemachine value to true to indicate 
                                          // thsi indicates this will only have 1 auton process
    }



    public void switchSteps() {

        Constants.telemetry.saveDouble("CLIMB SM Step Timer", timStepTimer.get() );  // capture timer on every step
        Constants.telemetry.saveInteger("CLIMB SM Step", iStep );
        String status = "unk";


        switch (iStep) {                            // switch statement.
            // it will look at iStep and find the case where it should run code.
            // if iStep not found, it will go to default section at bottom.

            case 0:                              // case 0 is where we can set up a bunch of stuff for the state machine.
                if (bStepFirstPass) {
                    sStepDescription = "Init";
                    Constants.telemetry.putString("CLIMB SM Desc", sStepDescription );
                }

                ClimbSubsystem.mLift1ResetDownPosition();

                timStepTimer.start();

                iStep++;                          // We are done with step, increment iStep

                break;                            // setup complete, jump out and go around again.


            /*******************************************************************
             * Here we are startign with the robot under the bar, 
             * Lift 1 is already extended and the Robot is back into the bar. 
             * This is all doen by the Carbon Based Units before we are started.
             * *****************************************************************
             * In Step1 we will...
             * 1) Pull down on Lift 1 until we are all the way down. 
             * 2) As we are pulling down we will move the shuttle out to the
             *    end of the arm so it is in position to grab the bar. 
             * 3) We will also start to lift the Dart and and attached arm to the 
             *    bar position. This will at first be done at a reduced speed. 
             *    This will prevent the arm from reaching the top before the 
             *    Lift 1 is in place. Once Lift 1 hits the bottom and is 
             *    all the way up, you will see the arm raising speed up. 
             ********************************************************************/
            case 1:                               
                if (bStepFirstPass) {             // use this to set stuff up
                    sStepDescription = "Pull down Stage 1, all others ready position";
                    Constants.telemetry.putString("CLIMB SM Desc", sStepDescription );

                    ClimbSubsystem.mLift1ResetDownPosition();       // hook is at the top and we are pulling down
                                                                    // the end game.
                    timStepTimer.reset();
                }

                status = "Step1: ";
                iCompletedStepParts =  0;

                /*************************************************
                 * Pull down on Lift 1 to raise the robot until
                 * we hit the proximity sensor. 
                 * 
                 * We can climb but not go down
                 *************************************************/

                if( ClimbSubsystem.mLift1DownHasBeenReached == false ){     // we are not pulled down all the way
                    ClimbSubsystem.mLift1Lock = true;                     // we can climb, pull down but not up
                    Inputs.climberLift1Down = true;                         // press and hold the button 
                    Inputs.climberLift1PowerOverride = 0.0;                 // to be clear, no override

                    // this is used to slow the climb until the robot bumper skids on the floor
                    // we hope this will take out most of the swing. 
                    if( ClimbSubsystem.checkLift1MotorRetroReflectivePosit() < Constants.ClimberConstants.kStep1SlowLiftRobotPosit ){
                        Inputs.climberLift1PowerOverride = 
                                Constants.ClimberConstants.kStep1SlowLiftRobotPower; // set Lift1 motor to climb slower.
                        status += " Lift1: Robot rising slowly to reduce swing, ";  // pulling up slowly so bumper can 
                                                                                    // drag on ground a reduce swing.

                    }else if( ClimbSubsystem.isLift1Down() == false){       // looking for signal from the prox sensor
                        status += " Lift1: Robot rising, ";                 // false means we should still be pulling robot up. 
                    }

                }else if(ClimbSubsystem.mLift1DownHasBeenReached == true){
                    status += " Lift1: Robot pulled full up, ";
                    ClimbSubsystem.mLift1Lock = true;                     // we can climb, pull down but not up
    
                    iCompletedStepParts++;                                      // this part is done
                }

                

                //The hand has to move out in not initially, but after the dart reaches a certain position
                if (Math.abs(ClimbSubsystem.getDartPosition() - Constants.ClimberConstants.kDartFullDownPosit) > Math.abs(ClimbSubsystem.getDartPosition() - Constants.ClimberConstants.kDartFullUpPosit)) {
                    Inputs.climberLift2Down = true;         // Push hand out to the end  -
                    Constants.telemetry.saveDouble("CLIMB SM Lift2 Target Posit", Constants.ClimberConstants.kStep1Lift2FullOutPosit );
                } else {
                    Inputs.climberLift2Down = false;         // Push hand out to the end  -
                    Constants.telemetry.saveDouble("CLIMB SM Lift2 Target Posit", Constants.ClimberConstants.kStep1Lift2FullOutPosit );
                }
                

                if( ClimbSubsystem.getLift2Position() >= Constants.ClimberConstants.kStep1Lift2FullOutPosit ){ // move will being pulled up
                    status += " Lift2: Moving hand away from robot, ";
                }else{
                    status += " Lift2: Hand at end of L2 arm, ";
                    Inputs.climberLift2Up = false;    // stop pushing either way just in case
                    Inputs.climberLift2Down = false;  // stop pushing out
                    iCompletedStepParts++;
                }


                /***************************************************************************
                 * This will swing the DART arm up towards, but under bar 2. 
                 * We want to do this while pulling robot up to save time later. 
                 ****************************************************************************/

                Inputs.climberDartUp = true;                  // press and hold the button +
                
                if( ClimbSubsystem.mLift1DownHasBeenReached == false && ClimbSubsystem.getDartPosition() >= Constants.ClimberConstants.kStep1DartUpToBarPosit + 0.1) {// The robot is lifted up to the bar. 
                    Inputs.climberDartUp = true;           // we are going up
                    status += " Dart coming up";
                }else if( ClimbSubsystem.mLift1DownHasBeenReached == false && ClimbSubsystem.getDartPosition() < Constants.ClimberConstants.kStep1DartUpToBarPosit + 0.1) {// The robot is lifted up to the bar. 
                    Inputs.climberDartUp = false;      
                    status += " Dart paused";     // we have stopped dart - waiting for lift1
                }else if (ClimbSubsystem.mLift1DownHasBeenReached == true && ClimbSubsystem.getDartPosition() >= Constants.ClimberConstants.kStep1DartUpToBarPosit) {
                    Inputs.climberDartUp = true; //we are darting up
                    status += " Dart coming up";
                }else {
                    status += " Dart: Arm is now in contact with bar 2. ";
                    Inputs.climberDartUp = false;           
                    iCompletedStepParts++;           
                }

                

                Constants.telemetry.saveDouble("CLIMB SM DART Target Posit", Constants.ClimberConstants.kStep1DartUpToBarPosit );

                if(iCompletedStepParts >= 3){
                    status = " Step1: Done; Dart in contact with High Rung; All ministeps are completed";
                    iStep++;                            // all parts are done, next step
                }

                Constants.telemetry.putString("CLIMB SM Desc", status );

                break;                                  // End of the step, jump out go around


            /*******************************************************************
             * In Step2 we will...
             * 1) Pull the Hand in to grab the pipe. Once grabbed it will keep
             *    pulling to ensure that it does not come off befor we want. 
             * 2) Next we will rotet the robot up by pulling Down on the Dart. 
             *    This will move the CG of the robot to reduce swing when we pull
             *    off of pipe 1. 
             ********************************************************************/
            case 2:             
                if (bStepFirstPass) {   // use this to set stuff up, only executed once. 
                    timStepTimer.reset();
                    sStepDescription = "Step2: Grab pipe 2 and swing up robot to pull off.";
                    Constants.telemetry.putString("CLIMBER SM Desc", sStepDescription );
                    gotoLift2.reset();  // reset the Lift2 goto for a new position
                }
                iCompletedStepParts = 0;
                status = "Step2: ";

                // lift 1: Engage brake to prevent us from falling
                ClimbSubsystem.mLift1Lock = true;                     // make sure the brake is engaged

                /******************************************************************
                 * Here the Hand (l2) pulls toward the robot to grab the pipe. 
                 ******************************************************************/
                Inputs.climberLift2Up = true;             // press and hold the button
                Constants.telemetry.saveDouble("CLIMB SM Lift2 Target Posit", Constants.ClimberConstants.kStep2Lift2GrabPipePosit );
                // Hand is moving up the arm towards the bar while position is < where we expect the pipe to be.  
                if( ClimbSubsystem.getLift2Position() < Constants.ClimberConstants.kStep2Lift2GrabPipePosit ){ // move while being pulled up
                    status += " Lift1: Grabbing pipe 2. ";
                    Constants.telemetry.putString("CLIMB SM Desc", status ); // pull down a little, keep it engaged 
                    break;      // go around until we are down here
                } else {                                    // we have grabbed the pipe at this point. 
                    Inputs.climberLift2PowerOverride = .1;  // once grabbed keep pulling at lower rate to maintain contact
                    status += " Lift1: Pipe 2 grabbed, pulling Hand (L2) in slowly, ";
                    iCompletedStepParts++;                  // part one is done
                }

                /*********************************************************************************************
                 * Rotate robot up by pulling down on the Dart. 
                 * This changes the CG of the robot resulting in less swing whe we come off the pipe. 
                 **********************************************************************************************/
                Inputs.climberDartDown = true;            // Pull down of DART to swing up robot
                Inputs.climberDartPowerOverride =  .15;

                Constants.telemetry.saveDouble("CLIMB SM DART Target Posit", Constants.ClimberConstants.kStep2DartSwingUpRobotPosit );


                // position numbers moving up. Keep going until we are above this posit. 
                if( ClimbSubsystem.getDartPosition() <= Constants.ClimberConstants.kStep2DartSwingUpRobotPosit){    // make sure we are up to bar
                    status += " Dart: Rotating Robot up to before release. ";
                    Constants.telemetry.putString("CLIMB SM Desc", status );
                    break;                              // go around again unto Dart is at the bar position
                }else{
                    Inputs.climberDartDown = false;   
                    iCompletedStepParts++;                  // part one is done
                    status += " Dart: Robot has rotated up. ";
                }

                if(  iCompletedStepParts != 2 ){
                    Constants.telemetry.putString("CLIMB SM Desc", status );
                } else {
                    iStep++;
                    Constants.telemetry.putString("CLIMB SM Desc", "Step2: Done!" );
                }

                break;                                  // step complete, jump out go around

                
            /*******************************************************************
             * In Step3 we will...
             * 1) Pull the Hand in to the robot to force us off Pipe 1. 
             * 2) Once off the pipe we will drop before we grab pipe 2. 
             *    Droppibg the robot raises up the arm above pipe 3. 
             ********************************************************************/

            case 3:             
                if (bStepFirstPass) {   
                    timStepTimer.reset();
                    sStepDescription = "Step3: Pull off first pipe";
                    Constants.telemetry.putString("CLIMB SM Desc", sStepDescription );
                    gotoLift2.reset();  // reset the Lift2 goto for a possible new position
                }

                iCompletedStepParts = 0;
                status = "Step 3: ";

                Constants.telemetry.saveDouble("CLIMB SM Lift2 Target Posit", Constants.ClimberConstants.kStep3Lift2PullRobotOffPosit );

                // pull the shuttle towards the robot, this is critical so we want to make sure. 
                // take where we are and where we want to be and return a direction 1=Up, -1=Down, 0=Done
                double direction = gotoLift2.getDirection(ClimbSubsystem.getLift2Position(), // where we are 
                                                        Constants.ClimberConstants.kStep3Lift2PullRobotOffPosit); 

                if( direction > 0){                      // 1-Pull Up towards robot
                    Inputs.climberLift2Up = true;   
                    status += " Lift2: Pulling hand towards robot to pull off the pipe. ";
                    Constants.telemetry.putString("CLIMB SM Desc", status );
                    break;                              // break out until we are where we want to be
                }else if( direction < 0 ){              // -1-Error this should not happen
                    status += " Lift2: Puling hand towards robot. Climb Auto: ERROR: Step3 L2-Shuttle too close the robot. It was not in a position where it should be. ";
                    System.out.println("Climb Auto: ERROR: Step3 L2-Shuttle too close the robot. It was not in a position where it should be.");
                    Constants.telemetry.putString("CLIMB SM Desc", status );
                    bIsDone = true;
                    break;
                }else {                                 // 0-Target reached
                    status += " Lift2: robot off pipe and swinging, ";
                    Constants.telemetry.putString("CLIMB SM Desc", status );
                    iCompletedStepParts++;
                }

                /********************************************************************
                 * We are off of pipe 1. Now drop the robot to reduce the swing. 
                 * We drop robot by telling the arm to move up, or extend the DART. 
                 ********************************************************************/

                Inputs.climberDartUp = true;   // Up on DART let the robot drop
                Constants.telemetry.saveDouble("CLIMB SM DART Target Posit", Constants.ClimberConstants.kStep3DartSwingDownRobotPosit );


                // we are off the pipe, swing the robot down
                if( ClimbSubsystem.getDartPosition() >= Constants.ClimberConstants.kStep3DartSwingDownRobotPosit){    // make sure we are up to bar
                    status += " Dart: moving up to rotate robot down. ";
                }else{
                    Inputs.climberDartUp = false;   
                    iCompletedStepParts++;              // we are down to where we want
                    status += " Dart: Robot rotated down. ";
                } 


                if(iCompletedStepParts == 2){            // all parts are done, next step
                    iStep++;
                    status = "Step 3: Done...";                 
                }           

                Constants.telemetry.putString("CLIMB SM Desc", status );

                break;                                  // step complete, jump out go around

            /*******************************************************************
             * In Step4 we will...
             * 1) Use Lift 2 to pull the robot up to and above Pipe 3. 
             * 2) Once off the pipe we will drop before we grab pipe 2. 
             *    Droppibg the robot raises up the arm above pipe 3. 
             ********************************************************************/

            case 4:             
                if (bStepFirstPass) {   
                    timStepTimer.reset();
                    sStepDescription = "Step4: Get into position above pipe 2 for final hook.";
                    Constants.telemetry.putString("CLIMB SM Desc", sStepDescription );
                }
                status = "Step4: ";
                iCompletedStepParts = 0;
                
                Inputs.climberLift2Up = true;           // Pull towards robot
                Constants.telemetry.saveDouble("CLIMB SM Lift2 Target Posit", Constants.ClimberConstants.kLift2FullInPosit );

                // Pull Robot up to pipe 2
                if( ClimbSubsystem.getLift2Position() <= Constants.ClimberConstants.kLift2FullInPosit){  // we are pulling up to robot
                    status += "Lift2: Pulling Hand full in to robot.";
                    Constants.telemetry.putString("CLIMB SM Desc", status );
                    break;                      // keep doing this until we get to the above the position
                }else{
                    Inputs.climberLift2Up = false;           // stop liftin' up for now
                    iCompletedStepParts++;
                    status += "Lift2: Hand should be pulled all the way in,";
                }

                
                Inputs.climberDartDown = true;   // Up on DART let the robot swing up
                Constants.telemetry.saveDouble("CLIMB SM DART Target Posit", Constants.ClimberConstants.kStep4DartSwingUpRobotPosit );



                // we are off the pipe, swing the robot down
                if( ClimbSubsystem.getDartPosition() <= Constants.ClimberConstants.kStep4DartSwingUpRobotPosit){    // make sure we are up to bar
                    status += " Dart: Rotating robot up to bring bar down, ";
                    Constants.telemetry.putString("CLIMB SM Desc", status );
                    break;
                }else{
                    Inputs.climberDartDown = false;   
                    iCompletedStepParts++;              // we are down to where we want
                    status += " Dart: Rotated robot up, Done ";
                }

                if(iCompletedStepParts == 2){            // all parts are done, next step
                    iStep++;
                    timStepTimer.reset();
                }       
                Constants.telemetry.putString("CLIMB SM Desc", status );

                break;


            case 5:             
                if (bStepFirstPass) {   
                    sStepDescription = "Step5: Pull up the hand on pipe 2 and release to pipe 3.";
                    Constants.telemetry.putString("CLIMB SM Desc", sStepDescription );
                    timStepTimer.reset();
                }

                if( timStepTimer.get() < 1.5 ){
                    Inputs.climberLift2Down = true;           // Pull away from pipe to release 
                    Constants.telemetry.putString("CLIMB SM Desc", "Step5: Pulling hand away from robot off pipe 2.");
                } else {    
                    Constants.telemetry.putString("CLIMB SM Desc", "Step5: Done! Robot should be off the pipe 2.");
                    timStepTimer.reset();
                    iStep++;
                }
                break;                                  // step complete, jump out go around

            case 6:             
                if (bStepFirstPass) {   
                    sStepDescription = "Step6: Drop Robot Down";
                    Constants.telemetry.putString("CLIMB SM Desc", sStepDescription );
                    timStepTimer.reset();
                }

                if( timStepTimer.get() < 1.5){           // wait for 2 seconds
                    Constants.telemetry.putString("CLIMB SM Desc", "Step6: Waiting for robot to slow swing.");
                    break;
                }

                Inputs.climberDartUp = true;   // Up on DART let the robot drop
                Constants.telemetry.saveDouble("CLIMB SM DART Target Posit", Constants.ClimberConstants.kStep3DartSwingDownRobotPosit );


                // we are off the pipe, swing the robot down
                if( ClimbSubsystem.getDartPosition() >= Constants.ClimberConstants.kStep3DartSwingDownRobotPosit) {    // make sure we are up to bar
                    Constants.telemetry.putString("CLIMB SM Desc", "Step6: Rotating robot back down.");
                }else{
                    Inputs.climberDartUp = false;   
                    Constants.telemetry.putString("CLIMB SM Desc", "Step6: Done! Robot should rotated be down.");
                    iStep = 99;  // we go to default step
                }

                break;                                  // step complete, jump out go around


            default:
                    sStepDescription = "Done: Climb Automation is complete.";
                    Constants.telemetry.putString("CLIMB SM Desc", sStepDescription );
                    bIsDone = true;

                }

    }
    

}
