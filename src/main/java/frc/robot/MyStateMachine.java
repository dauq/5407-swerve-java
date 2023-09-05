package frc.robot;

import edu.wpi.first.wpilibj.Timer;

/* MyStateMachine
        This is a generic implementation for a state machine. While originally being designed for FTC/FRC Autonomous
        it can be used for many purposes wherever many steps need to be followed in order. It is expected that
        this class will be extended to a new class. That means that all the variables and methods where are available.
        In your extended class you will override the methods auto0 to auto9 with your own versions. This allows
        this to support 10 different auton actions. auton0 is usually saved to indicate auton is not being used.

        When used you will doing some action or process so that on each pass you will call the state machine.
        This uses a step number to determine what state you are in. Step 0 is always the initial state. Step 1 is
        the first step where actions happen. In each step you can do sertain actions during that state.
        When the step is over you change the Step indicator is changed to move you to a new step.


        It recognizes that each step may require some setup. It maintains a flag called bStepFirstPass indicating that
        the step has changed since the last pass.



        */




class MyStateMachine {
    // These are all defined here as null so they can be instantiated when class constructor runs.
     Timer timStepTimer = null;  				// Keeps elapsed time to operation within a single step.

     static int iStep = 0; 									// tell us what step we are in.

     int  iLastStep = -1; 							
    // Keep track of the last step executed. When iStep != iLastStep it means
    // we are starting a new step.  We use this time to start or reset conditions
    // for that new step. Here we make it different than iStep so we can see it
    // at the beginning of the program.

     static boolean bIsDone = false; 						
    // tell us if the state machine is complete. Public so that outside can tell when down.
    // Look at where bAutonIsDone is used in the main class (not here).
    // notice this is public so that it can be seen outside the class.
    // now change it private and look at main code. Anywhere is tis used directly
    // will now be in error. Change it back to public.

     boolean bStepIsDone = false; 				    
    // tell us if the current step is complete. This used by your steps only. 
    // You control it. It is set to false at the start at the beginning 
    // of each pass of a step.

     int iCompletedStepParts = 0;
    // A way to tell if all parts of a step are complete. 
    // It is set to 0 at the start at the beginning of each pass of a step.
    // You manage it!!!
    // If a part is complete, increment it. At bottom of the step
    // test to see if your expected number of completed step matches.
    // If they do then that step is done. 

     boolean bStepFirstPass = false;					
    // A new step is starting, se set this to false to allow is to set the step up.
    // in the first pass.

     double dDelayStart = 0.0;						// allow for a delayed start, in seconds of the auton.

     boolean bStandAloneMachine = false;

     String sStepDescription = "";

     int iAutonId = -1;

    public MyStateMachine() {					 		// Constructor for the class. These are here so they are instantiated when class is created.
        timStepTimer = new Timer();
        timStepTimer.start();
        reset();								 		// Just making sure.
    }

    public  void reset(){							// reset the state machine so it can be used again if necessary.
        bIsDone = false;
        iStep = 0;
        iLastStep = -1;								// ensure this is different than iStep so we can tell the step changed.
        timStepTimer.reset();
        // no need to restart timers here as they are reset in steps.
    }

    public static boolean isDone(){
        return bIsDone;

    }

    public static int getStep(){
        return iStep;
    }

    public  double setDelayStartTime (double dDelaySeconds) {
        if( dDelaySeconds < 0.0 )                           //making sure delay is not negative
            dDelaySeconds = 0.0;

        dDelayStart = dDelaySeconds;

        return dDelayStart;
    }

    public  double adjustDelayStartTime( double dAdjustment ){   // here we and adjust delay up or down..
        dDelayStart += dAdjustment;                        // adjust the setting
        return setDelayStartTime( dDelayStart );           // now run it through here to make sure not < 0.0
    }

    public  double getDelayStartTime( ){
        return dDelayStart;
    }

    // Method defined here will be over ridden in the StateMachine class that extends this.
    // That calss will create their own version fo this method. When it is called by
    // the method executeSwitchSteps the overridden one should be executed basicallty passing
    // control to it. If no other switchSteps method is created then the local one here will run
    // and display the message below.
    public  void switchSteps(){

        System.out.println("*** StateMachine.stepList: Was not overridden by your instance of this class.!!!!***");
        bIsDone = true;

    }

    // Method used to make one pass through a standalone StateMachine Switch stack.
    public  void executeSwitchSteps(){			// execute the State machine as a single acting SM.

        updateSM();

        switchSteps();

    }

    public  void updateSM(){							// update the State machine for a single pass.
        // the class remembers the state of the pass after it is done.
        if( bIsDone == true)
            return;
                                                    // reset these becasue we are starting a pass at a step
        bStepIsDone = false;                        // reset to false 
        iCompletedStepParts = 0;                    // reset to 0 

            
        // VERY IMPORTANT AREA: We determine is we are in the first pass of a step.
        if( iStep != iLastStep){					// check if the step changed since the last pass, if change we are in a new step.
            bStepFirstPass = true;					// reset to true every time the step number changes. This is a new step.
            timStepTimer.reset();					// step changed, restart the step timer.
        } else {
            bStepFirstPass = false;					// Step did not change so we are not in the first pass of the step.
        }

        iLastStep = iStep;						    // Now we can save the current step to iLastStep to test again in the next pass.


        if( iStep == 0 && dDelayStart != 0.0){	// Sometimes we want to delay start our autonomuos because an alliance member
            // will be in our way.
            if( bStepFirstPass == true ){
                System.out.printf("StateMachine: Delaying start for %.2f seconds.\n", dDelayStart );
            }

            if( timStepTimer.get() < dDelayStart  )
                return;

        }

    }

    // This is used to execute the auto# method call for by the iAutoToRun number.
    public  void executeAuton(int iAutoToRun){		// execute the State machine for a single pass.
        // the class remembers the state of the pass after it is done.
        if( bIsDone == true)
            return;

        updateSM();

        switch(iAutoToRun){							// these are the 10 autons we are allowed to run.
            case 0:
                auton0();
                break;

            case 1:
                auton1();
                break;

            case 2:
                auton2();
                break;

            case 3:
                auton3();
                break;

            case 4:
                auton4();
                break;

            case 5:
                auton5();
                break;

            case 6:
                auton6();
                break;

            case 7:
                auton7();
                break;

            case 8:
                auton8();
                break;

            case 9:
                auton9();
                break;

            default:
                bIsDone = true;
                System.out.println("*** StateMachine.Invalid auto number was selected.***");
                System.out.println("*** StateMachine.Setting bIsDone to true.***");

        } // end of dispatcher method, we return back to caller here.

    }

    public  void auton0(){						// this is the default indicating no auton

        System.out.println("*** Auton: 0, No Auton Was Selected ***");
        bIsDone = true;

    }

    public  void auton1(){						// this will be over written in instance.

        System.out.println("*** StateMachine.auto1: Was not overridden by your instance if this class.!!!!***");
        System.out.println("*** StateMachine.Setting bIsDone to true.***");
        bIsDone = true;

    }

    public  void auton2(){						// this will be over written in instance.

        System.out.println("*** StateMachine.auto2: Was not overridden by your instance if this class.!!!!***");
        System.out.println("*** StateMachine.Setting bIsDone to true.***");
        bIsDone = true;

    }

    public  void auton3(){						// this will be over written in instance.

        System.out.println("*** StateMachine.auto3: Was not overridden by your instance if this class.!!!!***");
        System.out.println("*** StateMachine.Setting bIsDone to true.***");
        bIsDone = true;

    }

    public  void auton4(){						// this will be over written in instance.

        System.out.println("*** StateMachine.auto0: Was not overridden by your instance if this class.!!!!***");
        System.out.println("*** StateMachine.Setting bIsDone to true.***");
        bIsDone = true;

    }

    public  void auton5(){						// this will be over written in instance.

        System.out.println("*** StateMachine.auto0: Was not overridden by your instance if this class.!!!!***");
        System.out.println("*** StateMachine.Setting bIsDone to true.***");
        bIsDone = true;

    }

    public  void auton6(){						// this will be over written in instance.

        System.out.println("*** StateMachine.auto6: Was not overridden by your instance if this class.!!!!***");
        System.out.println("*** StateMachine.Setting bIsDone to true.***");
        bIsDone = true;

    }

    public  void auton7(){						// this will be over written in instance.

        System.out.println("*** StateMachine.auto7: Was not overridden by your instance if this class.!!!!***");
        System.out.println("*** StateMachine.Setting bIsDone to true.***");
        bIsDone = true;

    }

    public  void auton8(){						// this will be over written in instance.

        System.out.println("*** StateMachine.auto8: Was not overridden by your instance if this class.!!!!***");
        System.out.println("*** StateMachine.Setting bIsDone to true.***");
        bIsDone = true;

    }

    public  void auton9(){						// this will be over written in instance.

        System.out.println("*** StateMachine.auto9: Was not overridden by your instance if this class.!!!!***");
        System.out.println("*** StateMachine.Setting bIsDone to true.***");
        bIsDone = true;

    }



}



