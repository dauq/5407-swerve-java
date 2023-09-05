package frc.robot;

import edu.wpi.first.wpilibj.Timer;

public class MyTimedPower {
  
    boolean currentState = false; //This is true or false depending on the state of the button
    private Timer timDuration;
    double timeout = 0.0;                       //Amount of time power is actually applied
    boolean wasThisEverStarted = false;         //first time reached, we reset the timer
    double truePower = 0.0;
    double falsePower = 0.0;

    public MyTimedPower(double requestedTimout, double passedTruePower, double passedFalsePower) {
      timeout = requestedTimout;
      truePower = passedTruePower;
      falsePower = passedFalsePower;
  
      timDuration = new Timer(); //Creating a new version of the Timer class but named timDuration
      timDuration.start();
    }
  
    // overloaded version of constructor.  Here "this." refers to the above real constructor
    public MyTimedPower(double requestedTimout, double passedTruePower) { 
      this( requestedTimout, passedTruePower, -passedTruePower);          
    }

    public void resetTimer(){
      timDuration.reset();
    }

    public double getTime(){
      return timDuration.get();
    }
  
    public boolean wasThisEverStarted(){
      return wasThisEverStarted;
    }


    public double getPower(boolean buttonState) {
      
      if( wasThisEverStarted == false){
        timDuration.reset();
        wasThisEverStarted = true;
      }
      
      if (buttonState != currentState){
        timDuration.reset();
        currentState = buttonState;
      }
      
      if (timDuration.get() <= timeout){
        if (buttonState == true) {
          return truePower;
        } else {
          return falsePower;
        }
      } else {
        return 0.0;
      }
      
    }
    
    
  }
  