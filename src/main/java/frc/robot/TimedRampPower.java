package frc.robot;

import edu.wpi.first.wpilibj.Timer;

/****************************************************************************************
  TimedRampPower
  
  This is designed to allow you to calculate power for a device over a specific 
  amount of time while ramping it up in the beginning, running for a certain
  time and ramping it down as you approach the end time. The max power, min power,
  ramp up and down of overall percent of time is all magaged by you.
  
  If you have a non-zero ramp up time this will calculate increases for each 
  loop of the robot. If the calculated value is < min, min power will be applied. 
  The same process is followed on the ramp down power. This can be very useful
  to be sure that over time you will drive about the same distance. 
 
  This same class can be reset and reconfigured with different settings if you
  need to. You can create may of these if need be to cover multiple motors. 
  
  Parameters
    double dTotalTime    - Total seconds you want to run this for. 
    double *dMaxPower    - Max power to apply.  
    double *dMinPower    - Min power to apply. 
    double **dRampUpPct  - Percent of time you want to ramp up the power. (1.0 to 0.0)
    double **dRampDownPct- Percent of time you want to ramp down power. 0.0 no ramp down. 
 
   * If either of these powers are negative then both will be negative.
     Negative means the motor will go in the reverse direction.
     
   ** These will always be forced to positive. 
     
 ***************************************************************************************/

public class TimedRampPower{

    private boolean bIsDone = false;
  	private double dTotalTime = 0.0;
	private double dMaxPower = 0.0;
	private double dMinPower = 0.0;
    private double dRampUpPct = 0.0;
    private double dRampUpTimeEnd = 0.0;
    private double dTotalRampUpTime = 0.0;
    private double dRampDownPct = 0.0;
    private double dRampDownTimeStart = 0.0;
    private double dTotalRampDownTime = 0.0;
    private double dDirection = 1.0;
    private double dPower = 0.0;
  
    private double dPctTotalTime = 0.0;

  	Timer timDuration;
  
	/***************************************************************************
     * Class constructor
     ***************************************************************************/
	public TimedRampPower(double dTotalTime, double dMaxPower, double dMinPower, 
                     double dRampUpPct, double dRampDownPct){

      	timDuration = new Timer();  // run whether it is being used or not 
        timDuration.start();
      
        reconfigure(dTotalTime, dMaxPower, dMinPower, dRampUpPct, dRampDownPct);

    }

  	/****************************************************************************
      Overloaded version that uses a default of .10 ramp up and down values.
     ****************************************************************************/
  
  	public TimedRampPower(double dTotalTime, double dMaxPower, double dMinPower){ 
      	this(dTotalTime, dMaxPower, dMinPower, .10, .10);
    }

  	
	/***************************************************************************
     * Restart - Restart the process with the same parameters but changing 
     *           the total time of the process. 
     ***************************************************************************/
  	public void restart(double dTotalTime){ // restart this using a different time period
		this.dTotalTime = dTotalTime;
        timDuration.reset();
    }

  	/**************************************************************************
  	   Restart - Overloaded version that keeps all parameters the same. 
                 It just resets the time. 
     *************************************************************************/
  	public void restart(){ 					// restart the timer only
		this.restart( this.dTotalTime );	
    }


	/***************************************************************************
     * Reconfigure - This allows you to completely reconfigure the deployed class
     *               without rebuilding it. 
     ***************************************************************************/
     public void reconfigure(double dPassedTotalTime, 
                     double dPassedMaxPower, double dPassedMinPower, 
                     double dPassedRampUpPct, double dPassedRampDownPct){

 		    bIsDone 	= false;
        dTotalTime 	= dPassedTotalTime; 
        dMaxPower 	= dPassedMaxPower; 
        dMinPower 	= dPassedMinPower;

        dDirection = 1.0;
      	if( dMaxPower < 0.0 ){                                  // if max < 0, reverse direction
          dMaxPower = Math.abs(dMaxPower);		                  // save these as positive so code works
          dMinPower = Math.abs(dMinPower);
          dDirection = -1.0;				                            // save reverse direction 
                                                                // we apply the reverse direction 
                                                                // at the end of out power calculations
        }

        dRampUpPct 	 = Math.abs(dPassedRampUpPct);	// make sure always positive
        dRampDownPct = Math.abs(dPassedRampDownPct);

  		  dRampUpPct 	= Math.max(.000, dRampUpPct );  // apsolute min value so math works        	
	    	dRampUpPct 	= Math.min(.999, dRampUpPct );  // we don not use 1.0 here as we test for that seperatepy        	
        dRampUpTimeEnd 	 = dTotalTime * dRampUpPct;
        dTotalRampUpTime = dRampUpTimeEnd;  

        dRampDownPct = Math.max(.000, dRampDownPct );          	
        dRampDownPct = Math.min(.999, dRampDownPct );          	
        dRampDownTimeStart = dTotalTime - (dTotalTime * this.dRampDownPct);
        dTotalRampDownTime = dTotalTime - dRampDownTimeStart;  


    }
      
  
	/***************************************************************************
     * getRampedPower - Return the appropriate power based upon where you 
     *                  are in the total time.
     *                  Note: All calculatios are doen in positive power. 
     *                        If this in reverse then direction will be 
     *                        set to -1.0 and be applied at the end. 
     * Parameters:
     * seconds          Pass the seconds from your total ramp time.
     *
     ***************************************************************************/
	public double getRampedPower(double dSeconds){
  
      dPower = 0.0;
		  dPctTotalTime = 0.0;
      
        if( dSeconds > dTotalTime ){
          bIsDone = true;
          return 0.0;
        }
      
      	if( dSeconds <= dTotalRampUpTime ){					                      // ramping up low to high
          	if( dRampUpPct <= 0.0){							                          // no ramp up time
              dPower = dMaxPower;                                         // return full power
            } else {
              dPctTotalTime = dSeconds / dTotalRampUpTime;                // get percent of ramp up time
              dPower = dPctTotalTime * dMaxPower;                         // calcultate ramp up power
            }

        }else if( dSeconds >= dRampDownTimeStart){		                    // ramping down

          if( dRampDownPct <= 0.0){                                       // no ramp down power
            dPower = dMaxPower;                                           // return full power
          }else{
            double dRampSeconds = dSeconds - dRampDownTimeStart;          // how many seconds am I into ramp down time
            dPctTotalTime = 1.0 - (dRampSeconds / dTotalRampDownTime);    // get percent of ramp down time 
                                                                          // then sub from 1.0 to ramp down from hi to lo. 
            dPower = dPctTotalTime * dMaxPower;                           // calulate power besed on percent. 
          }
          
        }else{                                                            
          dPower = dMaxPower;								                              // in between, run at maxPower
          dPctTotalTime = 1.0;                                            // set this to keep cals cool at max Power
        }

        dPower = Math.min(dPower, dMaxPower);                             // return which ever is lower
        dPower = Math.max(dPower, dMinPower);                             // return which ever is higher
                                                                          // this is why we do not do cals in negative power.
      
        return dPower * dDirection;                                       // add direction back in here, -1. = reverse
	}

	/***************************************************************************
     * getRampedPower - Overloaded version. Returns the appropriate power based
     *                  upon where you are in the total time.
     * Parameters:
     * None             Not using your timer, will use the classes internal timer
     *
     ***************************************************************************/
	public double getRampedPower(){
  
      return this.getRampedPower(timDuration.get());  // use the classes timer

    }
  
	/***************************************************************************
     * isDone - Time is up. Calls continued calls will return 0.0 power. 
     ***************************************************************************/
    public boolean isDone(){
      return bIsDone;
    }
  
  	// used for testing
    public double getPctTime(){
      return dPctTotalTime;
    }
}
