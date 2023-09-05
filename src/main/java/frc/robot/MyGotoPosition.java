package frc.robot;


public class MyGotoPosition {

  double dCurrentDirection = 0.0;
  double dLowBound  = 0.0;
  double dHighBound = 1.0;
  String sName = "";
  int iLabelGroup = kLABEL_GROUP_SIMPLE;

  public static final int kLABEL_GROUP_SIMPLE = 0;
  public static final int kLABEL_GROUP_DOWN_UP = 1;
  public static final int kLABEL_GROUP_LEFT_RIGHT = 2;
  public static final int kLABEL_GROUP_OUT_IN = 3;

  public static final int kLABEL_POSIT_HIT_LOWER_BOUND = 0;
  public static final int kLABEL_POSIT_GOING_LOWER = 1;
  public static final int kLABEL_POSIT_STOPPED = 2;
  public static final int kLABEL_POSIT_GOING_HIGHER = 3;
  public static final int kLABEL_POSIT_HIT_HIGH_BOUND = 4;
  public static final int kLABEL_POSIT_HIT_TARGET = 5;

  static String aryLabels[][] = {
    		    {"[  ", "<<<", "Stop", ">>>", "  ]","[*]" },
            {"Botm", "\\/Down", "Stop", "/\\Up", "Top","[*]" },
            {"[Left", "<<Left", "Stop", ">>Right", "Right]","[*]" },
            {"[In", "<<In", "Stop", ">>Out", "Out]","[*]" }
               };
  
  
  String status = "";
  
  MyGotoPosition(String sName, double dLowBound, double dHighBound ){
    this.sName = sName;
    this.dLowBound  = dLowBound;
    this.dHighBound = dHighBound;
    this.iLabelGroup = kLABEL_GROUP_SIMPLE;
  }

  public void setLabelGroup(int iLabelGroup){
    this.iLabelGroup = iLabelGroup;
  }

  public double getDirection(  double dCurrentPosition, double dTargetPosition ){
    return getPower( 1.0, dCurrentPosition, dTargetPosition );
  }

  public double getPower( double dPower, double dCurrentPosition, double dTargetPosition ){
    
    dPower = Math.abs(dPower);                  // force this to positive

    if( dCurrentDirection == 0.0){							// we have no prefered direction
      
      if( dCurrentPosition == dTargetPosition ){			// no difference	
       	return 0.0;
      }else if( dCurrentPosition > dTargetPosition  ){ 
        dCurrentDirection = -1;		// moving to lowBound
      }else{
        dCurrentDirection = 1;		// moving to highBound
      }
    }

    if( dCurrentPosition < dLowBound )
        System.out.println("MyGotoP: (" + sName + "): " + 
                           			"Current Pos [" + String.valueOf(dCurrentPosition) + "] " + 
                           				"< lowBound [" + String.valueOf(dLowBound) + "]"   );

    if( dCurrentPosition > dHighBound )
        System.out.println("MyGotoP: (" + sName + "): " + 
                           			"Current Pos [" + String.valueOf(dCurrentPosition) + "] " + 
                           				"> highBound [" + String.valueOf(dHighBound) + "]"   );

    if( dCurrentDirection == 1 ){	// moving up
      if( dCurrentPosition >= dHighBound){ 
        status = aryLabels[iLabelGroup][kLABEL_POSIT_HIT_HIGH_BOUND];
        return 0.0;
      }
      if( dCurrentPosition >= dTargetPosition  ){
        status = aryLabels[iLabelGroup][kLABEL_POSIT_HIT_TARGET];
        return 0.0;
      }
    }

    if( dCurrentDirection == -1 ){		// moving down
      if( dCurrentPosition <= dLowBound){
        status = aryLabels[iLabelGroup][kLABEL_POSIT_HIT_LOWER_BOUND];
        return 0.0;
      }

      if( dCurrentPosition <= dTargetPosition  ){
        status = aryLabels[iLabelGroup][kLABEL_POSIT_HIT_TARGET];
        return 0.0;
      }

    }

    if( dCurrentDirection > 0.0)
      status = aryLabels[iLabelGroup][kLABEL_POSIT_GOING_HIGHER];
    else if( dCurrentDirection < 0.0)
      status = aryLabels[iLabelGroup][kLABEL_POSIT_GOING_LOWER];
    else
      status = aryLabels[iLabelGroup][kLABEL_POSIT_STOPPED];
      
    return dPower * dCurrentDirection;			// keep going
    
  }
  
  public String getIndicator(){
    return status;
  }
  
  public void reset(){
    dCurrentDirection = 0.0;
  }
    

}
