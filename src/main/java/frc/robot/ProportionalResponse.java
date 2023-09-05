package frc.robot;

public class ProportionalResponse {

  double dProportion = 0.0;
  double dMaxResponse = 0.0;
  double dMinResponse = 0.0;
  
  double dTarget = 0.0;
  double dTargetLowBound = 0.0;
  double dTargetHighBound = 0.0;
  
  double dWindowLowBound = 0.0;
  double dWindowHighBound = 0.0;
  String status = "";
  double dDiff = 0.0;
  boolean onTarget = false;

  
  public ProportionalResponse( double dTarget, double dTargetWidth, double dWindowWidth,  
              			 double dProportion,  double dMinResponse, double dMaxResponse){
    this.dMinResponse 	= dMinResponse;
    this.dMaxResponse 	= dMaxResponse;
    this.dTarget 		= dTarget; 
    this.dProportion 	= dProportion;

    dTargetLowBound 	= dTarget - dTargetWidth;
    dTargetHighBound 	= dTarget + dTargetWidth;
    dWindowLowBound 	= dTarget - dWindowWidth;
    dWindowHighBound 	= dTarget + dWindowWidth;

  }
  
  public double getCorrection(double dCurrPosition){
    return this.getCorrection( dCurrPosition, this.dTarget);
  }

  public double getCorrection(double dCurrPosition, double newTarget){
    this.dTarget = newTarget;
    onTarget = false; 
    dDiff = this.dTarget - dCurrPosition;

    if( dCurrPosition > dTargetLowBound && dCurrPosition < dTargetHighBound ){
      onTarget = true;
      status = "[*]";
      return 0.0;
    }
      
    // check if out side of bounds, if brign back a MaxPower
    if( dCurrPosition < dWindowLowBound ){			// below the window
      status = ">>>>";
      return dMaxResponse;
    } else if( dCurrPosition > dWindowHighBound ){	// above the window
      status = "<<<<";
      return -dMaxResponse;
    }

    double dProportionedPower = .0;

    if( dCurrPosition >= dWindowLowBound && dCurrPosition < dWindowHighBound ){			// below the window
      dProportionedPower = dDiff * dProportion;  // come back preportionally
    }

    if(dProportionedPower > 0.0 ){
      dProportionedPower = Math.max(dProportionedPower,dMinResponse);
    } else if(dProportionedPower < 0.0 ){
      dProportionedPower = Math.min(dProportionedPower, -dMinResponse);
    }

    return dProportionedPower;
  }
  
  public boolean isOnTarget(){
    return onTarget;
  }

  public double getDiff(){
    return dDiff;
  }
}
  

