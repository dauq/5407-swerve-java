package frc.robot;

public class RampPower{
	
	public double dMaxPower = 0.0;
	public double dMinPower = 0.0;
	public double dLastPower = 0.0;
	public double dRampIncrements = 0.0;
	public double dRampTime = 0.0;


	public RampPower(double dMaxPower, double dMinPower, double dRampTime){
		this.dMaxPower = dMaxPower; 
		this.dMinPower = dMinPower ;
		this.dRampTime = dRampTime;
		this.dRampIncrements = (dMaxPower-this.dMinPower) / (50*this.dRampTime);

    }

	public double getRampedPower(double dPower){

      	
		if( dPower < 0.0){
          	if( dLastPower > -dMinPower){
              	dLastPower = -dMinPower;
            } else {
              	dLastPower -= this.dRampIncrements;
				dLastPower  = Math.max(dLastPower,-dMaxPower);
            }
          
		}else if(dPower > 0.0){
          	if( dLastPower < dMinPower){
              	dLastPower = dMinPower;
            } else {
              dLastPower += this.dRampIncrements;
              dLastPower = Math.min(dMaxPower,dLastPower);
            }
		}else{
			dLastPower = 0.0;
		}

      return dLastPower;
	}

	public void printDebug(){
		System.out.println( 
			String.format("Max: % 8.4f, Min: % 8.2f, Last: % 8.2f, Inv: % 2.2f", 
				this.dMaxPower, this.dMinPower, this.dLastPower, this.dRampIncrements)
		);
	}
	
}
