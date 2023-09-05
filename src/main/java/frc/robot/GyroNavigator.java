package frc.robot;

import edu.wpi.first.math.controller.PIDController;

public class GyroNavigator {

    double maxPower = .50;

    PIDController pidControl;

    private double kP = Constants.GyroNavigateConstants.kPID_P;  // add to constants
    private double kI = Constants.GyroNavigateConstants.kPID_I;  // add to constants
    private double kD = Constants.GyroNavigateConstants.kPID_D;  // add to constants

    public GyroNavigator(){
        pidControl = new PIDController(kP, kI, kD);
    }
    
    public double calculatePower( double gyroAngle, double targetAngle ){
        double power = pidControl.calculate( gyroAngle, targetAngle ) * 
                                    Constants.GyroNavigateConstants.kPowerProportion;

        if( power > 0.0 )
            power = Math.max(power, Constants.GyroNavigateConstants.kMinPower );
        else if( power < 0.0 )
            power = Math.min(power, Constants.GyroNavigateConstants.kMinPower );

        if(Math.abs(power) < Constants.GyroNavigateConstants.kPowerDeadband) {         // dead band 
            power = 0.0;
        }

        return power;
    }


}
