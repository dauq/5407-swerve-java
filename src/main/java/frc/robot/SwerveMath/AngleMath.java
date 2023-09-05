package frc.robot.SwerveMath;


/**
 * Algorithms related to the angle calculations in swerve drive
 * ideally everything is in radians
 */
public class AngleMath {
    /**
    * calculates the shorter path to take to reach the setpoint, if it's faster to 
    * turn directly or turn the opposite direction and reverse the speed
    * @param current current absolute encoder reading 
    * @param target target radian
    * @param power desired state, power in meters per sec
    */
    public static double[] CalculateProperTurn(double current, double target, double power){
        double originalDifference = Math.abs(getContinuousError(target - current));
        double adjustedDifference = Math.abs(getContinuousError(capEncoderReading(target + Math.PI) - current));

        if (adjustedDifference < originalDifference){
            target = capEncoderReading(target + Math.PI);
            power = -power; 
        }

        double[] results = {current, target, power};

        return results;
    }

    /**
     * Wraps error around for continuous inputs. The original error is returned if continuous mode is
     * disabled.
     *
     * @param error The current error of the PID controller.
     * @return Error for continuous inputs.
     */
    static double getContinuousError(double error) {
        double m_inputRange = 2*Math.PI;
        if (Math.abs(error) > m_inputRange / 2) {
            if (error > 0) {
            return error - m_inputRange;
            } else {
            return error + m_inputRange;
            }
        }
        
        return error;
    }
    
    /**
     * adjusts the encoder reading to match with the expectation of PIDController (-PI, PI)
     * @param encoderInput the encoder reading in radians
     * @return adjusted encoder input, between PI and -PI
     */
    public static double capEncoderReading(double encoderInput){
        if (encoderInput < 0){
            encoderInput = -encoderInput;
            encoderInput = ((encoderInput + Math.PI) % (2*Math.PI)) - Math.PI;
            encoderInput = -encoderInput;
        }else{
            encoderInput = ((encoderInput + Math.PI) % (2*Math.PI)) - Math.PI;
        }
        return encoderInput;
    }
}