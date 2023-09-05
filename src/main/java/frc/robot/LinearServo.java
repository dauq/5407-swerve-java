package frc.robot;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;


public class LinearServo extends Servo{
 double m_speed = 0.8;
 double m_length;
 double setPos = 0;
 double curPos = 0;
 /**
 * Parameters for L16-R Actuonix Linear Actuators
 *
 * @param channel PWM channel used to control the servo
 * @param length max length of the servo [mm]
 * @param speed max speed of the servo [mm/second]
 */
 public LinearServo(int channel, int length/*, int speed*/) {
    super(channel);
    setBounds(2.0, 1.8, 1.5, 1.2, 1.0);
    m_length = length;
 }
 /**
Miniature Linear Servo Actuators -
User Guide (Rev 1)
Table of Contents Page 10
wcproducts.com

 * Run this method in any periodic function to update the position estimation of your
servo
 *
 * @param setpoint the target position of the servo [mm]
 */
 public void setPosition(double setpoint){


    
 }

 public void stopServo(){
     setSpeed(0);
 }

 public void runServo(double power){

     if (power > 0){
        setSpeed(1);
     } else if (power < 0){
        setSpeed(-1);
     } else {
        setSpeed(0);
     }
 }

 double lastTime = 0;
 /**
 * Run this method in any periodic function to update the position estimation of your
servo
 */
 public double time(){
     return Timer.getFPGATimestamp();
 }

 public void updateCurPos(){
    double dt = Timer.getFPGATimestamp() - lastTime;
    if (curPos > setPos + m_speed * dt){
        curPos -= m_speed *dt;
    } else if(curPos < setPos - m_speed *dt){
        curPos += m_speed *dt;
    }else{
        curPos = setPos;
    }

    lastTime = Timer.getFPGATimestamp();
 }
 /**
 * Current position of the servo, must be calling {@link #updateCurPos()
updateCurPos()} periodically
 *
 * @return Servo Position [mm]
 */
 public double getPosition(){
 return curPos;
 }

 /**
 * Checks if the servo is at its target position, must be calling {@link #updateCurPos()
updateCurPos()} periodically
 * @return true when servo is at its target
 */
 public boolean isFinished(){
   return curPos == setPos;
 }
}