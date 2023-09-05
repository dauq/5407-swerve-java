
package frc.robot;
import edu.wpi.first.wpilibj.motorcontrol.Spark;


public final class LEDDisplay {

  private static Spark m_ledDisplay = new Spark(3);

  public static final double kLEDDefault           = Constants.config.getDouble("LED_Default", 0.49);        // SCH Blue/Yellow chase
  public static final double kLEDOnTarget          = Constants.config.getDouble("LED_OnTarget",-0.07);       // SCH Blue
  public static final double kLEDShooting          = Constants.config.getDouble("LED_Shooting",-0.09);       // SCH Yellow
  public static final double kLEDEndGameStarted    = Constants.config.getDouble("LED_EndGameStarted",-0.35); // Larson Scanner, Red
  public static final double kLEDEndGameChange     = Constants.config.getDouble("LED_EndGameChange", 0.57);  // Pink
  public static final double kLEDClimbingSlow      = Constants.config.getDouble("LED_ClimbingSlow",-0.25);  // Heartbeat, Red
  public static final double kLEDClimbingMedium    = Constants.config.getDouble("LED_ClimbingMedium",-0.31);// Light Chase, Red
  public static final double kLEDClimbingFast      = Constants.config.getDouble("LED_ClimbingFast", -0.11); // Strobe, Red
  public static final double kLEDClimbDone         = Constants.config.getDouble("LED_ClimbDone", 0.99);     // Rainbow, Party Palette



  public LEDDisplay(){}   // all is static so no constructor


  public static void periodic(){

    double displayPWMValue = kLEDDefault;


      if( !Inputs.masterEndgameArm ){       // normal operation

        if(Inputs.shooterTargettingAuto)
          displayPWMValue =kLEDOnTarget;

        if(Inputs.driverTrigger || (Inputs.operatorTrigger && Inputs.operatorThumbSafety)){
          displayPWMValue =kLEDShooting;

        }

      } else if(Inputs.masterEndgameArm) {  // we are in the end game  

        displayPWMValue = kLEDEndGameStarted;

        if( Inputs.climberLift1Release ){       // quick button release
          displayPWMValue = kLEDEndGameChange; // blink pink on the release

        } else if( !Inputs.masterClimbAutoEnabled ){  // NOT using Climber automation, button not pressed

            if( Inputs.climberLift1Down || Inputs.climberLift1Up ){
              displayPWMValue = kLEDClimbingSlow;
            } else if( Inputs.climberDartUp || Inputs.climberDartDown ){
              displayPWMValue = kLEDClimbingMedium;
            }

        } else{

          if( ClimberAutomation.isDone() ){
              displayPWMValue = kLEDClimbDone;
          } else {
          switch( ClimberAutomation.iStep ){
              case 1: // pulling robot up
                displayPWMValue = kLEDClimbingSlow;
                break;

              case 2: // get Hand caught in the pipe, rotate robot up
              case 3: // pull robot off pipe 1, rotate robot down
                displayPWMValue = kLEDClimbingMedium;
                break;

              case 4: // grab pipe 3
              case 5: // pill robot off of pipe 3
                displayPWMValue = kLEDClimbingFast;
                break;

              case 6:   //robot swinging on pipe 3
              case 99:  // steps all played out, default, we are done
                displayPWMValue = kLEDClimbDone;
                break;

          }
        }    
      }

    }

    m_ledDisplay.set(displayPWMValue);  // write the value to the display

  }


}
