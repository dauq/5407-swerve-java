

// just telemetry to look back at files

package frc.robot;

import java.util.ArrayList; 
import java.io.BufferedReader; 
import java.io.FileReader;  

public class PlayBack{
  	int iSettingNumber = 0;		// index of all settign in listSettings
  	int iCurrMoment = 1;		// a moment is a time of 20ms on the robot. 
	int iTotalSettings = 0;
    public boolean bMomentsLoaded = false; 
	
    boolean isDone = false; 

	String pbFilePath = ""; 
  
  	ArrayList<String> listSettings = new ArrayList<String>();
  
	public PlayBack( String sFilePath, String sFileName ){
		pbFilePath =  sFilePath + '/' + sFileName;
		loadMoments();
	}
  
 	public boolean isPlaybackDone(){
      return isDone;
    }
  
  	private void loadMoments(){
   
    	try{
   			BufferedReader bufReader = new BufferedReader(new FileReader(pbFilePath));
    
	 		String line = bufReader.readLine(); 
     		while (line != null) { 
       	  		listSettings.add(line); 
          		line = bufReader.readLine(); 
        	} 
    	bufReader.close();
		iTotalSettings = listSettings.size();
        bMomentsLoaded = true; 

          
    }catch (Exception e){
      System.out.println(e);
      bMomentsLoaded = false; 
    }
      
   }
    
   public int getListSize(){
     return listSettings.size();
   }
      
   public boolean playNextMoment(){	// perfrom all the settings for this action
     
     while( true ){

		if(iSettingNumber >= iTotalSettings){
			 isDone = true;
			 return false;
		}

       String sSetting = listSettings.get(iSettingNumber); // get next robot variable setting in the Actions list

       String field[] = sSetting.split(",");	  	// break up the new Setting on comma Example: "1,3,0.5,INPUT Driver Power"
       												// action, robot field id (map), value to apply, name if the field. 

       int iNewMoment = Integer.parseInt(field[0]);	 // get the Moment Id (field[0]) from this setting.

       if( iNewMoment != iCurrMoment ){				// new moment <> current, end of this moment, get out perfroming actions. 
	     iCurrMoment++;								// this moment is done, get ready for the next moment
         return false;								// all settings in the curr moment are complete.
	   }
	   
       int iRobotFieldId = Integer.parseInt(field[1]);	// get the Robot field Id from field[1].
	   
	   String sSettingValue = "";
	   try{ 
       		sSettingValue = field[2];					// get the value of the setting, all values are a string, 
	   }catch( Exception e){
			iRobotFieldId = 0;
			sSettingValue = "";
		}												// we convert later. 

       updateSettings(iRobotFieldId, sSettingValue); 	// pass this to actually set the variable with the value.     

       iSettingNumber++;								// bump to the next setting in the list. 
	   
      }
     
    }
  
	public static void updateSettings(int iRobotVarId, String sCurrentValue){

		switch(iRobotVarId){
    
    
			case 1: // INPUT Driver Field Centric
				Inputs.fieldCentric = Boolean.parseBoolean( sCurrentValue );
				break;

			case 2: // INPUT Driver Trigger
				Inputs.driverTrigger = Boolean.parseBoolean( sCurrentValue );
				break;

			case 3: // INPUT Driver Power
				Inputs.driverPower = Double.parseDouble( sCurrentValue );
				break;

			case 4: // INPUT Driver Strafe
				Inputs.driverStrafe = Double.parseDouble( sCurrentValue );
				break;

			case 5: // INPUT Driver Turn
				Inputs.driverTurn = Double.parseDouble( sCurrentValue );
				break;

			case 6: // INPUT Intake Deploy
				Inputs.intakeDeploy = Boolean.parseBoolean( sCurrentValue );
				break;

			case 7: // INPUT Intake Retract
				Inputs.intakeRetract = Boolean.parseBoolean( sCurrentValue );
				break;

			case 8: // INPUT Operator Thumb Safety
				Inputs.operatorThumbSafety = Boolean.parseBoolean( sCurrentValue );
				break;
				
			case 9: // INPUT Operator Trigger
				Inputs.operatorTrigger = Boolean.parseBoolean( sCurrentValue );
				break;

			case 10: // INPUT Shooter Full Auto Off
				Inputs.shooterFullAutoModeOff = Boolean.parseBoolean( sCurrentValue );
				break;

			case 11: // INPUT Shooter Full Auto On
				Inputs.shooterFullAutoModeOn = Boolean.parseBoolean( sCurrentValue );
				break;

			case 12: // INPUT Turret Manual Adjust
				Inputs.turretAdjust = Double.parseDouble( sCurrentValue );
				break;

			case 13: // INPUT Lift1 Button Up
				Inputs.climberLift1Up = Boolean.parseBoolean( sCurrentValue );
				break;

			case 14: // INPUT Lift1 Button Down
				Inputs.climberLift1Down = Boolean.parseBoolean( sCurrentValue );
				break;

			case 15: // INPUT Lift2 Button Up
				Inputs.climberLift2Up = Boolean.parseBoolean( sCurrentValue );
				break;

			case 16: // INPUT Lift2 Button Down
				Inputs.climberLift2Down = Boolean.parseBoolean( sCurrentValue );
				break;

			case 17: // INPUT Dart Button Up
				Inputs.climberDartUp = Boolean.parseBoolean( sCurrentValue );
				break;

			case 18: // INPUT Dart Button Down
				Inputs.climberDartDown = Boolean.parseBoolean( sCurrentValue );
				break;

			//default:;

		}
	}
}