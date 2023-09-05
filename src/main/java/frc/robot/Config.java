package frc.robot;


/****************************************************************************************************
* General purpose tool to allow users have variable configs saved in a file on the robot.
* This is prefered to using Prefercnes and you can have all the fields you want in the file
* set as comments uing a # symbol. Then when you need to use the you just uncomment. Doing the same
* in Preferecnes is not an option. Preferecnes can still be used to make changes that need to be done
* quickly. 
*
*/


import java.util.HashMap;
import java.util.Map;
import java.util.TreeMap;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;

/************************
 * Class Constructor. You pass in the file name. Example: /c/MyConfigFile.cfg.
 * In the roboRio it is suggested you use the /c folder as it is available to the 
 * lvuser and only contain info text files.      
 * <p>
 * The config file uses a # symbol to indicate comment. The actual data lines have 2 required parts. 
 * The key is how you refer to your setting in code and the value is a text representation of the data. 
 * These are pipe separated. You can have as much white space as you like. 
 * The last pipe separated field is any comments. This is ignored when read. Below is an example...
 * <pre>    
 * shooter.ip_ClosePosition 	       | 100       | extra stuff
 * shooter.ip_OnTargetPosition         | 
 * shooter.dp_FastUpPower 		       | 30        | more extra stuff
 * shooter.dp_FastDownPower 	       | .5
 * shooter.dp_SlowUpPower		       | .2
 * shooter.dp_SlowDownPower 	       | .2
 *
 * #Camera stuff
 * #D_PickupPanPosition                |
 * #D_PickupTiltPosition               |
 * #D_LoweredPanPosition               |
 * #D_LoweredTiltPosition              |
 * #D_DrivingPanPosition               |
 * #D_DrivingTiltPosition              |
 * #D_TargetingPanPosition             |
 * #D_TargetingTiltPosition            |
 * #d_CameraLookingForwardPWMValue     |
 * #d_CameraLookingRearwardPWMValue    |
 * #I_CameraMaxFPS                     |
 * #S_CameraIpAddress                  |            | IPV 4 IP address
 * #S_CameraUser                       |
 * #S_CameraPassword                   |
 *
 * #Target
 * #I_FovPixelsX                       |
 * #I_FovPixelsY                       |
 *
 * #Telemetry values
 * Tele_FileName                       | FRC272_HH_Telemerty       | string just a file name. tele will add the .xls
 * #Tele_FilePath                      |             | string 
 * Tele_TimestampFile                  | true        | boolean
 * </pre> 
 *
 * You should get a copy of WINScp and load it on your driver station or programming computes. 
 * Then is is very easy to make and save changes on the robot and in your code.
 * <p>
 * This code will automatically try to read the config file when it is Constructed.
 * You can then call the load method anytime to reload the data. 
 * One suggested idea is to call it when you go into disabled init. 
 * Then it is read once every time you go into disable from teleop or auton. 
 * So you can make change, go into disabled and they will be read from the file. 
 * Then you have to all the appropriate method to get them into you variables.
 * See examples below...
 * <pre>
 *    // In robot class
 *     public void disabledInit() {
 *
 *    	telem.restartTimer();
 *    	config.load();
 *    	
 *    	this.telem.loadConfig(config);
 *    }
 *
 *    // In telemetry class
 *    public void loadConfig(Config config){
 *    	
 *        this.sp_FileName = config.getString("Tele_FileName", "telemetry");
 *        this.sp_FilePath = config.getString("Tele_FilePath", "/tmp"); 
 *        this.bp_TimestampFile = config.getBoolean("Tele_TimestampFile", false);
 *        
 *    }
 *
 */

public class Config {

    private Map<String, String> mapColumnData;  /** This is a string key, value map. You put in the key and get the value. */ 
    private Map<String, String> mapNewKeys;     /** This is a string key, value map. You put in the key and get the value. */ 
    private Map<String, String> mapDiffKeys; 	/** These are where live config values differ from defaults. */ 
	String s_FileName;
	String s_NewKeysFileName; 
	String s_DiffKeysFileName;
	
    public Config (String s_UserFilePath, String s_UserFileName){
	    s_FileName 			= s_UserFilePath + "/" + s_UserFileName;
	    s_NewKeysFileName 	= s_UserFilePath + "/" + "new_config_keys.txt";
	    s_DiffKeysFileName 	= s_UserFilePath + "/" + "keys_with_non_default_values.txt";
	    mapColumnData  = new HashMap<String, String>();	// when we load we want to reload this too to clear out the old data
		mapNewKeys 	   = new HashMap<String, String>();    // save all out keys and their default values
		mapDiffKeys    = new HashMap<String, String>();    // save all out keys and their default values
	    load();
    }

/************************
 * Used to open the config file and read in the variables. Everything is saved in a hash table (key value pair).
 * While reading anything that starts with a # will be ignored as well as blank lines even if filled with whitespace.
 * Message are sent to the console if you have it enabled so you can see problems in reading the file.    
 */
	public void load() {

		System.out.println("Config.load(): INFO: Loading data from file [" + this.s_FileName + "]");
	    mapColumnData.clear(); // when we load we want to clear out the old data

    	try{
    		
		    BufferedReader in = new BufferedReader(new FileReader(this.s_FileName));
		    String str;
		    Integer line = 0; 
		    while ((str = in.readLine()) != null){
		    	
		    	line++;
		    	
		    	str = str.trim();
		    	if(str.length() == 0 || str.charAt(0) == '#')
		    			continue;
		    	
		    	// break it up 
		    	String[] fields = str.split("\\|");
		    	
		    	if (fields.length < 2){
		    		System.out.println("Config.load(): ****ERROR Line " + line.toString() + " is not a comment (#) but does not have a pipe (|) delimiter!");
		    		continue;
		    	}	    	
		    	
		    	String key = fields[0].trim();
		    	String value = fields[1].trim();
	  	
		    	if( key.length() == 0){
		    		System.out.println("Config.load(): *WARN: Line " + line.toString() + " Key has no data in it, zero length, ignoring!!!");
		    		continue;
		    	}
		    	
		    	if( value.length() == 0){
		    		System.out.println("Config.load(): *WARN: Line " + line.toString() + " Value has no data in it, zero length, ignoring!!!");
		    		continue;
		    	}
		    	
		    	System.out.println("Config.load(): INFO: Line=" + line.toString() + "  Key=[" + key + "  Value=[" + value + "]"  );
		    	mapColumnData.put(key,value);			// save the data 
		    	
		    }

		    in.close();

		    
		} catch (IOException e) {
    		System.out.println("Config.load(): Issue!!!!: Failed to load the file " + this.s_FileName + 
    				"   Exception:" + e + "  Reason:" + e.getMessage() );
		}

    }


	/**
    * Called to tell Telemetry to save the rows and columns
    * that are the spreadsheet data. Note: It is expected that this is called
    * while in disabledPeriodic mode during testing or at the 
    * end of a competition round.
    * <p>
    * The best way is to have 2 buttons that must be pressed at the same time
    * while in disabledPeriodic mode. Possibly one on the driver stick and the 
    * other on the operator stick. We do not want this to be triggered
    * accidently during a round.
    *<p> 
    * Example:
    * <pre>
    * public void disabledPeriodic() {
    * 
    *	  // bla..bla..bla
    *	
    *     if( inputs.b_SaveTelemetry == true )
    *         telem.saveSpreadSheet();						// once done you cannot save more data there. 
    * }
    *</pre>
    *                      
    */
	public void saveNewKeys(){

		//System.out.println( "In Save New Keys...");			

		int longestKeyLength = 0;
		String key = "";
		String value = "";
		final String spaceFiller = "                                                                                                    ";

		TreeMap<String, String> sortedMap = new TreeMap<>(mapNewKeys);

		for (Map.Entry<String, String> entry : sortedMap.entrySet()){
			key = entry.getKey();
      		System.out.println( "key:" + key + "  Length:" + String.valueOf(key.length()));
			if( key.length()> longestKeyLength)
				longestKeyLength = key.length();
		}

        try {
			FileWriter fileHandle = new FileWriter( s_NewKeysFileName );

			if( mapNewKeys.isEmpty() == true ){
				fileHandle.write("# No new Keys, list is empty..\n");
			} else {

				fileHandle.write("# Below are unused config entries.\n");
				fileHandle.write("# They are here because someone put in a config entry in code to pull it but.");
              	fileHandle.write("# no entry exists for it in the config file. You can copy them and put then in you config file if you want.\n");
				fileHandle.write("# To use them you have to uncomment themm, just saying! :)\n\n\n");
              
				for (Map.Entry<String, String> entry : sortedMap.entrySet()){
					key 	= entry.getKey() + spaceFiller;			// we will truncate to make it nice!!!!
					value 	= entry.getValue() + spaceFiller;		// we will truncate to make it nice!!!!

                  	String sLine = key.substring(0, longestKeyLength) + " | " + value.substring(0,20) + " | "; 
					//System.out.println( "# " + sLine );			
					fileHandle.write( "# " + sLine + "\n");	 // # sets this as a comment
				}
			}
            
            fileHandle.close();						
            
		} catch (IOException e) {
			e.printStackTrace();
		}
 
		saveDiffKeys();

    }
    
    private String getValue( String key ){
    	
        if(mapColumnData.containsKey(key))		
            return mapColumnData.get(key);
        else
        	return "";
    }
    	
    /************************
 * Used to pull an int value from the config file.     
 * You can monitor the console to see if you values was pulled correctly.      
 */
	public int getInt( String key, int defaultValue){

		String sValue = getValue(key);

		if( sValue == ""){											// the key is not present in the file. 
			if(mapNewKeys.containsKey(key) == false)				// check to see if the key is in our New Keys map
				mapNewKeys.put(key,String.valueOf(defaultValue));	// if not add it and the default value. 

			return defaultValue;
		}

		try{ 
			double dValue = Double.parseDouble(sValue);  		// read as double first in case there is decimal
			int retValue = (int) dValue;	

			if(retValue != defaultValue)
				saveDifference(key, String.valueOf(retValue), String.valueOf(defaultValue) );

    		System.out.println("Config.getInt(): INFO: Key=" + key +  "  Config value=[" + sValue + "]");
    		return retValue;
		} catch( Exception e ) {
    		System.out.println("Config.getInt(): ***ERROR converting saved string value to int! Key=" + key +  "  Value String=[" + sValue + "]" + 
    				"   Exception:" + e + "  Reason:" + e.getMessage() );
    		return defaultValue;
		}

	}


    /************************
 * Used to pull a double value from the config file.     
 * You can monitor the console to see if you values was pulled correctly.      
 */
	public double getDouble( String key, double defaultValue){

		String sValue = getValue(key);

		if( sValue == ""){
			if(mapNewKeys.containsKey(key) == false)
				mapNewKeys.put(key,String.valueOf(defaultValue));
				
			return defaultValue;
		}
		
		try{
			double retValue = Double.parseDouble(sValue);

			if(retValue != defaultValue)
				saveDifference(key, String.valueOf(retValue), String.valueOf(defaultValue) );

    		System.out.println("Config.getDouble(): INFO: Key=" + key +  "  Config value=[" + sValue + "]");
    		return retValue;
		} catch( Exception e ) {
    		System.out.println("Config.getDouble(): ***ERROR converting saved string value to double! Key=" + key +  "  Value String=[" + sValue + "]" + 
    				"   Exception:" + e + "  Reason:" + e.getMessage() );
    		return defaultValue;
		}
		
	}


    /************************
 * Used to pull a string value from the config file.     
 * You can monitor the console to see if you values was pulled correctly.      
 */
	public String getString( String key, String defaultValue){

		String sValue = getValue(key);

		if( sValue == ""){
			if(mapNewKeys.containsKey(key) == false)
				mapNewKeys.put(key,String.valueOf(defaultValue));
				
			return defaultValue;
		}

  		System.out.println("Config.getString(): INFO: Key=" + key +  "  Config value=[" + sValue + "]");

		  if(sValue != defaultValue)
			  saveDifference(key, sValue, defaultValue );

		return sValue;	
		
	}

    /************************
 	* Used to pull a boolean (true,false) value from the config file.
 	* You can monitor the console to see if you values was pulled correctly.      
 	*/
	public boolean getBoolean( String key, boolean defaultValue){

		String sValue = getValue(key).toLowerCase();

		if( sValue == ""){
			if(mapNewKeys.containsKey(key) == false)
				mapNewKeys.put(key,String.valueOf(defaultValue));
				
			return defaultValue;
		}


		if(sValue != String.valueOf(defaultValue).toLowerCase()){
			saveDifference(key, sValue, String.valueOf(defaultValue).toLowerCase() );
		}

		if( sValue.equalsIgnoreCase("true")){
    		System.out.println("Config.getBoolean(): INFO: Key=" + key +  "  Config value=[" + sValue + "]");
			return true;
		} else if( sValue.equalsIgnoreCase("false")) {
    		System.out.println("Config.getBoolean(): INFO: Key=" + key +  "  Config value=[" + sValue + "]");
			return false;
		}else {
    		System.out.println("Config.getBoolean(): *WARN: Key=" + key +  "  Value String=[" + sValue + "]" + 
    					"  Not true or false, returning default value." );
			return defaultValue;
		}
	}

	private void saveDifference( String key, String retValue, String  defaultValue ){

		if(mapDiffKeys.containsKey(key) == false){				// check to see if the key is in our Diff Keys map
				String data = retValue + "," + defaultValue;    // package strings seperated by pipe. 
				mapDiffKeys.put(key,String.valueOf(data));		// add packaged string. 
		}
	}

	public void saveDiffKeys(){

		//System.out.println( "In Save New Keys...");			

		int longestKeyLength = 0;
		String key = "";
		final String spaceFiller = "                                                                                                    ";

		TreeMap<String, String> sortedMap = new TreeMap<>(mapDiffKeys);

		for (Map.Entry<String, String> entry : sortedMap.entrySet()){
			key = entry.getKey();
      		System.out.println( "key:" + key + "  Length:" + String.valueOf(key.length()));
			if( key.length()> longestKeyLength)
				longestKeyLength = key.length();
		}

        try {
			FileWriter fileHandle = new FileWriter( s_DiffKeysFileName );

			if( mapDiffKeys.isEmpty() == true ){
				fileHandle.write("# All Config entries are using their default values\n");
			} else {

				fileHandle.write("# Below are Config entries that are returning a non-default value.\n");
				fileHandle.write("# They are here because the config file entry is uncommented and \n");
              	fileHandle.write("# the value in the config file is different\nthan the default set in the code.\n\n");
				fileHandle.write("# This is fine, but at some point you may want to copy the value\n");
				fileHandle.write("# being used and set the code entry to use it as the default.\n\n");
				fileHandle.write("# WHY? If you lost config file and did not know what values were\n");
				fileHandle.write("#      being used you may be in a heap of trouble!!!!  Just saying...\n\n\n");
              
				for (Map.Entry<String, String> entry : sortedMap.entrySet()){
					key 	= entry.getKey() + spaceFiller;			// we will truncate to make it nice!!!!
					String temp = entry.getValue();
					String[] aryValues = temp.split(",");
					String usedValue = "";
					String defaultValue = "";

					try{
						usedValue = aryValues[0];
					} catch(Exception e){
						usedValue = "Used value error";
					}
					usedValue += spaceFiller;

					try{
						defaultValue = aryValues[1];
					} catch(Exception e){
						defaultValue = "default value error";
					}

                  	String sLine = key.substring(0, 50) + " | " + 
					  				usedValue.substring(0,20) + " | " + 
									"default: [ " + defaultValue + " ] " + temp;

					fileHandle.write( sLine + "\n");	 // # sets this as a comment
				}
			}
            
            fileHandle.close();						
            
		} catch (IOException e) {
			e.printStackTrace();
		}
 
    }
    

}
