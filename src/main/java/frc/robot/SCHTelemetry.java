package frc.robot;

import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Date;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/****************************************************************************************************
* General purpose tool to allow user to record and save robot variable data to .xls spreadsheet.
*
*<p>
*<pre>
* Changes ********************************************************************************************
*</pre>
*<p>			 
* @author 
* @version 
*   
*/
public class SCHTelemetry { 


    private String[] listRows;			
    private List<String> listNewHeadings;
    private List<String> listColumns;		   /** Arrays of columnNames, think ladder. Each step is data. They are numbered starting at 0.
     											* Order is maintained. This is how the column values can be added in any order and be
     											* printed in the correct column. 
     											*/
    
    private Map<String, String> dictColumnData; /** This is a string key, value map. You put in the key and get the value. */ 
    private int i_TotalRows;
    
    private FileWriter fileHandle;
    private Timer  tim_Time;
    public boolean  b_TimestampFile;        // Do you want to time stamp the output file?
    private String  s_FileName;			    /** Default: telemetry.<b>Note: xls is added to the end.</b> */
    private String  s_FilePath;				/** Default: /tmp folder.(linked to /var/volitile/tmp) Lost after reboot, /home/lvuser is writable.*/
    private String  s_ColumnsFileName = 
                    "TelemetryColumns.txt";//** file where column headings are stored */
    private Boolean b_FullDetails;			/** Do you want full details file name used at events*/

    public final int ki_MaxColumns 	= 300;				/** Number of columns. Increase if you need to. */ 
    public final int ki_MaxRows 	= 50  * 60 * 3; 	/** Default 9000: 50 times per second * 60 seconds * 3 minutes */

    private String s_TelemType = "new";
    private String s_FileFormat = ".csv";   // was ".xls";
    private String s_FieldDelimiter = ",";  // was "\t";
    private Boolean supressSmartDashbaordDisplay = true;

    private Boolean sawRecordMarker = false;


	/** Class Contructor initialize you variables here. 
	 * Here the size of the arrays can be adjusted to allow for more columns
	 * and more rows. The constants ki_MaxRos and ki_MaxColumns control that.
    */
    
    public SCHTelemetry(String sPassedFilePath, String sPassedFileName, String sPassedColumnsFileName ) {

    	tim_Time = new Timer();
    	tim_Time.start();
    	

        listColumns = new ArrayList<>();
        listNewHeadings = new ArrayList<>();
        dictColumnData = new HashMap<String, String>();
        b_TimestampFile =  false;
        s_FilePath = sPassedFilePath;					
        s_FileName = sPassedFileName;
        s_ColumnsFileName = sPassedColumnsFileName;

        loadConfig();                                   // load my local config entries for telemetry

        createNewList();
        loadColumnNames();
 
    }


    public void loadColumnNames(){

        String fullFilePath = s_FilePath + "/" + s_ColumnsFileName;
        System.out.println("Telemetry.load(): INFO: Loading loading Columns from file [" + fullFilePath + "]");

        try{
            
            BufferedReader in = new BufferedReader(new FileReader(fullFilePath));
            String columnName;
            Integer line = 0; 
            while ((columnName = in.readLine()) != null){
                
                line++;
                
                columnName = columnName.trim();

                if( columnName.startsWith("#") )                                 // skip comments in the file
                    continue;

                if(columnName.length() == 0 ) {                                  // empty string leave a blank column
                    columnName = "mt " + line.toString();                  
                }                                                        
                
                System.out.println("Telemetry.load(): INFO: Heading [" + columnName + "]"  );
                listColumns.add(columnName);
                
            }

            in.close();

            
        } catch (IOException e) {
            System.out.println("telemetry.load(): Issue!!!!: Failed to load the headings file " + fullFilePath + 
                    "   Exception:" + e + "  Reason:" + e.getMessage() );
        }

    }


    public void setCSVFileFormat( Boolean bSetCSVFormat ){

        if( bSetCSVFormat == true ){
            s_FileFormat = ".csv";
            s_FieldDelimiter = ",";         // comma delimited file
        } else {
            s_FileFormat = ".xls";
            s_FieldDelimiter = "\t";        // tab delimited file
        }

    }

    public void supressDashBoardDisplay( boolean supress){
        supressSmartDashbaordDisplay = supress;
    }

    private void createNewList(){
        i_TotalRows = 0;
        listRows = new String[ ki_MaxRows + 2 ];
        s_TelemType = "new";
        sawRecordMarker = false;
    }

    public void restartTimer() {							// they can restart our internal time ever time we switch modes. 
        tim_Time.reset();
    }

    private void saveColumnName(String columnName){

      if( listColumns.contains(columnName) == true)       // we already know about this column
          return;
    
      if( listNewHeadings.contains(columnName) == false){      // is this not  on our headings list?
              listNewHeadings.add(columnName);                 // save it if not. 
      }

    }

    /**
    * Called to add a column header to the Telemetry object. The field should   
    * have spaces between words to allow them to easily word wrap in 
    * Excel. The exact spelling is needed to add column data.<br> 
    * Example: I Left Driver Power 
    * <b>Note: The I tells us this is from the Inputs Class.</b>
    * <p>
    * The order these are entered in will be the columns of the
    * final spread sheet. The order the data is add has no impact on this.
    * <p>
    * It is assumed that you will add a method to each class like 
    * addTelemetryHeaders(). In that class you will add the different fields
    * you want to track. This is best called in the constructor of the Robot
    * class after all the other classes have been initialized.
    * <p>
    * Example:
    * <pre>
    *   public void addTelemetryHeaders(LCTelemetry telem ){
    *       telem.addColumn("A Program Number");
    *       telem.addColumn("A Started");
    *       telem.addColumn("A Step Timer"); 
    *   }
    * <p>
    * @param	columnName string at the top of this column.  
    */
    public void addColumn(String columnName) {				// add a column to the list of columns
        //listColumns[i_listColumnsIndex++] = columnName;		// this is an array so we use an index i_listColumnsIndex++ to tell use where to put it 
        return;                                             // ignored in favor of static column name
        //listColumns.add(columnName);	
    }														// the ++ on the right tells Java to use the current value but then 
    														// increment AFTER you use it. 
    														// If it was ++i_listColumnsIndex it means increment and use value. 

    /**
    * Called to add an Integer to a column in the Telemetry instance.
    * The columnName is the exact spelling of the name entered in an  
    * addColumn call. The order this data is added will not change the
    * order of the columns.
    *<p>
    * Example:
    *<pre>
    *   public void writeTelemetryValues(LCTelemetry telem ){
    *       telem.saveInteger("A Program Number",this.ip_ProgramNumber);
    *       telem.saveTrueBoolean("A Started",this.b_Started);
    *       telem.saveTrueBoolean("A Is Done",this.b_IsDone); 
    *       telem.saveDouble("A Auton Timer",this.tim_AutonTimer.get());
    *       telem.saveString("A Program Desc",this.s_ProgramDescription);
    *       telem.saveString("A Step Desc",this.s_StepDescription);
    *       telem.saveInteger("A Step",this.i_Step);
    *       telem.saveDouble("A Step Timer",this.tim_StepTimer.get());
    *   }
    *</pre>
    * <p>		
    * @param	columnName string at the top of this column.  
    * @param	value Integer number to be written.  
    */
    public void saveInteger(String columnName, int value){
        dictColumnData.put(columnName,  String.format("%s",value) );		// format as an integer
        saveColumnName(columnName);
    }
    
    public void putNumber( String columnName, int value, boolean forceDisplay){ // integer version
        saveInteger(columnName, value );		                                
        
        if( !supressSmartDashbaordDisplay || forceDisplay )
            SmartDashboard.putNumber( columnName, value  );
    }

    public void putNumber( String columnName, int value){            // overloaded version, no forced display
        putNumber( columnName, value, false);
    }

    
    
    /**
    * Called to add an Double float point number to a column in the
    * Telemetry instance. The columnName is the exact spelling   
    * of the name entered in an addColumn call. The order this data 
    * is added will not change the order of the columns. The resulting 
    * data will have 2 decimal points of precision.
    * <p>
    * Default precision is .2f or 2 decimal places. To save with more
    * precision convert to string and save as a sting. See saveString.
    *  
    * 
    * @param	columnName string at the top of this column.  
    * @param	value Double number to be written.  
    */
    public void saveDouble(String columnName, double value, int decimals ){
        String formatPercision = "%." + String.valueOf(decimals) + "f";
        dictColumnData.put(columnName,   String.format(formatPercision,value) );		// format as floating point with x decimals. 
        saveColumnName(columnName);

    }

    public void saveDouble(String columnName, double value){
        saveDouble(columnName, value, 4 );		                // format as floating point with 4 decimals. 
    }

    public void putNumber( String columnName, double value, boolean forceDisplay){    // save and show on Dashboard
        saveDouble(columnName, value, 4 );		                // format as floating point with 4 decimals. 
        if( !supressSmartDashbaordDisplay || forceDisplay )
            SmartDashboard.putNumber(columnName, value );
    }

    public void putNumber( String columnName, double value){    // overloaded version for doubles
        putNumber( columnName, value, false);
    }

    /**
    * Called to add a string to a column in the
    * Telemetry instance. The columnName is the exact spelling   
    * of the name entered in an addColumn call. The order this data 
    * is added will not change the order of the columns. No additional
    * formatting is done on the data. 
    * <p>
    * Example: See saveString()
    * <p>
    * If you have a double that you want to save to more precision
    * convert to a sting and save as a string. As in saving an 
    * accelerometer value you may want 6 decimal places. 
    * <pre>
    * telem.saveString("S ACCEL X", String.format("%.6f",this.d_BACC_XAxis) );
    * </pre>
    * <p>
    * @param	columnName string at the top of this column.  
    * @param	value String to be written.  
    */
    public void saveString(String columnName, String value){
        dictColumnData.put(columnName,"\"" + value + "\"");								// no formatting needed
        saveColumnName(columnName);

        if( columnName.charAt(0) == '*'){
            if( columnName.startsWith("*REC") )
                sawRecordMarker = true;
        }
    }

    public void putString( String columnName, String value, boolean forceDisplay){
        saveString(columnName, value );		
        if( !supressSmartDashbaordDisplay || forceDisplay )
            SmartDashboard.putString( columnName, value  );
    }

    public void putString( String columnName, String value){    // overloaded version
        putString( columnName, value, false  );
    }


    /**
    * Called to add a true/false boolean to a column in the
    * Telemetry instance. The columnName is the exact spelling   
    * of the name entered in an addColumn call. The order this data 
    * is added will not change the order of the columns. No additional
    * formatting is done on the data but Excel will fore these to CAPs. 
    * <p>
    * Example: See saveInteger()
    * <p>
    * @param	columnName string at the top of this column.  
    * @param	value Boolean to be written.  
    */
    public void saveBoolean(String columnName, Boolean value){			 
        dictColumnData.put(columnName, value.toString());					// save the Boolean as a string 
        saveColumnName(columnName);
    }

    public void putBoolean( String columnName, Boolean value, boolean forceDisplay){
        saveBoolean(columnName, value );		
        if( !supressSmartDashbaordDisplay || forceDisplay )
            SmartDashboard.putBoolean( columnName, value  );
    }

    public void putBoolean( String columnName, Boolean value){              // overloaded version 
        putBoolean( columnName, value, false  );
    }

    
    /**
    * Called to add a true boolean to a column in the Telemetry instance.
    * If the item is false then an empty field will be added. If true 
    * then the field will say true. Excel will set this to all CAPS. 
    * <p> 
    * The columnName is the exact spelling of the name entered in an 
    * addColumn call. The order this data is added will not change
    * the order of the columns.  
    * <p>
    * Example: See saveInteger()
    * <p>
    * @param	columnName string at the top of this column.  
    * @param	value boolean to be written.  
    */
    public void saveTrueBoolean(String columnName, Boolean value){		// we only want true or blank easier to read
        if(value == true)
            dictColumnData.put(columnName,"true");
        else
            dictColumnData.put(columnName,"");
    }

    public void putTrueBoolean( String columnName, Boolean value, boolean forceDisplay){
        saveTrueBoolean(columnName, value );	
        if( !supressSmartDashbaordDisplay || forceDisplay )
            SmartDashboard.putBoolean( columnName, value  );
    }

    public void putTrueBoolean( String columnName, Boolean value ){      // overloaded version 
        putBoolean( columnName, value, false );
    }

    
    /**
    * Called to add a false boolean to a column in the Telemetry instance.
    * If the item is true then an empty field will be added. If false 
    * then the field will say false. Excel will set this to all CAPS. 
    * <p> 
    * The columnName is the exact spelling of the name entered in an 
    * addColumn call. The order this data is added will not change
    * the order of the columns.  
    * <p>
    * Example: See saveInteger()
    * <p>
    * @param	columnName string at the top of this column.  
    * @param	value boolean to be written.  
    */
    public void saveFalseBoolean(String columnName, Boolean value){		// we only want false or blank, easier to read. 
        if(value == true)
            dictColumnData.put(columnName,"");
        else
            dictColumnData.put(columnName,"false");
    }

    public void putFalseBoolean( String columnName, Boolean value, boolean forceDisplay){
        saveFalseBoolean(columnName, value );	
        SmartDashboard.putBoolean( columnName, value  );
    }

    public void putFalseBoolean( String columnName, Boolean value){         // overloaded version 
        saveFalseBoolean(columnName, value );	
    }

    
    /**
    * Called to tell telemetry to save the current saved
    * data into a row. This will cause the data to be saved under the
    * appropriate headers. Any columns missing data will show blank.  
    * <p>  
    * This is expected to be called at the bottom of the TeleopPeriodic
    * and the autonomousPeriodic methods of the Robot class.  
    * <p>
    * Example:
    * <p>
    * <pre>
    *     public void autonomousPeriodic() {
    *         
    *         bla...bla...bla
    *         
    *         inputs.writeTelemetryValues(telem);
    *         robotbase.writeTelemetryValues(telem);
    *         wedge.writeTelemetryValues(telem);
    *         telem.writeRow();	                          // write a record now
    *     }
	* </pre>
    */
    public void writeRow(){										// save a row of data to memory

    	
        if( this.i_TotalRows >= this.ki_MaxRows )				// if we have already written MAXRows to memory then 
            return;												// return without adding more rows. The calculation for 
        														// total rows should be good for a match of 50 cycles per second * 180 seconds ( 3 minutes) = 9000
        														// they should not save during disabled mode

        String[] listColumnData = new String[ki_MaxColumns];	// temporary list of each of the column's data. Explained why later
        														// this is cleaned up when the this function is over. 
        
        int i_TotalColumns = 0;									// index for the column data being added to the temp list
        
       
        listColumnData[i_TotalColumns++] =						// Save the Telemetry generated columns first in the proper order, notice increment  
        		String.format( "%.2f", tim_Time.get() );
        
        listColumnData[i_TotalColumns++] = 
        			String.format( "%.2f", RobotController.getBatteryVoltage() );



        String s_Mode = "unk";                                                      // default in case mode is not known here
        if (DriverStation.isAutonomous() == true) 			s_Mode = "auton";		// this form is allowed as Java ignores whitespace between things
        else if (DriverStation.isTeleop() == true)          s_Mode = "teleop";		// we don't like to do it as it may not be clear and harder to debug.
        else if (DriverStation.isDisabled() == true) 		s_Mode = "disable";		// added here for your viewing pleasure and a learning moment 
        else if (DriverStation.isTest() == true) 		    s_Mode = "test";		// added here for your viewing pleasure and a learning moment 

        if( this.i_TotalRows == 0){								// first row
        	this.s_TelemType = s_Mode;	    					// need a string to hold this Mode
            restartTimer();                                     // restart the timer so we capture this passes time. 
        }
        	
        listColumnData[i_TotalColumns++] = s_Mode;				// add mode to the column data list 

        for(String columnName : listColumns){					// go through all the headers looking for their data in the HashMap or dictionary 

        	if( columnName == null )							// if we hit a header that had no value (null), end of the header list, break out of the loop  
        			break; 
        	
        	String sValue = "";									// Create a tmp sting to hold the column data. "" assumes this column does not have any data 

            if(dictColumnData.containsKey(columnName))			// use the columnName to see if there is any column data for this header
                sValue = dictColumnData.get(columnName);		// yes their is, save it to sValue
            	
            if (sValue == null)									// this is an error, get out.
            	break;
            
            listColumnData[i_TotalColumns++] = sValue;			// add the value to the list. The important point here is this is how the 
            													// actual data is saved in the same position as the header
            													// no matter when the column data was written to the class 
        }
        
        while( i_TotalColumns < listColumnData.length  )		// here we set all of the rest of the column data in this list to ""
            listColumnData[i_TotalColumns++] = "";				// if we did not do this then the word null would show up for all the extra columns
        														// this extra data is annoying but is the price we pay for speed. 
        
        listRows[this.i_TotalRows++] = 
                String.join(s_FieldDelimiter, listColumnData);	// save this row to the row list 
                                                                // This is why we use the array listColumnData. Using the join method to save to the 
                                                                // list of rows is very, very fast and does not use any robot cycles. 
                                                                // we still run at 50 cycles per second. Other methods we use in development
                                                                // made us run at 20 cycles per second.
        
        this.dictColumnData.clear();							// now clear out the old column data for the next pass. 
                                                                // this does not clear the column headings just the data. 
            
    }

    
    /**
    * Called to tell telemetry to read its Preferences.
    * The available fields are...<br>
    * 1) Tele_FileName - Default: telemetry - Name of the file to be written. This will have a .xls extension added.
    * <p>                     
    * 2) Tele_FilePath - Default: /tmp - Location on the roboRio where 
    *                    the file is written. This is in /var/volitile/tmp.
    *                    This will be erased when powered off. So get it 
    *                    before you power off. The folder /home/lvuser
    *                    is writable and is not volitile. <b><u><i>Use at your own risk.</i></u></b>  
    * <p>                     
    * 3) Tele_TimestampFile - Default: false - Use to tell the system to add a current time stamp to the file. 
    * 					 This is applied just before the data is written. <br>
    * 					 Example:  telemetry.2015-12-14-13-25-23.xls
    */
    public void loadConfig(){
    	
        this.s_FileName      = Constants.config.getString("_TELEM_FileName", this.s_FileName);
        this.s_FilePath      = Constants.config.getString("_TELEM_FilePath", this.s_FilePath); 
        this.b_TimestampFile = Constants.config.getBoolean("_TELEM_TimestampFile", false);
        this.b_FullDetails   = Constants.config.getBoolean("_TELEM_FullDetails", false);
        
    }
 
    
    /**
    * Call see what the file name will be. You can include on 
    * the SmartDashboard so you can see what the name will look like based upon
    * the preferences.<br> 
    * If you have the time stamp option enabled the time will 
    * be when the final file is written. 
     * @param Paths 
    * 
    *  @return String of the full file name with .xls extension.  
     * @throws IOException
    */
    public String getFileName(){										// build up the file name

    	if(s_FileName.length() == 0 )   // this should have been defined above
    		s_FileName = "telemetry";
    		
    	String s_FullFileName = s_FilePath + "/telemFiles";
        try{
            Files.createDirectories(Paths.get(s_FullFileName));
        }catch(Exception e){
            return s_FilePath + "/telem_default.xls";
        }

        s_FullFileName += "/";

        if(this.b_FullDetails == false){
            s_FullFileName += s_FileName;

        } else {
            s_FullFileName += "telemetry_Event-" +  DriverStation.getEventName().toString();
            s_FullFileName += "_" + DriverStation.getMatchType().toString();
            s_FullFileName += "_Match-" + String.valueOf(DriverStation.getMatchNumber());
            s_FullFileName += "-(" + DriverStation.getAlliance().toString() + ")";
        }

    	if(this.b_TimestampFile == true)
    		s_FullFileName += "_" +  new SimpleDateFormat("yyyy-MM-dd-HH-mm-ss").format(new Date());

  	    s_FullFileName += "_" + this.s_TelemType;
       
        if( sawRecordMarker )                       // telem came in with a recording marker
            s_FullFileName += "_recording"; 	

        s_FullFileName += s_FileFormat; 	

        return s_FullFileName;
    }

    
    /**
    * Called to show a few of the fields that can aid you in setup.<br> 
    * <b>Note: This only works while in disabledPeriodic mode.</b> 
    * <p>  
    * 1) Tele_FileName - Final File name. Note: Is timestamp is used the
    *                    timestamp will updated until you actually write 
    *                    the file.  
    * <p>
    * 2) Tele_DataSaved - a boolen that will show true when the spreadsheet
    * 					 is written. Once written you cannot write again 
    * 					 until the code is restarted.
    * <p> 
    * 					 Example:  /tmp/telemetry.2015-12-14-13-25-23.xls
    * <p>                      
    * @param	b_MinDisplay Ignored but kept for compatibility with
    *                      other LC functions with this name.    
    */
    public void outputToDashBoard(Boolean b_MinDisplay){

    	if(DriverStation.isDisabled()){									// only acted upon and displayed in disabled mode to save cycles 
    		SmartDashboard.putString("Tele_FileName", getFileName() );
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
    *	
    *</pre>
    *                      
    */
    public void saveSpreadSheet(){
    	
    	if( this.i_TotalRows == 0)
    		return;
    	
    
        try {
			fileHandle = new FileWriter( getFileName() );

			String headerRow =  "Timer" + s_FieldDelimiter +        	// these are common fields LCTelemetry adds for us. 
                                "Batt Volts" + s_FieldDelimiter +
                                "Mode"+ s_FieldDelimiter;		
			
            for (String column : listColumns) {
            	headerRow += column + s_FieldDelimiter;				// inserts a delimter easily read as a column in excel. 
            }
            
        	fileHandle.write(headerRow + "\n");						// write the header. the \n is a new line indicating the end of a line or row in the sheet. 
			
        	
            for (int i = 0; listRows[i] != null; i++) {
            	fileHandle.write(listRows[i] + "\n");				// write each row of telemetry data. Teh \n is a new line character.   
            }
            
            fileHandle.close();						
            
		} catch (IOException e) {
			e.printStackTrace();
		}
 
        createNewList();
        saveUndeclariedColumnHeadings();
            
    }

    
    private void saveUndeclariedColumnHeadings(){

        try {
            
			fileHandle = new FileWriter( s_FilePath + "/" + "telem_new_column_headings.txt" );

            Collections.sort(listNewHeadings);                 // sort the headings lexicongraphically, fancy name fro alpha sort. :)
            for (String heading : listNewHeadings) {
				
				fileHandle.write( heading.trim() + "\n"  );        // just write it out
            }
            
            fileHandle.close();						
            
		} catch (IOException e) {
			e.printStackTrace();
		}


    }


}

