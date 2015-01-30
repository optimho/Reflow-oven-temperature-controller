
/* REFLOW OVEN CONTROLLER


http://www.optimho.co.nz/reflow_controller


/* Thanks to Marc Alexander for the framwork that I modified
   Example code for the Freetronics LCD & Keypad Shield:
   
     http://www.freetronics.com/products/lcd-keypad-shield
   
   by Marc Alexander, 7 September 2011
   This example code is in the public domain.
   
   ---------------------------------------------------------------------
   
   This program demonstrates button detection, LCD text/number printing,
   and LCD backlight control on the Freetronics LCD & Keypad Shield, connected to an Arduino board.
   
   After powerup, the screen looks like this:
   
        ------------------
       |Freetronics 16x2|
        |Btn:          0 | <- This time value counts up the number of seconds since reset (overflows at 99)
        ------------------
 
   When a button is pressed, a label appears for it:
   
        ------------------
       |Freetronics 16x2|
        |Btn:RIGHT     0 |
        ------------------
       Labels are LEFT, UP, DOWN, RIGHT and SELECT-FLASH.
        SELECT-FLASH makes the LCD backlight flash off and on when held down.  
   
   Pins used by LCD & Keypad Shield:
   
     A0: Buttons, analog input from voltage ladder
     A1: Measure Temperature
     D3: LCD Backlight (high = on, also has pullup high so default is on)
     D4: LCD bit 4
     D5: LCD bit 5
     D6: LCD bit 6
     D7: LCD bit 7
     D8: LCD RS
     D9: LCD E
     D12: Temperature Control relay On/Off
     
   
   ADC voltages for the 5 buttons on analog input pin A0:
   
     RIGHT:  0.00V :   0 @ 8bit ;   0 @ 10 bit
     UP:     0.71V :  36 @ 8bit ; 145 @ 10 bit
     DOWN:   1.61V :  82 @ 8bit ; 329 @ 10 bit
     LEFT:   2.47V : 126 @ 8bit ; 505 @ 10 bit
     SELECT: 3.62V : 185 @ 8bit ; 741 @ 10 bit
 */
 /*--------------------------------------------------------------------------------------
   Includes
 --------------------------------------------------------------------------------------*/
 #include <Wire.h>
 #include <LiquidCrystal.h>   // include LCD library
 #include <PID_v1.h>          // PID control library
 #include <Timer.h>           // Timer
 /*--------------------------------------------------------------------------------------
   Defines
 --------------------------------------------------------------------------------------*/
 // Pins in use
 #define BUTTON_ADC_PIN           A0  // A0 is the button ADC input
 #define LCD_BACKLIGHT_PIN         3  // D3 controls LCD backlight
 // ADC readings expected for the 5 buttons on the ADC input
 #define RIGHT_10BIT_ADC           0  // right
 #define UP_10BIT_ADC            145  // up
 #define DOWN_10BIT_ADC          329  // down
 #define LEFT_10BIT_ADC          505  // left
 #define SELECT_10BIT_ADC        741  // right
 #define BUTTONHYSTERESIS         10  // hysteresis for valid button sensing window
 //return values for ReadButtons()
 #define BUTTON_NONE               0  // 
 #define BUTTON_RIGHT              1  // 
 #define BUTTON_UP                 2  // 
 #define BUTTON_DOWN               3  // 
 #define BUTTON_LEFT               4  // 
 #define BUTTON_SELECT             5  // 
 #define TEMP_ADC_PIN            A1   // Temperature measurement 
 #define HEATER_ELEMENT_ON       12   // Turn on heater
 //some example macros with friendly labels for LCD backlight/pin control, tested and can be swapped into the example code as you like
 #define LCD_BACKLIGHT_OFF()     digitalWrite( LCD_BACKLIGHT_PIN, LOW )
 #define LCD_BACKLIGHT_ON()      digitalWrite( LCD_BACKLIGHT_PIN, HIGH )
 #define LCD_BACKLIGHT(state)    { if( state ){digitalWrite( LCD_BACKLIGHT_PIN, HIGH );}else{digitalWrite( LCD_BACKLIGHT_PIN, LOW );} }
 
 
 /*--------------------------------------------------------------------------------------
   Variables
 --------------------------------------------------------------------------------------*/
 byte buttonJustPressed  = false;         //this will be true after a ReadButtons() call if triggered
 byte buttonJustReleased = false;         //this will be true after a ReadButtons() call if triggered
 byte buttonWas          = BUTTON_NONE;   //used by ReadButtons() for detection of button events
 byte element            = false;         //used by heating element true=element on false=element off
 float gTemp_Calibration =  1.315;            //Calibration constant to convert temperature to degrees
 
 //Define Variables we'll be connecting to
 double Setpoint, Input, Output;
 //Specify the links and initial tuning parameters
 PID myPID(&Input, &Output, &Setpoint,600,20,400, REVERSE);
 int WindowSize = 5000;
 unsigned long windowStartTime;
 
 
 //Temperature and delay settings contained in these arrays.
 double Ctemp[6]={40,150,185,225,0,0}; //Temperatures to become the setpoint in degrees C                 
 double Cdwell[6]={50000,180000,90000,30000,50000,50000}; //this is the delay at a specified temperature, in Milli Seconds
                                               
 //------------------------------------------------------------------------------------------------------------
 struct record {
   byte go;
   double startTime;
   double temp;
   double duration;
   int Lenght;
 };
 
 typedef struct record DataControl;
 //variables
    byte button;
    byte timestamp;
    float TemperatureMeasured;
    DataControl aRec;
    int gIndex=0;

 
 /*--------------------------------------------------------------------------------------
   Init the LCD library with the LCD pins to be used
 --------------------------------------------------------------------------------------*/
 LiquidCrystal lcd( 8, 9, 4, 5, 6, 7 );   //Pins for the freetronics 16x2 LCD shield. LCD: ( RS, E, LCD-D4, LCD-D5, LCD-D6, LCD-D7 )
 /*--------------------------------------------------------------------------------------
   setup()
   Called by the Arduino framework once, before the main loop begins
 --------------------------------------------------------------------------------------*/
 void setup()
 {
    Serial.begin(9600);
    windowStartTime = millis();
    Setpoint = 0;                           //initialize the variables we're linked to PID control
    myPID.SetOutputLimits(0, WindowSize);     //tell the PID to range between 0 and the full window size 
    myPID.SetMode(AUTOMATIC);                 //turn the PID on
    //button adc input
    pinMode( BUTTON_ADC_PIN, INPUT );         //ensure A0 is an input
    digitalWrite( BUTTON_ADC_PIN, LOW );      //ensure pullup is off on A0
    //button Temperature adc input
    pinMode( TEMP_ADC_PIN, INPUT );           //ensure A1 is an input
    digitalWrite( TEMP_ADC_PIN, LOW );        //ensure pullup is off on A0
    
    //Temperature Relay to switch on heating element
    pinMode( HEATER_ELEMENT_ON, OUTPUT );     // heater element is an input
    digitalWrite( HEATER_ELEMENT_ON, LOW );   // Switch heater element off.  
    
    //lcd backlight control
    digitalWrite( LCD_BACKLIGHT_PIN, HIGH );    //backlight control pin D3 is high (on)
    pinMode( LCD_BACKLIGHT_PIN, OUTPUT );       //D3 is an output
    //set up the LCD number of columns and rows: 
    lcd.begin( 16, 2 );
   
   
    //Print some initial text to the LCD.
    lcd.setCursor( 9, 0 );
    lcd.print ("set:");
    lcd.setCursor( 0, 0 );   //top left
    lcd.print( "" );
    lcd.setCursor( 0, 1 );   //bottom left
    lcd.print( "" );
    lcd.setCursor(9, 1);              //note the value will be flickering/faint on the LCD
    lcd.print ("deg:"); 
    
    /*Initialises the record structure with data that will build a reflow curve.
    Thus 100 degrees for 3 minutes then 220 for 1 minute and so on.
    */

   aRec.go=false;                   // boolean indication if the reflow function is on
   aRec.startTime=0;                // Time stamp
   aRec.temp=0;         // Temperature Setpoints
   aRec.duration=Cdwell[gIndex];    // time at each temperature
   aRec.Lenght=5;                   // records in array
   
 }
 /*--------------------------------------------------------------------------------------
   loop()
   Arduino main loop
 --------------------------------------------------------------------------------------*/
 void loop()
 {
                                                                   //  Serial.print(gIndex);
                                                                    //Serial.print("\n");
                                                                    //Serial.print (aRec.duration);
                                                                    //Serial.print("\n");
      //aRec.go=true;                                                 // Testing remove after testing
                                                                    // Serial.print ("\n");
                                                                    //Serial.print (aRec.go);   
    TempControl(aRec);                                              // automatic sepoint function
 
    //get the latest button pressed, also the buttonJustPressed, buttonJustReleased flags
    button = ReadButtons();
    //TemperatureMeasured = ReadTemperature();
    TemperatureMeasured = (ReadTemperature() +25.00) * gTemp_Calibration;
    Input=TemperatureMeasured;
  
    //blank the demo text line if a new button is pressed or released, ready for a new label to be written
    if( buttonJustPressed || buttonJustReleased )
    {

    //show text label for the button pressed
    }
    switch( button )
    {
       case BUTTON_NONE:
       {
          break;
       }
       case BUTTON_RIGHT:
       {
          
          //Serial.print("Auto Setpoint :");
          //Serial.print(Setpoint);
          //Serial.print("\n");
          aRec.go=true;
          aRec.startTime=millis();
          
          break;
       }
       case BUTTON_UP:
       {
          lcd.setCursor( 0, 0 );
          lcd.print( "man      ");
          element=true;
            //  Serial.print("Manual setpoint +:");
            //  Serial.print(Setpoint);
            //  Serial.print("\t");
              aRec.go=false;
          //digitalWrite( HEATER_ELEMENT_ON, HIGH ); //Switch Heater element On
          aRec.temp++;

          break;
       }
       case BUTTON_DOWN:
       {
          lcd.setCursor( 0, 0 );
          lcd.print( "man      " );
          element=false;
              //Serial.print("Setpoint :");
              //Serial.print(Setpoint);
              //Serial.print("\t");
              aRec.go=false;
          //digitalWrite( HEATER_ELEMENT_ON, LOW ); //Switch Heater element OFF
          //Setpoint=Setpoint-1;
          aRec.temp--;

          break;
       }
       case BUTTON_LEFT:
       {

         //Serial.print("Setpoint :");
         //Serial.print(Setpoint);
         //Serial.print("\t");
         aRec.go=false;
         gIndex=0;
         //aRec.startTime=0;
         aRec.temp=0;
         break;
      }
      case BUTTON_SELECT:
      {
         lcd.setCursor( 0, 0 );
         lcd.print( "SELECT  " );
          //Serial.print(Setpoint);
          //Serial.print("\t");    
         aRec.go=true;
         break;
       }
       default:
      {
         break;
      }
    }
  


    lcd.setCursor(13, 1);
    //lcd.setCursor(5, 1);
    lcd.print( "    " );          //quick hack to blank over default left-justification from lcd.print()
    lcd.setCursor(13, 1);

    if( TemperatureMeasured <= 35 )
       lcd.print( "<35" );   //quick trick to right-justify this 2 digit value when it's a single digi
       
    if (TemperatureMeasured > 35); //if > 35 disply temperature
        lcd.print (round (Input));
        //lcd.print (Output);
        
         
 /************************************************
  * PID Control 
  ************************************************/         
    //Setpoint=aRec.temp;                     //use the latest setpoint
    //Serial.print("Setpoint :");
    //Serial.print(Setpoint);
    //Serial.print("t");
    //Serial.print("Sec :");
    //Serial.print(millis()/1000);
    //Serial.print("\n");
    //Serial.print("gIndex :");
    //Serial.print(gIndex);
    //Serial.print("\n");
    if (Setpoint<0)
       Setpoint =0;

   
    //Serial.print(Setpoint);
    myPID.Compute(); // calculate PID action 
    
     
     /************************************************
     * turn the output pin on/off based on pid output
     ************************************************/
    if(millis() - windowStartTime>WindowSize)
    { //time to shift the Relay Window
     windowStartTime += WindowSize;
    }
    if(Output < millis() - windowStartTime) 
    {
      digitalWrite(HEATER_ELEMENT_ON,HIGH);
       lcd.setCursor( 0, 1 );
       lcd.print( "on " );
    }
    else 
    {
       digitalWrite(HEATER_ELEMENT_ON,LOW);
       lcd.setCursor( 0, 1 );
       lcd.print( "off" );
    }
     
        
    //clear the buttonJustPressed or buttonJustReleased flags, they've already done their job now.
    if( buttonJustPressed )
       buttonJustPressed = false;
    if( buttonJustReleased )
       buttonJustReleased = false;
 }
 
 /*--------------------------------------------------------------------------------------
   ReadButtons()
   Detect the button pressed and return the value
   Uses global values buttonWas, buttonJustPressed, buttonJustReleased.
 --------------------------------------------------------------------------------------*/
 byte ReadButtons()
 {
    unsigned int buttonVoltage;
    byte button = BUTTON_NONE;   // return no button pressed if the below checks don't write to btn
    
    //read the button ADC pin voltage
    buttonVoltage = analogRead( BUTTON_ADC_PIN );
    //sense if the voltage falls within valid voltage windows
    if( buttonVoltage < ( RIGHT_10BIT_ADC + BUTTONHYSTERESIS ) )
    {
       button = BUTTON_RIGHT;
    }
    else if(   buttonVoltage >= ( UP_10BIT_ADC - BUTTONHYSTERESIS )
            && buttonVoltage <= ( UP_10BIT_ADC + BUTTONHYSTERESIS ) )
    {
       button = BUTTON_UP;
    }
    else if(   buttonVoltage >= ( DOWN_10BIT_ADC - BUTTONHYSTERESIS )
            && buttonVoltage <= ( DOWN_10BIT_ADC + BUTTONHYSTERESIS ) )
    {
       button = BUTTON_DOWN;
    }
    else if(   buttonVoltage >= ( LEFT_10BIT_ADC - BUTTONHYSTERESIS )
            && buttonVoltage <= ( LEFT_10BIT_ADC + BUTTONHYSTERESIS ) )
    {
       button = BUTTON_LEFT;
    }
    else if(   buttonVoltage >= ( SELECT_10BIT_ADC - BUTTONHYSTERESIS )
            && buttonVoltage <= ( SELECT_10BIT_ADC + BUTTONHYSTERESIS ) )
    {
       button = BUTTON_SELECT;
    }
    //handle button flags for just pressed and just released events
    if( ( buttonWas == BUTTON_NONE ) && ( button != BUTTON_NONE ) )
    {
    //the button was just pressed, set buttonJustPressed, this can optionally be used to trigger a once-off action for a button press event
    //it's the duty of the receiver to clear these flags if it wants to detect a new button change event
       buttonJustPressed  = true;
       buttonJustReleased = false;
    }
    if( ( buttonWas != BUTTON_NONE ) && ( button == BUTTON_NONE ) )
    {
       buttonJustPressed  = false;
       buttonJustReleased = true;
    }
    
    //save the latest button value, for change event detection next time round
    buttonWas = button;
    
    return( button );
 }

/*--------------------------------------------------------------------------------------
 Read Temperature
--------------------------------------------------------------------------------------*/
int ReadTemperature()
{
  
  int Read_Temperature;
  //ReadTemperature=0;
  
  int ReadTemperature_ = analogRead (TEMP_ADC_PIN);
  Read_Temperature = ReadTemperature_;
  return (Read_Temperature);
}

/*--------------------------------------------------------------------------------------
 Drive Automatic reflow curve Temperature 
--------------------------------------------------------------------------------------*/
void TempControl (struct record programs)
{
  
      
      if (aRec.go==true) {
        
        aRec.temp=Ctemp[gIndex];                                   //Make sure that that we have the correct point loaded
        aRec.duration=Cdwell[gIndex];                              //Make sure that we have the correct duration loaded
        //Serial.print("IIIIIIIIIII");
        lcd.setCursor( 0, 0 );
        lcd.print( "Auto on " );                                //If the auto button (right arrow) is selected the selected program starts
                                                                //Serial.print("aRec.duration + aRec.startTime \n");
                                                                // Serial.print(aRec.duration + aRec.startTime);
                                                                // Serial.print("\n");
                                                                // Serial.print("\n");
                                                                //Serial.print("\n");
                                                                //Serial.print("milli sec \n");
                                                                //Serial.print(millis());
                                                                //Serial.print("\n");
      
      if (millis()>=(aRec.duration+aRec.startTime))             //check that time has not elapsed
      {                                                         //If it has 
                                                                //Serial.print("aRec.duration: ");
                                                                //Serial.print(aRec.duration);
                                                                //Serial.print("\n");
                                                                //Serial.print("gIndex before increment: ");
                                                                //Serial.print(gIndex);
                                                                //Serial.print("\n");
                                                                //Serial.print("mills ");
                                                                //Serial.print(millis());
                                                                //Serial.print("\n");
                                                                // Serial.print (aRec.duration);
                                                                // Serial.print("\n");
                                                      // Get the next point in the reflow curve
      aRec.startTime=millis();                       // Time stamp the time that this was done
      aRec.duration=Cdwell[gIndex+1];
      //Serial.print("aRec.duration: ");
      //Serial.print(aRec.duration);
      //Serial.print("\n");
      gIndex++;
      //Serial.print("\n");
      //Serial.print("gIndex: After increment ");
      //Serial.print(gIndex);
      //Serial.print("\n");
      //Serial.print("aRec.statrTime ");
      //Serial.print(aRec.startTime);
      //Serial.print("\n");
      }
      
      if (gIndex>aRec.Lenght){                                  //If it is the last point int the array
        gIndex=0;                                                   //Select index 0 again, this is off
        aRec.go=false;                                          //Turn off the reflow 
        aRec.startTime=0;                                       //Reset the time stamp to 0
        Setpoint=aRec.temp;
      }
      
                            }

if (aRec.go==false){       

    lcd.setCursor( 0, 0 );
    lcd.print( "Man    " );  //If Reflow is selected off                             
 
//Serial.print("xxxxxxxxxx");
//Serial.print("\n");
 gIndex=0;
 aRec.startTime=0;                                             //make sure index is zeo
}                                                                  //make sure start time is at zero
 
 //Serial.print("&&&");
 //aRec.temp=Ctemp[gIndex];                                   //Make sure that that we have the correct point loaded
 //aRec.duration=Cdwell[gIndex];                              //Make sure that we have the correct duration loaded
 if (aRec.temp>250)aRec.temp=250;
   
 if (aRec.temp<0)aRec.temp=0;
 
 Setpoint=aRec.temp;
 
// Serial.print(aRec.temp);
// Serial.print("&&&");
 lcd.setCursor(13, 0);
 lcd.print( "   " );          //quick hack to blank over default left-justification from lcd.print()
 lcd.setCursor(13, 0);          //note the value will be flickering/faint on the LCD
 lcd.print( Setpoint,0);          // display setpoint on LCD
    
    
}

 

