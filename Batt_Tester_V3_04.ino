/*
 * The code and hardware is based on a design from John Lowen <john@vwlowen.co.uk>
 * http://www.vwlowen.co.uk/arduino/battery-tester/battery-tester.htm
 *
 * I wanted the same functionality, but a much simpler hardware solution
 * and mostly footprint. Besides 3.7V Lithium Lithium Ion and LiPo cells, I also 
 * wanted to test NiMH and NiCad's (both 1.2V), lead cells (12V) and
 * normal batteries (1.5V). Luckily the PC program allows that, and the hardware 
 * supports that too.
 * 
 * see my blog at http://www.paulvdiyblogs.net/
 * 
 * paulv march 2019
 * 
 */


#include <Wire.h>
#include <Adafruit_INA219.h>
#include <SPI.h>              // Nokia 5110 lcd driver
#include <Adafruit_GFX.h>     // Nokia 5110 lcd driver
#include <Adafruit_PCD8544.h> // Nokia 5110 lcd driver


//**************************************
// If Arduino Nano, use old bootloader!
//**************************************

String SW_VERSION = "3.04"; // Changed write_to_pc() and getTime() to stay compatible with 
                            // the V2.2c from John's PC program

boolean manual = false;     // false is with PC control
                            // manual mode is only used during testing

/*************************************/
// Default 'set point' variables for manual mode. I use this mode only during testing
int target_mA = 100;
float cutoff_voltage = 3.0;
int time_limit = 180;
float kP = 50;              // Simple PID 'Proportional' control term
int tolerance = 1;          // Deadband to stop 'hunting' around target value
float offset = 0.0;
/************************************/

// Definitions for the LCD
// Using software SPI (slower updates, more flexible pin options):
// pin 7 - Serial clock out (SCLK)
// pin 6 - Serial data out (DIN)
// pin 5 - Data/Command select (D/C)
// pin 4 - LCD chip select (CS)
// pin 3 - LCD reset (RST)
Adafruit_PCD8544 lcd = Adafruit_PCD8544(7, 6, 5, 4, 3);

/* 
 * The Nokia 5110 LCD display is a graphic 84x48 pixel display.
 * You drive the display with pixels, so even characters are made-up of individual pixels.
 * 
 * The standard font allows the display of 15 characters per line
 * on 5 lines. The characters are 5 pixels wide and 7 high.
 * Because the character are made up of pixels, it means that you cannot "erase" characters 
 * by overwriting them with a space. You have to use a reqtangular pixel filling routine 
 * in order to do that.
 * 
 * The easiest way to drive the display in this application, that uses no graphics, is to 
 * fully erase it and then build it up again. The refresh rate is fast enough, even by using the 
 * slower software bit-banging SPI mode. It does not disturb the viewing.
 * 
 */
 
// LCD display character and line position definitions
int c_pos_1  = 0;   // character position in pixels
int c_pos_2  = 1*6;
int c_pos_3  = 2*6;
int c_pos_4  = 3*6;
int c_pos_5  = 4*6;
int c_pos_6  = 5*6;
int c_pos_7  = 6*6;
int c_pos_8  = 7*6;
int c_pos_9  = 8*6;
int c_pos_10  = 9*6;
int c_pos_11  = 10*6;
int c_pos_12  = 11*6;
int c_pos_13  = 12*6;
int c_pos_14  = 13*6;

int line_1  = 0;  // line position in pixels
int line_2  = 10;
int line_3  = 20;
int line_4  = 30;
int line_5  = 40;


// Current shunt and voltage measurements
Adafruit_INA219 ina219;
/*
 * Adafruit INA219 Breakout board
 * I2C connections:
 *    SCL pin goes to Nano A5
 *    SDA pin goes to Nano A4 
 * VCC to +5V
 * GND to GND
 * Vin- and Vin+ are not used
 */
float shuntvoltage = 0;
float busvoltage = 0;
double current_mA = 0;
float loadvoltage = 0;
float power_mW = 0;
/*
 * The following variable displays the cell voltage.
 * There is a constant R that can be set in the read_INA routine that can 
 * be used to account for wiring or PCB trace losses in the current loop. 
 * If you can measure it.
 * You can also use a very accurate DVM to compare the cell voltage
 * with what you measure and tweak this variable to match the displayed
 * cell voltage. If you care for this precision...
 */
float vR;

// lapse timer for test duration and mAh calculation
unsigned long  startMillisec;           // Variables for discharge timer.
unsigned long  sampleTime = 10000;      // Default samples to PC time (ms)
unsigned long millis_PC_wait;           // Timer for samples to PC
unsigned long millisCalc_mAh;           // Timer for mAh calc. and LCD write.
float last_hours = 0.0;                 // Working variables for time and mAh
float mAh_soFar = 0.0;

int days, hours;
int mins, secs;
int tMins;

// Beeper
int end_sounder = A2;     // Digital output for sounder
boolean sounded = false;  // flag to limit beeping
int beep = 1;             // value coming from PC, no longer used

// Variables and flags to terminate test
int cancel = 0;
boolean timed_out = false;
boolean high_current = false;
boolean cutoff_voltage_reached = false;
String error_code = "";
boolean end_of_test = false;

/* a hack to create up to 16-bit PWM signals:
 * https://forum.arduino.cc/index.php?topic=332431.0
 * The above one is WRONG! Below is the correct one.
 * https://arduino.stackexchange.com/questions/12718/increase-pwm-bit-resolution
 *
 * I use the 13-bit version because the resolution of the 10-bit counter is too 
 * course to drive the error variations.
 */
void setupFastPWM() {
  /* Changing ICR1 will effect the resolution and the frequency.
  ICR1 = 0xffff; (65535) 16-bit resolution  244 Hz
  ICR1 = 0x7fff; (32767) 15-bit resolution  488 Hz
  ICR1 = 0x3fff; (16383) 14-bit resolution  977 Hz
  ICR1 = 0x1fff;  (8192) 13-bit resolution 1953 Hz 
  ICR1 = 0x0fff;  (4096) 12-bit resolution 3908 Hz
  */
  DDRB |= (1 << DDB1) | (1 << DDB2);
  TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11);
  TCCR1B = (1 << WGM12) | (1 << WGM13) | (1 << CS10);
  OCR1A = 0;
  ICR1 = 0x1fff; /* TOP counter value (freeing OCR1A)*/
}

/* xx-bit version of analogWrite(). Works only on pins 9 and 10. */
void analogWriteFast(uint8_t pin, uint16_t val)
{
  switch (pin) {
    case  9: OCR1A = val; break;
    case 10: OCR1B = val; break;
  }
}

// PWM setup
const byte pwm_pin = 9;   // PWM DAC, only pins 9 and 10 are allowed with the fast PWM
int pwm = 4000;           // Starting value for 13-bit DAC. PWM is typ. @ 4540
float pid_error;          // the error between current setiing and actual current, 
                          // used in the "PID" calculation to drive the MOSFET


void setup(void) {
  Serial.begin(9600);
  lcd.begin();  // initialize the lcd
  lcd.setContrast(53);
  lcd.clearDisplay();
  lcd.setCursor(c_pos_1, line_1);   // 1st pos, first line
  lcd.print("Cell Test ");
  lcd.setCursor(c_pos_11, line_1);
  lcd.print(SW_VERSION);
  lcd.display();  // show the buffer on the lcd display
  
  // Initialize the INA219.
  // By default the initialization will use the largest range (32V, 2A).  However
  // you can call a setCalibration function to change this range (see comments).
  ina219.begin();

  pinMode(end_sounder, OUTPUT);                 // Output to sounder.
  digitalWrite(end_sounder, LOW);
  
  // Set up DAC pin as output
  pinMode(pwm_pin, OUTPUT);
  setupFastPWM();
  analogWriteFast(pwm_pin, pwm);  // and set to zero PWM out (off)

  if (!manual) {                                // If not manual, must be under PC control.
    lcd.setCursor(c_pos_1, line_3);             // I only use manual during testing.
    lcd.print("Waiting for");                   // Prompt that we're waiting to receive
    lcd.setCursor(c_pos_1, line_4);             // the load settings from the application
    lcd.print("PC settings...");                // that's running on the PC.
    lcd.display();                              // Show it on the lcd

    /*
     * Wait for the settings coming from the PC
     */
    while (Serial.available() == 0 ) ;          // Wait for settings params from PC
    if (Serial.available() > 0) {               // Data available on serial port from PC
      target_mA = Serial.parseInt();            // so read each one in turn into variables.
      cutoff_voltage = Serial.parseFloat();     // Minimum battery voltage to end test
      time_limit = Serial.parseInt();           // maximum time allowed for the test
      sampleTime = Serial.parseInt() * 1000;    // Interval to send data to PC
      kP = Serial.parseInt();                   // kP - control loop Proportional value
      offset = Serial.parseFloat();             // no longer used
      tolerance = Serial.parseInt();            // no longer used
      beep = Serial.parseInt();                 // Sounder (0=off, 1=on)no longer used
      cancel = Serial.parseInt();               // Will = 0. Clear Cancel flag
                     
    }
    Serial.flush(); // Make sure the serial port is empty to avoid
                    // false 'Cancel' messages in the control loop.
  }
  
  /*
   * With the higher pwm resolution, it takes longer to ramp-up to a large current setting
   * shorten the ramp-up time
   */
  if (target_mA > 99){
    kP = 50;  // the maximum
  }
  
  // These lines echo the received values to the LCD and display them for 5 seconds 
  lcd.clearDisplay();
  lcd.setCursor(c_pos_1,line_1);                // first line
  lcd.print("Current: "); lcd.print(target_mA);
  lcd.setCursor(c_pos_1, line_2);               // second line
  lcd.print("Cutoff V: "); lcd.print(cutoff_voltage); 
  lcd.setCursor(c_pos_1, line_3);               // third line
  lcd.print("T-limit: "); lcd.print(time_limit); 
  lcd.setCursor(c_pos_1, line_4);               // fourth line
  lcd.print("Sample t= "); lcd.print(sampleTime/1000);
  lcd.setCursor(c_pos_1, line_5);               // fifth line
  lcd.print("Tol= "); lcd.print(tolerance); lcd.print(" kP= "); lcd.print((int)kP); 
  lcd.display();
  delay(5000);
  lcd.clearDisplay(); // from now on we use write_to_lcd() to display the running data 
  
  // bleep once to signal the start of the test
  digitalWrite(end_sounder, HIGH);
  delay(20);
  digitalWrite(end_sounder, LOW);

  startMillisec = millis();   // get millisec timestamp for the starting point
}


void loop(void) {

  // get the data from the INA219
  readINA219();
  
  /* 
   * This is a very simplified "PID" routine to drive the MOSFET with a
   * PWM value, based on the difference of the set target_mA value and the measured actual_mA 
   * current value.
   */
  pid_error = abs(target_mA - current_mA);
  pid_error = (pid_error / target_mA) * 100;
  
  if ((!end_of_test) && (pid_error > tolerance)) {    // If out of tolerance (deadband to stop 'hunting')
    pid_error = pid_error - offset;                   // Bias (long term error compensation)
    pid_error = (pid_error * kP) / 100;               // 'proportional' factor reduces impact of 'raw' error.
    pid_error = constrain(pid_error, 0.0, 50.0);      // limit to max incremental steps
 
    if (current_mA > target_mA){
      pid_error = - pid_error;                        // Determine if it's a pos or neg error.
    }   
    pwm =  abs(pwm + round(pid_error));
    pwm = constrain(pwm, 0, 8192-1);                  // constrain to 13-bit max
  } 
  
  if (manual) {Serial.println(current_mA);} // so we can plot it with the Arduino Serial Plotter

  /* check if the cell voltage has reached the set cut off voltage
   * and abort the cycle if it has.
   * end_of_test is used to stop further processing.
   */
  if ((!end_of_test) && (loadvoltage < cutoff_voltage)) {
    delay(3000);    // account for a short dip when we start the de-charging process
    readINA219();   // read the values again
    if (loadvoltage < cutoff_voltage) {
      analogWriteFast(pwm_pin, 0);  // turn PWM off.                                
      cutoff_voltage_reached = true;
      target_mA = 0;
      error_code = "Cutoff Voltage";  // display this on line 5 of theLCD
      end_of_test = true;
      if (!manual) {Serial.print("MSGSTTest FinishedMSGEND");} // inform the PC
    }
  }

  /* 
   * Check if the measured current has overshot the target value
   * by more than 100%. If so, we have a problem so abort.
   */
  if ((!end_of_test) && (current_mA > (target_mA * 2.0))) {
     analogWriteFast(pwm_pin, 0);  // turn PWM off.
     target_mA = 0;
     error_code = "ERROR - Hi mA";  // display this on line 5 of the lcd
     end_of_test = true;
     if (!manual) {Serial.println("MSGSTError - High mAMSGEND");} // inform the PC
  }  

  // If the cycle takes too long, terminate it
  if ((!end_of_test) && (tMins > time_limit)) {
    analogWriteFast(pwm_pin, 0);  // turn PWM off.
    timed_out = true;
    target_mA = 0;
    error_code = "ERR-Timed Out"; // display this on line 5 of the lcd
    end_of_test = true;  
    if (!manual) {Serial.print("MSGSTTime ExceededMSGEND");} // inform the PC
  }

  // if Data available on serial port from PC, check for a manual abort
  if (Serial.available() > 0) {
      cancel = Serial.parseInt(); // 999 will calcel the test. 0 will clear Cancel flag

      if (cancel == 999) {        // 999 from the PC means 'Cancel' the test.
        analogWriteFast(pwm_pin, 0);  // turn PWM off.
        target_mA = 0;
        error_code = "CANCELLED"; // display this on line 5 of the lcd
        end_of_test = true;
        if (!manual) {Serial.print("MSGSTUser cancelledMSGEND");} // inform the PC
      }       
    }
    Serial.flush();
    
  /*
   * If all is OK and there are no error conditions, outout the new PWM value to adjust the current.
   */
  if ((cancel == 0) && (!timed_out) && (!high_current) && (!cutoff_voltage_reached) && (!end_of_test)) {
    analogWriteFast(pwm_pin, pwm);  // Adjust the 13-bit PWM to the calculated error correction value.
  }
  else {  // if the process is terminated, sound the beeper, but only once
    if (!sounded) {
      sounded = true;
      for (int i = 0; i< 3; i++) {
        digitalWrite(end_sounder, HIGH);
        delay(40);
        digitalWrite(end_sounder, LOW);
        delay(40);
      }
    }
  }

  /* 
   * Calculate the elapsed time and the mAh used each 
   * second round the loop.
   */
  getTime();
  // calculate the mAh capacity so far of the cell
  if (millis() > millisCalc_mAh + 1000) {
    float this_hours = (millis() - startMillisec) / (1000.0 * 3600.0);
    mAh_soFar = mAh_soFar + ((this_hours - last_hours) * current_mA);
    last_hours = this_hours;  
    millisCalc_mAh = millis();
  }

  // check if the seriallink to the PC has hung
  if (millis() > millis_PC_wait + sampleTime) {   // If the Sample-to-PC time has
      if (!manual) {write_to_pc();}               // elapsed, send data to the PC
      millis_PC_wait = millis();                  // and reset the elapsed time
  }
  //write_to_pc();  // This is for debug only. Making this active will continue to 
                    // update the PC display and the clock continues to run
                    // even though the measurement has terminated.
  // finally, update the lcd with the fresh values

  write_to_lcd();
    
} // end of loop



/* 
 * Get Current and Voltage from Adafruit INA219 breakout board.
 * 
 * The INA219 was not originally designed to be used in this kind of an application.
 * It was supposed to help calculate and display the battery capacity for laptops,
 * tablets and phones. In these applications, precision is not really required.
 * 
 * In this application, the INA219 current reading, which is very jittery to begin with,
 * is used to drive a MOSFET in the linear region. Thge MOSFET is used as a variable resistor.
 * A very small voltage change (single mVolts) applied to the Gate will result in a quite large 
 * current change. This is why I used a 13-bit PWM, to get better resolution.
 * The INA219 readings need to be averaged out a number of times to get reasonably stable values.
 * 
 */
void readINA219() {           // Obtain the INA219 readings.
  float R = 0.02;             // "Tweak" this value to compensate for circuit resistance losses.
  float temp_mA = 0.0;
  float temp_V = 0.0;
  float temp_shunt = 0.0;
  shuntvoltage = 0;
  busvoltage = 0;
  current_mA = 0;

  for (int i = 0; i< 10; i++) {               // attempt to pre-filter the readings
    temp_shunt = ina219.getShuntVoltage_mV(); // Voltage accross the shunt in mV.
    delayMicroseconds(600);
    shuntvoltage += temp_shunt; // Sum results
  }
  shuntvoltage = shuntvoltage / 10;
     
  for (int i = 0; i< 10; i++) {          // attempt to pre-filter the readings
    temp_V = ina219.getBusVoltage_V();   // Voltage from INA219 minus to gnd in V
    delayMicroseconds(600);
    busvoltage += temp_V; // Sum results
  } 
  busvoltage = busvoltage / 10;

  // the readings for the current are very jittery
  for (int i = 0; i< 20; i++) {         // attempt to pre-filter the readings
    temp_mA = ina219.getCurrent_mA();   // Current through the shunt in mA
    delayMicroseconds(600);
    current_mA += temp_mA; // Sum results  
  }
  current_mA = current_mA / 20;

  vR = R * current_mA / 1000;                               // Circuit/wire resistance factor
  loadvoltage = busvoltage  + (shuntvoltage/1000) + vR;     // Total cell voltage 
}


/* 
 * Write the obtained values to the Nokia 5110 LCD screen.
 * 
 * The easiest way is to just erase the screen and build it up before
 * sending it out again. 
 * 
 * The text for the LCD display is first put into a buffer before it gets transferred to
 * the screen by invoking the display() function.
 * 
 * To make the display of numbers in various sizes more pleasing, I took care to 
 * position the numbers right-justified. This is the only "complexity" in this code.
 * 
 */
void write_to_lcd() {

  // display title and version
  lcd.clearDisplay();
  lcd.setCursor(c_pos_1, line_1);   // 1st pos, first line
  lcd.print("Cell Test ");
  lcd.setCursor(c_pos_11, line_1);
  lcd.print(SW_VERSION);

  // display cell voltage
  int h_pos;
  if (loadvoltage < 100) h_pos = c_pos_1;
  if (loadvoltage < 10) h_pos = c_pos_2;  
  lcd.setCursor(h_pos,line_2);
  lcd.print(loadvoltage, 2);            // 2 decimal places
  lcd.setCursor(c_pos_6,line_2);
  lcd.print("V");

  // display discharge current
  current_mA = int(current_mA);
  if (current_mA >= 1000) h_pos = c_pos_8;
  if (current_mA < 1000) h_pos = c_pos_9;
  if (current_mA < 100) h_pos = c_pos_10;
  if (current_mA < 10) h_pos = c_pos_11;
  lcd.setCursor(h_pos,line_2);
  lcd.print(current_mA,0);
  lcd.setCursor(c_pos_12,line_2);
  lcd.print("mA");

  // display pwm
  if (pwm < 10000) h_pos = c_pos_1;
  if (pwm < 100) h_pos = c_pos_2;
  if (pwm < 10) h_pos = c_pos_3;
  lcd.setCursor(h_pos,line_3);
  lcd.print(pwm);
  lcd.setCursor(c_pos_5, line_3); 
  lcd.print("pwm");

  // display pid error
  h_pos = c_pos_10;
  if (pid_error < 0) h_pos = c_pos_9; // if negative, create room
  lcd.setCursor(h_pos, line_3);
  if (pid_error >= 1000) {
     lcd.print((int) pid_error); // no room for decimals
  }else{
    lcd.print(pid_error, 1);    // 1 decimal
  }

  // display current mAh value
  if (mAh_soFar >= 1000) h_pos = c_pos_1;
  if (mAh_soFar < 1000) h_pos = c_pos_2;
  if (mAh_soFar < 100) h_pos = c_pos_3;
  if (mAh_soFar < 10) h_pos = c_pos_4;
  lcd.setCursor(h_pos, line_4);
  lcd.print((int)mAh_soFar);
  lcd.setCursor(c_pos_5,line_4);       // 1st pos, 3rd line  
  lcd.print("mAh");

  // display running time
  if (hours < 10){
    lcd.setCursor(c_pos_10, line_4);       //13th pos, 3rd line
  }else{
    lcd.setCursor(c_pos_9, line_4);       //12th pos, 3rd line
  }
  lcd.print((int) hours);
  lcd.setCursor(c_pos_11, line_4);       //12th pos, 3rd line
  lcd.print(":");
    if (mins < 10){
      lcd.setCursor(c_pos_12, line_4);
      lcd.print(0);             // filling "0"
      lcd.setCursor(c_pos_13, line_4);   //14th pos, 3rd line
  }else{
    lcd.setCursor(c_pos_12, line_4);     //13th pos, 3rd line
  }
  lcd.print((int) mins); 

  lcd.setCursor(c_pos_1, line_5);         //first pos, 5th line
  lcd.print(error_code);
  
  lcd.display();    
}


/*
 * Send values to the PC
 * Formatting of the time elapsed to the PC is done at this end as it's easier than
 * having to parse/unparse the string twice at both ends. 
 * Values with a decimal can cause issues depending on the region setting of the PC.
 * English regions use the "." for decimals, other countries use the ",".
 * The PC program does not handle the current_mA properly, that's why we're simply
 * sending this value as an integer. We don't need this precision, problem solved.
 */
/* =================== Send values to PC =================================== */

void write_to_pc() {
  
  /* Send time elapsed to PC. Formatting is done this end as it's easier than
     having to parse/unparse the string twice at the other end.  */


 String message = "GRAPHVS";
 message += days;
 message += ":";
 if (hours < 10) message += "0"; message += hours; message += ":";
 if (mins < 10) message += "0"; message += mins; message += ":";
 if (secs < 10) message += "0"; message += secs; message += "!";

 message += mAh_soFar; message += "!";
 message += (int) current_mA; message += "!";
 message += loadvoltage; message += "GRAPHVEND";

 Serial.print(message);
 
 Serial.flush();        // Flush serial port before it returns to the main loop.
}


/*
 * Generic routine to calculate hours, minutes and seconds between two millis() values.
 */
void getTime() { 
     
 long day = 86400000; // 86400000 milliseconds in a day
 long hour = 3600000; // 3600000 milliseconds in an hour
 long minute = 60000; // 60000 milliseconds in a minute
 long second =  1000; // 1000 milliseconds in a second

      
 unsigned long timeNow =  millis() - startMillisec;
 tMins = timeNow / minute;
 
 days = timeNow / day ;                               
 hours = (timeNow % day) / hour;                      
 mins = ((timeNow % day) % hour) / minute ;        
 secs = (((timeNow % day) % hour) % minute) / second;
}
