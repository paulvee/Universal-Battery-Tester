/*================================
micro:  		Arduino Nano with bootloader Uno    (Arduino Uno)
Display:		TFT ST7789 240x240
Sensor:			INA219
Power:			
Data:			23/01/2023

File programma:	H:\arduino-1.8.13\portable\sketchbook\Battery_Test_Aldo-v3\Battery_Test_Aldo-v3.ino
Annotazioni:	
 ST7789 240x240 IPS (without CS pin) connections (only 6 wires required):
 #01 GND -> GND
 #02 VCC -> VCC (3.3V only!)
 #03 SCL -> D13				52 Mega
 #04 SDA -> D11				51 Mega
 #05 RES -> D9
 #06 DC  -> D8
 #07 BLK -> NC
 
 * Adafruit INA219 Breakout board
 * I2C connections:
 *    SCL pin goes to Nano A5
 *    SDA pin goes to Nano A4 
 * VCC to +5V
 * GND to GND
 * Vin- and Vin+ are not used
===================================*/
#include <Button.h>
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <EEPROM.h>
#include <SPI.h>
//#include <Adafruit_GFX.h>
#include <Arduino_ST7789_Fast.h>

// pin I/O Arduino
#define end_sounder  A2 //buzzer
#define potPin  A0		  //potenziometro 

Button BT1(2);
Button BT2(4);

// pin display ST7789 
#define TFT_DC    8
#define TFT_RST   7 
// dimensione schermo display
#define SCR_WD 240
#define SCR_HT 240
// define what kind of fonts should be used
#define USE_RRE_FONTS 1

Arduino_ST7789 lcd = Arduino_ST7789(TFT_DC, TFT_RST);

#include "RREFont.h"
#include "rre_term_10x16.h"
#include "rre_bold13x20.h"
#include "rre_bold13x20v.h"
#include "rre_bold13x20no.h"

RREFont font;
// needed for RREFont library initialization, define your fillRect
void customRect(int x, int y, int w, int h, int c) { return lcd.fillRect(x, y, w, h, c); }  

//-----------------------------------------------------------------------------

unsigned long ms = 0;
int stato = 10;
boolean FIRST = true;
boolean ONCE = true;

int target_mA; 			    //= 100;
float cutoff_voltage; 		//= 3.2;
float kP;     				//= 50;
float v1,v2,v3;
int a1, a2, a3; 
float b1, b2, b3;
byte cifra[4];
char str_h[3];				//stringa per ore 
char str_m[3];				//stringa per minuti
char str_s[3];				//stringa per secondi


//================== variabili per  RUN  ==================
int time_limit = 360;       // 6 hours. Test relies on the..    
float offset = 0.0;
int tolerance = 1;

//int pwm = 0;
float error;
boolean manual = false;

String SW_VERSION = "3.04";

// Current shunt and voltage measurements
Adafruit_INA219 ina219;

float shuntvoltage = 0;
float busvoltage = 0;
double current_mA = 0;
float loadvoltage = 0;
float power_mW = 0;
/*
  * La seguente variabile visualizza la tensione della cella.
  * C'è una costante R che può essere impostata nella routine read_INA che può
  * essere utilizzato per tenere conto delle perdite di cablaggio o traccia PCB nel loop di corrente.
  * Se puoi misurarlo.
  * Puoi anche utilizzare un DVM molto accurato per confrontare la tensione della cella
  * con ciò che misuri e modifica questa variabile in modo che corrisponda a quanto visualizzato
  * tensione di cella. Se ci tieni a questa precisione...e cell voltage
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
boolean sounded = false;  // flag to limit beeping
int beep = 1;             // value coming from PC, no longer used

// Variables and flags to terminate test
int cancel = 0;
boolean timed_out = false;
boolean high_current = false;
boolean cutoff_voltage_reached = false;
String error_code = "";
boolean end_of_test = false;

//------------------------------------------------------------
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
//----------------------------------------------------------

/* xx-bit version of analogWrite(). Works only on pins 9 and 10. */
void analogWriteFast(uint8_t pin, uint16_t val)
{
  switch (pin) {
    case  9: OCR1A = val; break;
    case 10: OCR1B = val; break;
  }
}
//--------------------------------------------------------------------

// PWM setup
const byte pwm_pin = 9;   // PWM DAC, only pins 9 and 10 are allowed with the fast PWM
int pwm = 2000;           // Starting value for 13-bit DAC. PWM is typ. @ 4540
float pid_error;          // the error between current setiing and actual current, 
                          // used in the "PID" calculation to drive the MOSFET
//==========================================================

void setup() 
{
  Serial.begin(9600);
  lcd.init(SCR_WD, SCR_HT);
  font.init(customRect, SCR_WD, SCR_HT); // custom fillRect function and screen width and height values 


//inizializzazione EEPROM solo prima volta quindi remmare
  //EEPROM.put(0, target_mA);			//scrivi target_mA
  //EEPROM.put(10, cutoff_voltage);		//scrivi cutoff_voltage
  //EEPROM.put(20, kP);					//scrivi kP  
  EEPROM.get(0, target_mA);
  EEPROM.get(10, cutoff_voltage);
  EEPROM.get(20, kP);
  
// Inizializza  INA219.
  // Per impostazione predefinita, l'inizializzazione utilizzerà l'intervallo più ampio (32 V, 2 A). Tuttavia
  // puoi chiamare una funzione setCalibration per modificare questo intervallo (vedi commenti).
  ina219.begin(); 
    
  BT1.begin();
  BT2.begin();
  pinMode(end_sounder, OUTPUT);       // Output to sounder.
  digitalWrite(end_sounder, LOW);

// Set up DAC pin come output
  pinMode(pwm_pin, OUTPUT);

}

//const uint16_t lnCol  = RGBto565(255,150,255);
const uint16_t ln2Col = RGBto565(180,180,180);		//bianco
const uint16_t labCol = RGBto565(250,250,250);		//grigio
const uint16_t v1Col  = RGBto565(100,250,100);		//verdino
const uint16_t v4Col  = RGBto565(255,0,0);			//rosso
const uint16_t v2Col  = RGBto565(235, 164, 52);		//arancio
const uint16_t v3Col  = RGBto565(66, 135, 245);		//azzurro
const uint16_t v5Col  = RGBto565(255, 255, 0);		//giallo

int mode=0,lastMode=-1;

void setBigNumFont()
{

  font.setFont(&rre_Bold13x20v);

  font.setSpacing(1);
  font.setScale(1,2);
  font.setDigitMinWd(16);
}


void drawField(int x, int y, int w, int h, char *label, uint16_t col=v3Col)   //disegna i campi
{
  lcd.drawRect(x,y+7,w,h-7,col);
  font.setFont(&rre_term_10x16);
  font.setScale(1);
  font.setColor(labCol,BLACK);
  int wl = font.strWidth(label);
  font.printStr(x+(w-wl)/2,y,label);
}


void RUN_SETUP(){				//stato 10
  if (FIRST) {
	lcd.fillScreen(BLACK);
	setBigNumFont();
	font.setFont(&rre_term_10x16);
	font.setScale(2);
	font.setColor(ln2Col);
	font.printStr(10, 15, "Battery  Test");
	font.printStr(65, 65, " v3.1.0");
	font.setColor(v5Col);
	font.printStr(10, 140, "BT1 >>  Setup");
	font.printStr(10, 190, "BT2 >>  Run");
	FIRST = false;
  }	
  //leggi pulsante BT1
  if (BT1.pressed()) {
    //entra in displaySetup  stato 0
    stato = 0;  
    FIRST = true;
  }

  //leggi pulsante BT2
  if (BT2.pressed()) {
    //entra in displayRun  stato 20
    stato = 20;  
    FIRST = true;
  }	  
}	

void displaySetup(){
  if (FIRST) {
    lcd.fillScreen(BLACK);
	drawField(    0,  0,235,78," Target_mA ");
	drawField(    0, 81,235,78," Cutoff_voltage ");
	drawField(    0,162,240,78," Proportional Kp ");	
    FIRST = false;
  }
  setBigNumFont();
  int wv=font.strWidth("8888");										//lunghezza stringa      88.8
  int wv2=font.strWidth("8.88");   
  font.setFont(&rre_term_10x16);
  font.setScale(1,2);
  font.setColor(v1Col); font.printStr(134,36,"mA");
  font.setColor(v1Col); font.printStr(141,115,"V");
  font.setColor(v1Col); font.printStr(141,194,"%");
  
  //visualizza i dati
  showVal(target_mA, 48,24, 4,0, v1Col);
  showVal(cutoff_voltage, 48,106, 4,2, v1Col);
  showVal(kP, 48,187, 4,1, v1Col);
  
  //leggi pulsante BT1
  if (BT1.pressed()) {
    //imposta target_mA
    stato = 1;  
 
    FIRST = true;
  }
  
  //leggi pulsante BT2
  if (BT2.pressed()) {
    //ritorna a displaySetup  stato 10
    stato = 10;  
    FIRST = true;
  }
  
}	

void showVal(float v, int x, int y, int w,  int p, uint16_t col)
{
  setBigNumFont();
  font.setColor(col,BLACK);
  char txt[10];
  dtostrf(v,w,p,txt);
  font.printStr(x,y,txt);
}


void loop()
{
  switch(stato){
	case 10:
	  RUN_SETUP();	 
	  break;
	case 0:
	  displaySetup();	 
	  break;  
	case 1:
	  setCurrent();
	  break;
	case 2:
	  setVoltage();
	  break;
	case 3:
	  setKP();
	  break;
	case 20:
	  displayRun();
	  break;
	case 30:
	  Messaggio();
	  break;
	case 40:
	  result();
	  break;
  }

}


void setCurrent(){
  if (FIRST) {
    v1 = target_mA;
	showVal(v1, 48,24, 4,0, v4Col);
    //formatCifre( target_mA, 1);
    FIRST = false;	
  }
 
  
  delay(1000);
  while (BT1.read()) {       //leggi potenziometro
	a1 = map(analogRead(potPin), 0, 1023, 0, 1500);	
	v1 = a1;	
	showVal(v1, 48,24, 4,0, v4Col);
	delay(200);
  }

  
  target_mA = v1;                 //salva valore target_mA
  EEPROM.put(0, target_mA);				//scrivi target_mA
  showVal(v1, 48,24, 4,0, v1Col);
  stato = 2;
  FIRST = true;
}   	


void setVoltage(){
  if (FIRST) {
    v2 = cutoff_voltage;
	showVal(v2, 48,106, 4,2, v4Col);
    //formatCifre( target_mA, 1);
    FIRST = false;	
  }
 
  
  delay(1000);
  while (BT1.read()) {       //leggi potenziometro
	a2 = map(analogRead(potPin), 0, 1023, 0, 450);	
	b2 = float(a2);
	v2 = b2/100;	
	showVal(v2, 48,106, 4,2, v4Col);
	delay(200);
  }

  
  cutoff_voltage = v2;              //salva valore target_mA
  EEPROM.put(10, cutoff_voltage);		//scrivi cutoff_voltage
  showVal(v2, 48,106, 4,2, v1Col);
  stato = 3;
  FIRST = true; 
}   	


void setKP(){
  if (FIRST) {
    v3 = kP;
	showVal(v3, 48,187, 4,1, v4Col);  //formatCifre( target_mA, 1);
    
    FIRST = false;	
  }
 
  
  delay(2000);
  while (BT1.read()) {    //leggi potenziometro
	a3 = map(analogRead(potPin), 0, 1023, 0, 800);
	b3 = float(a3);		
	v3 = b3/10;	
	showVal(v3, 48,187, 4,1, v4Col);
	delay(200);
  }

  
  kP = v3;              //salva valore target_mA
  EEPROM.put(20, kP);		//scrivi cutoff_voltage
  showVal(v3, 48,187, 4,1, v1Col);
  stato = 10;
  FIRST = true;
}


void displayRun(){
  if (FIRST) {
    lcd.fillScreen(BLACK);
	drawField(0, 0, 115, 78, " Voltage ");
	drawField(120, 0, 115, 78, " Current ");
	drawField(0, 81, 115, 78,"  PWM  ");
	drawField(120, 81, 115, 78, "  PID  ");
	drawField(0, 162, 115, 78," Capacity ");
	drawField(120, 162, 115, 78," Time ");

	//--------------------------------------
	
    setupFastPWM();					        // imposta registri DAC
    analogWriteFast(pwm_pin, pwm);  // and set to zero PWM out (off)
	
	//* Con la risoluzione pwm più alta, ci vuole più tempo per passare a un'impostazione di corrente elevata
    //* abbreviare il tempo di accelerazione
	if (target_mA > 99){
		kP = 50;  // the maximum
    }
	
	// bleep once to signal the start of the test
    digitalWrite(end_sounder, HIGH);
    delay(300);
	digitalWrite(end_sounder, LOW);
	
	startMillisec = millis();   // get millisec timestamp for the starting point
	//---------------------------------------
	
	FIRST = false;
  }
  setBigNumFont();  
  font.setFont(&rre_term_10x16);
  font.setScale(1,2);
  font.setColor(v1Col); font.printStr(95, 36, "V");
  font.setColor(v1Col); font.printStr(210, 36, "mA");
  font.setColor(v1Col); font.printStr(80, 195, "mAh");
  
  // inizia test
  // get the data from the INA219
  readINA219();
  //Serial.println("ciclo iniziato");
   /*
    * Questa è una routine "PID" molto semplificata per pilotare il MOSFET con un
    * valore PWM, basato sulla differenza tra il valore target_mA impostato e il valore actual_mA misurato
    * valore corrente.
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
  

//----------------------- Rilevamento Errori  o  Fine Test---------------------------------------------

	// check if the cell voltage has reached the set cut off voltage
	// and abort the cycle if it has.
	// end_of_test is used to stop further processing.

  if ((!end_of_test) && (loadvoltage < cutoff_voltage)) {
    delay(3000);    // tenere conto di un breve calo quando iniziamo il processo di scaricamento
    readINA219();   // read the values again
    if (loadvoltage < cutoff_voltage) {
      analogWriteFast(pwm_pin, 0);  // turn PWM off.                                
      cutoff_voltage_reached = true;
      target_mA = 0;
      error_code = "  END TEST";  // display this on line 5 of theLCD
	                 
      end_of_test = true;
    }
  } 

	// Check if the measured current has overshot the target value
	// by more than 100%. If so, we have a problem so abort. 
  if ((!end_of_test) && (current_mA > (target_mA * 2.0))) {
     analogWriteFast(pwm_pin, 0);  // turn PWM off.
     
	 target_mA = 0;
     error_code = "ERR - Hi mA";  // display this on line 5 of the lcd
     end_of_test = true;
  }  

  // If the cycle takes too long, terminate it
  if ((!end_of_test) && (tMins > time_limit)) {
    analogWriteFast(pwm_pin, 0);  // turn PWM off.
    timed_out = true;
    target_mA = 0;
    error_code = "ERR-Time Out"; // display this on line 5 of the lcd
    end_of_test = true;  
  }
  
  //* Se tutto è OK e non ci sono condizioni di errore, 
  //* uscire dal nuovo valore PWM per regolare la corrente.
  if ((cancel == 0) && (!timed_out) && (!high_current) && (!cutoff_voltage_reached) && (!end_of_test)) {
    analogWriteFast(pwm_pin, pwm);  // Adjust the 13-bit PWM to the calculated error correction value.
  }
  else {  // if the process is terminated, sound the beeper, but only once
    if (!sounded) {
      sounded = true;
      for (int i = 0; i< 3; i++) {
        digitalWrite(end_sounder, HIGH);
        delay(300);
        digitalWrite(end_sounder, LOW);
        delay(100);
      } 
    }		
	//entra in Messaggio  stato 30
	stato = 30;
	FIRST = true;
  }
  
//----------------------------------------------------------------------------------------------------- 

 
  // * Calcola il tempo trascorso e i mAh utilizzati ciascuno
  // * secondo giro del giro.
    
  getTime();
  // calculate the mAh capacity so far of the cell
  if (millis() > millisCalc_mAh + 1000) {
    float this_hours = (millis() - startMillisec) / (1000.0 * 3600.0);
    mAh_soFar = mAh_soFar + ((this_hours - last_hours) * current_mA);
    last_hours = this_hours;  
    millisCalc_mAh = millis();
  }

  // finally, update the lcd with the fresh values
  write_to_lcd();
  
  //leggi pulsante BT2  
  if (BT2.pressed()) {
	analogWriteFast(pwm_pin, 0);  // turn PWM off.
    target_mA = 0;
	cancel = 1;
	end_of_test = true;  //false;
	
	//entra in Messaggio  stato 30
	stato = 30;
    FIRST = true;
  }	  
} 					// end of displayRun 
  

 /*
  * Ottieni corrente e tensione dalla scheda breakout Adafruit INA219.
  *
  * L'INA219 non è stato originariamente progettato per essere utilizzato in questo tipo di applicazione.
  * Doveva aiutare a calcolare e visualizzare la capacità della batteria per i laptop,
  * tablet e telefoni. In queste applicazioni, la precisione non è realmente richiesta.
  *
  * In questa applicazione, la lettura corrente INA219, che all'inizio è molto nervosa,
  * viene utilizzato per pilotare un MOSFET nella regione lineare. Thge MOSFET viene utilizzato come resistenza variabile.
  * Un cambiamento di tensione molto piccolo (singolo mVolt) applicato al Gate si tradurrà in un cambiamento abbastanza grande
  * modifica corrente. Questo è il motivo per cui ho utilizzato un PWM a 13 bit, per ottenere una risoluzione migliore.
  * Le letture INA219 devono essere mediate un numero di volte per ottenere valori ragionevolmente stabili.
  *
  */
void readINA219() {   // Obtain the INA219 readings.
  float R = 0.09;     //modifica questo valore per compensare le perdite 
                      //di resistenza del circuito.
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
  * Scrivere i valori ottenuti sullo schermo LCD 
  *
  * Il modo più semplice è semplicemente cancellare lo schermo e costruirlo prima
  * inviandolo di nuovo.
  *
  * Il testo per il display LCD viene prima inserito in un buffer prima di essere trasferito
  * lo schermo invocando la funzione display().
  *
  * Per rendere più piacevole la visualizzazione dei numeri in varie dimensioni, ho provveduto a
  * posizionare i numeri giustificati a destra. Questa è l'unica "complessità" in questo codice.
  *
  */
void write_to_lcd() {

  //visualizza i dati
  showVal(loadvoltage, 10, 24, 4, 2, v5Col);
  showVal(current_mA, 130, 24, 4, 0, v5Col);
  showVal(pwm, 10, 106, 4, 0, v2Col);
  showVal(pid_error, 130, 106, 4,1, v2Col);
  showVal(mAh_soFar, 10, 187, 4, 0, v4Col);
  
  //converti valori tempi in stringa
  sprintf(str_h, "%02d", hours);
  sprintf(str_m, "%02d", mins);
  String string_h = str_h; 
  String string_m = str_m;
  String s = string_h + ":" + string_m;		//stringa composta  hh:mm


  int buffer_len = 6;
  char buffer[buffer_len];

  s.toCharArray(buffer, buffer_len);
 
  font.printStr(130, 187, buffer);			//visualizza tempo trascorso 

}


/*
 * Routine generica per calcolare ore, minuti e secondi tra due valori millis().
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


void Messaggio(){
  if (FIRST) {
    lcd.fillScreen(BLACK);
	drawField(    0, 81,235,88," Message");
	setBigNumFont();
	font.setFont(&rre_term_10x16);
	font.setScale(2);
	font.setColor(v2Col);
	
	//converti stringa in char
	if (cancel == 0) {
		String m = error_code;		//stringa messaggio errore 
		int buffer1_len = 13;
		char buffer1[buffer1_len];
		m.toCharArray(buffer1, buffer1_len);	
		font.printStr(10, 115, buffer1);
	}
	else { 
		font.printStr(10, 115, "  CANCELLED");		
	}		
	FIRST = false;
  }
  //leggi pulsante BT2
  if (BT2.pressed()) {
    //entra in result  stato 40
    cancel = 0;
	stato = 40;  
    FIRST = true;
  }
}


void result(){
  if (FIRST) {
    lcd.fillScreen(BLACK);
	drawField(0, 0, 115, 78, " Voltage ");
	drawField(120, 0, 115, 78, " Current ");
	drawField(0, 81, 115, 78,"  PWM  ");
	drawField(120, 81, 115, 78, "  PID  ");
	drawField(0, 162, 115, 78," Capacity ");
	drawField(120, 162, 115, 78," Time ");

	setBigNumFont();  
	font.setFont(&rre_term_10x16);
	font.setScale(1,2);
	font.setColor(v1Col); font.printStr(95, 36, "V");
	font.setColor(v1Col); font.printStr(210, 36, "mA");
	font.setColor(v1Col); font.printStr(80, 195, "mAh");
	
	write_to_lcd();
	FIRST = false;
  }
  //leggi pulsante BT2
  if (BT2.pressed()) {
    //entra in run/setup  stato 10
    stato = 10;  
    FIRST = true;
  }	  	
}		
