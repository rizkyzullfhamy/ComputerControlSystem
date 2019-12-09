  #include <LiquidCrystal.h>
  #include <Encoder.h>
  #include <CaptureTimer.h>

  #define ctINPin     52 
  #define samplingPer   10
  #define ENCODER_OPTIMIZE_INTERRUPS      // INTERRUPT ENCODER
  #define dirMotor 2                      //Directional MOTOR
  #define pwmMotor 3                      // PWM MOTOR
  #define btnRIGHT 0                      // Tombol Kanan
  #define btnUP 1                         // Tombol Atas
  #define btnDOWN 2                       // Tombol Bawah
  #define btnLEFT 3                       // Tombol Kiri
  #define btnSELECT 4                     // Tombol Ok
  #define btnNONE 5                       // None

  // PID DEFINE //
  #define TimeSampling 0.01
  #define KP 0.0289     //0.0082048
  #define KI 1.81       //1.2572
  #define KD 0.000116   //1.3387e-05

  // VAR GLOBAL PID //
   long SetPoint;
   float Proportional, Integral, Derivative, PID;
   static float Err = 0, LastErr = 0, IntegralErr = 0, DerivativeErr = 0;

  // VAR GLOBAL RPM_LCD //
   LiquidCrystal lcd (8 , 9, 4, 5, 6, 7);
   float duty;
   float rpmp, periode, freq, rpmf;
   Encoder encModule(24,25);
   long positionModule = 0;
   long newEncModule;

  // VAR GLOBAL Voltage~CurrentDivider //
   float Voltage = 0;
   float vRead = 0;
   float R1  = 30000;
   float R2  = 3300;
   float iRead = 0, Current = 0;
   int mVperAmp = 185, ACSoffset = 1650;

  // VAR GLOBAL RX TX //
   char buff[200];
   char inbyte; 
   unsigned long prevmillis = 0; 
   static int mode = 1;

  // VAR GLOBAL BUTTON //
   int lcd_key = 0;
   int adc_key_in = 0;

//=========================== FUNCTION PROGRAM ===========================//
void SetPID(){
    //if (millis() - prevmillis > 10) {
    //prevmillis = millis();
    Err = newEncModule - rpmf;
    IntegralErr += (Err * TimeSampling);
    if (IntegralErr >= 1000) IntegralErr = 1000;
    else if(IntegralErr <= -1000) IntegralErr = -1000;

    DerivativeErr = (Err - LastErr) / TimeSampling;
    LastErr = Err;
    
    Proportional = Err * KP;
    Integral = IntegralErr * KI;
    Derivative = DerivativeErr * KD;
    
    PID = Proportional + Integral + Derivative;
    
    if (PID > 255) PID = 255;
    else if (PID < 0) PID = 0;
    //Serial.print("PID = "); Serial.println(PID);
    digitalWrite (dirMotor, LOW);
    analogWrite (pwmMotor, PID); 
    delay(10);
    //}
}
int read_LCD_buttons() {
    adc_key_in = analogRead(0);
    if (adc_key_in > 1000) return btnNONE;
    if (adc_key_in < 50) return btnRIGHT;
    if (adc_key_in < 250) return btnUP;
    if (adc_key_in < 450) return btnDOWN;
    if (adc_key_in < 650) return btnLEFT;
    if (adc_key_in < 950) return btnSELECT;
   // return btnNONE;
    return mode;
}
void Volt_Current(){
  // voltage
  vRead  = analogRead(10) * 3.3 /1024;
  Voltage = vRead / (R2 / (R1+R2));
 // Voltage = Voltage * ((float) newEncModule/255);
}
void periodes(){  
  uint32_t Time;
  if (CaptureTimer::getTickCapture(&Time) == true)
  {
    periode = (float)Time / 1000000;
    rpmp = 60 / (periode * 200);
  }
}
void frekuensi(){  
  uint16_t ticks;
  (void) CaptureTimer::getFreq(&ticks);
  freq = ticks;
  rpmf = freq * 60 / 200;
}
void Send_Data(){
    //sprintf(buff, "\r\nPWM = %d | Duty Cycle = %.2f | RPMF = %d  | RPMP = %d | Voltage = %.2f", 
    //            newEncModule,duty,(int)rpmf,(int)rpmp,Voltage);
    sprintf(buff, "%d %d %d\r\n", PID, (int)rpmf, newEncModule);
    Serial.print(buff);
}

void setup(){
  Serial.begin(115200);
  // put your setup code here, to run once:
  lcd.begin(16,2);
  lcd.setCursor(0,0);
  lcd.print("ENC   = ");
  lcd.setCursor(0,1);
  lcd.print("RPM   = ");

  // initialize the capture timer pin & period
  CaptureTimer::initCapTicks(samplingPer, ctINPin);
  CaptureTimer::initCapTime(ctINPin);
}
void loop(){
    if(Serial.available()){
        inbyte = Serial.read();
         if (inbyte == '1'){
           mode = 1;
         }else if(inbyte == '2'){
           mode = 2;
         }
      }else{
        //mode = read_LCD_buttons();
      }
        
    if(mode == 1){
        newEncModule = encModule.read() * 5;
        if (newEncModule != positionModule){
            if (newEncModule >= 4500 ){newEncModule = 4500;}  // Batas PWM Out
            else if(newEncModule <=0){newEncModule = 0;}

            positionModule = newEncModule;
    }
        Volt_Current();
        frekuensi();
        periodes();
        SetPID();
        //PLOT PROGRAM//
          Send_Data(); 
    }
    else if(mode == 2){

    }
}
