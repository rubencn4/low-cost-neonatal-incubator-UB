#include <PID_v1.h>

#include <Wire.h>

#include <SparkFun_SHTC3.h>

SHTC3 mySHTC3;

#include <LiquidCrystal_I2C.h>

#include "HX711.h" //Weight sensor library
HX711 scale; //Configure the Weight sensor
#define RELAY_PIN 6

//Define Variables we'll be connecting to
double set_temp, Input, Output;

//Specify the links and initial tuning parameters
double Kp = 50, Ki = 5, Kd = 0;
PID myPID( & Input, & Output, & set_temp, Kp, Ki, Kd, DIRECT);

int DOUT_w = A2; //DT pin of the weight sensor
int CLK_w = A3; //SCK pin of the weight sensor
#define bombadown 7
#define bombaup 8
#define analog_hum A1
#define analog_temp A0
int button = 11; //Button to change between variables and setpoint visualization
int alarmPin = 12; // the number of the LED pin

//Led blink
unsigned long tled = 0; // will store last time LED was updated
int ledState = LOW;
int alarm_1_up = 39;
int alarm_1_low = 25;

int WindowSize = 5000;
unsigned long windowStartTime;

unsigned long t0_lectu; // Time used to write the temp, weight and HR values on console
int interval_lectu = 10000; //The measures will be displayed in the LCD every 2 seconds

long interval_fan = 900000; //The measures will be displayed in the LCD every 2 seconds
int c_fan=0;

// set the LCD number of columns and rows
int lcdColumns = 16;
int lcdRows = 2;
unsigned long t0 = millis();
int Tm = 200; //Sampling freq in us
int a = 0, b = 0;

float total_rh = 0.0, total_Temp = 0.0, total_W = 0.0;
float mean_rh = 0.0, mean_temp = 0.0, mean_w = 0.0;
int i = 0;

// set LCD address, number of columns and rows
// if you don't know your display address, run an I2C scanner sketch
LiquidCrystal_I2C lcd(0x27, lcdColumns, lcdRows);

void setup() {
  windowStartTime = millis();
  //tell the PID to range between 0 and the full window size
  myPID.SetOutputLimits(0, WindowSize);
  //turn the PID on
  myPID.SetMode(AUTOMATIC);

  Serial.begin(9600);
  Wire.begin();
  errorDecoder(mySHTC3.begin());
  pinMode(bombaup, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(bombadown, OUTPUT);
  pinMode(button, INPUT);

  scale.begin(DOUT_w, CLK_w);
  Serial.print("Reading the ADC value:  ");
  Serial.println(scale.read());
  Serial.println("Do not put any object on the scale");
  Serial.println("...");
  scale.set_scale(483610); // We use the scale we commuted
  scale.tare(20); //The weight is the tare
  Serial.println("Ready to weigh");

  lcd.init();
  // turn on LCD backlight                      
  lcd.backlight();

}

void loop() {

  float RH, T, W;
  SHTC3_Status_TypeDef result = mySHTC3.update();
  RH = mySHTC3.toPercent(); //Read humidity data                       
  T = mySHTC3.toDegC(); //Read temperature data 
  W = scale.get_units(); //Accumulation of the Weight                   
  total_rh = total_rh + RH;
  total_Temp = total_Temp + T;
  total_W = total_W + W;
  i++;
  // Mean filter
  if (millis() - (t0) >= Tm) {
    mean_rh = total_rh / i;
    mean_temp = total_Temp / i;
    mean_w = total_W / i;
    
    int analogTemp = analogRead(A0);
    int Percentage_temp =  map(analogTemp, 0, 1023, 25 * 100, 45 * 100);
    set_temp = (Percentage_temp / 100.00); //Convert Percentage_temp to % and Round for displaying it
    Input = mean_temp;
    myPID.Compute();

    // Temperature PID Control
    if (millis() - windowStartTime > WindowSize) { //time to shift the Relay Window
      windowStartTime += WindowSize;
    }

    if (Output < millis() - windowStartTime) {
      digitalWrite(RELAY_PIN, LOW);
    } else {
      digitalWrite(RELAY_PIN, HIGH);
    }

    t0 = millis();

    int set_rh = analogRead(analog_hum) / 10.23; // RH setpoint in %
    int buttonState = digitalRead(button); // Read the state of the button
    if (buttonState == HIGH) { //If the button is pushed we show the setpoints. Fer atach interrupt
        //Write the values:
        lcd.setCursor(0, 0);
        String str_temp_sp = "Sp. Temp: " + String(set_temp, 1) + (char) 223 + "C  ";
        lcd.print(str_temp_sp);
        lcd.setCursor(0, 1);
        String str_hum_sp = "Sp. RH:     " + String(set_rh) + " % ";
        lcd.print(str_hum_sp);
      }
      if (buttonState == LOW) { //If the button is not pushed we show the current weight, temperature and RH
        lcd.setCursor(0, 0);
        String str_temp = "T:" + String(mean_temp,1) + (char) 223 + "C  ";
        lcd.print(str_temp);
        lcd.setCursor(9, 0);
        String str_hum_val = " RH:" + String(mean_rh,0) + "%  ";
        lcd.print(str_hum_val);
        lcd.setCursor(0, 1);
        String str_w = "Weight:   " + String(abs(mean_w)) + "Kg";
        lcd.print(str_w);
      }
    // Humidity Control
    if (mean_rh < (set_rh + a)) {
      a = 2;
      b = 0;
      digitalWrite(bombaup, HIGH);
      digitalWrite(bombadown, LOW);

    } else {

      digitalWrite(bombaup, LOW);
      a = 0;
      b = 0;
    }

    if (mean_rh >= set_rh + 10 + b) {
      b = 2;
      a = 0;
      digitalWrite(bombadown, HIGH);

    }
    // Internal alarm system
    if (mean_temp > alarm_1_up || mean_temp < alarm_1_low) {
        digitalWrite(alarmPin, HIGH);
    }
    // Sending the results to serial monitor
    if (millis() - t0_lectu >= interval_lectu) {
        Serial.println(String(mean_rh) + "," + String(mean_temp) + "," + String(abs(mean_w), 2) + "," + set_rh + "," + String(set_temp,1));
        t0_lectu += interval_lectu;
      }
    mean_rh = 0;mean_temp = 0;mean_w = 0;
    total_Temp = 0;total_rh = 0;total_W=0;
    i = 0;

  }
 
}
void errorDecoder(SHTC3_Status_TypeDef message) // The errorDecoder function prints "SHTC3_Status_TypeDef" results in a human-friendly way
{
  switch (message) {
  case SHTC3_Status_Nominal:
    Serial.print("Nominal");
    break;
  case SHTC3_Status_Error:
    Serial.print("Error");
    break;
  case SHTC3_Status_CRC_Fail:
    Serial.print("CRC Fail");
    break;
  default:
    Serial.print("Unknown return code");
    break;
  }
}
