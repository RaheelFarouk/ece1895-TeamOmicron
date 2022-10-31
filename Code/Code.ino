// ECE 1895 - Design Project 2 - Team Omicron
// Veronica Bella, Raheel Farouk, Maggie Killmeyer

// libraries
#include <LiquidCrystal_I2C.h>
#include <DFRobot_RGBLCD1602.h>
#include <SPI.h>  // for SD card
#include <SD.h>   // for SD card

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#include "SoftwareSerial.h"
#include "DFRobotDFPlayerMini.h"


// Macros for pin numbers
// input pins
#define GYRO_SCL_PIN A5
#define GYRO_SDA_PIN A4
#define SLIDER_PIN A0
#define ENCODER_A_PIN 6
#define ENCODER_B_PIN 7

// output pins
#define LED_0_PIN 3
#define LED_1_PIN 4
#define LED_2_PIN 5
#define LCD_SCL_PIN A5
#define LCD_SDA_PIN A4
#define SD_CARD_DO D12
#define SD_CARD_DI D11
#define SoftwareSerialTX 9
#define SoftwareSerialRX 8

const int delayTime = 5;   // time between games of bop it after win or loss
const int maxStartTime = 5;  // most time allowed for successful action

const int TWIST_IT = 0;
const int PUSH_IT = 1;
const int SHAKE_IT = 2;

int aEncoderState;
int aEncoderLastState;

DFRobot_RGBLCD1602 lcd(16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display
Adafruit_MPU6050 mpu;

SoftwareSerial mySoftwareSerial(SoftwareSerialRX, SoftwareSerialTX); // RX, TX
DFRobotDFPlayerMini myDFPlayer;

void setup() {
  Serial.begin(9600);

  pinMode(GYRO_SCL_PIN, INPUT);
  pinMode(GYRO_SDA_PIN, INPUT);
  //pinMode(ACEL_PIN, INPUT);

  pinMode(SLIDER_PIN, INPUT);
  pinMode(ENCODER_A_PIN, INPUT);
  pinMode(ENCODER_B_PIN, INPUT);

  pinMode(LED_0_PIN, OUTPUT);
  pinMode(LED_1_PIN, OUTPUT);
  pinMode(LED_2_PIN, OUTPUT);

  pinMode(LCD_SCL_PIN, OUTPUT);
  pinMode(LCD_SDA_PIN, OUTPUT);

  //pinMode(SPEAKER_PIN, OUTPUT);

  lcd.init();
  lcd.setBacklight(true);
  lcd.setRGB(255,255,255);



  //Setting up the Gyro?/Accel
  // Try to initialize MPU6050!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  //begin the communicationwith the mp3 module
  mySoftwareSerial.begin(9600);
  if (!myDFPlayer.begin(mySoftwareSerial)) {  //Use softwareSerial to communicate with mp3.
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1.Please recheck the connection!"));
    Serial.println(F("2.Please insert the SD card!"));
    while(true){
      delay(0); // Code to compatible with ESP8266 watch dog.
    }
  }
  Serial.println(F("DFPlayer Mini online."));
  
  myDFPlayer.volume(15);  //Set volume value. From 0 to 30

}

void loop() {

  // if(playGame()){
  //   winner();
  // } else{
  //   loser();
  // }

  // delay(delayTime*1000);
  playGame();
}

/**
  Turns on or off all leds
  @param mode can be either HIGH (on) or LOW (off)
*/
void setLeds(int mode){
  digitalWrite(LED_0_PIN, mode);
  digitalWrite(LED_1_PIN, mode);
  digitalWrite(LED_2_PIN, mode);
}

/**
  Sets up SD card and anything needed for audio
*/
void setupAudio(){
  SD.begin();
  SD.open("", FILE_READ);
}

/**
  Verifies that the slider was pushed to desired amount within the alloted time frame
  @param maxTime maximum ammount of time to complete twist it action
  @return true if the push it action was completed in time
*/
bool verifySlider(int maxTime){
  unsigned long startTime = millis();
  int startPos = analogRead(SLIDER_PIN);

  while (millis() - startTime < maxTime){
    // check other action inputs, for false positives
    
    aEncoderState = digitalRead(ENCODER_A_PIN);
    if (aEncoderState != aEncoderLastState){     
     if (digitalRead(ENCODER_B_PIN) != aEncoderState) { 
       return false;
     } else {
       return false;
     }
    }

    // verify no gyro
    /* Get new sensor events with the readings */
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    if(a.acceleration.z<-15.0){ //-10.0 is the acceleration threshold
      return true;
    }

    if (abs(analogRead(SLIDER_PIN) - startPos) >= 500) return true;
  }

  return false;
}

/**
  Verifies that the encoder was twisted to desired amount within the alloted time frame
  @param maxTime maximum ammount of time to complete twist it action
  @return true if the twist it action was completed in time
*/
bool verifyEncoder(int maxTime){
  unsigned long startTime = millis();
  int counter = 0;

  while(millis() - startTime < maxTime){
    // check other action inputs, for false positives
    int startPos = analogRead(SLIDER_PIN);
    if (abs(analogRead(SLIDER_PIN) - startPos) >= 20) return false;  

    // verify no gyro
    /* Get new sensor events with the readings */
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    if(a.acceleration.z<-15.0){ //-10.0 is the acceleration threshold
      return true;
    }

    aEncoderState = digitalRead(ENCODER_A_PIN);
    if (aEncoderState != aEncoderLastState){     
     if (digitalRead(ENCODER_B_PIN) != aEncoderState) { 
       counter ++;
     } else {
       counter --;
     }
     //Serial.print(counter);
     if (abs(counter) >= 500) return true; // Threshold for twist it command
   }    
  }

  return false;
}

bool verifyAccel(int maxTime){
  unsigned long startTime = millis();
  int counter = 0;

  while(millis() - startTime < maxTime){
    // check other action inputs, for false positives
    int startPos = analogRead(SLIDER_PIN);
    if (abs(analogRead(SLIDER_PIN) - startPos) >= 20) return false;  

    //check encoder input
    aEncoderState = digitalRead(ENCODER_A_PIN);
    if (aEncoderState != aEncoderLastState){     
      if (digitalRead(ENCODER_B_PIN) != aEncoderState) { 
        return false;
      } else {
        return false;
      }
    }
    
    /* Get new sensor events with the readings */
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    if(a.acceleration.z<-15.0){ //-10.0 is the acceleration threshold
      return true;
    }
  }

  return false;
}

/**
  Basic game function, player gets action and must follow action
  @return true if player reaches 100 actions
*/
bool playGame(){
  int count = 0;
  int maxTime = maxStartTime;

  while (count <= 99){
    int action = random(2);
    //Serial.print(count);
    switch (action){
      case TWIST_IT:

        lcd.print("TWIST IT!");
        //myDFPlayer.play(); //enter track number in brackets

        aEncoderLastState = digitalRead(ENCODER_A_PIN);
        if (!verifyEncoder(maxTime*1000)){
          lcd.clear();
          return false;
        }
      break;

      case PUSH_IT:
        lcd.print("PUSH IT!");
        if (!verifySlider(maxTime*1000)){
          lcd.clear();
          return false;
        }
      break;

      case SHAKE_IT:
        lcd.print("PUSH IT!");
        if (!verifyAccel(maxTime*1000)){
          lcd.clear();
          return false;
        }
      break;
    }

    count++;
    //maxTime = maxTime / 1.025;    // arbitrary value to speed up
    delay(500);                     // wait half a second in between giving commands
  }

  return true;
}

/**
  Whatever happens when the player wins
*/
void winner(){
  setLeds(HIGH);

  // play winner sound

  setLeds(LOW);
}

/**
  Whatever happens when the player loses
*/
void loser(){
  setLeds(LOW);

  // play loser sound
}