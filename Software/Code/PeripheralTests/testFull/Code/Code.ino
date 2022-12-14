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
int selector;

bool firstPowerOn = true;

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
  lcd.setBacklight(false);
  lcd.setRGB(127,255,255);
  lcd.autoscroll();


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
  mpu.setHighPassFilter(MPU6050_HIGHPASS_5_HZ);

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
  
  myDFPlayer.volume(20);  //Set volume value. From 0 to 30
  myDFPlayer.play(1);
  delay(10);

  //get encoder last state
  aEncoderLastState = digitalRead(ENCODER_A_PIN);

}

void loop() {

  lcd.print("Choose a Game Mode by turning the knob");

  delay(delayTime*1000);
  lcd.clear();
  int choice = menu();
  lcd.clear();

  switch (choice){
    case 0:
      
      lcd.print("PLAY GAME MODE ACTIVATED");
      delay(2000);
      if(playGame(9, 6, 8)){
        winner();
      } else{
        loser();
      }
      break;

    case 1:
      lcd.print("CHAOS MODE ACTIVATED");
      delay(2000);
      lcd.clear();
      lcd.print("Follow the written actions, NOT sound cues");
      delay(2000);
      if(playGame(9, 6, 8)){
        winner();
      } else{
        loser();
      }
      break;

    case 2:
      lcd.print("TUTORIAL MODE ACTIVATED");
      break;    
  }

  firstPowerOn = false;

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

    if((a.acceleration.z)<-18.0){ //-10.0 is the acceleration threshold
      return false;
    }

    int sliderPos = abs(analogRead(SLIDER_PIN) - startPos);
    if (sliderPos >= 500){
      return true;
    } else if(sliderPos >= 275){
      myDFPlayer.play(7);
    }
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
  int startPos = analogRead(SLIDER_PIN);

  while(millis() - startTime < maxTime){
    // check other action inputs, for false positives
    
    if (abs(analogRead(SLIDER_PIN) - startPos) >= 20) return false;  
    startPos = analogRead(SLIDER_PIN);

    // verify no gyro
    /* Get new sensor events with the readings */
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    if((a.acceleration.z)<-18.0){ //-10.0 is the acceleration threshold
      return false;
    }

    aEncoderState = digitalRead(ENCODER_A_PIN);
    if (aEncoderState != aEncoderLastState){     
     if (digitalRead(ENCODER_B_PIN) != aEncoderState) { 
       counter ++;
       myDFPlayer.play(5);
      //  return true;
     } else {
       counter --;
      //  return true;
     }
     //Serial.print(counter);
     if (abs(counter) >= 5){
       return true; // Threshold for twist it command
       lcd.clear();  
     } 

     //lcd.print("passed counter");
   }    
  }

  return false;
}

bool verifyAccel(int maxTime){
  unsigned long startTime = millis();
  int counter = 0;
  int startPos = analogRead(SLIDER_PIN);

  while(millis() - startTime < maxTime){
    /* Get new sensor events with the readings */
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    if(abs((g.gyro.z+g.gyro.y+g.gyro.x)/3)>3){
      if((a.acceleration.z)<-18.0){ //-10.0 is the acceleration threshold
        delay(100);
        lcd.clear();
        return true;
      }
    }

    //lcd.print(a.acceleration.z);
    // check other action inputs, for false positives
    if (abs(analogRead(SLIDER_PIN) - startPos) >= 20) return false;  
    startPos = analogRead(SLIDER_PIN);

    //check encoder input
    aEncoderState = digitalRead(ENCODER_A_PIN);
    if (aEncoderState != aEncoderLastState){     
      if (digitalRead(ENCODER_B_PIN) != aEncoderState) { 
        return false;
      } else {
        return false;
      }
    }
  }

  return false;
}

/**
  Basic game function, player gets action and must follow action
  @return true if player reaches 100 actions
*/
bool playGame(int twistAudio, int pushAudio, int shakeAudio){
  int count = 0;
  int maxTime = maxStartTime;

  myDFPlayer.play(3); // does player pause code?????

  while (count <= 99){
    lcd.print("SCORE: " + count);
    int action = random(3);
    lcd.clear();

    switch (action){
      case TWIST_IT:
        lcd.print("TWIST IT!");
        myDFPlayer.play(twistAudio); //enter track number in brackets
        delay(20);       

        aEncoderLastState = digitalRead(ENCODER_A_PIN);
        if (!verifyEncoder(maxTime*1000)){
          return false;
        }
        lcd.clear();
      break;

      case PUSH_IT:
        lcd.print("PUSH IT!");
        myDFPlayer.play(pushAudio);
        delay(20);
        
        if (!verifySlider(maxTime*1000)){
          lcd.clear();
          return false;
        }
        lcd.clear();
      break;

      case SHAKE_IT:
        lcd.print("SHAKE IT!");
        myDFPlayer.play(shakeAudio);
        delay(20);
        if (!verifyAccel(maxTime*1000)){
          lcd.clear();
          return false;
        }
        lcd.clear();
      break;
    }

    count++;
    myDFPlayer.play(2);
    //maxTime = maxTime / 1.025;    // arbitrary value to speed up
    delay(500);                     // wait half a second in between giving commands
  }

  return true;
}

/**
  Whatever happens when the player wins
*/
void winner(){

  lcd.clear();
  // play winner sound
  lcd.print("You Win");

}

/**
  Whatever happens when the player loses
*/
void loser(){
  lcd.clear();
  lcd.print("You Lose");

  myDFPlayer.play(4);


  // play loser sound
}


void menuEncoder() {
	
	// Read the current state of CLK
	aEncoderState = digitalRead(ENCODER_A_PIN);
	// If last and current state of CLK are different, then pulse occurred
	// React to only 1 state change to avoid double count
	if (aEncoderState != aEncoderLastState  && aEncoderState == 1){
		// If the DT state is different than the CLK state then
		// the encoder is rotating CCW so decrement
		if (digitalRead(ENCODER_B_PIN) != aEncoderState) {
			selector --;
			//currentDir ="CCW";
		} else {
			// Encoder is rotating CW so increment
			selector ++;
			//currentDir ="CW";
		}
		//Serial.print("Direction: ");
		//Serial.print(currentDir);
		Serial.print(" | Counter: ");
		Serial.println(selector);
	}
	// Remember last CLK state
	aEncoderLastState = aEncoderState;
	// Put in a slight delay to help debounce the reading
	delay(1);
}

/**
  Menu to select what mode to be in
  @return int game mode index
*/
int menu(){
  selector = 0;
  char *menuOptions[] = {"Play Game", "CHAOS Mode", "Tutorial"};
  int length = sizeof(menuOptions)/sizeof(menuOptions[0]);
  
  //lcd.print(menuOptions[selector]);

  int startPos = analogRead(SLIDER_PIN);
  int oldValue;

  while ((abs(analogRead(SLIDER_PIN) - startPos) < 500)){
  //while (analogRead(SLIDER_PIN) < 500){

    menuEncoder();
    
     if (selector < 0){
       selector = length-1;
     } else if (selector > length-1){
       selector = 0;
     }
    
    if(selector != oldValue){
      lcd.clear();
      lcd.print(menuOptions[selector]); 
      oldValue = selector;
    }
    
  }

  return selector;

}
