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
const int maxStartTime = 8;  // most time allowed for successful action in seconds

const int TWIST_IT = 0;
const int PUSH_IT = 1;
const int SHAKE_IT = 2;

int aEncoderState;
int aEncoderLastState;
int selector;
int score;

DFRobot_RGBLCD1602 lcd(16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display
Adafruit_MPU6050 mpu;

SoftwareSerial mySoftwareSerial(SoftwareSerialRX, SoftwareSerialTX); // RX, TX
DFRobotDFPlayerMini myDFPlayer;

void setup() {
  Serial.begin(9600);

  pinMode(GYRO_SCL_PIN, INPUT);
  pinMode(GYRO_SDA_PIN, INPUT);

  pinMode(SLIDER_PIN, INPUT);
  pinMode(ENCODER_A_PIN, INPUT);
  pinMode(ENCODER_B_PIN, INPUT);

  pinMode(LED_0_PIN, OUTPUT);
  pinMode(LED_1_PIN, OUTPUT);
  pinMode(LED_2_PIN, OUTPUT);

  pinMode(LCD_SCL_PIN, OUTPUT);
  pinMode(LCD_SDA_PIN, OUTPUT);

  lcd.init();
  lcd.setBacklight(false);
  lcd.setRGB(245,120,66);


  //Setting up the Gyro?/Accel
  // Try to initialize MPU6050!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(30);
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
  
  myDFPlayer.volume(28);  //Set volume value. From 0 to 30
  delay(200);
  myDFPlayer.play(1);
  delay(20);

  lcd.print("Welcome to TOAST MASTER 1895");
  delay(5000);
  lcd.clear();
  delay(200);

  // myDFPlayer.play(1);
  // delay(10000);
  // myDFPlayer.play(2);
  // delay(10000);
  // myDFPlayer.play(3);
  // delay(10000);
  // myDFPlayer.play(4);
  // delay(10000);
  // myDFPlayer.play(5);
  // delay(10000);
  // myDFPlayer.play(6);
  // delay(10000);
  // myDFPlayer.play(7);
  // delay(10000);
  // myDFPlayer.play(8);
  // delay(10000);
  // myDFPlayer.play(9);
  // delay(10000);
  // myDFPlayer.play(10);
  // delay(10000);
  

  //get encoder last state
  aEncoderLastState = digitalRead(ENCODER_A_PIN);

}


void loop() {
  // Menu instructions for entering Mode
  lcd.print("Turn knob to");
  lcd.setCursor(0, 2);
  lcd.print("choose Mode");
  delay(3*1000);
  lcd.clear();
  lcd.print("To select Mode");
  lcd.setCursor(0, 2);
  lcd.print("push toast down");
  delay(3*1000);

  int choice = menu();

  switch (choice){
    case 0:
      lcd.clear();
      lcd.print("PLAY GAME MODE");
      lcd.setCursor(0, 2);
      lcd.print("ACTIVATED");
      delay(2000);

      if(playGame(9, 6, 8)){
        winner();
      } else{
        loser();
      }
      break;

    case 1:
      lcd.clear();
      lcd.print("CHAOS MODE ACTIVATED");
      delay(2000);
      lcd.clear();
      delay(2000);

      // all posible audio permutations, this is less neat that a shuffle array but with 
      // ATMEGA chip limitations the randomizer should be better
      int list[][3] = {{9, 6, 8}, {9, 8, 6},
                      {6, 8, 9}, {6, 9, 8},
                      {8, 9, 6}, {8, 6, 9}};

      int audioCues = random(6);
      if(playGame(list[audioCues][0], list[audioCues][1], list[audioCues][2])){
        winner();
      } else{
        loser();
      }
      break;

    case 2:
      lcd.clear();
      lcd.print("TUTORIAL MODE ACTIVATED");
      myDFPlayer.play(11);
      delay(2000);
      break;    
  }

  delay(5000);
  lcd.clear();

}

/**
  Verifies that the slider was pushed to desired amount within the alloted time frame
  @param maxTime maximum ammount of time to complete twist it action
  @return true if the push it action was completed in time
*/
bool verifySlider(int maxTime){
  unsigned long startTime = millis();
  int startPos = analogRead(SLIDER_PIN);
  int counter = 0;

  while (millis() - startTime < maxTime){
    // check other action inputs, for false positives
    
    aEncoderState = digitalRead(ENCODER_A_PIN);
    if (aEncoderState != aEncoderLastState){     
     if (digitalRead(ENCODER_B_PIN) != aEncoderState) { 
       counter ++;
     } else {
       counter --;
     }

     if (abs(counter) >= 10){
       return false; // Threshold for twist it command
       lcd.clear();  
     }
    }

    // verify no gyro
    /* Get new sensor events with the readings */
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    if((a.acceleration.z)<-25.0){ //-10.0 is the acceleration threshold
      return false;
    }

    int sliderPos = abs(analogRead(SLIDER_PIN) - startPos);
    if (sliderPos >= 500){
      return true;
    } else if(sliderPos >= 275){
      myDFPlayer.play(7);
      delay(25);
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
    
    if (abs(analogRead(SLIDER_PIN) - startPos) >= 200) return false;  
    startPos = analogRead(SLIDER_PIN);

    // verify no gyro
    /* Get new sensor events with the readings */
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    if((a.acceleration.z)<-25.0){ //-10.0 is the acceleration threshold
      return false;
    }

    aEncoderState = digitalRead(ENCODER_A_PIN);
    if (aEncoderState != aEncoderLastState){     
     if (digitalRead(ENCODER_B_PIN) != aEncoderState) { 
       counter ++;
       myDFPlayer.play(5);
     } else {
       counter --;
     }
     if (abs(counter) >= 5){
       return true; // Threshold for twist it command
       lcd.clear();  
     } 
   }    
  }

  return false;
}

bool verifyAccel(int maxTime){
  unsigned long startTime = millis();
  int counter = 0;
  int startPos = analogRead(SLIDER_PIN);

  while(millis() - startTime < maxTime){
    
    //lcd.print(a.acceleration.z);
    // check other action inputs, for false positives
    if (abs(analogRead(SLIDER_PIN) - startPos) >= 200){
      return false; 
    }  
    startPos = analogRead(SLIDER_PIN);

    //check encoder input
    aEncoderState = digitalRead(ENCODER_A_PIN);
    if (aEncoderState != aEncoderLastState){     
      if (digitalRead(ENCODER_B_PIN) != aEncoderState) { 
        counter ++;
      } else {
        counter --;
      }
      //Serial.print(counter);
      if (abs(counter) >= 15){
        return false; // Threshold for twist it command
        // lcd.print("Fail on encoder");
        // delay(1000);
        lcd.clear();  
      } 
    }

    /* Get new sensor events with the readings */
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    if(abs((g.gyro.z+g.gyro.y+g.gyro.x)/3)>3){
      if((a.acceleration.z)<-18.0){ //-10.0 is the acceleration threshold
        //delay(100);
        lcd.clear();
        return true;
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
  score = 0;
  int maxTime = maxStartTime;

  myDFPlayer.play(3); // does player pause code?????
  delay(20);
  delay(6000);

  // while loop to control the number of needed successful actions
  while (score <= 99){
    
    int action = random(3);
    lcd.clear();

    // switch case that gets a command number from random generator to verify the selected action
    switch (action){
      case TWIST_IT:
        lcd.print("TWIST IT!");
        myDFPlayer.play(9); //enter track number in brackets
        delay(50);       

        aEncoderLastState = digitalRead(ENCODER_A_PIN);
        if (!verifyEncoder(maxTime*1000)){
          delay(150);
          return false;
        }
        delay(150);
        lcd.clear();
      break;

      case PUSH_IT:
        lcd.print("PUSH IT!");
        myDFPlayer.play(6);
        delay(50);
        
        if (!verifySlider(maxTime*1000)){
          delay(150);
          lcd.clear();
          return false;
        }
        delay(150);
        lcd.clear();
      break;

      case SHAKE_IT:
        lcd.print("SHAKE IT!");
        myDFPlayer.play(8);
        delay(50);
        if (!verifyAccel(maxTime*1000)){
          delay(150);
          lcd.clear();
          return false;
        }
        delay(20);
        lcd.clear();
      break;
    }

    score++;
    myDFPlayer.play(2);
    delay(25);
    lcd.clear();
    lcd.print("SCORE: " + (String)score);
    maxTime = maxTime-0.06;         // arbitrary value to speed up
    delay(500);                     // wait half a second in between giving commands
  }

  return true;
}

/**
  Display message to player when player wins and 99 commands have been completed
*/
void winner(){

  lcd.clear();
  lcd.print("You Win");
  delay(3000);
  lcd.clear();
  lcd.print("Toast Master");
  lcd.setCursor(0, 2);
  lcd.print("DEFEATED!!");
  delay(3000);

}

/**
  WDisplays message and final score and plays audio clip when the player fails to complete 99 command successfully 
*/
void loser(){
  lcd.clear();
  lcd.print("You Lose");
  lcd.setCursor(0, 2);
  lcd.print("FINAL SCORE: " + (String)score);
  myDFPlayer.play(4);
  delay(1000);
}

/**
  Encoder for menu needs to have more precision to reliably select game mode so this function helps with debouncing
*/
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
		} else {
			// Encoder is rotating CW so increment
			selector ++;
		}
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
  
  int startPos = analogRead(SLIDER_PIN);
  int oldValue;

  while (analogRead(SLIDER_PIN) < 500){

    menuEncoder();
    
    // if statements for making wrap around menu, user can never go out of bounds for the menu
    if (selector < 0){
      selector = 2;
    } else if (selector > 2){
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
