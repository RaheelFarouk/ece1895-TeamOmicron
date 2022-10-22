// ECE 1895 - Design Project 2 - Team Omicron
// Veronica Bella, Raheel Farouk, Maggie Killmuer

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
#define SPEAKER_PIN 9

// libraries
#include <SPI.h>  // for SD card
#include <SD.h>   // for SD card

const int delayTime = 5;   // time between games of bop it after win or loss
const int maxStartTime = 5;  // most time allowed for successful action

const int TWIST_IT = 0;
const int PUSH_IT = 1;
const int SHAKE_IT = 2;

int aEncoderState;
int aEncoderLastState;

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

  pinMode(SPEAKER_PIN, OUTPUT);
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
  Verifies that correct action was completed in time, no penalty for wrong input
  @param action_pin pin to check for correct action
  @param led_pin led pin corresponding to action to be turned off if correct action done
  @param maxTime time in seconds to complete action
  @return true if the correct action was completed in time
*/
bool verifyAction(int actionPin, int ledPin, int maxTime){
  digitalWrite(ledPin, HIGH);
  unsigned long startTime = millis();

  while(millis() - startTime < maxTime){
    if(digitalRead(actionPin)){
      digitalWrite(ledPin, LOW);
      return true;
    } 
  }
  
  return false;
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

/**
  Basic game function, player gets action and must follow action
  @return true if player reaches 100 actions
*/
bool playGame(){
  int count = 0;
  int maxTime = maxStartTime;

  while (count <= 99){
    //int action = random(3);
    int action = PUSH_IT;
    //Serial.print(count);
    switch (action){
      case TWIST_IT:
        aEncoderLastState = digitalRead(ENCODER_A_PIN);
        if (!verifyEncoder(maxTime*1000)){
          Serial.println("FAILED TWISTED");
          return false;
        } else{
          Serial.println("TWISTED");
        }
      break;

      case PUSH_IT:
        if (!verifySlider(maxTime*1000)){
          Serial.println("FAILED PUSH");
        } else{
          Serial.println("PUSHED");
        }
      break;

      case SHAKE_IT:
        //if (!verifyAction(ACCEL_PIN, LED_2_PIN, maxTime*1000)) return false;
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
