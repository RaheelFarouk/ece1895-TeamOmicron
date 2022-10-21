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

// // action names
// #define TWIST_IT (0)
// #define PUSH_IT (1)
// #define SHAKE_IT (2)

// libraries
#include <SPI.h>  // for SD card
#include <SD.h>   // for SD card

const int delay_time = 5;   // time between games of bop it after win or loss
const int max_start_time = 5;  // most time allowed for successful action
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

  // if(play_game()){
  //   winner();
  // } else{
  //   loser();
  // }

  // delay(delay_time*1000);
  play_game();
}

/**
  Turns on or off all leds
  @param mode can be either HIGH (on) or LOW (off)
*/
void set_leds(int mode){
  digitalWrite(LED_0_PIN, mode);
  digitalWrite(LED_1_PIN, mode);
  digitalWrite(LED_2_PIN, mode);
}

/**
  Sets up SD card and anything needed for audio
*/
void setup_audio(){
  SD.begin();
  SD.open("", FILE_READ);
}



/**
  Verifies that correct action was completed in time, no penalty for wrong input
  @param action_pin pin to check for correct action
  @param led_pin led pin corresponding to action to be turned off if correct action done
  @param max_time time in seconds to complete action
  @return true if the correct action was completed in time
*/
bool verify_action(int actionPin, int ledPin, int maxTime){
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

*/
bool verify_encoder(int maxTime){
  // int a = digitalRead(ENCODER_A_PIN);
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
     if (abs(counter) >= 10000) return true;
   }    
  }

  return false;
}

/**
  Basic game function, player gets action and must follow action
  @return true if player reaches 100 actions
*/
bool play_game(){
  int count = 0;
  int max_time = max_start_time;

  while (count <= 99){
    //int action = random(3);
    int action = TWIST_IT;
    Serial.print(count);
    switch (action){
      case TWIST_IT:
        aEncoderLastState = digitalRead(ENCODER_A_PIN);
        if (!verify_encoder(max_time*1000)){
          Serial.println(": FAILED");
          return false;
        } else{
          Serial.println(": TWISTED");
        }
      break;

      case PUSH_IT:
        if (!verify_action(SLIDER_PIN, LED_1_PIN, max_time*1000)) return false;
      break;

      case SHAKE_IT:
        //if (!verify_action(ACCEL_PIN, LED_2_PIN, max_time*1000)) return false;
      break;
    }

    count++;
    max_time = max_time / 1.025;    // arbitrary value to speed up
    delay(500);
  }

  return true;
}

/**
  Whatever happens when the player wins
*/
void winner(){
  set_leds(HIGH);

  // play winner sound

  set_leds(LOW);
}

/**
  Whatever happens when the player loses
*/
void loser(){
  set_leds(LOW);

  // play loser sound
}
