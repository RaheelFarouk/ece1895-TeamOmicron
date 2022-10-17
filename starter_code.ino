// ECE 1895 - Design Project 2 - Team Omicron
// Veronica Bella, Raheel Farouk, Maggie Killmuer

// Macros for pin numbers
// input pins
#define GYRO_PIN 
#define ACEL_PIN
#define SLIDER_PIN
#define ENCODER_PIN

// output pins
#define LED1_PIN
#define LED2_PIN
#define LED3_PIN
#define LCD_PIN
#define SPEAKER_PIN

// action names
#define TWIST_IT 0
#define PUSH_IT 1
#define SHAKE_IT 2

// libraries
#include <SPI.h>  // for SD card
#include <SD.h>   // for SD card

const int delay_time = 5;   // time between games of bop it after win or loss
const int max_start_time = 10;  // most time allowed for successful action


/**
  Set up all digital and analog pins for input
*/
void setup_inputs(){
  pinMode(GRYO_PIN, INPUT);
  pinMode(ACEL_PIN, INPUT);

  pinMode(SLIDER_PIN, INPUT);
  pinMode(ENCODER_PIN, INPUT);
}

/**
  Set up all led, screen, and speaker pins for output
*/
void setup_outputs(){
  pinMode(LED0_PIN, OUTPUT);
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);

  pinMode(LCD_PIN, OUTPUT);

  pinMode(SPEAKER_PIN, OUTPUT);
}

/**
  Turns on or off all leds
  @param mode can be either HIGH (on) or LOW (off)
*/
void set_leds(int mode){
  digitalWrite(LED0_PIN, mode);
  digitalWrite(LED1_PIN, mode);
  digitalWrite(LED2_PIN, mode);
}

/**
  Sets up SD card and anything needed for audio
*/
void setup_audio(){
  SD.begin();
  SD.open("", FILE_READ);
}

/**
  Calls functions to set up all I/O
*/
void setup() {
  setup_inputs();
  setup_outputs();

  setup_audio();
}

/**
  Set up all digital and analog pins for input
*/
void setup_inputs(){
  pinMode(GRYO_PIN, INPUT);
  pinMode(ACEL_PIN, INPUT);

  pinMode(SLIDER_PIN, INPUT);
  pinMode(ENCODER_PIN, INPUT);
}

/**
  Set up all led, screen, and speaker pins for output
*/
void setup_outputs(){
  pinMode(LED0_PIN, OUTPUT);
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);

  pinMode(LCD_PIN, OUTPUT);

  pinMode(SPEAKER_PIN, OUTPUT);
}

/**
  Turns on or off all leds
  @param mode can be either HIGH (on) or LOW (off)
  @return
*/
void set_leds(int mode){
  digitalWrite(LED0_PIN, mode);
  digitalWrite(LED1_PIN, mode);
  digitalWrite(LED2_PIN, mode);
}

/**
  Sets up SD card and anything needed for audio
  @return
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
bool verify_action(int action_pin, int led_pin, int max_time){
  digitalWrite(led_pin, HIGH);
  unsigned long start_time = millis();

  while(millis() - start_time < max_time){
    if(digitalRead(action_pin)){
      digitalWrite(led_pin, LOW);
      return true;
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
    int action = random(3);

    switch (action){
      case TWIST_IT:
        if (!verify_action(ENCODER_PIN, LED0_PIN, max_time*1000)) return false;
      break;

      case PUSH_IT:
        if (!verify_action(SLIDER_PIN, LED1_PIN, max_time*1000)) return false;
      break;

      case SHAKE_IT:
        if (!verify_action(ACCEL_PIN, LED2_PIN, max_time*1000)) return false;
      break;
    }

    count++;
    max_time = max_time / 1.025;    // arbitrary value to speed up
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

/**
  main loop
*/
void loop() {

  if(play_game()){
    winner();
  } else{
    loser();
  }

  delay(delay_time*1000);
}
