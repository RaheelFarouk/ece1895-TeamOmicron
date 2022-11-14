// Rotary Encoder Inputs
#define ENCODER_A_PIN 6
#define ENCODER_B_PIN 7
#define SW 4

int selector = 0;
int aEncoderState;
int aEncoderLastState;
String currentDir ="";

void setup() {
	
	// Set encoder pins as inputs
	pinMode(ENCODER_A_PIN,INPUT);
	pinMode(ENCODER_B_PIN,INPUT);

	// Setup Serial Monitor
	Serial.begin(9600);

	// Read the initial state of CLK
	aEncoderLastState = digitalRead(ENCODER_A_PIN);
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
			currentDir ="CCW";
		} else {
			// Encoder is rotating CW so increment
			selector ++;
			currentDir ="CW";
		}
		Serial.print("Direction: ");
		Serial.print(currentDir);
		Serial.print(" | Counter: ");
		Serial.println(selector);
	}
	// Remember last CLK state
	aEncoderLastState = aEncoderState;
	// Put in a slight delay to help debounce the reading
	delay(1);
}