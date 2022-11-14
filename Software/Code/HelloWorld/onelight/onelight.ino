// Maggie Killmeyer
// Veronica Bella
=======
//Raheel Farouk

void setup() {
  // put your setup code here, to run once:
  pinMode(6, OUTPUT);
  pinMode(9, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(digitalRead(9)==HIGH){
    digitalWrite(6,HIGH);
    delay(500);
    digitalWrite(6,LOW);
    delay(500);
    
    
  }else{
    digitalWrite(6, LOW);
  }
  
  

}
