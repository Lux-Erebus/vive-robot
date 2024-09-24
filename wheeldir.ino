void setup() {
  // put your setup code here, to run once:
Serial.begin(115200);
pinMode(19,OUTPUT);
pinMode(23,OUTPUT);
pinMode(18,OUTPUT);
pinMode(10,OUTPUT);
ledcSetup(1,10000,12);
ledcAttachPin(9,1); //pin 9 for speed control
float duty=0.8;
ledcWrite(1,duty*4096);
digitalWrite(19,LOW);
  digitalWrite(23,HIGH);
  digitalWrite(18,HIGH);
  digitalWrite(10,LOW);
}

void loop() {
  // put your main code here, to run repeatedly:

}
