int fsrPin = 0;
int fsrReading;


void setup(void) {
  Serial.begin(9600);
}

void loop(void) {
  fsrReading = analogRead(fsrPin); 
//Serial.print("Sensor value = ");
//Serial.println(fsrReading);
  float force = 15.311 * exp(0.005199 * fsrReading) * 9.8 * 0.001 - 0.15;
  //Serial.print("Force value = ");
  Serial.println(force);
  //Serial.println("--------------------");
  //delay(1000);
}
