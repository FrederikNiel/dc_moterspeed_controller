void encoder() {
  // Quad sensor uses 4 pins, GND, 3.3V, 2 Bool datapins
  // datapins are 4,5
  bool old1 = 0, old2 = 0;
#define sensorinput1 4
#define sensorinput2 5

  int i = 0;
  int counter = 0;
  Serial.print("Encoder Included!");
  pinMode(4, INPUT_PULLUP); //Setting D4 as a interrupt pin
  attachInterrupt(digitalPinToInterrupt(4), directionprog, CHANGE); //If changes registred at D4, then run prog "directionprog"

}
//interrupt service routines (ISRs)


void directionprog() // For reading motor information
{
    Serial.println("Encoder running!");
    bool dataread1 = digitalRead(sensorinput1);
    if (old1 != dataread1)
    {
      if (dataread1 == digitalRead(sensorinput2))
      {
        counter++;
      }
      else
      {
        counter--;
      }
    }
    Serial.println(counter);

}
