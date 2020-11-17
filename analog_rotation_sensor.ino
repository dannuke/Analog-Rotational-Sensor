//notes by Dan Davis
// introducing the const int code: declaration of a constant
//equipment: Analog Rotation Sensor DFR 0058, Digital LED, Arduino, IDE serial monitor. Breadboard optional
//direct connect to the Arduino
//reference https://wiki.dfrobot.com/Analog_Rotation_Sensor_V2__SKU__DFR0058_
//connect the rotational sensor leads to 5 volts, GND, and signal pin to analaog A0 on the Arduino 
//connect the digital LED to 3.3 volts, GND,  and output to pin ~9
//by rotating the  potentiometer, the LED will fade or get bright. Also, see the affect using the serial monitor
// if a breadboard is used, connect the 5 volts and GND , for the sensor,  with leads to the breadboard rail, and the 3.3 volts  and GND for the LED to the otherside rails of the breaboard.

const int analogInPin = A0;  // Analog input pin that the potentiometer is attached to
const int analogOutPin = 9; // Analog output pin that the LED is attached to

int sensorValue = 0;        // value read from the pot
int outputValue = 0;        // value output to the PWM (analog out)

void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);
}

void loop() {
  // read the analog in value:
  sensorValue = analogRead(analogInPin);
  // map it to the range of the analog out:
  outputValue = map(sensorValue, 0, 1023, 0, 255);
  // change the analog out value:
  analogWrite(analogOutPin, outputValue);

  // print the results to the serial monitor:
  Serial.print("sensor = ");
  Serial.print(sensorValue);
  Serial.print("\t output = ");
  Serial.println(outputValue);

  // wait 2 milliseconds before the next loop
  // for the analog-to-digital converter to settle
  // after the last reading:
  delay(20);
}
