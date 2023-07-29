#include <SoftwareSerial.h>    //Include the library for bluetooth serial communication

//Decleare the two resistor value for the voltage divider to detect input battery voltage
#define R1 100000.0    //100k
#define R2 47000.0     //47k

//Pins for the HC05
#define RX 11
#define TX 12

char i;  //A character type global variable to store the incoming serial data from HC05 module.

//motor 1,3 control pins decleration
int enA = 10;
const int inPut1 = 2;
const int inPut2 = 3;

//motor 2,4 control pins decleration
int enB = 9;
const int inPut3 = 4;
const int inPut4 = 5;

//pin for buzzer
const int buzz = 13;

//pin for LED light
const int led = 7;

//Pin to detect the state of the bluetooth module
const int bt_st = 8;

//Pin to detect the output of the voltage divider
const int batt = A0;
float Vin;    //Variable to store battery voltage
long previousMillis = -1000 * 10;  // -1000*10=-10sec. To read the first value. If you use 0 then you will take the first value after 10sec.
long interval = 1000 * 10;         // Interval at which to read battery voltage, change it if you want! (10*1000=10sec)
unsigned long currentMillis;       //Insigned long type variable to store the current time

int vSpeed = 200;

SoftwareSerial BT(RX, TX);  // RX, TX of HC-05

float batt_volt_read() {
  currentMillis = millis();
  if (currentMillis - (previousMillis) > (interval)) {
    previousMillis = currentMillis;
    float volt = 0.0;
    //Calculate the input battery voltage
    int adc_val = analogRead(batt);
    volt = adc_val * (5.0 / 1024.0);
    Vin = volt / float(R2 / (R1 + R2));
  }
  return Vin;
}

//user defined function one named halt() to stop the bot
void halt() {
  analogWrite(enA, 0);
  analogWrite(enB, 0);

  digitalWrite(inPut1, LOW);
  digitalWrite(inPut2, LOW);

  digitalWrite(inPut3, LOW);
  digitalWrite(inPut4, LOW);
}

//user defined function four named left() to turn the bot to left direction
void forward() {
  analogWrite(enA, vSpeed);
  digitalWrite(inPut1, LOW);
  digitalWrite(inPut2, HIGH);


  analogWrite(enB, vSpeed);
  digitalWrite(inPut3, HIGH);
  digitalWrite(inPut4, LOW);
}

//user defined function five named right() to turn the bot to right direction
void back() {
  analogWrite(enA, vSpeed);
  digitalWrite(inPut1, HIGH);
  digitalWrite(inPut2, LOW);



  analogWrite(enB, vSpeed);
  digitalWrite(inPut3, LOW);
  digitalWrite(inPut4, HIGH);
}

//user defined function three named back() to move the bot to backward direction
void left() {
  analogWrite(enA, vSpeed);
  digitalWrite(inPut1, LOW);
  digitalWrite(inPut2, HIGH);


  analogWrite(enB, vSpeed);
  digitalWrite(inPut3, LOW);
  digitalWrite(inPut4, HIGH);
}

//user defined function two named forward() to move the bot to fornt direction
void right() {
  analogWrite(enA, vSpeed);
  digitalWrite(inPut1, HIGH);
  digitalWrite(inPut2, LOW);


  analogWrite(enB, vSpeed);
  digitalWrite(inPut3, HIGH);
  digitalWrite(inPut4, LOW);
}

void blink() {
  digitalWrite(led, LOW);
  delay(500);
  digitalWrite(led, HIGH);
  delay(500);
  digitalWrite(led, LOW);
  delay(500);
}

//--------------------------------------SETUP----------------------------------------//

void setup() {
  Serial.begin(9600);
  BT.begin(9600);
  //configure all the decleared pins as output
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(inPut1, OUTPUT);
  pinMode(inPut2, OUTPUT);
  pinMode(inPut3, OUTPUT);
  pinMode(inPut4, OUTPUT);
  pinMode(led, OUTPUT);
  pinMode(buzz, OUTPUT);
  pinMode(bt_st, INPUT);
  //arduino built in library function to silence the buzzer
  noTone(buzz);
}

//--------------------------------------LOOP-----------------------------------------//

//main function which runs over and over
void loop() {
  //Read the input voltage
  Vin = batt_volt_read();
  //Serial.println(Vin);

  //If the battery voltage is in between 10.75V to 12.75V
  if (Vin > 10.75 && Vin < 12.75) {
    digitalWrite(buzz, LOW);

    //Check weather the bluetooth module is connected or not
    if (digitalRead(bt_st) == HIGH) {
      //The "Serial.available( )" function in Arduino gets the stored bytes from the serial port that are available for reading.
      //It is the data, which is already stored and arrived in the serial buffer.
      if (BT.available())  //If there is serial data available in the buffer
      {
        i = BT.read();  //Store the received serial data into the variable named "i"
        //Serial.println(i);

        halt();  //Stop firstly

        //Convert the speed slider value into actual PWM value viz., 0 - 250
        if (i == '0') {
          vSpeed = 0;
        } else if (i == '1') {
          vSpeed = 120;
        } else if (i == '2') {
          vSpeed = 160;
        } else if (i == '3') {
          vSpeed = 200;
        } else if (i == '4') {
          vSpeed = 250;
        }
        //Constructing a switch-case
        switch (i) {
          case 'F':
            forward();  //Go forward

            break;

          case 'B':
            back();  //go backward

            break;

          case 'L':  //Go left
            left();

            break;

          case 'R':
            right();  //Go right

            break;

          case 'V':
            tone(buzz, 3000);  //Make sound through buzzer with 1.5 kHz tone
            break;

          case 'v':
            noTone(buzz);  //Silence the buzzer

            break;

          case 'W':
            digitalWrite(led, HIGH);  //Front led on

            break;

          case 'w':
            digitalWrite(led, LOW);  //Front led off

            break;
        }
      }
    } else {
      blink();    //Blink LED if HC05 is not connected to any network
      halt();    //Stops the robot immedietly after HC05 lost connection
    }
  }

  else {
    digitalWrite(buzz, HIGH);    //If the battery voltage in outside the safe limit, buzzer turns ON
    halt();    //Stops the robot immedietly
  }
}
