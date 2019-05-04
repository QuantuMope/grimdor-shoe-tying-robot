// Include all necessary libraries for Adafruit Motorshield v2.3 and ULN2003.
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Stepper.h>

// Create motorshield object.
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Create and define NEMA17 motor object.
Adafruit_StepperMotor *NEMA17 = AFMS.getStepper(200, 2);

// Create and define 28BYJ-48 motor object.
const int stepsPerRevolution = 2048;
Stepper s28BYJ48 = Stepper(stepsPerRevolution,8,10,9,11);

// Define Arduino button pins and boolean states.
int i = 0;
int j = 0;
int p = 0;
int t1 = 0;
int t2 = 0;
int firstjump = 0;
int firstadjust = 0;
int firstlimitadjust = 0;
int firstINjumpback = 0;
int firstOUTjumpback = 0;
int secondjump = 0;
int secondlimitadjust = 0;
int secondINjumpback = 0;
int secondOUTjumpback = 0;
int JUMP = 0;
char Direction = FORWARD;
const int limitINPin = 4;
const int limitOUTPin = 6;
const int buttonPin = 2;
const int redledPin =  12;
const int greenledPin = 13;
boolean lastButton = LOW;
boolean currentButton = LOW;
boolean ledOn = false;

void setup() {
  Serial.begin(9600);

  // Connect to Adafruit Motorshield to Arduino.
  AFMS.begin();

  // Boost I2C rate to 400kHz for faster speed.
  Wire.begin();
  Wire.setClock(400000);

  // Set speeds (rpm) for both motors.
  NEMA17->setSpeed(1);
  s28BYJ48.setSpeed(14);

  // Define output and inputs.
  pinMode(redledPin, OUTPUT);
  pinMode(greenledPin, OUTPUT);
  pinMode(limitINPin, INPUT);
  pinMode(limitOUTPin, INPUT);
  pinMode(buttonPin, INPUT);
}

// Button debounce function for proper pushbutton read.
boolean debounce(boolean last)
{
  boolean current = digitalRead(buttonPin);
  if (last != current)
  {
    delay(5);
    current = digitalRead(buttonPin);
  }
  return current;
}

// Clutch function. Input is either 13,31,12,23,21,32. First number is starting shaft, second number is destinatinon shaft. 1 being closest to the shoe.
// Input 130 exists for a specific part of the shoe tying algorithm in which the drive shaft must microstep the opposite direction.
int clutch(int j)
{
  int clutchstep;

  // Number of steps to jump between gears ending right before gear mesh begins.
  JUMP = 825;

  Direction = FORWARD;

  // Step numbers for each clutch case
  if (j == 31){
    clutchstep = 4;
    firstjump = JUMP;
    firstadjust = 100;
    firstlimitadjust = 0;
    firstINjumpback = 0;
    firstOUTjumpback = 0;
    p = 25;
    t1 = 0;
    t2 = 5000;
    secondjump = JUMP;
    secondlimitadjust = 1;
    secondINjumpback = -50;
    secondOUTjumpback = 0;
  }
  else if (j == 13){
    clutchstep = -4;
    firstjump = -JUMP;
    firstadjust = -100;
    firstlimitadjust = 0;
    firstINjumpback = 0;
    firstOUTjumpback = 0;
    p = 25;
    t1 = 0;
    t2 = 5000;
    secondjump = -JUMP;
    secondlimitadjust = -1;
    secondINjumpback = 0;
    secondOUTjumpback = 80;
  }
  else if (j == 32){
    clutchstep = 4;
    firstjump = JUMP;
    firstadjust = 100;
    firstlimitadjust = 0;
    firstINjumpback = 0;
    firstOUTjumpback = 0;
    p = 0;
    t1 = 0;
    t2 = 0;
    secondjump = 0;
    secondlimitadjust = 0;
    secondINjumpback = 0;
    secondOUTjumpback = 0;
  }
  else if (j == 21){
    clutchstep = 4;
    firstjump = JUMP;
    firstadjust = 0;
    firstlimitadjust = 1;
    firstINjumpback = -50;
    firstOUTjumpback = 0;
    p = 0;
    t1 = 5000;
    t2 = 0;
    secondjump = 0;
    secondlimitadjust = 0;
    secondINjumpback = 0;
    secondOUTjumpback = 0;
  }
  else if (j == 12){
    clutchstep = -4;
    firstjump = -JUMP;
    firstadjust = -100;
    firstlimitadjust = 0;
    firstINjumpback = 0;
    firstOUTjumpback = 0;
    p = 0;
    t1 = 0;
    t2 = 0;
    secondjump = 0;
    secondlimitadjust = 0;
    secondINjumpback = 0;
    secondOUTjumpback = 0;
  }
  else if (j == 23){
    clutchstep = -4;
    firstjump = -JUMP;
    firstadjust = 0;
    firstlimitadjust = -1;
    firstINjumpback = 0;
    firstOUTjumpback = 80;
    p = 0;
    t1 = 5000;
    t2 = 0;
    secondjump = 0;
    secondlimitadjust = 0;
    secondINjumpback = 0;
    secondOUTjumpback = 0;
  }
  else if (j == 130){
    clutchstep = -4;
    firstjump = -JUMP;
    firstadjust = -100;
    firstlimitadjust = 0;
    firstINjumpback = 0;
    firstOUTjumpback = 0;
    p = 25;
    t1 = 0;
    t2 = 5000;
    secondjump = -JUMP;
    secondlimitadjust = -1;
    secondINjumpback = 0;
    secondOUTjumpback = 80;
    Direction = BACKWARD;
  }
    // First initial Jump
    NEMA17->setSpeed(5);
    s28BYJ48.step(firstjump);
    // Dual motor movement to prevent gear catching
    while(i < (25))
    {
    NEMA17->step(2, FORWARD, MICROSTEP);
    s28BYJ48.step(clutchstep);
    i++;
    }
    i=0;
    // Adjust for proper realignment after passing through gear
    s28BYJ48.step(firstadjust);
    // Infinite clutch movement until limit switch activation
    while(i < (t1))
    {
    s28BYJ48.step(firstlimitadjust);
    i++;
    if (digitalRead(limitINPin) == 0){
      i = 5000;
    }
    if (digitalRead(limitOUTPin) == 0){
      i = 5000;
    }
    }
    i=0;
    // Jump back values after limitswitch is hit
    s28BYJ48.step(firstINjumpback);
    s28BYJ48.step(firstOUTjumpback);

    // Another iteration to move into second gear position for clutch(13) and clutch(31). If not, this part is disabled because p = 0.
    s28BYJ48.step(secondjump);
    while(i < (p))
    {
    // Dual motor movement during clutch action to prevent gear catching
    NEMA17->step(2, Direction, MICROSTEP);
    s28BYJ48.step(clutchstep);
    i++;
    }
    i=0;
    // Infinite clutch movement until limit switch activation
    while(i < (t2))
    {
    s28BYJ48.step(secondlimitadjust);
    i++;
    if (digitalRead(limitINPin) == 0){
      i = 5000;
    }
    if (digitalRead(limitOUTPin) == 0){
      i = 5000;
    }
    }
    i=0;
    // Jumpback values after limitswitch is hit
    s28BYJ48.step(secondINjumpback);
    s28BYJ48.step(secondOUTjumpback);
}

void loop() {
  currentButton = debounce(lastButton);

  // Logic statement analyzing button states.
  if (lastButton == LOW && currentButton == HIGH)
  {
    ledOn = !ledOn;
  }
  lastButton = currentButton;

  // During standby, only green LED is on. When button is pressed, red LED turns on and green LED turns off.
  digitalWrite(redledPin,ledOn);
  digitalWrite(greenledPin,!ledOn);

  // Logic statement that engages if button has been pressed.

  // Y-AXIS 7 INCHES ---> 5650 STEPS FORWARD IS UP!
  // 90° ROTATE CW   ---> 1500 STEPS FORWARD DOUBLE @ 240RPM    FROM FRONTSIDE     BACKWARDS - CLOCKWISE   FORWARDS - CCW
  // X-AXIS          ---> use 75RPM (safe not too fast but not too slow) SINGLE is fine    - IS LEFT TO RIGHT
  // CLUTCH          ---> + is going in - is going out
  Serial.print(digitalRead(limitOUTPin));
  if (ledOn == 1)
  {
    // SHOE TYING ALGORITHM

    NEMA17->setSpeed(40);
    NEMA17->step(160,BACKWARD,DOUBLE);  // X-axis move out: Grab laces
    clutch(32);
    NEMA17->setSpeed(150);
    NEMA17->step(6800,FORWARD,DOUBLE);  // Y-axis Move up: Move up past shoe
    clutch(21);
    NEMA17->setSpeed(120);
    NEMA17->step(1400,FORWARD,DOUBLE);  // 90 degrees rotate: Have hooks facing up
    clutch(13);
    NEMA17->setSpeed(150);
    NEMA17->step(260,FORWARD,DOUBLE);   // X-axis move in: Move in until laces are past hooks
    clutch(31);
    NEMA17->setSpeed(150);
    NEMA17->step(3090,FORWARD,DOUBLE);  // 225 degrees rotate: Rotate so that hooks can grab laces
    clutch(13);
    NEMA17->setSpeed(40);
    NEMA17->step(110,BACKWARD,DOUBLE);  // X-axis move out: Pull out until laces are engaged in the hooks
    clutch(31);
    NEMA17->setSpeed(120);
    NEMA17->step(670,BACKWARD,DOUBLE);   // 45 Degrees rotate: Rotate down so that hooks are pointing down to avoid collision
    clutch(130);                         // Clutch: 1 to 3 SPECIAL CASE
    NEMA17->setSpeed(30);
    NEMA17->step(27,BACKWARD,DOUBLE);    // X-axis move out: move out in 8 individual spurts seperated by 500ms
    delay(500);
    NEMA17->step(27,BACKWARD,DOUBLE);
    delay(500);
    NEMA17->step(27,BACKWARD,DOUBLE);
    delay(500);
    NEMA17->step(27,BACKWARD,DOUBLE);
    delay(500);
    NEMA17->step(27,BACKWARD,DOUBLE);
    delay(500);
    NEMA17->step(27,BACKWARD,DOUBLE);
    delay(500);
    NEMA17->step(27,BACKWARD,DOUBLE);
    delay(500);
    NEMA17->step(27,BACKWARD,DOUBLE);
    clutch(32);
    NEMA17->setSpeed(200);
    NEMA17->step(1000,BACKWARD,DOUBLE);  // Y-axis move down: move down to be more level with shoe
    clutch(21);
    NEMA17->setSpeed(120);
    NEMA17->step(1370,FORWARD,DOUBLE);   // 90 degrees rotate: Rotate so hooks are side to side
    clutch(13);
    NEMA17->setSpeed(150);               // X-axis mvoe out: move out in 3 large spurts so that laces shoot through to the opposite side
    NEMA17->step(90,BACKWARD,DOUBLE);
    delay(500);
    NEMA17->step(90,BACKWARD,DOUBLE);
    delay(500);
    NEMA17->step(120,BACKWARD,DOUBLE);
    clutch(32);
    NEMA17->setSpeed(200);
    NEMA17->step(6000,BACKWARD,DOUBLE);  // Y-axis move down:  Go down to reset

    //*** FIRST LACE IS DONE / SECOND KNOT START ***

    clutch(23);
    NEMA17->setSpeed(40);
    NEMA17->step(215,FORWARD,DOUBLE);   // X-axis move in : reset
    clutch(32);
    NEMA17->setSpeed(150);
    NEMA17->step(6650,FORWARD,DOUBLE);  // Y-axis Move up: Move up past shoe
    clutch(21);
    NEMA17->setSpeed(120);
    NEMA17->step(1175,FORWARD,DOUBLE);  // 90 degrees rotate: Have hooks facing up
    clutch(13);
    NEMA17->setSpeed(150);
    NEMA17->step(255,FORWARD,DOUBLE);   // X-axis move in:  Move in until laces are past hooks
    clutch(31);
    NEMA17->setSpeed(120);
    NEMA17->step(3000,FORWARD,DOUBLE);  // 225 degrees rotate:   Rotate so that hooks can grab laces
    clutch(13);
    NEMA17->setSpeed(40);
    NEMA17->step(95,BACKWARD,DOUBLE);   // X-axis move out just in case:   Move out until hooks catch laces
    clutch(31);
    NEMA17->setSpeed(120);
    NEMA17->step(575,BACKWARD,DOUBLE);   //45 Degrees rotate: rotate so that hooks are facing down to avoid collision
    clutch(130);                         // Clutch: 1 to 3 SPECIAL CASE
    NEMA17->setSpeed(30);
    NEMA17->step(26,BACKWARD,DOUBLE);    // X-axis move out: move out in 3 spurts
    delay(500);
    NEMA17->step(26,BACKWARD,DOUBLE);
    delay(500);
    NEMA17->step(26,BACKWARD,DOUBLE);
    clutch(32);
    NEMA17->setSpeed(200);
    NEMA17->step(1275,BACKWARD,DOUBLE);  // Y-axis Move down to level with shoe
    clutch(23);
    NEMA17->setSpeed(40);
    NEMA17->step(290 ,BACKWARD,DOUBLE);   // X-axis move out:  Pull until Shoeknot is done

    // Reverts boolean variables back to standby values.
    ledOn = 0;
    lastButton = LOW;
    i=0;
  }
}
