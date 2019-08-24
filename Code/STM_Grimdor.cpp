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
int clutchstep = 0;
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

// Clutch function. Input is either 13,31,12,23,21,32.
// First number is starting shaft, second number is destinatinon shaft. 1 being closest to the shoe.
// Input 130 exists for a specific part of the shoe tying algorithm in which the drive shaft must microstep the opposite direction.
int clutch(int j)
{
  int clutchstep;

  // Number of steps to jump between gears ending right before gear mesh begins.
  JUMP = 825;

  direction = FORWARD;

  // Step numbers for each clutch case
  switch(j)
  {
    case 31:
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
      break;

    case 13:
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
      break;

    case 32:
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
      break;

    case 21:
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
      break;

    case 12:
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
      break;

    case 23:
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
      break;

    case 130:
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
      direction = BACKWARD;
      break;
  }

    // First initial Jump
    NEMA17->setSpeed(5);
    s28BYJ48.step(firstjump);

    // Dual motor "shimmy" to prevent gear catching.
    while(i < (25))
    {
      NEMA17->step(2, FORWARD, MICROSTEP);
      s28BYJ48.step(clutchstep);
      i++;
    }
    i=0;

    // Adjust for proper realignment after gear teeth mesh.
    s28BYJ48.step(firstadjust);

    // Clutch translation until limit switch activation.
    while(i < (t1))
    {
      s28BYJ48.step(firstlimitadjust);
      i++;
      if ((digitalRead(limitINPin) == 0) || (digitalRead(limitOUTPin) == 0))
      {
        i = 5000;
      }
    }
    i=0;

    // Jump back values after limitswitch is hit.
    s28BYJ48.step(firstINjumpback);
    s28BYJ48.step(firstOUTjumpback);

    // Another iteration to move into second gear position for clutch(13) and clutch(31). If not, this part is disabled because p = 0.
    s28BYJ48.step(secondjump);
    while(i < (p))
    {
      // Dual motor "shimmy" to prevent gear catching.
      NEMA17->step(2, direction, MICROSTEP);
      s28BYJ48.step(clutchstep);
      i++;
    }
    i=0;

    // Clutch translation until limit switch activation.
    while(i < (t2))
    {
      s28BYJ48.step(secondlimitadjust);
      i++;
      if ((digitalRead(limitINPin) == 0) || (digitalRead(limitOUTPin) == 0))
      {
        i = 5000;
      }
    }
    i=0;

    // Jumpback values after limitswitch is hit.
    s28BYJ48.step(secondINjumpback);
    s28BYJ48.step(secondOUTjumpback);
}

int nema_move(int speed, int steps, char direction)
{
  NEMA17->setSpeed(speed);
  NEMA17->step(steps, direction, DOUBLE);
}

void loop() {
  currentButton = debounce(lastButton);

  // Logic statement analyzing button states.
  if (lastButton == LOW && currentButton == HIGH)
  {
    ledOn = !ledOn;
  }
  lastButton = currentButton;

  // Green LED is on during standby when system is ready.
  // When button is pressed and operation beings, red LED turns on and green LED turns off.
  digitalWrite(redledPin,ledOn);
  digitalWrite(greenledPin,!ledOn);

  // Y-AXIS 7 INCHES ---> FORWARD is up. BACKWARD is down.
  // 90° ROTATE CW   ---> 1500 STEPS FORWARD DOUBLE @ 240RPM    FROM FRONTSIDE     BACKWARDS - CLOCKWISE   FORWARDS - CCW
  // X-AXIS          ---> LEFT TO RIGHT.
  // CLUTCH          ---> + is going in - is going out.

  Serial.print(digitalRead(limitOUTPin));
  if (ledOn == 1)
  {
    // SHOE TYING ALGORITHM

    // X-axis move out: Grab laces.
    nema_move(40, 160, BACKWARD);
    clutch(32);
    // Y-axis Move up: Move up past shoe.
    nema_move(150, 6800, FORWARD);
    clutch(21);
    // 90 degrees rotate: Have hooks facing up.
    nema_move(120, 1400, FORWARD);
    clutch(13);
    // X-axis move in: Move in until laces are past hooks.
    nema_move(150, 260, FORWARD);
    clutch(31);
    // 225 degrees rotate: Rotate so that hooks can grab laces.
    nema_move(150, 3090, FORWARD);
    clutch(13);
    // X-axis move out: Pull out until laces are engaged in the hooks.
    nema_move(40, 110, BACKWARD);
    clutch(31);
    // 45 Degrees rotate: Rotate down so that hooks are pointing down to avoid collision.
    nema_move(120, 670, BACKWARD);
    // Clutch: 1 to 3 SPECIAL CASE
    clutch(130);
    // X-axis move out: move out in 8 individual spurts seperated by 500ms.
    NEMA17->setSpeed(30);
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
    delay(500);
    NEMA17->step(27,BACKWARD,DOUBLE);
    clutch(32);
    // Y-axis move down: move down to be more level with shoe.
    nema_move(200, 1000, BACKWARD);
    clutch(21);
    // 90 degrees rotate: Rotate so hooks are side to side.
    nema_move(120, 1370, FORWARD);
    clutch(13);
    // X-axis mvoe out: move out in 3 large spurts so that laces shoot through to the opposite side.
    NEMA17->setSpeed(150);
    NEMA17->step(90,BACKWARD,DOUBLE);
    delay(500);
    NEMA17->step(90,BACKWARD,DOUBLE);
    delay(500);
    NEMA17->step(120,BACKWARD,DOUBLE);
    clutch(32);
    // Y-axis move down:  Go down to reset.
    nema_move(200, 6000, BACKWARD);
    clutch(23);


    //*** FIRST LACE IS DONE / SECOND KNOT START ***


    // X-axis move in : reset.
    nema_move(40, 215, FORWARD);
    clutch(32);
    // Y-axis Move up: Move up past shoe.
    nema_move(150, 6650, FORWARD);
    clutch(21);
    // 90 degrees rotate: Have hooks facing up.
    nema_move(120, 1175, FORWARD);
    clutch(13);
    // X-axis move in:  Move in until laces are past hooks.
    nema_move(150, 255, FORWARD);
    clutch(31);
    // 225 degrees rotate: Rotate so that hooks can grab laces.
    nema_move(120, 3000, FORWARD);
    clutch(13);
    // X-axis move out just in case: Move out until hooks catch laces.
    nema_move(40, 95, BACKWARD);
    clutch(31);
    // 45 Degrees rotate: rotate so that hooks are facing down to avoid collision.
    nema_move(120, 575, BACKWARD);
    // Clutch: 1 to 3 SPECIAL CASE.
    clutch(130);
    // X-axis move out: move out in 3 spurts.
    NEMA17->setSpeed(30);
    NEMA17->step(26,BACKWARD,DOUBLE);
    delay(500);
    NEMA17->step(26,BACKWARD,DOUBLE);
    delay(500);
    NEMA17->step(26,BACKWARD,DOUBLE);
    clutch(32);
    // Y-axis Move down to level with shoe.
    nema_move(200, 1275, BACKWARD);
    clutch(23);
    // X-axis move out:  Pull until shoeknot is done.
    nema_move(40, 290, FORWARD);

    // Reverts boolean variables back to standby values.
    ledOn = 0;
    lastButton = LOW;
    i=0;
  }
}
