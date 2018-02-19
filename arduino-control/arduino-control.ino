#include <Servo.h>
//#include <Adafruit_GPS.h>
#include "Motor.h"
#include "JetsonCom.h"
#include "Accelerator.h"
#include "StateMachine.h"

#define MAX_ANGLE 50

#define ENGINE_ON_PIN 2
#define ENGINE_CRANK_PIN 3

#define HALT 1
#define PARK 2
#define BRAKE 3
#define STEERING 4
#define POWER 5
#define IGNITE 6
#define DRIVE 7
#define GO 8

int ignitionOn_ = 0;
int engineCrankOn_ = 0;


int state = GO;

/* Variables for each motor */
Motor gear_lever;
Motor brake_lever;
Motor steering_motor;
Accelerator accelerator;

int switch_pwm;
int left_pwm;
int right_pwm;
int dir = 0;

int velocity = 0;
int angle = 0;
bool deadman = true;

void setIgnitionON(){
  ignitionOn_ = 1;
  digitalWrite(ENGINE_ON_PIN, 1);
}

void setIgnitionOFF(){
  ignitionOn_ = 0;
  digitalWrite(ENGINE_ON_PIN, 0);
}

void setIgnitionCrankON(){
  Serial.println("setIgnitionCrankON");
  engineCrankOn_ = 1;
  digitalWrite(ENGINE_CRANK_PIN, 1);
}

void setIgnitionCrankOFF(){
  Serial.println("setIgnitionCrankOFF");
  engineCrankOn_ = 0;
  digitalWrite(ENGINE_CRANK_PIN, 0);
}

void wait(int ms) {
  while(ms--) {
    switch_pwm = pulseIn(7, HIGH);
  
    if (switch_pwm < 1500) {
      setIgnitionOFF();
//      brake_lever.setTarget(BRAKE_ON_POS);
      state = HALT;
      break;
    }
    delay(1);
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);

  Serial.print("Subaru init started....");
  
  Servo gear;
  gear.attach(12);
  gear_lever = Motor(&gear, A0);
  gear_lever.min = 0;
  gear_lever.max = 1023;
  gear_lever.PTERM = 1.0;
  gear_lever.setTarget(GEAR_P_POS);

  Servo brake;
  brake.attach(11);
  brake_lever = Motor(&brake, A1);
  brake_lever.min = 0;
  brake_lever.max = 1023;
  brake_lever.setTarget(BRAKE_ON_POS);

  Servo steering;
  steering.attach(10);
  steering_motor = Motor(&steering, A2);
  steering_motor.min = 0;
  steering_motor.max = 1023;
  steering_motor.invert = true;
  steering_motor.PTERM = 0.4;

  Servo __accl;
  __accl.attach(9); 
  accelerator = Accelerator(&__accl);
  accelerator.setTarget(ACCELERATOR_ZERO + 10);

  pinMode(5, INPUT);
  pinMode(6, INPUT);
  pinMode(7, INPUT);
}

void loop() {  
  gear_lever.update();
  brake_lever.update();
  steering_motor.update();

  switch_pwm = pulseIn(7, HIGH);

//  Serial.print("Kill switch = ");
//  Serial.println(switch_pwm > 1500);

  if (switch_pwm < 1500) {
    setIgnitionOFF();
    brake_lever.setTarget(BRAKE_ON_POS);
    gear_lever.setTarget(GEAR_P_POS);
    state = HALT;
  }

  switch (state) {
    case HALT:
      Serial.println("HALT");
      accelerator.setTarget(ACCELERATOR_ZERO + 10);
      if (switch_pwm > 1500) {
        state = PARK;
      }
      break;
    case PARK:
      /* Put in park */
      gear_lever.setTarget(GEAR_P_POS);
      Serial.println("Putting in Park");
      wait(200);
      state = BRAKE;
      break;
    case BRAKE:
      /* Put in brake */
      brake_lever.setTarget(BRAKE_ON_POS);
      Serial.println("Turning on brake");
      wait(200);
      state = STEERING;
      break;
    case STEERING:
      // 500 midpoint
      // 250 left
      // 750 right
      steering_motor.setTarget(500);
      Serial.println("Setting steering to 500");
      wait(500);
      state = POWER;
      break;
    case POWER:
      setIgnitionON();
      Serial.println("Set Ignition On");
      wait(200);
      state = IGNITE;
      break;
    case IGNITE:
      setIgnitionON();
      Serial.println("Set Ignition On");
      wait(200);
      Serial.println("Starting engine...");
      setIgnitionCrankON();
      accelerator.setTarget(45);
      wait(150);
      setIgnitionCrankOFF();
      Serial.println("Started");
      wait(100);
      state = DRIVE;
      break;
    case DRIVE:
      dir = pulseIn(4, HIGH);
      if (dir < 1500) {
        gear_lever.setTarget(GEAR_D_POS);
      } else {
        gear_lever.setTarget(GEAR_R_POS);
      }
      
      brake_lever.setTarget(BRAKE_OFF_POS);
      Serial.println("Putting in Drive");
      state = GO;
      wait(200);
      break;
    case GO:
      switch_pwm = pulseIn(7, HIGH);
      left_pwm = pulseIn(5, HIGH);
      right_pwm = pulseIn(6, HIGH);
      dir = pulseIn(4, HIGH);
      if (dir < 1500) {
        gear_lever.setTarget(GEAR_D_POS);
      } else {
        gear_lever.setTarget(GEAR_R_POS);
      }
      velocity = map(left_pwm,983,2001,0,100);
      angle = map(right_pwm,983,2001,250,750);
      angle -= MAX_ANGLE;
      Serial.print("Velocity to ");
      Serial.println(velocity);
      accelerator.setTarget(velocity);
      steering_motor.setTarget(angle);
      break;
  }

  delay(10);
}
