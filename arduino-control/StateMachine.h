//
// Created by rue011 on 17/02/18.
//
#ifndef SUBARU_CAR_STATE_MACHINE_HPP
#define SUBARU_CAR_STATE_MACHINE_HPP

#define ENGINE_ON_PIN 2
#define ENGINE_CRANK_PIN 3

#include <Servo.h>
#include "Defines.h"
#include <Math.h>

struct Actuators {
  Accelerator servo_acc; // Accelerator
  Motor gear_lever;
  Motor brake_lever;
};

class StateMachine{
  public:
    StateMachine(Actuators *actuators);
    StateMachine();
    void initialize();
    void update();

    bool requestState(State requested_state);

    void achieveCurrentState();
    void changeState();
    void emergencyStop();
    Gear getGear();

  State getState(){
    return state_;
  }


  private:
    State state_; // enumarotor of the state
    Gear  gear_;
    double brake_;
    bool engineCrankOn_, ignitionOn_, motorOn_; // igni_start: 0,1 to start the motor,
    double acc_;
    Actuators *actuators_;

    // We dont want to acces the states of the StateMachine
    //  void setState(State state){state_=state}
    // Checks if all defined coditoin in states
    bool checkState();
    bool checkStop();
    bool checkHalt();
    bool checkIgnition();
    bool checkStart();
    bool checkRunning();
    bool checkDrivinng();


    void setIgnitionON();
    void setIgnitionOFF();
    void setIgnitionCrankON();
    void setIgnitionCrankOFF();
};

StateMachine::StateMachine() {}

StateMachine::StateMachine(Actuators *actuators) {
  actuators_ = actuators;
  initialize();
}

void StateMachine::setIgnitionON(){
  ignitionOn_ = 1;
  digitalWrite(ENGINE_ON_PIN, 1);
}

void StateMachine::setIgnitionOFF(){
  ignitionOn_ = 0;
  digitalWrite(ENGINE_ON_PIN, 0);
}

void StateMachine::setIgnitionCrankON(){
  Serial.println("setIgnitionCrankON");
  engineCrankOn_ = 1;
  digitalWrite(ENGINE_CRANK_PIN, 1);
}

void StateMachine::setIgnitionCrankOFF(){
  Serial.println("setIgnitionCrankOFF");
  engineCrankOn_ = 0;
  digitalWrite(ENGINE_CRANK_PIN, 0);
}


// Intialisation of the StateMachine
void StateMachine::initialize(){
  state_ = HALT;
  setIgnitionOFF();
  setIgnitionCrankOFF();
  //ToDO:: Check MOTORS  for current state
  update();
}

// Updates the state of the motors
void StateMachine::update(){
//ToDo: Get the actuator values
 gear_ = getGear();
 brake_ = actuators_->brake_lever.getPosition();
}

Gear StateMachine::getGear() {
  int gear_pos = actuators_->gear_lever.getPosition();
  if(abs(GEAR_P_POS - gear_pos) <= GEAR_OFFSET )
    return PARK;
  if(abs(GEAR_R_POS - gear_pos) <= GEAR_OFFSET )
    return REVERSE;
  if (abs(GEAR_N_POS - gear_pos) <= GEAR_OFFSET )
    return NEUTRAL;
  if(abs(GEAR_D_POS - gear_pos) <= GEAR_OFFSET )
    return DRIVE;
  else
    return UKNOWN;

}
// State request from the main file,
bool StateMachine::requestState(State requested_state) {
  update();
  if (STOP == requested_state) {
    emergencyStop();
    return true;
  } else if(requested_state == state_ && checkState()) { // Are in the requested state and all the state condiftions have been met
    return true;
  } else {
   // This is where Switch to the next state, and start acchiving the state;
    if( checkState() ){
      Serial.println("Passed move test, changing state");
      changeState();
      return false;
    }
    else if(!checkState()){
      achieveCurrentState();
      return false;
    }
    else{}
//  assert(false);
  }
//  assert(false);
}

void StateMachine::emergencyStop(){
  state_ = STOP;
  actuators_->brake_lever.setTarget(BRAKE_ON_POS);
  setIgnitionOFF();
}


void StateMachine::achieveCurrentState(){
  double t = 0;
  double delta_t = 0.2;
  switch(state_){
    default:
//      assert(false);
      break;
    case STOP:
      // Only reset
    //ToDo:: Read motor values
      actuators_->brake_lever.setTarget(BRAKE_ON_POS);
      digitalWrite(ENGINE_ON_PIN, 0);
      break;

    case HALT:
      Serial.println("achieveCurrentState: HALT");
      acc_ = 0;
      actuators_->servo_acc.setTarget(ACCELERATOR_ZERO);
      actuators_->gear_lever.setTarget(GEAR_P_POS);
      setIgnitionOFF();
      setIgnitionCrankOFF();
      break;

    case IGNITION:
      Serial.println("achieveCurrentState: IGNITION");
      setIgnitionON();
      break;

    case START:
      Serial.println("achieveCurrentState: START");
      setIgnitionCrankON();
      delay(1500);
      setIgnitionCrankOFF();
      delay(1000);
//      while(t < 0.5) {
//        setIgnitionCrankON();
//        t += delta_t;
//      }
//      setIgnitionCrankOFF();
//      delay(3000);
      break;

    case RUNNING:
      Serial.println("achieveCurrentState: RUNNING");
      actuators_->gear_lever.setTarget(GEAR_D_POS);
      actuators_->brake_lever.setTarget(BRAKE_OFF_POS);
      break;

    case DRIVING:
      Serial.println("achieveCurrentState: DRIVING");
      break;
  }
}

void StateMachine::changeState(){
  switch(state_){

    default: //This should never happen
      emergencyStop();
      break;

    case STOP:
      state_ = HALT;
      achieveCurrentState();
      break;

    case HALT:
      state_ = IGNITION;
      achieveCurrentState();
      break;

    case IGNITION:
      state_ = START;
      achieveCurrentState();
      break;

    case START:
      state_ = RUNNING;
      achieveCurrentState();
      break;

    case RUNNING:
      state_ = DRIVING;
      achieveCurrentState();
      break;
  }
}

bool StateMachine::checkState(){
  switch(state_){
    default: //This should never happen
      emergencyStop();
      return false;
    case STOP:
      return checkStop();
    case HALT:
      return checkHalt();
    case IGNITION:
      return checkIgnition();
    case START:
      return checkStart();
    case RUNNING:
      return checkRunning();
    case DRIVING:
      return checkDrivinng();
  }
}


bool StateMachine::checkStop(){
  if (brake_ > BRAKE_ON_POS && false == ignitionOn_) {
    Serial.println("checkStop(): true");
    return true;
  }else{
    Serial.println("checkStop(): false");
    return false;
  }
}

bool StateMachine::checkHalt(){
  Serial.print("Ignition: ");
  Serial.print(ignitionOn_);
  Serial.print(" Accelerator: ");
  Serial.print(acc_);
  Serial.print(" Gear: ");
  Serial.print(gear_);
  Serial.print(" Brake: ");
  Serial.println(brake_);
  if (ignitionOn_ == false && 0 == acc_ && gear_ == PARK &&
      brake_ > BRAKE_ON_POS && false == engineCrankOn_) {
    Serial.println("checkHalt() = True");
    return true;
  } else {
    Serial.println("checkHalt() = False");
    return false;
  }
}

bool StateMachine::checkIgnition(){
  if (ignitionOn_ == true && 0 == acc_ && gear_ == PARK &&
      brake_ > BRAKE_ON_POS && false == engineCrankOn_){
        Serial.println("checkIgnition() = True");
    return true;
  }else{
        Serial.println("checkIgnition() = False");
    return false;
  }

}
bool StateMachine::checkStart(){
//  if (ignitionOn_ == true && 0 == acc_ && gear_ == PARK && false == engineCrankOn_){
//        Serial.println("checkStart() = True");
//    return true;
//  }else{
//        Serial.println("checkStart() = False");
//    return false;
  return true;
//  }
}
bool StateMachine::checkRunning() {
  if (ignitionOn_ == true && 0 == acc_ && gear_ == PARK  && false == engineCrankOn_){
    Serial.println("checkRunning() = True");
    return true;
  }else{
    return false;
    Serial.println("checkRunning() = False");
  }
}


bool StateMachine::checkDrivinng(){
  if (ignitionOn_ == true && 0 == acc_ && gear_ == PARK &&
      100 == brake_ && false == engineCrankOn_){
    return true;
  }else{
    return false;
  }
}

#endif //SUBARU_CAR_STATE_MACHINE_HPP


