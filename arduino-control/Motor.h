class Motor {
  public:
    Motor(Servo* servo, int feedback_pin);
    Motor();
    void update();
    void setTarget(int x);
    int max = 1023;
    int min = 0;
    void setMax(int x);
    void setMin(int x);
    int getPosition();
    bool debug = false;
    double PTERM = 0.5;
    bool invert = false;
  private:
    int motor_pin;
    int feedback_pin;
    int current_speed = 90;
    int target = 450;
    double error = 0;
    Servo* servo;
};

Motor::Motor() {
}

Motor::Motor(Servo* servo, int feedback_pin) {
  this->motor_pin = motor_pin;
  this->feedback_pin = feedback_pin;
  this->servo = servo;
  pinMode(this->feedback_pin, INPUT);
}

void Motor::update() {
  int sensorPos = analogRead(this->feedback_pin);
  int diff = this->target - sensorPos;
  int term = 90 + diff*PTERM;

  // Limit to 180 and 0
  if (term > 180)
    term = 180;
  if (term < 0)
    term = 0;

  if (this->debug) {
    Serial.print("Error: ");
    Serial.println(diff);
    Serial.print("Setting motor to: ");
    Serial.println(term);
  }

  if (sensorPos < this->min) {
    term = 100;
  }

  if (sensorPos > this->max) {
    term = 80;
  }

  // Finally write out to the stepper motor
  if (this->invert) {
    this->servo->write(180 - term);
  } else {
    this->servo->write(term);
  }
}

void Motor::setTarget(int x) {
  if (x > this->max)
    x = this->max;
  if (x < this->min)
    x = this->min;
  this->target = x;
//  Serial.print("Writing to motor ");
//  Serial.println(this->target = x);
}

void Motor::setMax(int x) {
  this->max = x;
}

void Motor::setMin(int x){
  this->min = x;
}

int Motor::getPosition() {
  int a = analogRead(this->feedback_pin);
  if (this->debug) {
    Serial.print("motorPosition: ");
    Serial.println(a);
  }
  return a;
}

