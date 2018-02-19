#define ACCELERATOR_MULTIPLIER 0.25

class Accelerator {
  public:
    Accelerator();
    int min = 0;
    int max = 100;
    int motor_pin;
    Accelerator(Servo* servo);
    void Accelerator::setTarget(int x);
  private:
    Servo* servo;
};

Accelerator::Accelerator() {
}

Accelerator::Accelerator(Servo* servo) {
  this->servo = servo;
}

void Accelerator::setTarget(int x) {
//  x *= ACCELERATOR_MULTIPLIER;
  if (x > this->max)
    x = this->max;
  if (x < this->min)
    x = this->min;
  this->servo->write(x);
}
