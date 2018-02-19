/* Ignores the dead man switch variable from the jetson */
#define IGNORE_DEAD_MAN 0

//#define GPS_PRINTOUT 0

/* Defines for the gear positions */
#define GEAR_P_POS 160
#define GEAR_R_POS 310
#define GEAR_N_POS 360
#define GEAR_D_POS 460
//#define GEAR_1_POS 600
//#define GEAR_2_POS 700

/* Defines for the brake positions */
#define BRAKE_ON_POS 300
#define BRAKE_OFF_POS 600

#define ACCELERATOR_ZERO 70
#define ACCELERATOR_MAX 85

#define GEAR_OFFSET 100

enum State {
  HALT,
  STOP,
  IGNITION,
  START,
  RUNNING,
  DRIVING
};

enum Gear{
  PARK,
  REVERSE,
  DRIVE,
  NEUTRAL,
  UKNOWN
};


float mapfloat(long x, long in_min, long in_max, long out_min, long out_max)
{
 return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}
