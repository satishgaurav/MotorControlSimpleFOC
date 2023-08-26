/*
    Odrive robotics' hardware is one of the best  BLDC motor foc supporting hardware out there.

    This is an example code that can be directly uploaded to the Odrive using the SWD programmer.
    This code uses an encoder with 8192 cpr and a BLDC motor with 12 pole pairs connected to the M0 interface of the Odrive.

    This is a short template code and the idea is that you are able to adapt to your needs not to be a complete solution. :D
*/
#include <SimpleFOC.h>

// Odrive M0 motor pinout
#define M0_INH_A PA8
#define M0_INH_B PA9
#define M0_INH_C PA10
#define M0_INL_A PB13
#define M0_INL_B PB14
#define M0_INL_C PB15
// M0 currnets
#define M0_IB PC0
#define M0_IC PC1
// Odrive M0 encoder pinout
#define M0_ENC_A PB4
#define M0_ENC_B PB5
#define M0_ENC_Z PC9

// Odrive M1 motor pinout
#define M1_INH_A PC6
#define M1_INH_B PC7
#define M1_INH_C PC8
#define M1_INL_A PA7
#define M1_INL_B PB0
#define M1_INL_C PB1
// M0 currnets
#define M1_IB PC2
#define M1_IC PC3
// Odrive M1 encoder pinout
#define M1_ENC_A PB6
#define M1_ENC_B PB7
#define M1_ENC_Z PC15

// M1 & M2 common enable pin
#define EN_GATE PB12

// SPI pinout
#define SPI3_SCL PC10
#define SPI3_MISO PC11
#define SPI3_MOSO PC12

// Motor instance
BLDCMotor motor = BLDCMotor(12);
BLDCDriver6PWM driver = BLDCDriver6PWM(M0_INH_A, M0_INL_A, M0_INH_B, M0_INL_B, M0_INH_C, M0_INL_C, EN_GATE);

// instantiate the commander
Commander command = Commander(Serial);
void doMotor(char *cmd) { command.motor(&motor, cmd); }

Encoder encoder = Encoder(M0_ENC_A, M0_ENC_B, 8192, M0_ENC_Z);

// velocity set point variable
float target_velocity = 2;
void onTarget(char *cmd) { command.scalar(&target_velocity, cmd); }

// Interrupt routine intialisation
// channel A and B callbacks
void doA() { encoder.handleA(); }
void doB() { encoder.handleB(); }
void doI() { encoder.handleIndex(); }

void setup()
{

  // pwm frequency to be used [Hz]
  driver.pwm_frequency = 20000;
  // power supply voltage [V]
  driver.voltage_power_supply = 21;

  // driver init
  driver.init();
  // link the motor and the driver
  motor.linkDriver(&driver);

  // initialize encoder sensor hardware
  encoder.init();
  encoder.enableInterrupts(doA, doB, doI);
  // link the motor to the sensor
  motor.linkSensor(&encoder);

  // set control loop type to be used
  motor.controller = MotionControlType::velocity;

  // velocity PI controller parameters
  // default P=0.5 I = 10
  motor.PID_velocity.P = 0.2;
  motor.PID_velocity.I = 20;
  // default voltage_power_supply
  motor.voltage_limit = 6;

  // velocity low pass filtering
  // default 5ms - try different values to see what is the best.
  // the lower the less filtered
  motor.LPF_velocity.Tf = 0.01;

  // link the motor to the sensor
  motor.linkSensor(&encoder);

  Serial.begin(115200);
  // comment out if not needed
  motor.useMonitoring(Serial);
  motor.monitor_variables = _MON_CURR_Q | _MON_CURR_D;
  motor.monitor_downsample = 1000;

  // add target command T
  command.add('M', doMotor, "motor M0");
  // add target command T
  command.add('T', onTarget, "target velocity");

  // initialise motor
  motor.init();

  // init FOC
  motor.initFOC();
  delay(1000);
}

void loop()
{

  // foc loop
  motor.loopFOC();
  // motion control
  motor.move(target_velocity);
  // monitoring
  motor.monitor();
  // user communication
  command.run();
}