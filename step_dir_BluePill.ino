/**
 * A position control example using step/dir interface to update the motor position
 */

#include <SimpleFOC.h>
#include <PciManager.h>
#include <PciListenerImp.h>
// BLDC motor & driver instance

//BLDCMotor motor = BLDCMotor(16);
//BLDCDriver3PWM driver = BLDCDriver3PWM(9, 5, 6, 8);

BLDCMotor motor = BLDCMotor(8);
BLDCDriver3PWM driver = BLDCDriver3PWM(PA10, PA9, PA8, PA6);

// Stepper x`motor & driver instance
//StepperMotor motor = StepperMotor(50);
//StepperDriver4PWM driver = StepperDriver4PWM(9, 5, 10, 6,  8);

// encoder instance
//Encoder encoder = Encoder(2, 3, 800);
Encoder encoder = Encoder(PB8, PB9, 400);
// channel A and B callbacks
void doA() { encoder.handleA(); }
void doB() { encoder.handleB(); }

// StepDirListener( step_pin, dir_pin, counter_to_value)
StepDirListener step_dir = StepDirListener(PB5, PB4, 2.0f*_PI/200.0);
//PciListenerImp listenStep(step_dir.pin_step, onStep);

void onStep() { step_dir.handle(); }

void setup() {

  // initialize encoder sensor hardware
  encoder.init();
  encoder.enableInterrupts(doA, doB);
  // link the motor to the sensor
  motor.linkSensor(&encoder);

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 24;
  driver.init();
  // link the motor and the driver
  motor.linkDriver(&driver);

  // aligning voltage [V]
  motor.voltage_sensor_align = 2;
  // index search velocity [rad/s]
  motor.velocity_index_search = 3;

  // set motion control loop to be used
  motor.controller = MotionControlType::angle;

  // contoller configuration
  // default parameters in defaults.h
  // velocity PI controller parameters
  motor.PID_velocity.P = 0.1f;
  motor.PID_velocity.I = 10.0f;
  motor.PID_velocity.D = 0;
  // default voltage_power_supply
  motor.voltage_limit = 3;
  // jerk control using voltage voltage ramp
  // default value is 300 volts per sec  ~ 0.3V per millisecond
  motor.PID_velocity.output_ramp = 1000;

  // velocity low pass filtering time constant
  motor.LPF_velocity.Tf = 0.01f;

  // angle P controller
  motor.P_angle.P = 15;
  //  maximal velocity of the position control
  motor.velocity_limit = 1000;

  // use monitoring with serial
  Serial.begin(115200);
  // comment out if not needed
  motor.useMonitoring(Serial);

  // initialize motor
  motor.init();
  // align encoder and start FOC
  motor.initFOC();

 // // init step and dir pins
 // step_dir.init();
 // // enable interrupts
 //  PciManager.registerListener(&listenStep);
  //step_dir.enableInterrupt(onStep);
  // attach the variable to be updated on each step (optional)
  // the same can be done asynchronously by caling motor.move(step_dir.getValue());
  //step_dir.attach(&motor.target);

  // init step and dir pins
  step_dir.init();
  // enable interrupts
  step_dir.enableInterrupt(onStep);
  // attach the variable to be updated on each step (optional)
  // the same can be done asynchronously by caling motor.move(step_dir.getValue());
  step_dir.attach(&motor.target);

  Serial.println(F("Motor ready."));
  Serial.println(F("Listening to step/dir commands!"));
  _delay(1000);
}

void loop() {

  // main FOC algorithm function
  motor.loopFOC();
   //Serial.print("\t");
   // Serial.println(step_dir.getValue());
  // Motion control function
  motor.move();
}