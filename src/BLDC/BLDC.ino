#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>
#include <encoders/smoothing/SmoothingSensor.h>
#include <BLDCCustomMotor.h>
/*
ESC_BLDC_2024
By Noturno and Bail
The code is focused on controlling a BLDC or PMSM motor using Simple Field Oriented Control (Simple FOC) for a Prototype vehicle intended to compete in SEM Brazil 2024.

Powered by SimpleFOC:
https://docs.simplefoc.com/code

PHASE SEQUENCY:
PHASE A - BLUE
PHASE B - GREEN
PHASE C - YELLOW

*/

#define DEBUG_ON 1
#define DEBUG_OFF 0
byte debugMode = DEBUG_ON;

#define DBGLN(...) debugMode == DEBUG_ON ? Serial.println(__VA_ARGS__) : NULL
#define DBG(...) debugMode == DEBUG_ON ? Serial.print(__VA_ARGS__) : NULL

const int POWER_SUPPLY = 42;
const int DRIVER_LIMIT = 42;
const int MOTOR_LIMIT = 41;

const int PWM_FREQ = 20000;
const int MOTOR_KV = 10.5;
const int TURNOFF_SPEED = 50; // rad/s
const int TURNON_SPEED = 13;  // rad/s
const int SPEED_CYCLES = 250;
const float ACCEL_RAMP = 0.001;

const int POLE_PAIRS = 88;  
const float PHASE_RESISTANCE = 0.098;
const float PHASE_INDUCTANCE = 0.0000644;
const float PM_FLUX_LINKAGE = 0.001;

bool MAX_VEL_TRIGGERED = false;
float target_torque = 0;
float offset_angle = 3.14f;
// BLDC motor & driver instance
// BLDC motor instance BLDCMotor(polepairs, (R), (KV))
BLDCMotor motor = BLDCMotor(88, 0.098, MOTOR_KV, PHASE_INDUCTANCE);
//BLDCCustomMotor motor = BLDCCustomMotor(POLE_PAIRS, PHASE_RESISTANCE, MOTOR_KV, PM_FLUX_LINKAGE);
// PWM pins
BLDCDriver6PWM driver = BLDCDriver6PWM(PA8, PB13, PA9, PB14, PA10, PB15);

// Hall sensor instance
HallSensor sensor = HallSensor(PA15, PB3, PB4, 88);
// Hall sensor interpolation
SmoothingSensor smooth = SmoothingSensor(sensor, motor);
// Sensor Hall pins and poles configurated

// Interrupt routine intialisation
void doA() { sensor.handleA(); }
void doB() { sensor.handleB(); }
void doC() { sensor.handleC(); }

// Hardware interrupts instantiated (all the code are running in the library)

// PID controllers for current
PIDController &pid_d = motor.PID_current_d;
PIDController &pid_q = motor.PID_current_q;

int cycle = 0;

// LowsideCurrentSense(shunt_resistance, gain, adc_a, adc_b, adc_c)
LowsideCurrentSense current_sense = LowsideCurrentSense(0.002, 40, PA7, PB0, PB1);
// low side current sensors (for all phases) instantiated
// LowPassFilter d_lpf = LowPassFilter(time_constant);
// LowPassFilter q_lpf = LowPassFilter(time_constant);
// instantiate the commander for managing via Serial
// TODO: Check performance impact
Commander command = Commander(Serial);
void doTarget(char *cmd) { command.scalar(&target_torque, cmd); }
void doOffset(char *cmd) { command.scalar(&offset_angle, cmd); }
void doMotor(char *cmd) { command.motor(&motor, cmd); }
void onPid(char *cmd)
{
  command.pid(&pid_d, cmd);
  command.pid(&pid_q, cmd);
}

void setup()
{
  // enable more verbose output from library for debugging
  if (debugMode)
  {
    SimpleFOCDebug::enable(&Serial);
    Serial.begin(115200);
  }
  // Initial delay for connecting to serial
  delay(3000);

  // Custom Start configs
  // Tun on Motor Driver Enable
  SIMPLEFOC_DEBUG("Driver Enable ");
  pinMode(PA4, OUTPUT);
  digitalWrite(PA4, HIGH);

  delay(500);
  // nFault PIN - Attach interrupt to trigger shutdown function if driver fault is detected
  pinMode(PB10, INPUT);                                                         
  attachInterrupt(digitalPinToInterrupt(PB10), ChecknFaultProtection, FALLING);
  pinMode(PA6, INPUT);

  // Current sense driver calibration
  SIMPLEFOC_DEBUG("Driver Current Calibration, please wait... ");
  pinMode(PB8, OUTPUT);
  digitalWrite(PB8, LOW);
  delay(100);
  digitalWrite(PB8, HIGH);
  delay(100);
  SIMPLEFOC_DEBUG("Driver Current calibrated!");
  digitalWrite(PB8, LOW);
  delay(500);
  // Driver current sense calibrated

  // Blink x2
  SIMPLEFOC_DEBUG("Finishing some things...");
  pinMode(PC13, OUTPUT);
  digitalWrite(PC13, LOW);
  delay(1000);
  digitalWrite(PC13, HIGH);
  delay(1000);
  digitalWrite(PC13, LOW);
  delay(1000);
  SIMPLEFOC_DEBUG("So far, so good!");
  digitalWrite(PC13, HIGH);
  // Visual confirmation of all custom hardware initialization set properly
  // Hardware Custom Configs Finished

  // Library Start
  // initialize encoder sensor hardware
  sensor.init();
  sensor.enableInterrupts(doA, doB, doC);
  // Hall sensor initialized and motor start position founded

  // link the Hall sensor to motor
  motor.linkSensor(&smooth);

  // Driver settings:
  // PWM Frequency Configuration [Hz] - Between 25 kHz and 50 kHz
  driver.pwm_frequency = PWM_FREQ;
  // Pwm work Frequency configurated

  // power supply voltage [V]
  driver.voltage_power_supply = POWER_SUPPLY;
  driver.voltage_limit = DRIVER_LIMIT;

  // dead_zone [0,1] - default 0.02 - 2%
  driver.dead_zone = 0.025;

  // Diver initialization Routine
  SIMPLEFOC_DEBUG("Driver init...");
  driver.init();

  motor.linkDriver(&driver);

  // Circuit configurations
  // Link the driver with the current sense
  current_sense.linkDriver(&driver);
  current_sense.init();

  //  Motor directions and alignment configurations
  motor.sensor_direction = Direction::CW; // should be implemented for skipping align routine
  motor.zero_electric_angle = offset_angle;
  // motor Aligned and direction of rotation configuration completed

  // FOC modulation settings:
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.torque_controller = TorqueControlType::foc_current;
  motor.controller = MotionControlType::torque;

  // Current PID settings
  // TODO: Autotuning
  //Iq
  motor.LPF_current_q.Tf = 0.005f;
  motor.PID_current_q.P = 1.5f; 
  motor.PID_current_q.I = 100;
  motor.PID_current_q.limit = POWER_SUPPLY;
  motor.PID_current_q.output_ramp = 250;
  //Id
  motor.LPF_current_d.Tf = 0.005f;
  motor.PID_current_d.P = 1.5f;
  motor.PID_current_d.I = 100;
  motor.PID_current_q.limit = POWER_SUPPLY;
  motor.PID_current_d.output_ramp = 250;

  // Setting the motor limits
  motor.voltage_limit = MOTOR_LIMIT; // Volts - default driver.voltage_limit     - Hard limit on output voltage, in volts. Effectively limits PWM duty cycle proportionally to power supply voltage.

  // Configurating Serial to control motor

  motor.useMonitoring(Serial);
  //   motor.monitor_variables = _MON_TARGET | _MON_VEL;
  // display variables
  // SimpleFOCDebug::enable();
  // motor.monitor_variables = _MON_TARGET | _MON_VEL | _MON_ANGLE;
  // Choosed infos to plot and debbug


  // Circuit configurations finished

  // Initializing motor
  motor.init();

  // Link current sense to motor
  motor.linkCurrentSense(&current_sense);

  current_sense.skip_align = true;
  // Start FOC
  motor.initFOC();

  // control procedure via Serial
  command.add('T', doTarget, "target Current");
  command.add('O', doOffset, "offset");
  command.add('C', onPid, "my pid");

  SIMPLEFOC_DEBUG("Motor ready.");
}

void loop()
{

  motor.loopFOC();

  cycle = cycle + 1;
  if (cycle == SPEED_CYCLES)
  {
    //CalculateButtonThrottle();

    //float error = motor.target - motor.current.q;
    //motor.zero_electric_angle = offset_angle;
    PulseAndGlide();
    PhaseCurrent_s currents = current_sense.getPhaseCurrents();
    cycle = 0;
    DBG(motor.target); // milli Amps
    DBG(" , ");
    DBGLN(motor.current.q); // milli Amps
    // DBG(" , ");
    // DBGLN(motor.shaft_velocity);
    //DBG(" , ");
    //DBGLN(motor.electrical_angle);

    command.run();
  }
}

void CalculateButtonThrottle()
{
    int button_pressed = digitalRead(PA6);
    if(button_pressed == HIGH){
      target_torque = constrain(target_torque + ACCEL_RAMP, 0, 1);
      SIMPLEFOC_DEBUG("pressed");
    }
    else {
      target_torque = constrain(target_torque - ACCEL_RAMP*5, 0, 1);
    }
    SIMPLEFOC_DEBUG("TARGET TORQUE:", target_torque);
}

// Implements pulse and glide strategy - Car should accelerate to defined  TURNOFF_SPEED and then coast until TURNON_SPEED is reached
void PulseAndGlide()
{
if (MAX_VEL_TRIGGERED)
    {
      if (motor.shaft_velocity < TURNON_SPEED)
      {
        SIMPLEFOC_DEBUG("RE-ENABLING");
        motor.move(target_torque);
        MAX_VEL_TRIGGERED = false;
      }
      else
      {
        motor.move(0);
      }
    }
    else
    {
      if (motor.shaft_velocity > TURNOFF_SPEED)
      {
        SIMPLEFOC_DEBUG("SHUTTING DOWN!");
        motor.move(0);
        MAX_VEL_TRIGGERED = true;
      }
      else
      {
        motor.move(target_torque);
      }
    }
}


void ChecknFaultProtection()
{
  if (digitalRead(PB10) == LOW)
  {
    digitalWrite(PA4, LOW);
    SIMPLEFOC_DEBUG("Driver Fault Detected - Shutting down");
  }
}
