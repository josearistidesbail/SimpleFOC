#include "BLDCCustomMotor.h"
#include "./communication/SimpleFOCDebug.h"

// This class is for operating the BLDC motor only in FOC current and SVPWM modulation, to run the algorithm more efficiently
// It also implements Flux/Field Weakening operation using CVCP ( Constant Voltage, Constant Power) algorithm
// By Bail

// BLDCCustomMotor( int pp , float R)
// - pp            - pole pair number
// - R             - motor phase resistance
// - KV            - motor kv rating (rmp/v)
// - L             - motor phase inductance
BLDCCustomMotor::BLDCCustomMotor(int pp, float _R, float _KV, float _inductance, float _phi_pm, float _base_speed)
    : FOCMotor()
{
  // save pole pairs number
  pole_pairs = pp;
  // save phase resistance number
  phase_resistance = _R;
  // save back emf constant KV = 1/KV
  // 1/sqrt(2) - rms value
  KV_rating = NOT_SET;
  if (_isset(_KV))
    KV_rating = _KV;
  // save phase inductance
  phase_inductance = _inductance;
  phi_pm = _phi_pm;
  base_speed = _base_speed;
  // torque control type is voltage by default
  torque_controller = TorqueControlType::foc_current;
}

/**
  Link the driver which controls the motor
*/
void BLDCCustomMotor::linkDriver(BLDCDriver *_driver)
{
  driver = _driver;
}

// init hardware pins
int BLDCCustomMotor::init()
{
  if (!driver || !driver->initialized)
  {
    motor_status = FOCMotorStatus::motor_init_failed;
    SIMPLEFOC_DEBUG("MOT: Init not possible, driver not initialized");
    return 0;
  }
  motor_status = FOCMotorStatus::motor_initializing;
  SIMPLEFOC_DEBUG("MOT: Init");

  // sanity check for the voltage limit configuration
  if (voltage_limit > driver->voltage_limit)
    voltage_limit = driver->voltage_limit;
  // constrain voltage for sensor alignment
  if (voltage_sensor_align > voltage_limit)
    voltage_sensor_align = voltage_limit;

  // update the controller limits
  if (current_sense)
  {
    // current control loop controls voltage
    PID_current_q.limit = voltage_limit;
    PID_current_d.limit = voltage_limit;
  }
  if (_isset(phase_resistance) || torque_controller != TorqueControlType::voltage)
  {
    // velocity control loop controls current
    PID_velocity.limit = current_limit;
  }
  else
  {
    // velocity control loop controls the voltage
    PID_velocity.limit = voltage_limit;
  }
  P_angle.limit = velocity_limit;

  // if using open loop control, set a CW as the default direction if not already set
  if ((controller == MotionControlType::angle_openloop || controller == MotionControlType::velocity_openloop) && (sensor_direction == Direction::UNKNOWN))
  {
    sensor_direction = Direction::CW;
  }

  _delay(500);
  // enable motor
  SIMPLEFOC_DEBUG("MOT: Enable driver.");
  enable();
  _delay(500);
  motor_status = FOCMotorStatus::motor_uncalibrated;
  return 1;
}

// disable motor driver
void BLDCCustomMotor::disable()
{
  // disable the current sense
  if (current_sense)
    current_sense->disable();
  // set zero to PWM
  driver->setPwm(0, 0, 0);
  // disable the driver
  driver->disable();
  // motor status update
  enabled = 0;
}
// enable motor driver
void BLDCCustomMotor::enable()
{
  // enable the driver
  driver->enable();
  // set zero to PWM
  driver->setPwm(0, 0, 0);
  // enable the current sense
  if (current_sense)
    current_sense->enable();
  // reset the pids
  PID_velocity.reset();
  P_angle.reset();
  PID_current_q.reset();
  PID_current_d.reset();
  // motor status update
  enabled = 1;
}

/**
  FOC functions
*/
// FOC initialization function
int BLDCCustomMotor::initFOC()
{
  int exit_flag = 1;

  motor_status = FOCMotorStatus::motor_calibrating;

  // align motor if necessary
  // alignment necessary for encoders!
  // sensor and motor alignment - can be skipped
  // by setting motor.sensor_direction and motor.zero_electric_angle
  if (sensor)
  {
    exit_flag *= alignSensor();
    // added the shaft_angle update
    sensor->update();
    shaft_angle = shaftAngle();

    // aligning the current sensor - can be skipped
    // checks if driver phases are the same as current sense phases
    // and checks the direction of measuremnt.
    if (exit_flag)
    {
      if (current_sense)
      {
        if (!current_sense->initialized)
        {
          motor_status = FOCMotorStatus::motor_calib_failed;
          SIMPLEFOC_DEBUG("MOT: Init FOC error, current sense not initialized");
          exit_flag = 0;
        }
        else
        {
          exit_flag *= alignCurrentSense();
        }
      }
      else
      {
        SIMPLEFOC_DEBUG("MOT: No current sense.");
      }
    }
  }
  else
  {
    SIMPLEFOC_DEBUG("MOT: No sensor.");
    if ((controller == MotionControlType::angle_openloop || controller == MotionControlType::velocity_openloop))
    {
      exit_flag = 1;
      SIMPLEFOC_DEBUG("MOT: Openloop only!");
    }
    else
    {
      exit_flag = 0; // no FOC without sensor
    }
  }

  if (exit_flag)
  {
    SIMPLEFOC_DEBUG("MOT: Ready.");
    motor_status = FOCMotorStatus::motor_ready;
  }
  else
  {
    SIMPLEFOC_DEBUG("MOT: Init FOC failed.");
    motor_status = FOCMotorStatus::motor_calib_failed;
    disable();
  }

  return exit_flag;
}

// Calibarthe the motor and current sense phases
int BLDCCustomMotor::alignCurrentSense()
{
  int exit_flag = 1; // success

  SIMPLEFOC_DEBUG("MOT: Align current sense.");

  // align current sense and the driver
  exit_flag = current_sense->driverAlign(voltage_sensor_align, modulation_centered);
  if (!exit_flag)
  {
    // error in current sense - phase either not measured or bad connection
    SIMPLEFOC_DEBUG("MOT: Align error!");
    exit_flag = 0;
  }
  else
  {
    // output the alignment status flag
    SIMPLEFOC_DEBUG("MOT: Success: ", exit_flag);
  }

  return exit_flag > 0;
}

// Encoder alignment to electrical 0 angle
int BLDCCustomMotor::alignSensor()
{
  int exit_flag = 1; // success
  SIMPLEFOC_DEBUG("MOT: Align sensor.");

  // check if sensor needs zero search
  if (sensor->needsSearch())
    exit_flag = absoluteZeroSearch();
  // stop init if not found index
  if (!exit_flag)
    return exit_flag;

  // v2.3.3 fix for R_AVR_7_PCREL against symbol" bug for AVR boards
  // TODO figure out why this works
  float voltage_align = voltage_sensor_align;

  // if unknown natural direction
  if (sensor_direction == Direction::UNKNOWN)
  {

    // find natural direction
    // move one electrical revolution forward
    for (int i = 0; i <= 500; i++)
    {
      float angle = _3PI_2 + _2PI * i / 500.0f;
      setPhaseVoltage(voltage_align, 0, angle);
      sensor->update();
      _delay(2);
    }
    // take and angle in the middle
    sensor->update();
    float mid_angle = sensor->getAngle();
    // move one electrical revolution backwards
    for (int i = 500; i >= 0; i--)
    {
      float angle = _3PI_2 + _2PI * i / 500.0f;
      setPhaseVoltage(voltage_align, 0, angle);
      sensor->update();
      _delay(2);
    }
    sensor->update();
    float end_angle = sensor->getAngle();
    // setPhaseVoltage(0, 0, 0);
    _delay(200);
    // determine the direction the sensor moved
    float moved = fabs(mid_angle - end_angle);
    if (moved < MIN_ANGLE_DETECT_MOVEMENT)
    { // minimum angle to detect movement
      SIMPLEFOC_DEBUG("MOT: Failed to notice movement");
      return 0; // failed calibration
    }
    else if (mid_angle < end_angle)
    {
      SIMPLEFOC_DEBUG("MOT: sensor_direction==CCW");
      sensor_direction = Direction::CCW;
    }
    else
    {
      SIMPLEFOC_DEBUG("MOT: sensor_direction==CW");
      sensor_direction = Direction::CW;
    }
    // check pole pair number
    pp_check_result = !(fabs(moved * pole_pairs - _2PI) > 0.5f); // 0.5f is arbitrary number it can be lower or higher!
    if (pp_check_result == false)
    {
      SIMPLEFOC_DEBUG("MOT: PP check: fail - estimated pp: ", _2PI / moved);
    }
    else
    {
      SIMPLEFOC_DEBUG("MOT: PP check: OK!");
    }
  }
  else
  {
    SIMPLEFOC_DEBUG("MOT: Skip dir calib.");
  }

  // zero electric angle not known
  if (!_isset(zero_electric_angle))
  {
    // align the electrical phases of the motor and sensor
    // set angle -90(270 = 3PI/2) degrees
    setPhaseVoltage(voltage_align, 0, _3PI_2);
    _delay(700);
    // read the sensor
    sensor->update();
    // get the current zero electric angle
    zero_electric_angle = 0;
    zero_electric_angle = electricalAngle();
    // zero_electric_angle =  _normalizeAngle(_electricalAngle(sensor_direction*sensor->getAngle(), pole_pairs));
    _delay(20);
    if (monitor_port)
    {
      SIMPLEFOC_DEBUG("MOT: Zero elec. angle: ", zero_electric_angle);
    }
    // stop everything
    setPhaseVoltage(0, 0, 0);
    _delay(200);
  }
  else
  {
    SIMPLEFOC_DEBUG("MOT: Skip offset calib.");
  }
  return exit_flag;
}

// Encoder alignment the absolute zero angle
// - to the index
int BLDCCustomMotor::absoluteZeroSearch()
{
  // sensor precision: this is all ok, as the search happens near the 0-angle, where the precision
  //                    of float is sufficient.
  SIMPLEFOC_DEBUG("MOT: Index search...");
  // search the absolute zero with small velocity
  float limit_vel = velocity_limit;
  float limit_volt = voltage_limit;
  velocity_limit = velocity_index_search;
  voltage_limit = voltage_sensor_align;
  shaft_angle = 0;
  while (sensor->needsSearch() && shaft_angle < _2PI)
  {
    angleOpenloop(1.5f * _2PI);
    // call important for some sensors not to loose count
    // not needed for the search
    sensor->update();
  }
  // disable motor
  setPhaseVoltage(0, 0, 0);
  // reinit the limits
  velocity_limit = limit_vel;
  voltage_limit = limit_volt;
  // check if the zero found
  if (monitor_port)
  {
    if (sensor->needsSearch())
    {
      SIMPLEFOC_DEBUG("MOT: Error: Not found!");
    }
    else
    {
      SIMPLEFOC_DEBUG("MOT: Success!");
    }
  }
  return !sensor->needsSearch();
}

// Iterative function looping FOC algorithm, setting Uq on the Motor
// The faster it can be run the better
// In this class, it has been stripped of the other modes
void BLDCCustomMotor::loopFOC()
{
  // update sensor - do this even in open-loop mode, as user may be switching between modes and we could lose track
  //                 of full rotations otherwise.
  if (sensor)
    sensor->update();

  // if disabled do nothing
  if (!enabled)
    return;

  // Needs the update() to be called first
  // This function will not have numerical issues because it uses Sensor::getMechanicalAngle()
  // which is in range 0-2PI
  electrical_angle = electricalAngle();

  if (!current_sense)
    return;
  // read dq currents
  current = current_sense->getFOCCurrents(electrical_angle);

  // filter values
  current.q = LPF_current_q(current.q);
  current.d = LPF_current_d(current.d);

  // calculate the phase voltages
  // float iq_ref = current_sp;
  // float id_ref = 0;
  // If we are over the base speed, activate CVCP - Constant voltage, constant power algorithm
  // if(shaft_velocity > base_speed)
  // {
  //   id_ref = (phi_pm/phase_inductance)*((shaft_velocity/base_speed) -1);
  //   iq_ref = (shaft_velocity/base_speed)*iq_ref;
  // }
  // Calculates d an q voltage and adds axis decoupling components
  // float electrical_velocity = shaft_velocity * pole_pairs;
  // voltage.q = constrain(PID_current_q(current_sp - current.q) + shaft_velocity * pole_pairs * phase_inductance * current.d, -voltage_limit, voltage_limit);
  // voltage.d = constrain(PID_current_d(-current.d) - electrical_velocity * phase_inductance * current.q + electrical_velocity * phi_pm, -voltage_limit, voltage_limit);

  // d voltage - lag compensation - TODO verify
  // if(_isset(phase_inductance)) voltage.d = _constrain( voltage.d - current_sp*sh,aft_velocity*pole_pairs*phase_inductance, -voltage_limit, voltage_limit);

      // calculate the phase voltages
  voltage.q = PID_current_q(current_sp - current.q);
  voltage.d = PID_current_d(-current.d);
      // d voltage - lag compensation - TODO verify
  //if(_isset(phase_inductance)) voltage.d = _constrain( voltage.d - current_sp*shaft_velocity*pole_pairs*phase_inductance, -voltage_limit, voltage_limit);
    
  // set the phase voltage - FOC heart function :)
  setPhaseVoltage(voltage.q, voltage.d, electrical_angle);
}

// Iterative function running outer loop of the FOC algorithm
// Behavior of this function is determined by the motor.controller variable
// It runs either angle, velocity or torque loop
// - needs to be called iteratively it is asynchronous function
// - if target is not set it uses motor.target value
void BLDCCustomMotor::move(float new_target)
{

  // set internal target variable
  if (_isset(new_target))
    target = new_target;

  // shaft angle/velocity need the update() to be called first
  // get shaft angle
  shaft_angle = shaftAngle(); // read value even if motor is disabled to keep the monitoring updated but not in openloop mode
  // get angular velocity
  shaft_velocity = shaftVelocity(); // read value even if motor is disabled to keep the monitoring updated

  // if disabled do nothing
  if (!enabled)
    return;

  // Sets current target
  current_sp = target;
}

// Method using FOC to set Uq and Ud to the motor at the optimal angle
// Function implementing Space Vector PWM and Sine PWM algorithms
//
// Function using sine approximation
// regular sin + cos ~300us    (no memory usage)
// approx  _sin + _cos ~110us  (400Byte ~ 20% of memory)
void BLDCCustomMotor::setPhaseVoltage(float Uq, float Ud, float angle_el)
{

  float center;
  int sector;
  float _ca, _sa;

  // Sinusoidal PWM modulation
  // Inverse Park + Clarke transformation
  _sincos(angle_el, &_sa, &_ca);

  // Inverse park transform
  Ualpha = _ca * Ud - _sa * Uq; // -sin(angle) * Uq;
  Ubeta = _sa * Ud + _ca * Uq;  //  cos(angle) * Uq;

  // Clarke transform
  Ua = Ualpha;
  Ub = -0.5f * Ualpha + _SQRT3_2 * Ubeta;
  Uc = -0.5f * Ualpha - _SQRT3_2 * Ubeta;

  center = driver->voltage_limit / 2;

  // TODO: Check if this is proper SVPWM,check how native stm32 does it

  // discussed here: https://community.simplefoc.com/t/embedded-world-2023-stm32-cordic-co-processor/3107/165?u=candas1
  // a bit more info here: https://microchipdeveloper.com/mct5001:which-zsm-is-best
  // Midpoint Clamp
  float Umin = min(Ua, min(Ub, Uc));
  float Umax = max(Ua, max(Ub, Uc));
  center -= (Umax + Umin) / 2;

  if (!modulation_centered)
  {
    float Umin = min(Ua, min(Ub, Uc));
    Ua -= Umin;
    Ub -= Umin;
    Uc -= Umin;
  }
  else
  {
    Ua += center;
    Ub += center;
    Uc += center;
  }

  // set the voltages in driver
  driver->setPwm(Ua, Ub, Uc);
}

// Function (iterative) generating open loop movement for target velocity
// - target_velocity - rad/s
// it uses voltage_limit variable
float BLDCCustomMotor::velocityOpenloop(float target_velocity)
{
  // get current timestamp
  unsigned long now_us = _micros();
  // calculate the sample time from last call
  float Ts = (now_us - open_loop_timestamp) * 1e-6f;
  // quick fix for strange cases (micros overflow + timestamp not defined)
  if (Ts <= 0 || Ts > 0.5f)
    Ts = 1e-3f;

  // calculate the necessary angle to achieve target velocity
  shaft_angle = _normalizeAngle(shaft_angle + target_velocity * Ts);
  // for display purposes
  shaft_velocity = target_velocity;

  // use voltage limit or current limit
  float Uq = voltage_limit;
  if (_isset(phase_resistance))
  {
    Uq = _constrain(current_limit * phase_resistance + fabs(voltage_bemf), -voltage_limit, voltage_limit);
    // recalculate the current
    current.q = (Uq - fabs(voltage_bemf)) / phase_resistance;
  }
  // set the maximal allowed voltage (voltage_limit) with the necessary angle
  setPhaseVoltage(Uq, 0, _electricalAngle(shaft_angle, pole_pairs));

  // save timestamp for next call
  open_loop_timestamp = now_us;

  return Uq;
}

// Function (iterative) generating open loop movement towards the target angle
// - target_angle - rad
// it uses voltage_limit and velocity_limit variables
float BLDCCustomMotor::angleOpenloop(float target_angle)
{
  // get current timestamp
  unsigned long now_us = _micros();
  // calculate the sample time from last call
  float Ts = (now_us - open_loop_timestamp) * 1e-6f;
  // quick fix for strange cases (micros overflow + timestamp not defined)
  if (Ts <= 0 || Ts > 0.5f)
    Ts = 1e-3f;

  // calculate the necessary angle to move from current position towards target angle
  // with maximal velocity (velocity_limit)
  // TODO sensor precision: this calculation is not numerically precise. The angle can grow to the point
  //                        where small position changes are no longer captured by the precision of floats
  //                        when the total position is large.
  if (abs(target_angle - shaft_angle) > abs(velocity_limit * Ts))
  {
    shaft_angle += _sign(target_angle - shaft_angle) * abs(velocity_limit) * Ts;
    shaft_velocity = velocity_limit;
  }
  else
  {
    shaft_angle = target_angle;
    shaft_velocity = 0;
  }

  // use voltage limit or current limit
  float Uq = voltage_limit;
  if (_isset(phase_resistance))
  {
    Uq = _constrain(current_limit * phase_resistance + fabs(voltage_bemf), -voltage_limit, voltage_limit);
    // recalculate the current
    current.q = (Uq - fabs(voltage_bemf)) / phase_resistance;
  }
  // set the maximal allowed voltage (voltage_limit) with the necessary angle
  // sensor precision: this calculation is OK due to the normalisation
  setPhaseVoltage(Uq, 0, _electricalAngle(_normalizeAngle(shaft_angle), pole_pairs));

  // save timestamp for next call
  open_loop_timestamp = now_us;

  return Uq;
}
