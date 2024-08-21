#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>
#include <encoders/smoothing/SmoothingSensor.h>

/*
ESC_BLDC_2024
By Noturno and Bail
Código Voltado para o controle de um motor BLDC ou PMSM utilizando Simple Field Oriented Controlling (Simple FOC), para veículo Prototype destinado a concorrer na SEM Brasil 2024 

Código seguindo diretrizes da Biblioteca, disponível em:
https://docs.simplefoc.com/code

LEMBRETE DE SEQUENCIA DE FASES:
FASE A - AZUL
FASE B - VERDE
FASE C - AMARELO
 
 */

int i=0;

#define DEBUG_ON 1
#define DEBUG_OFF 0
byte debugMode = DEBUG_OFF;

#define DBGLN(...) debugMode == DEBUG_ON ? Serial.println(__VA_ARGS__) : NULL
#define DBG(...) debugMode == DEBUG_ON ? Serial.print(__VA_ARGS__) : NULL


const int POWER_SUPPLY = 42;
const int DRIVER_LIMIT = 42;
const int MOTOR_LIMIT = 41;

const int PWM_FREQ = 20000;
const int MOTOR_KV = 10.5;
const int TURNOFF_SPEED = 50; // rad/s
const int TURNON_SPEED = 13; // rad/s
const int SPEED_CYCLES = 250;
const float ACCEL_RAMP = 0.001;

bool MAX_VEL_TRIGGERED = false;
float target_torque = 0;
float offset_angle = 3.14f;
// BLDC motor & driver instance
  // BLDC motor instance BLDCMotor(polepairs, (R), (KV))
BLDCMotor motor = BLDCMotor(88,0.098, MOTOR_KV);
BLDCDriver6PWM driver = BLDCDriver6PWM(PA8, PB13, PA9, PB14, PA10, PB15);
// Motor Instantiated

// Hall sensor instance
HallSensor sensor = HallSensor(PA15, PB3, PB4, 88);
SmoothingSensor smooth = SmoothingSensor(sensor, motor);  
// Sensor Hall pins and poles configurated

// Interrupt routine intialisation
  void doA(){sensor.handleA();}
  void doB(){sensor.handleB();}
  void doC(){sensor.handleC();}
// Hardware interrupts instantiated (all the code are running in the library)

// global variabe to control velocity initiated

PIDController& pid_d = motor.PID_current_d;
PIDController& pid_q = motor.PID_current_q;


//  LowsideCurrentSense(shunt_resistance, gain, adc_a, adc_b, adc_c)
  LowsideCurrentSense current_sense = LowsideCurrentSense(0.002, 40, PA7, PB0, PB1);
// low side current sensors (for all phases) instantiated 

// instantiate the commander- For this code, we aren't controll via serial, so this part are commented
  Commander command = Commander(Serial);
  void doTarget(char* cmd) { command.scalar(&target_torque, cmd); }
  void doOffset(char* cmd) { command.scalar(&offset_angle, cmd); }
  void doMotor(char* cmd) { command.motor(&motor, cmd); }
  void onPid(char* cmd){ command.pid(&pid_d,cmd); command.pid(&pid_q,cmd);}
// To control velocity of the motor via Serial Monitor/Plotter

void setup() { 
  delay(3000); // delay para possibilitar a verificação do inicio das configs via serial plott
  
// use monitoring with serial 
  Serial.begin(115200);

// enable more verbose output from library for debugging
  SimpleFOCDebug::enable(&Serial);


// Custom Start configs
 
// Tun on Motor Driver Enable
    SIMPLEFOC_DEBUG("Driver Enable ");
    pinMode(PA4, OUTPUT); // liga o anable do driver
    digitalWrite(PA4,HIGH); 

    delay(500);
// Motor Driver Enable to work
    //nFault PIN
    pinMode(PB10, INPUT); // Set the pin to input mode
    attachInterrupt(digitalPinToInterrupt(PB10), ChecknFaultProtection, FALLING); // Attach interrupt
//Button pin
    pinMode(PA6, INPUT);

// Current sense driver calibration
    SIMPLEFOC_DEBUG("Driver Current Calibration, please wait... ");
    pinMode(PB8, OUTPUT);
    digitalWrite(PB8,LOW);
    delay(100);
    digitalWrite(PB8,HIGH); 
    delay(100);
    SIMPLEFOC_DEBUG("Driver Current calibrated!");
    digitalWrite(PB8,LOW);
    delay(500);
// Driver current sense calibrated

    // Blink x2
    SIMPLEFOC_DEBUG("Finishing some things...");     
    pinMode(PC13, OUTPUT);
    digitalWrite(PC13,LOW);
    delay(1000);
    digitalWrite(PC13,HIGH); 
    delay(1000);
    digitalWrite(PC13,LOW);
    delay(1000);
    SIMPLEFOC_DEBUG("So far, so good!");
    digitalWrite(PC13,HIGH); 
// Visual confirmation of all custom hardware initialization set properly
// Hardware Custom Configs Finished

  
  
  
// Library Start
// initialize encoder sensor hardware
  sensor.init();
  sensor.enableInterrupts(doA, doB, doC); 
// Hall sensor initialized and motor start position founded
  
// link the Hall sensor to motor 
  motor.linkSensor(&smooth);
// Sensor (hall) linked to motor

// driver configurations
  // PWM Frequency Configuration [Hz] - Between 25 kHz and 50 kHz
    driver.pwm_frequency = PWM_FREQ;
  // Pwm work Frequency configurated
    
  // power supply voltage [V]
    driver.voltage_power_supply = POWER_SUPPLY;  
    driver.voltage_limit = DRIVER_LIMIT;
  // Driver voltages set
    
  // dead_zone [0,1] - default 0.02 - 2%
    driver.dead_zone = 0.025;
  // Dead time working configured
  
  // Diver initialization Routine
    Serial.print("Driver init ");
    driver.init(); 
    motor.linkDriver(&driver);
  // driver initialized and linked to motor
// Driver configurations finished

// Circuit configurations 
  // Link the driver with the current sense
    current_sense.linkDriver(&driver);
    current_sense.init();  
  // Current sense linked to motor and initialized
  
  //  Motor directions and alignment configurations     
    motor.sensor_direction = Direction::CW;// should be implemented for skipping align routine
    motor.zero_electric_angle = offset_angle;  
  // motor Aligned and direction of rotation configuration completed
  
  // choose FOC modulation type 
    //motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
    motor.foc_modulation = FOCModulationType::SpaceVectorPWM; 
    motor.torque_controller = TorqueControlType::foc_current;
    motor.controller = MotionControlType::torque;
  // Modulation configurated
  
    motor.PID_current_q.P = 0.25;
    motor.PID_current_q.I= 10;
    motor.LPF_current_q.Tf = 0.01;
    motor.PID_current_q.limit = POWER_SUPPLY;
    motor.PID_current_q.output_ramp = 250;
  //Second PID  
    motor.PID_current_d.P= 0.25;
    motor.PID_current_d.I = 10;
    motor.LPF_current_d.Tf = 0.01;
    motor.PID_current_q.limit = POWER_SUPPLY;
    motor.PID_current_d.output_ramp = 250;
    // PID Configurations set (still needs to be refineted)
      
    // ** Setting the Circuit limits **
      motor.voltage_limit = MOTOR_LIMIT; // Volts - default driver.voltage_limit     - Hard limit on output voltage, in volts. Effectively limits PWM duty cycle proportionally to power supply voltage.
      motor.motion_downsample = 100;
    // ** Circuit Limits configurated **
  
  // Configurating Serial to control motor
    
    motor.useMonitoring(Serial);
 //   motor.monitor_variables = _MON_TARGET | _MON_VEL; 
  //display variables
   //SimpleFOCDebug::enable();
   //motor.monitor_variables = _MON_TARGET | _MON_VEL | _MON_ANGLE; 
  // Choosed infos to plot and debbug
        
  // downsampling
  //  motor.monitor_downsample = 100; // default 10
// Circuit configurations finished

// Final Configurations of Library
  // initialize motor
    motor.init();
  // motor initialized

  // Link current sense to motor
    motor.linkCurrentSense(&current_sense);
    current_sense.skip_align = true; // when debbuged uncommented for skip align procedure (safety reasons)
  // Motor and Current sense linked
    
  // Start FOC
    motor.initFOC();

  // control procedure via Serial 
    command.add('T', doTarget, "target Current");
    command.add('O', doOffset, "offset");
    command.add('C',onPid,"my pid");

    SIMPLEFOC_DEBUG("Motor ready.");
    SIMPLEFOC_DEBUG("Set the target voltage using serial terminal:");

// Final Configurarion conclude
}


void loop() {

  motor.loopFOC();
  // user communication
  
    i = i+1;
    if (i == SPEED_CYCLES)
    {
     // ChecknFaultProtection();
      int button_pressed = digitalRead(PA6);

      // if(button_pressed == HIGH){
      //   target_torque = constrain(target_torque + ACCEL_RAMP, 0, 1);
      //   SIMPLEFOC_DEBUG("pressed");
      // }
      // else {
      //   target_torque = constrain(target_torque - ACCEL_RAMP*5, 0, 1);
      // }
      //SIMPLEFOC_DEBUG("TARGET TORQUE:", target_torque);
      float error = motor.target - motor.current.q;
      motor.zero_electric_angle = offset_angle;
      if(MAX_VEL_TRIGGERED)
      {
        if(motor.shaft_velocity < TURNON_SPEED)
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
        if(motor.shaft_velocity > TURNOFF_SPEED)
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
      
      i=0;      
      DBG(motor.target); // milli Amps
      DBG(" , ");
      DBG(motor.current.q); // milli Amps
      DBG(" , ");
      DBG(motor.shaft_velocity);
      DBG(" , ");
      DBGLN(motor.electrical_angle);
      
      command.run();   
    }  
}

void Coast()
{
  if(MAX_VEL_TRIGGERED)
  {
    if(motor.shaft_velocity < TURNON_SPEED)
    {
      SIMPLEFOC_DEBUG("RE-ENABLING");
      motor.move(target_torque);
      MAX_VEL_TRIGGERED = false;
    }
  }
  else
  {
    if(motor.shaft_velocity > TURNOFF_SPEED)
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
    
/* -- Verificar o tempo de loop 
    unsigned long startTime = micros();
    // Código da função loop
    unsigned long endTime = micros();
    unsigned long duration = endTime - startTime;
    Serial.println(duration);  // Exibe o tempo em microsegundos
*/  
}

void ChecknFaultProtection()
{
     if (digitalRead(PB10)== LOW)
      {
        digitalWrite(PA4,LOW);
        SIMPLEFOC_DEBUG("Driver Fault Detected - Shutting down");
      }
}
