#include <Arduino.h>
#include <SimpleFOC.h>
#include <USBSerial.h>
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
// BLDC motor & driver instance
  // BLDC motor instance BLDCMotor(polepairs, (R), (KV))
    BLDCMotor motor = BLDCMotor(88,0.098, 10.5);
    BLDCDriver6PWM driver = BLDCDriver6PWM(PA8, PB13, PA9, PB14, PA10, PB15);
// Motor Instantiated

// Hall sensor instance
  HallSensor sensor = HallSensor(PA15, PB3, PB4, 88);
// Sensor Hall pins and poles configurated

// Interrupt routine intialisation
  void doA(){sensor.handleA();}
  void doB(){sensor.handleB();}
  void doC(){sensor.handleC();}
// Hardware interrupts instantiated (all the code are running in the library)

// velocity set point variable (rad/s)
  float target_velocity = 0;
// global variabe to control velocity initiated

PIDController& pid_d = motor.PID_current_d;
PIDController& pid_q = motor.PID_current_q;


//  LowsideCurrentSense(shunt_resistance, gain, adc_a, adc_b, adc_c)
LowsideCurrentSense current_sense = LowsideCurrentSense(0.02, 10, PA7, PB0, PB1);
// low side current sensors (for all phases) instantiated 

// instantiate the commander- For this code, we aren't controll via serial, so this part are commented
Commander command = Commander(Serial);
  void doTarget(char* cmd) { command.scalar(&target_velocity, cmd); }
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
    Serial.println("Driver Enable ");
    pinMode(PA4, OUTPUT); // liga o anable do driver
    digitalWrite(PA4,HIGH); 
    delay(500);
// Motor Driver Enable to work

// Current sense driver calibration
    Serial.println("Driver Current Calibration, please wait... ");
    pinMode(PB8, OUTPUT);
    digitalWrite(PB8,LOW);
    delay(100);
    digitalWrite(PB8,HIGH); 
    delay(100);
    Serial.println("Driver Current calibrated!");
    digitalWrite(PB8,LOW);
    delay(500);
// Driver current sense calibrated

    // Blink x2
    Serial.println("Finishing some things...");     
    pinMode(PC13, OUTPUT);
    digitalWrite(PC13,LOW);
    delay(1000);
    digitalWrite(PC13,HIGH); 
    delay(1000);
    digitalWrite(PC13,LOW);
    delay(1000);
    Serial.println("So far, so good!");
    digitalWrite(PC13,HIGH); 
// Visual confirmation of all custom hardware initialization set properly
// Hardware Custom Configs Finished

  
  
  
// Library Start
// initialize encoder sensor hardware
  sensor.init();
  sensor.enableInterrupts(doA, doB, doC); 
// Hall sensor initialized and motor start position founded
  
// link the Hall sensor to motor 
  motor.linkSensor(&sensor);
// Sensor (hall) linked to motor

// driver configurations
  // PWM Frequency Configuration [Hz] - Between 25 kHz and 50 kHz
    driver.pwm_frequency = 25000;
  // Pwm work Frequency configurated
    
  // power supply voltage [V]
    driver.voltage_power_supply = 13;  
    driver.voltage_limit = 13;
  // Driver voltages set
    
  // dead_zone [0,1] - default 0.02 - 2%
    driver.dead_zone = 0.05;
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
    //motor.sensor_direction = Direction::CW;// should be implemented for skipping align routine
    motor.voltage_sensor_align = 1.5;  
  // motor Aligned and direction of rotation configuration completed
  
  // choose FOC modulation type 
    //motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
    motor.foc_modulation = FOCModulationType::SinePWM; 
  // Modulation configurated
  
  // Setting motion control loop to be used
      // set torque mode:
      motor.torque_controller = TorqueControlType::foc_current;
      motor.controller = MotionControlType::torque;
      //motor.controller = MotionControlType::velocity;
      //motor.controller = MotionControlType::angle
  // Finished configurations of control. 

  // contoller configuration (default parameters in defaults.h)
    // foc current control parameters (Arduino UNO/Mega)
      //First PID 
        motor.PID_current_q.P = 5;
        motor.PID_current_q.I= 500;
        motor.LPF_current_q.Tf = 0.01;
      //Second PID  
        motor.PID_current_d.P= 5;
        motor.PID_current_d.I = 500;
        motor.LPF_current_d.Tf = 0.01;
    // PID Configurations set (still needs to be refineted)
      
    // ** Setting the Circuit limits **
      motor.voltage_limit = 32; // Volts - default driver.voltage_limit     - Hard limit on output voltage, in volts. Effectively limits PWM duty cycle proportionally to power supply voltage.
      //motor.current_limit = 6.0; // Amps - default 0.2Amps  
      motor.motion_downsample = 100;
      motor.velocity_limit = 26.9;    // ~ 46,23 rad/s ≈ 38 km/h   
    // ** Circuit Limits configurated **
        
    // jerk control using voltage voltage ramp (default value is 300 volts per sec  ~ 0.3V per millisecond)
      //motor.PID_velocity.output_ramp = 100;

    // velocity low pass filtering time constant
      //motor.LPF_velocity.Tf = 0.02f; // the lower the less filtered 
  // Controller configurations finished
  
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
  //  current_sense.skip_align = true; // when debbuged uncommented for skip align procedure (safety reasons)
  // Motor and Current sense linked
    
  // Start FOC
    motor.initFOC();

  // control procedure via Serial 
    command.add('T', doTarget, "target Current");
    command.add('C',onPid,"my pid");
    Serial.println(F("Motor ready."));
    Serial.println(F("Set the target voltage using serial terminal:"));
    //_delay(1000);


// Final Configurarion conclude
}


void loop() {
  motor.loopFOC();

 
  // this function can be run at much lower frequency than loopFOC() function
  // You can also use motor.move() and set the motor.target in the code
  
  motor.move(target_velocity);

  // user communication
  command.run();   
  
    i = i+1;
    if (i == 1000)
    {    
      if (digitalRead(PB10)== LOW)
      {
        digitalWrite(PA4,LOW);
      } 
      i=0;      
      float error = motor.target/motor.current.q;
      Serial.print(motor.target); // milli Amps
      Serial.print(" , ");
      Serial.print (motor.current.q); // milli Amps
      Serial.print(" , ");
      Serial.print (motor.current.d); // milli Amps
      Serial.print(" , ");
      Serial.println (error);
    } 
/* -- Verificar o tempo de loop 
    unsigned long startTime = micros();
    // Código da função loop
    unsigned long endTime = micros();
    unsigned long duration = endTime - startTime;
    Serial.println(duration);  // Exibe o tempo em microsegundos
*/  
}
