#include <HardwareSerial.h>
#include <TMCStepper.h>


#define DIAG_PIN         19          // STALL motor 2
#define EN_PIN           5          // Enable
#define DIR_PIN          23          // Direction
#define STEP_PIN         14          // Step
#define SERIAL_PORT_2      Serial2    // TMC2208/TMC2224 HardwareSerial port
#define DRIVER_ADDRESS   0b00       // TMC2209 Driver address according to MS1 and MS2
#define R_SENSE          0.11f      // E_SENSE for current calc.  
#define STALL_VALUE      100        // [0..255]

hw_timer_t * timer1 = NULL;
TMC2209Stepper driver2(&SERIAL_PORT_2, R_SENSE , DRIVER_ADDRESS );


void IRAM_ATTR onTimer() {

  digitalWrite(STEP_PIN, !digitalRead(STEP_PIN));
} 

void setup() {
  Serial.begin(250000);         // Init serial port and set baudrate
  while(!Serial);               // Wait for serial port to connect
  Serial.println("\nStart...");
  SERIAL_PORT_2.begin(115200);
  
  pinMode(DIAG_PIN ,INPUT);
  pinMode(EN_PIN ,OUTPUT);
  pinMode(STEP_PIN ,OUTPUT);
  pinMode(DIR_PIN ,OUTPUT);

  digitalWrite(EN_PIN ,LOW);
  digitalWrite(DIR_PIN ,LOW);

  //Begin Driver
  driver2.begin();

  // Sets the slow decay time (off time) [1... 15]. This setting also limits
  // the maximum chopper frequency. For operation with StealthChop,
  // this parameter is not used, but it is required to enable the motor.
  // In case of operation with StealthChop only, any setting is OK.
  driver2.toff(4);

  // Comparator blank time. This time needs to safely cover the switching
  // event and the duration of the ringing on the sense resistor. For most
  // applications, a setting of 16 or 24 is good. For highly capacitive
  // loads, a setting of 32 or 40 will be required.
  driver2.blank_time(24);
  
  driver2.rms_current(500); 
  driver2.microsteps(16);
  
  // Lower threshold velocity for switching on smart energy CoolStep and StallGuard to DIAG output
  driver2.TCOOLTHRS(0xFFFFF); // 20bit max

  // CoolStep lower threshold [0... 15].
  // If SG_RESULT goes below this threshold, CoolStep increases the current to both coils.
  // 0: disable CoolStep
  driver2.semin(0);

  // CoolStep upper threshold [0... 15].
  // If SG is sampled equal to or above this threshold enough times,
  // CoolStep decreases the current to both coils.
  driver2.semax(2);
  
  driver2.shaft(false);

  // Sets the number of StallGuard2 readings above the upper threshold necessary
  // for each current decrement of the motor current.
  driver2.sedn(0b01);

  // StallGuard4 threshold [0... 255] level for stall detection. It compensates for
  // motor specific characteristics and controls sensitivity. A higher value gives a higher
  // sensitivity. A higher value makes StallGuard4 more sensitive and requires less torque to
  // indicate a stall. The double of this value is compared to SG_RESULT.
  // The stall output becomes active if SG_RESULT fall below this value.
  driver2.SGTHRS(STALL_VALUE);

  //The step pulse is triggered as an interrupt
  activate_interrupt();
}

void loop() {
 static uint32_t last_time=0;
 uint32_t ms = millis();
 if((ms-last_time) > 100) { //run every 0.1s
    last_time = ms;

    Serial.print("0 ");
    Serial.print(driver2.SG_RESULT(), DEC);
    Serial.print(" ");
    Serial.println(driver2.cs2rms(driver2.cs_actual()), DEC);
  }
}

void activate_interrupt(){
  {
    cli();//stop interrupts
    timer1 = timerBegin(3, 8,true); 
    timerAttachInterrupt(timer1, &onTimer, true); //link interrupt with function onTimer
    timerAlarmWrite(timer1, 2000, true); //This value can be changed if we want to change speed of stepper. Increasing speeds gives more stable results
    timerAlarmEnable(timer1);    //Enable timer        
    sei();//allow interrupts
  }
}
