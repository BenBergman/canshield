
// Controlling a servo position using a potentiometer (variable resistor) 
// by Michal Rinott <http://people.interaction-ivrea.it/m.rinott> 

// Servo for throttle - 180 deg is no throttle tension
// hall effect sensor for engine RPM - 24 teeth
// output to starter

// motor control is main loop
// CAN polling should be an interrupt

#include <Servo.h> 
#include <PID_v1.h>
#include <Bounce.h>


/**************** Constants ****************/

// PID parameters
const double Kp = 0.02;
const double Ki = 0.1;
const double Kd = 0.00;

// Engine RPM parameters
const int MAX_RPM = 4000;
const int MIN_RPM = 1200;
const int TARGET_RPM = 5000;

// Throttle positions
const int ZERO_THROTTLE = 179;
const int MAX_THROTTLE = 0;
const int PID_DIRECTION = REVERSE; // if the zero point is 179, this must be REVERSE, but if 0, use DIRECT

const byte SOC_MIN = 85;   // if SOC below 85%, start the engine
const byte SOC_MAX = 90;   // if SOC above 90%, refuse to start engine

// State machine states
const int START =                  0;
const int ENGINE_NOT_RUNNING =     1;
const int HIGH_SOC =               2;
const int ENGINE_REQUESTED =       3;
const int START_ENGINE =           4;
const int START_ENGINE_DELAY =     5;
const int START_ENGINE_RPM =       6;
const int START_ENGINE_REATTEMPT = 7;
const int RUN_ENGINE_DELAY =       8;
const int RUN_ENGINE_RPM =         9;
const int SUSTAIN_ENGINE =        10;
const int KILL_ENGINE =           11;
const int KILL_ENGINE_WAIT =      12;
const int KILL_ENGINE_FOREVER =   13;

// State machine thresholds
const int STARTER_TIMER_THRESHOLD = 5000; // in ms; wait before checking if engine has started
const int STARTER_RPM_THRESHOLD = 1300; // X RPM; must reach X RPM before cutting starter
const int STARTER_ATTEMPTS_MAX = 5;
const int STARTER_TIMER_RUN_THRESHOLD = 2000; // in ms; wait before assuming engine started
const int RUN_RPM_DIFFERENCE = 200; // X RPM; allow engine to drop X RPM from threshold
const int SUSTAIN_RPM_THRESHOLD = 500; // X RPM; if engine X RPM below desired, kill engine
const long KILL_ENGINE_TIMER = 5000; // X ms; keep the kill switch engaged this long

// Pin assignments
const int GENERATOR_REQUEST_PIN = 18;
const int KILL_SWITCH =           56;
const int STARTER_PIN =           55;
const int SERVO_PIN =             54;
const int FAKE_SERVO =             5; // for testing with motor emulator
const int HALL_EFFECT_INTERRUPT =  4; // interrupt 4 is digital pin 19
const int CRIO_START_STOP_INTERRUPT = 5; // interrupt 5 is digital pin 18
const int START_BUTTON =          57;


const int KILL_SWITCH_DEAD = HIGH;
const int KILL_SWITCH_LIVE = LOW;

const int DEBOUNCE_TIME = 5; // ms

/************ End of Constants ************/




#define SIZE 100

char temp[SIZE]; // temp data is NEVER considered valid; just used to appease methods






int pwm;

Bounce startButton = Bounce(START_BUTTON, DEBOUNCE_TIME);
 
Servo myServo;  // create servo object to control a servo 

//PID variables
double Setpoint;  // This is the desired RPM of the engine
double Input;     // This is the current RPM of the engine
double Output;    // This is the value sent to the servo

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, PID_DIRECTION);  // create PID for engine RPM control; K inputs are P, I, and D parameters.
 
int potpin = 2;  // analog pin used to connect the potentiometer
int val;    // variable to read the value from the analog pin 

int teeth = 24; // 24 teeth on hall effect sensor wheel

volatile int rpmcount;
volatile unsigned long rpm;
unsigned long timeold;

unsigned long rpmold;

unsigned long denom;
unsigned long numer;

unsigned long rpmTimer;
const unsigned long rpmTimeout = 300;

unsigned int state;
int starterAttempts;
unsigned long starterTimer;

int generatorRequest;
boolean sustainEngine;
volatile boolean startRequested = false;
volatile boolean stopRequested = false;
volatile boolean killEngine = true;

unsigned long killEngineTimer = 0;

// values to send to analog outputs
// BMS
byte maxCellV;
byte minCellV;
byte soc = 87;
// DC/DC
byte generatorVin;
byte dcdcVout;
byte dcdcIout;

 
void setup() 
{ 
  pinMode(GENERATOR_REQUEST_PIN, INPUT);
  pinMode(KILL_SWITCH, OUTPUT);
  pinMode(STARTER_PIN, OUTPUT);
  pinMode(SERVO_PIN, OUTPUT);
  pinMode(FAKE_SERVO, OUTPUT);
  pinMode(START_BUTTON, INPUT);

  digitalWrite(STARTER_PIN, LOW);
  digitalWrite(KILL_SWITCH, KILL_SWITCH_DEAD);
      
  myServo.attach(SERVO_PIN);  // attaches the servo on pin 9 to the servo object 

  // initialize PID
  Input = MAX_THROTTLE;
  myServo.write(ZERO_THROTTLE);
  analogWrite(FAKE_SERVO, ZERO_THROTTLE);
  Setpoint = 0;

  
  // limit the Output range to that which the servo can achieve
  //myPID.SetOutputLimits(ZERO_THROTTLE, MAX_THROTTLE);
  myPID.SetOutputLimits(0, 179);
  myPID.SetMode(AUTOMATIC);
  
  Serial.begin(115200);
  Serial.flush();

  attachInterrupt(HALL_EFFECT_INTERRUPT, rpm_fun, RISING); 
  attachInterrupt(CRIO_START_STOP_INTERRUPT, crioStartStop, RISING);

  rpmcount = 0;
  rpm = 0;
  rpmold = 0;
  timeold = 0;

  rpmTimer = millis();

  sustainEngine = false;

  Serial.println("Start main loop");
} 




void rpm_fun()
{
  rpmcount++;
}




void crioStartStop()
{
  if (startRequested == false && stopRequested == false)
  {
    startRequested = true;
  }
  else
  {
    if (stopRequested == false) state = KILL_ENGINE;
    startRequested = false;
    stopRequested = true;
  }
}





 
void loop() 
{ 
  startButton.update();
  if (startButton.risingEdge()) // assumes unpressed in LOW
  {
    crioStartStop();
  }
  

  switch (state)
  {


    case START:
      Serial.println("START");

      /**/
      checkRPM();
      if (rpm >= 900) 
      {
        startRequested = true;
        state = SUSTAIN_ENGINE;  
      }
      else /**/ state = ENGINE_NOT_RUNNING;
      break;
    


    case ENGINE_NOT_RUNNING: 
      Serial.println("ENGINE_NOT_RUNNING");

      if (soc < SOC_MIN) state = START_ENGINE;
      else state = HIGH_SOC;
      break;
    


    case HIGH_SOC:
      Serial.println("HIGH_SOC");

      //generatorRequest = digitalRead(GENERATOR_REQUEST_PIN);
      //if (generatorRequest == HIGH) state = ENGINE_REQUESTED;
      if (startRequested == true) state = ENGINE_REQUESTED;
      else state = START;
      break;
    


    case ENGINE_REQUESTED:
      Serial.println("ENGINE_REQUESTED");

      if (soc > SOC_MAX) state = START;
      else state = START_ENGINE;
      break;
    


    case START_ENGINE:
      Serial.println("START_ENGINE");

      starterAttempts = 0;
      digitalWrite(KILL_SWITCH, KILL_SWITCH_LIVE);
      digitalWrite(STARTER_PIN, HIGH);
      starterTimer = millis();
      state = START_ENGINE_DELAY;
      break;
    


    case START_ENGINE_DELAY:
      Serial.println("START_ENGINE_DELAY");

      // Make sure any state that points here first starts the timer!
      if (millis() - starterTimer > STARTER_TIMER_THRESHOLD) state = START_ENGINE_RPM;
      break;
      


    case START_ENGINE_RPM:
      Serial.println("START_ENGINE_RPM");

      // CHECK ENGINE RPM
      checkRPM();
      if (rpm > STARTER_RPM_THRESHOLD) 
      {
        digitalWrite(STARTER_PIN, LOW);
        starterTimer = millis();
        state = RUN_ENGINE_DELAY;
      }
      else 
      {
        starterTimer = millis();
        state = START_ENGINE_REATTEMPT;
      }
      break;



    case START_ENGINE_REATTEMPT:
      Serial.println("START_ENGINE_REATTEMPT");

      // Might move to START_ENGINE_DELAY but does not start timer.
      // States prior to START_ENGINE_REATTEMPT must start timer!
      starterAttempts ++;
      if (starterAttempts > STARTER_ATTEMPTS_MAX) 
      {
        digitalWrite(STARTER_PIN, LOW);
        //state = START;
        state = KILL_ENGINE_FOREVER; // assumed that if engine wouldn't start, we should abandon trying
      }
      else state = START_ENGINE_DELAY;
      break;



    case RUN_ENGINE_DELAY:
      Serial.println("RUN_ENGINE_DELAY");

      // Make sure any state that points here first starts the timer!
      if (millis() - starterTimer > STARTER_TIMER_RUN_THRESHOLD) state = RUN_ENGINE_RPM;
      break;



    case RUN_ENGINE_RPM:
      Serial.println("RUN_ENGINE_RPM");

      // CHECK ENGINE RPM
      checkRPM();
      if (rpm > (STARTER_RPM_THRESHOLD - RUN_RPM_DIFFERENCE)) state = SUSTAIN_ENGINE;
      else state = START_ENGINE_REATTEMPT;
      break;



    case SUSTAIN_ENGINE:
      //Serial.println("SUSTAIN_ENGINE");

      // START DC/DC (might need to be done before getting into this state)
      // slowly increase RPM setpoint to 6000
      //    if ramping up, starting from roughly 1300 and going to 6000, could setup timer and raise setpoint by the # of ms since started until hitting 6000
      // kill engine if RPM more than <threshold> below setpoint???????


      // send DC/DC ON command
      turnDCDCOn();

      // how we get the kill signal needs to be investigated
      //generatorRequest = digitalRead(GENERATOR_REQUEST_PIN);
      //if (generatorRequest == LOW) 
      if (stopRequested == true) 
      {
        if (soc < SOC_MIN)
        {
          // assumed that driver will not want any charging after this point
          state = KILL_ENGINE_FOREVER;
        }
        else
        {
          state = KILL_ENGINE;
        }
        break;
      }

      sustainEngine = true;

      // set the setpoint

      /******** Set my potentiometer ********
      val = analogRead(potpin);            // reads the value of the potentiometer (value between 0 and 1023) 
      Setpoint = map(val, 0, 1023, MIN_RPM, MAX_RPM);     // scale it to use it with the servo (value between 0 and 180) 
      // ^ using the potentiometer to change the setpoint
      /**************************************/

      /************* Hard coded *************/
      Setpoint = TARGET_RPM;
      /**************************************/

      checkRPM();

      //if (rpm < Setpoint - SUSTAIN_RPM_THRESHOLD) state = KILL_ENGINE;
      break;



    case KILL_ENGINE:
      Serial.println("KILL_ENGINE");

      // STOP DC/DC
      // kill engine
      
      sustainEngine = false;
      // signal engine kill
      digitalWrite(KILL_SWITCH, KILL_SWITCH_DEAD);
      // send DC/DC OFF command
      turnDCDCOff();

      if (soc < SOC_MIN)
      {
        state = KILL_ENGINE_FOREVER;
        break;
      }

      
      killEngineTimer = millis();
      state = KILL_ENGINE_WAIT;
      break;


      
    case KILL_ENGINE_WAIT:
      Serial.println("KILL_ENGINE_WAIT");

      // stops after timout AND stopped engine (assume engine dead if rpm below 30
      if (millis() - killEngineTimer > KILL_ENGINE_TIMER && rpm <= 30)
      {
        if (soc < SOC_MIN) state = KILL_ENGINE_FOREVER;
        else 
        {
          stopRequested == false;
          state = START;
        }
      }
      break;


      
    case KILL_ENGINE_FOREVER:
      sustainEngine = false;
      digitalWrite(KILL_SWITCH, KILL_SWITCH_DEAD);
      turnDCDCOff();
      break;



    default:
      Serial.println("default");
      break;
  }

  delay(15); // waits for the servo to get there   <---- remove this once integrated with DAQ
} 


void checkRPM()
{
  if (rpmcount >= 24)
  {
    // Update RPM every 20 counts, increase this for better RPM resolution,
    // decrease for faster update (adjust rpm calc as needed)

    Serial.print("Pot: ");
    Serial.print(val, DEC);
    Serial.print("  rpmcount: ");
    Serial.print(rpmcount, DEC);

    // RPM = count / teeth / T
    numer = rpmcount * 60000; // 60 s/min * 1000 ms/s
    rpmcount = 0;
    denom = teeth * (millis() - timeold);
    timeold = millis();
    rpm = numer / denom;
    rpmold = rpm;
    Input = rpm;

    Serial.print("  RPM: ");
    Serial.print(rpm, DEC);
    Serial.print("  PID:");
    Serial.print("  SetPoint: ");
    Serial.print(Setpoint, DEC);
    Serial.print("  In: ");
    Serial.print(Input, DEC);
    Serial.print("  Out: ");
    Serial.print(Output, DEC);
    Serial.println();

    if (sustainEngine == true)
    {
      myPID.Compute();
      myServo.write(Output); // sets the servo position according to the scaled value; 0-179 deg

      /* used for motor emulator */
      pwm = Output; 
      analogWrite(FAKE_SERVO, pwm);
      /**/
    }
    else
    {
      myServo.write(ZERO_THROTTLE); // cuts throttle
      /* used for motor emulator */
      pwm = Output; 
      analogWrite(FAKE_SERVO, ZERO_THROTTLE);
      /**/
    }
    rpmTimer = millis();
  }
  else if (millis() - rpmTimer > rpmTimeout)
  {
    // no significant rotations so assuming not spinning
    rpm = 0;
  }
}

void turnDCDCOn()
{
  getDCDC();
  send_command("AT CP 0C\r", temp);
  send_command("AT SH 5F 82 81\r", temp);
  Serial2.flush();
  send_command("FF 20\r", temp); // second byte is power setpoint (in deciwatts)
}

void turnDCDCOff()
{
  getDCDC();
  send_command("AT CP 0C\r", temp);
  send_command("AT SH 5F 82 81\r", temp);
  Serial2.flush();
  send_command("00 00\r", temp); // second byte is power setpoint (in deciwatts)
}










//sends a CAN command
int send_command(char *cmd, char *result)
{
  //Serial2.flush(); 
  Serial2.print(cmd);
  delay(15);
  return 1;//read_data(result);
}

void getDCDC()
{
  // prepare elm to use long CAN addresses
  send_command("AT PP 2C SV 40\r", temp);
  send_command("AT PP 2C ON\r", temp);
  send_command("AT WS\r", temp);
  Serial2.flush();
  delay(60); // takes some time to reset ELM but this might not be needed once motor control integrated
}

