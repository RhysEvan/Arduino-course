/*
  Op3Mech controller

  Designed to be used with the CNC shield V3.

  Created 26/07/2020

  By Jona Gladines
  For Op3Mech
*/

#include <SoftwareSerial.h>                         //libraries for the processing of the serial command and to controll the stepper motors
#include <SerialCommand.h>
#include <AccelStepper.h>

SerialCommand SCmd;                                 // The SerialCommand object

AccelStepper steppers[4] = {
  AccelStepper(AccelStepper::DRIVER, 54, 55),
  AccelStepper(AccelStepper::DRIVER, 60, 61),
  AccelStepper(AccelStepper::DRIVER, 46, 48)
};

int stepperEnablePin = 8;                           //pin to enable stepper drivers on the CNC shield,must be tied low to enable drivers
int uddir = 1;
unsigned long lastMillis;
bool b_move_complete = true;
const byte limitSwitch_x = 3; //pin for the microswitch using attachInterrupt()
const byte limitSwitch_y = 14; //pin for the microswitch using attachInterrupt()
const byte limitSwitch_z = 18;

bool switchFlipped = false; //stores the status for flipping
bool previousFlip = true; //stores the previous state for flipping - needed for the direction change

int lockx = 0;
int locky = 0;
int lockz = 0;

int VRx = A3;
int VRy = A4;
int SW = 2;
int XAVG = 0;
int YAVG = 0;
int mapX = 0;
int mapY = 0;

void setup() {

  pinMode(38, OUTPUT);
  digitalWrite(38, LOW);

  pinMode(56, OUTPUT);
  digitalWrite(56, LOW);

  pinMode(62, OUTPUT);
  digitalWrite(62, LOW);

  for (int i = 0; i <= 3; i++) {                    //set the maximum speed and acceleration for the stepper motors
    steppers[i].setMaxSpeed(500);
    steppers[i].setAcceleration(2000);
  }

  SCmd.addCommand("M", move_stepper);
  SCmd.addCommand("V", change_velocity);
  SCmd.addCommand("STOP", stop_all);
  SCmd.addCommand("Home", homing);
  SCmd.addCommand("Info", send_info);
  SCmd.addCommand("Pos", send_position);
  SCmd.addCommand("Ready", check_move_complete);
  SCmd.addCommand("Position", check_position);
  SCmd.addCommand("completed?", is_complete);
  
  SCmd.addDefaultHandler(unrecognized);

  Serial.begin(115200);
  Serial.println("Robotic Arm");
  //

  pinMode(limitSwitch_x, INPUT_PULLUP); // internal pullup resistor (debouncing)
  pinMode(limitSwitch_y, INPUT_PULLUP); // internal pullup resistor (debouncing)
  pinMode(limitSwitch_z, INPUT_PULLUP); // internal pullup resistor (debouncing)
  

}

void loop() {
  SCmd.readSerial();

  for (int i = 0; i <= 3; i++) {                    //set the maximum speed and acceleration for the stepper motors
    steppers[i].run();

  }
}
/*
void flipCheck()
{
  if(switchFlipped == true)
  {    
     //Serial.println(previousFlip); //This was just a control flag for debugging
    

  }
}
*/


// This gets set as the default handler, and gets called when no other command matches.
void unrecognized()
{
  Serial.println("Not recognized");            //returns not ok to software

}


void send_info() {
  Serial.println("Robot Arm");
}


void send_position() {
  Serial.println("Robot Arm");
}

void limitswitch(){
  
  if (digitalRead(limitSwitch_x) == 0 && lockx==0) {
    steppers[0].setCurrentPosition(0);
    stop_spec(0);
    lockx = lockx+1;    
    }
  if (digitalRead(limitSwitch_y) == 0 && locky==0) {
    steppers[1].setCurrentPosition(0);
    stop_spec(1);
    locky = locky+1;
    }
  if (digitalRead(limitSwitch_z) == 0 && lockz==0) {
    steppers[2].setCurrentPosition(0);
    stop_spec(2);
    lockz = lockz+1;
    }
   
  
  if (digitalRead(limitSwitch_x) == 1 && lockx==1) {
    steppers[0].setCurrentPosition(0);
    stop_spec(0);
    lockx = lockx-1;    
    }
  if (digitalRead(limitSwitch_y) == 1 && locky==1) {
    steppers[1].setCurrentPosition(0);
    stop_spec(1);
    locky = locky-1;
    }
  if (digitalRead(limitSwitch_z) == 1 && lockz==1) {
    steppers[2].setCurrentPosition(0);
    stop_spec(2);
    lockz = lockz-1;
    }
   
  }

void change_velocity()    //function called when a serial command is received
{
  char *arg;
  float velocity;

  arg = SCmd.next();
  if (arg == NULL) {
    Serial.println("Not recognized: No Velocity given");
    return;
  }

  velocity = atoi(arg);
  if (velocity == 0) {
    Serial.println("Not recognized: Velocity parameter could not get parsed");
    return;
  }

  for (int i = 0; i <= 3; i++) {
    steppers[i].setMaxSpeed(velocity);
  }

}

void check_move_complete() {


  if (b_move_complete) {
    Serial.println("Ready for next command");
    return;
  }

  bool b_all_done = true;
  for (int i = 0; i <= 3; i++) {
    if (steppers[i].distanceToGo() > 0) {
      b_all_done = false;
    }
  }

  if (b_all_done) {
    Serial.println("Ready for next command");
    b_move_complete = true;
  }
  else {
    Serial.println("Busy");
  }

}

void stop_all() {
  for (int i = 0; i <= 3; i++) {
    stop_spec(i);
  }
}
void homing(){
  for (int i = 0; i <= 3; i++) {                    //set the maximum speed and acceleration for the stepper motors
    steppers[i].move(-100000);
  }
  }

void stop_spec(int value) {steppers[value].move(0);}

void changejoystick(){
  
  int xValue = analogRead(VRx);
  int yRead = analogRead(VRy);
  int yValue = -yRead;
  int SW_state = digitalRead(SW);
  mapX = map(xValue, 0, 1023, -512, 512);
  mapY = map(yValue, 0, -1023, 512, -512);
  }

void joystick(){
  Serial.println(mapX);
  Serial.println(mapY);
  int currentx = steppers[0].currentPosition();
  int currenty = steppers[1].currentPosition();
  int SW_state = digitalRead(SW);
    if (abs(mapX)>100) {
      if (mapX > 0){steppers[0].moveTo(10+currentx);}
      else {steppers[0].moveTo(-10+currentx);}
      }
    
    if (abs(mapY)>100){
      Serial.println("yes");
      if (mapY > 0){steppers[1].moveTo(10+currenty);}
      else {steppers[1].moveTo(-10+currenty);}
      }
}

void move_stepper() {

  char *arg;
  int step_idx;
  float distance;

  arg = SCmd.next();
  if (arg == NULL)  {
    Serial.println("Not recognized: Stepper Number" );
    return;
  }

  step_idx = atoi(arg);
  if (step_idx < 0) {
    Serial.print("Not recognized:");   Serial.println(step_idx);  return;

    Serial.print("ID ");
    Serial.print(step_idx );
  }

  arg = SCmd.next();
  if (arg == NULL)   {
    Serial.println("Not recognized: No hieght parameter given");
    return;
  }

  distance = atof(arg);
  if (distance == 0) {
    Serial.println("Not recognized: Height parameter not parsed");
    return;
  }

  Serial.print("moving ");
  Serial.print(distance);
  
  distance = distance * 100;
  steppers[step_idx].moveTo(distance);
  b_move_complete = false;
}
void is_complete() {
    if (steppers[0].distanceToGo() == 0) {
      if (steppers[1].distanceToGo() == 0) {
        if (steppers[2].distanceToGo() == 0) {
              Serial.println("Complete");
        }
        }
        }
  }

void check_position() {
  int M0;
  int M1;
  int M2;
  M0 = steppers[0].currentPosition();
  M1 = steppers[1].currentPosition();
  M2 = steppers[2].currentPosition();
  Serial.println(String(M0) + " " + String(M1) + " " + String(M2));
}
