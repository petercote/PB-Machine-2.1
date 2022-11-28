//potentiometer pins
int feedPin=A15; //ball feed interval potentiometer
int spinPin=A7; //ball spin potentiometer
int velocityPin=A2; //ball velocity potentiometer


//button pins
int StartStopPin=7; //start/stop button connects to arduino pin 7
int loftupPin=6; //loft up button pin 
int loftdwnPin=5; //loft down button pin


//hardware pins
int topMotorDir1=4; //top motor controller pin
int topMotorDir2=8; //top motor controller pin
int bottomMotorDir1=9; //bottom motor controller pin
int bottomMotorDir2=10; //bottom motor controller pin
int feedDir=2; //arduino pin 2 leads to dir+ terminal on ball feed stepper driver
int feedPulse=3; //arduino pin 3 leads to pul+ terminal on ball feed stepper driver
int loftDir=12; // arduino pin 12 leads to dir+ terminal on loft stepper driver
int loftPulse=13; // arduino pin 13 leads to pul+ terminal on loft stepper driver


//varaiables
int ballSpeed; //read value of ball speed potientiometer
int spinValue; //read value of ball spin potentiometer
int feedRate; //read value of feed rate potentiometer
unsigned long feedSpeed; //delay value that determines speed of ball feed stepper motor
unsigned long previousStep = 0; //time of previous step in millis
float topMotorSpeed; //motor speed of top motor
float bottomMotorSpeed; //motor speed of bottom motor
int topSpinMap;
int backSpinMap; 
float topSpinPercent;
float backSpinPercent;
int loftupStatus; //read value from loft up button
int loftdwnStatus; //read value from loft down button
int StartStopStatus; //read value from start/stop button
int loftSpeed=30; //delay time between loft stepper pulses
unsigned long currentMillis = 0;
int pulseStatus = LOW;
int debounce=100; //delays to allow button to debounce
int pgmState=1; //a read value of 1 means the program is not running
int buttonNew; //a read value of 0 means the program is running
int buttonOld=1; //ensures that the program does not run on power up



void setup() {
  // put your setup code here, to run once:

//potentiometer pin modes here
pinMode(feedPin,INPUT);
pinMode(spinPin,INPUT);
pinMode(velocityPin,INPUT);

//button pin modes here
pinMode(StartStopPin,INPUT_PULLUP); //start stop button set as input with pullup resistor
pinMode(loftupPin,INPUT_PULLUP); //loft up button set as input with pullup resistor
pinMode(loftdwnPin,INPUT_PULLUP); //loft down button set as input with pullup resistor

//hardware signal pin modes here
pinMode(topMotorDir1,OUTPUT);
pinMode(topMotorDir2,OUTPUT);
pinMode(bottomMotorDir1,OUTPUT);
pinMode(bottomMotorDir2,OUTPUT);
pinMode(feedDir,OUTPUT);
pinMode(feedPulse,OUTPUT);
pinMode(loftDir,OUTPUT);
pinMode(loftPulse,OUTPUT);

//Start serial monitor
Serial.begin(115200);

}

void loop() {
buttonNew=digitalRead(StartStopPin);
if(buttonOld==1 && buttonNew==0){
  if(pgmState==1){
    //ENTER PROGRAM HERE





currentMillis = millis();

//reads potiometer values
ballSpeed=analogRead(velocityPin); //reads the value of the ball velocity potentiometer 0-1023
spinValue=analogRead(spinPin); //reads the value of the ball spin potentiometer 0-1023
feedRate=analogRead(feedPin); //reads the value of the ball feed potentiometer 0-1023

//reads button values
loftupStatus=digitalRead(loftupPin); //reads the status of loft up button and defines the variable
loftdwnStatus=digitalRead(loftdwnPin); //reads the status of the loft down button and defines the variable
StartStopStatus=digitalRead(StartStopPin); //reads the status of the start/stop button


//FlAT SPIN CONTROL
//No spin (flat ball) control, if spin potentiometer has no spin, then top and bottom motors should be the same speed
if (spinValue<=541 && spinValue>=481) {
  topMotorSpeed=map(ballSpeed, 0, 1023, 15, 255); //remaps top motor analog read range to analog write range with a speed minimum above zero
  bottomMotorSpeed=map(ballSpeed, 0, 1023, 15, 255); //remaps bottom motor analog read range to analog write range with a speed minimum above zero
}

//BACKSPIN CONTROL
//if spin potentiometer has backspin, then the top motor will run slower
if (spinValue<481)  {
  backSpinMap=map(spinValue, 0, 480, 70, 95); 
  backSpinPercent=backSpinMap/100.; ////remaps the potentiometer values so that the bottom motor will run at 70-95 percent of the speed of the top motor  
  topMotorSpeed=(ballSpeed/4.011)*backSpinPercent;
  bottomMotorSpeed=map(ballSpeed, 0, 1023, 15, 255); //remaps bottom motor analog read range to analog write range with a speed minimum above zero
}

//TOPSPIN CONTROL
//if spin potentiometer has topspin, then the bottom motor will run slower
if (spinValue>541) {
  topSpinMap=map(spinValue, 542, 1023, 95, 70); 
  topSpinPercent=topSpinMap/100.; //remaps the potentiometer values so that the bottom motor will run at 70-95 percent of the speed of the top motor  
  bottomMotorSpeed=(ballSpeed/4.011)*(topSpinPercent);
  topMotorSpeed=map(ballSpeed, 0, 1023, 15, 255); //remaps top motor analog read range to analog write range with a speed minimum above zero
}


//BALL VELOCITY
//sends pwm signal to ball speed dc motor controller
analogWrite(topMotorDir1,topMotorSpeed);
analogWrite(bottomMotorDir1,bottomMotorSpeed);

//LOFT UP CONTROL
//while loft up button is pushed it will execute the below commands in a continuous loop
while (loftupStatus == LOW) {     
   // stepper motor/driver commands to adjust loft up
  digitalWrite(loftDir, HIGH); //High and Low sets direction of loft stepper motor
  digitalWrite(loftPulse, HIGH); //moves motor one setp
  delayMicroseconds(loftSpeed); //time between steps.  Changing this time will increase or decrease speed of motor
  digitalWrite(loftPulse, LOW); //resets stepper pulse so that the next pulse will move the stepper motor another step
  delayMicroseconds(loftSpeed); //time between steps.  Changing this time will increase or decrease speed of motor
  loftupStatus=digitalRead(loftupPin); //reads the status of loft up button and defines the variable
}

//LOFT DOWN CONTROL
//while loft down button is pushed it will execute the below commands in a continuous loop
while (loftdwnStatus == LOW) {
  // stepper motor/driver commands to adjust loft down
  digitalWrite(loftDir, LOW); //High and Low sets direction of loft stepper motor
  digitalWrite(loftPulse, HIGH); //moves motor one step
  delayMicroseconds(loftSpeed); //time between steps.  Changing this time will increase or decrease speed of motor
  digitalWrite(loftPulse, LOW); //resets stepper pulse so that the next pulse will move the stepper motor another step
  delayMicroseconds(loftSpeed); //time between steps.  Changing this time will increase or decrease speed of motor
  loftdwnStatus=digitalRead(loftdwnPin); //reads the status of the loft down button and defines the variable
}

//BALL FEED CONTROL
feedSpeed=map(feedRate, 0, 1023, 40, 1600);
digitalWrite(feedDir, LOW); //High and Low sets direction of loft stepper motor
digitalWrite(feedPulse,pulseStatus);

digitalWrite(feedDir, LOW); //High and Low sets direction of loft stepper motor
digitalWrite(feedPulse,HIGH);
delayMicroseconds(feedSpeed);
digitalWrite(feedPulse,LOW);
delayMicroseconds(feedSpeed);
    
    pgmState=0; //changes program state
  }
  else{
    pgmState=1;
    analogWrite(topMotorDir1,0);
    analogWrite(bottomMotorDir1,0);
  }

}
buttonOld=buttonNew;
delay(debounce);

}
