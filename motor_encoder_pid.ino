#include <PID_v1.h>
#define MotEnable 6 //Motor Enamble pin Runs on PWM signal
#define MotFwd  5  // Motor Forward pin
#define MotRev  7 // Motor Reverse pin
String readString; //This while store the user input data
int User_Input = 0; // This while convert input string into integer
int encoderPin1 = 2; //Encoder Output 'A' must connected with intreput pin of arduino.
int encoderPin2 = 3; //Encoder Otput 'B' must connected with intreput pin of arduino.
volatile int lastEncoded = 0; // Here updated value of encoder store.
volatile long encoderValue = 0; // Raw encoder value
int PPR = 1600;  // Encoder Pulse per revolution.
int angle = 360; // Maximum degree of motion.
int REV = 0;          // Set point REQUIRED ENCODER VALUE
int lastMSB = 0;
int lastLSB = 0;
double kp = 5 , ki =  1, kd = 0.01;             // modify for optimal performance
double input = 0, output = 0, setpoint = 0;
PID myPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);  

void setup() {
  pinMode(MotEnable, OUTPUT);
  pinMode(MotFwd, OUTPUT); 
  pinMode(MotRev, OUTPUT); 
  Serial.begin(9600); //initialize serial comunication

  pinMode(encoderPin1, INPUT_PULLUP); 
  pinMode(encoderPin2, INPUT_PULLUP);

  digitalWrite(encoderPin1, HIGH); //turn pullup resistor on
  digitalWrite(encoderPin2, HIGH); //turn pullup resistor on

  //call updateEncoder() when any high/low changed seen
  //on interrupt 0 (pin 2), or interrupt 1 (pin 3) 
  attachInterrupt(0, updateEncoder, CHANGE); 
  attachInterrupt(1, updateEncoder, CHANGE);

    
  TCCR1B = TCCR1B & 0b11111000 | 1;  // set 31KHz PWM to prevent motor noise
  myPID.SetMode(AUTOMATIC);   //set PID in Auto mode
  myPID.SetSampleTime(1);  // refresh rate of PID controller
  myPID.SetOutputLimits(-125, 125); // this is the MAX PWM value to move motor, here change in value reflect change in speed of motor.
}

void loop() {
  while (Serial.available()) { //Check if the serial data is available.
    delay(3);                  // a small delay
    char c = Serial.read();  // storing input data
    readString += c;         // accumulate each of the characters in readString
  }
 
  if (readString.length() >0) { //Verify that the variable contains information
  
   Serial.println(readString.toInt());  //printing the input data in integer form
   User_Input = readString.toInt();   // here input data is store in integer form
}
REV = map (User_Input, 0, 360, 0, 1600); // mapping degree into pulse
Serial.print("this is REV - "); 
Serial.println(REV);               // printing REV value  

setpoint = REV;                    //PID while work to achive this value consider as SET value
  input = encoderValue ;           // data from encoder consider as a Process value
 Serial.print("encoderValue - ");
 Serial.println(encoderValue);
  myPID.Compute();                 // calculate new output
  pwmOut(output);  

  // Check if the difference between setpoint and encoder value is within a certain threshold
  if (abs(setpoint - encoderValue) < 10) { // Adjust the threshold value as needed
    finish(); // Stop the motor
  }
}

void pwmOut(int out) {       
  int slowedPWM = map(abs(out), 0, 10, 0, 5);                        
  if (out > 0) {                         // if REV > encoderValue motor move in forward direction.    
    analogWrite(MotEnable, out);         // Enabling motor enable pin to reach the desire angle
    reverse();                           // calling motor to move forward
  }
  else if (out < 0) {
    analogWrite(MotEnable, abs(out));          // if REV < encoderValue motor move in forward direction.                      
    forward();                            // calling motor to move reverse
    // Serial.println("test 2");
  }else{
    finish();   
  }
 readString=""; // Cleaning User input, ready for new Input
}
void updateEncoder(){
  int MSB = digitalRead(encoderPin1); //MSB = most significant bit
  int LSB = digitalRead(encoderPin2); //LSB = least significant bit

  int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  int sum  = (lastEncoded << 2) | encoded; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue ++;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue --;

  lastEncoded = encoded; //store this value for next time

}

void forward () {
  analogWrite(MotFwd, 25); 
 analogWrite(MotRev, 0); 
  
}

void reverse () {
  analogWrite(MotFwd, 0); 
 analogWrite(MotRev, 25); 
  
}
void finish () {
  analogWrite(MotFwd, 0); 
 analogWrite(MotRev, 0); 
  
}