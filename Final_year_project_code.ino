#include <TimerFreeTone.h>// Library for the Buzzer
#include <AFMotor.h>  // Library for the motors
#include <Servo.h>    // Library for the servo motor
#include <NewPing.h>  // Library for the ultrasonic sensor
// #include <IRremote.h> //Library for the IR reciever



//Ultrasonic sensor
#define TriggerPin A3
#define EchoPin A2
#define max_distance 50 //50 at first
//Infrared Sensor
#define IrLeft A0 //Left line sensor
#define IrRight A5 //Right line sensor
//Pezzo Buzzer
#define buzzer A1
//motor
#define MaxSpeed 200
#define MaxSpeed_Offset 20
//speed of the motoes
#define SPEED1 180 //61 forward
#define SPEED2 180 //102 left right
#define SPEED3 180
//creates an instance of the Servo class for controlling the servo motor.
Servo servo;

const int Motor_1 = 1; 
const int Motor_2 = 2; 
const int Motor_3 = 3; 
const int Motor_4 = 4; 

//creates an instance of the NewPing class for reading from the HC-SR04 ultrasonic sensor
NewPing sonar(TriggerPin, EchoPin, max_distance); 

//creates instances of the AF_DCMotor class for controlling the four motors of the robot.
//Pulse Width Modulation and is a technique used to control the speed of DC motors. By varying the pulse width of the signal sent to the motor, the effective voltage supplied to the motor is varied, which in turn varies the speed of the motor.
//AF_DCMotor(motor#, frequency)
AF_DCMotor motor1(Motor_1, MOTOR12_1KHZ);// create motor object, 1KHz pwm....1,2 4,8,16,32,64
AF_DCMotor motor2(Motor_2, MOTOR12_1KHZ);
AF_DCMotor motor3(Motor_3, MOTOR34_1KHZ);
AF_DCMotor motor4(Motor_4, MOTOR34_1KHZ);

//declares variables for storing the distances detected by the sensors.
int distance =0;
int LeftDistance = 0;
int RightDistance = 0;
int IRLEFT=0;
int IRRIGHT =0;
int set = 6;
String STATE = " ";
char val;
//boolean variable for determining the direction of the turn
boolean Object;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);//initializes the serial communication
  Serial.println("Start");

  servo.attach(10);//attaches the Servo object to pin 10
  servo.write(95); //sets the initial position of the servo to 90 degrees

  pinMode(IrLeft, INPUT); // sets the pins for the infrared sensors as inputs
  pinMode(IrRight, INPUT);

  pinMode(buzzer, OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
  //If both sensors detect a line, it calls the objectAvoid() function and moves forward

  if(Serial.available() > 0){
    val = Serial.read();
    Stop();
    if (val == 'F'){
      moveForward();
    }
    if(val == 'B'){
      moveBackward();
    }
    if(val == 'L'){
      moveLeft();
    }
    if(val == 'R'){
      moveRight();
    }
    if(val == 'T'){
      Stop();
    }
  }
    
  // printValues();

  // // Reset the printed values to zero
  // distance = 0;
  // LeftDistance = 0;
  // RightDistance = 0;


  // IRLEFT = digitalRead(IrLeft);
  // IRRIGHT= digitalRead(IrRight);

  // if (!digitalRead(IrLeft) == 0 && !digitalRead(IrRight) == 0 ) {
  //   objectAvoid();
  // }
  // //If only the right sensor detects a line, it calls the objectAvoid() function, movesright , and prints "TR" to the serial monitor.
  // else if (digitalRead(IrLeft) == 0 && !digitalRead(IrRight) == 0 ) {
  //   objectAvoid();
  //   STATE = "Turn Right";
  //   moveBackward();
  //   delay(1);
  //   moveRight();   
  // }
  // //If only the left sensor detects a line, it calls the objectAvoid() function, moves left, and prints "TL" to the serial monitor.
  // else if (!digitalRead(IrLeft) == 0 && digitalRead(IrRight) == 0 ) {
  //   objectAvoid();
  //   STATE = "Turn Left";
  //   moveBackward();
  //   delay(1);
  //   moveLeft();
  // }
  // //If neithber sensors detect line, the motors stops.
  // else if (digitalRead(IrLeft) == 0 && digitalRead(IrRight) == 0 ) {
  //   //Stop();
  //   moveBackward();
  //   delay(10);
  //   TimerFreeTone(buzzer,10,100);
  //   STATE= "Stop";
  // }
}
//function for detecting obstacles and avoiding them
// void objectAvoid() {
//   distance = getDistance();
//   //calls the getDistance() function and assigns its return value to the variable distance
//   //checks whether the distance variable is less than or equal to 15 centimeters, which is the threshold distance for detecting obstacles
//   if (distance <= set) {
//     Stop();
//     STATE = "Stop";
//     moveBackward();
//     delay(300);
//     Stop();
//     // delay(100);
//     TimerFreeTone(buzzer,700,500);
//     TimerFreeTone(buzzer,261,500);

//     //These lines call the lookLeft() and lookRight() functions to measure the distance to the left and right of the robot using a servo-mounted ultrasonic sensor.
//     LeftDistance = lookLeft();
//     RightDistance = lookRight();

//     // if right distance is less than or equal to the left distance, the robot will turn left to avoid the obstacle.
//     if (RightDistance <= LeftDistance) {
//       Object = true;//Set the object variable to true to indicate that an obstacle has been detected on the right
//       TimerFreeTone(buzzer,392,1000); 
//       turn(); // call the turn() function to turn left
//       STATE = "Move Left";
//     //If the right distance is greater than the left distance, the robot will turn right to avoid the obstacle  
//     } else {
//       Object = false;
//       TimerFreeTone(buzzer,440,1000);
//       turn();
//       STATE = "Move Right";
//     }
//     delay(100);
//   }
//     //If no obstacle is detected within the threshold distance, the robot will move forward.
//   else {
//     STATE = "Move Forward";
//     moveForward();//moveForward() function to move the robot forward.
//   }

// }
// // function to turn the robot when an obstacle is detected
// void turn(){
//   if (Object == false){//obstacle is detected on the left side so you move right
//     moveRight();
//     delay(630);//700
//     moveForward();
//     delay(1800);//800
//     moveLeft();
//     delay(670);//720
//     moveForward();
//     delay (600);
//     //check if the IR sensor on the left side detects the line.If the line is detected, it calls the loop() function. If not, it calls the moveForward() function again
//     if (digitalRead(IrLeft) == 1) {
//       loop();
//     } else {
//       moveForward();
//     }
//   }
//   else if (Object == true) {
//     moveLeft();
//     delay(670);//700
//     moveForward();
//     delay(1800);//800
//     moveRight();
//     delay(780);//720
//     moveForward();
//     delay (600);
//     if (digitalRead(IrRight) == 1) {
//       loop();
//     } else {
//       moveForward();
//     }
//   }
// }


// // function responsible for measuring the distance to an obstacle using a servo-mounted ultrasonic sensor.
// int getDistance(){
//   delay(50);
//   int cm = sonar.ping_cm();//use the ping_cm() function of the NewPing library to measure the distance to an obstacle in centimeters and store it.
//   //If the measured distance is zero or -ve, the distance is set to 100 centimeters.
//   if (cm == 0) {
//     cm = 100;
//   }
//   return cm;//distance is returned
// }

// //detect object at the left side
// int lookLeft(){
//   servo.write(180);// set the servo position to 170 degrees which corresponds to the leftmost position.
//   delay(500);//wait 500 milliseconds to allow time for the ultrasonic sensor to measure the distance.
//   int dist = getDistance();
//   delay(200); //waits for 100 milliseconds.
//   servo.write(95);//sets the servo motor back to the center position (90 degrees)
//   return dist;//returns the distance value.
//   delay(100);
// }

// //detect object at the Right side
// int lookRight(){
//   servo.write(10);// set the servo position to 10 degrees which corresponds to the right position.
//   delay(500);//wait 300 milliseconds to allow time for the ultrasonic sensor to measure the distance.
//   int dist = getDistance();
//   delay(200);
//   servo.write(95);
//   return dist;
//   delay(100);
// }
//MOTOR SPEED
void set_Motorspeed(char g) {
  if (g == 'L' || g == 'R' ) {
    motor1.setSpeed(SPEED2);//130
    motor2.setSpeed(SPEED2);
    motor3.setSpeed(SPEED2);
    motor4.setSpeed(SPEED2);
  }else if(g == 'B'){
    motor1.setSpeed(SPEED3);//100
    motor2.setSpeed(SPEED3);
    motor3.setSpeed(SPEED3);
    motor4.setSpeed(SPEED3);
  }
  else {
    motor1.setSpeed(SPEED1);//110
    motor2.setSpeed(SPEED1);
    motor3.setSpeed(SPEED1);
    motor4.setSpeed(SPEED1);
  }
}

//This function sets all the motors to run in the forward direction by calling the FORWARD parameter of the run() method of the AF_DCMotor library.
void moveForward(){
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
  set_Motorspeed('F');
}
//This function sets all the motors to run in the backward direction by calling the BACKWARD parameter of the run() method of the AF_DCMotor library
void moveBackward(){
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
  set_Motorspeed('B');
}
//function is used to make the robot turn right. It makes the left motors turn backward and the right motors turn forward by calling the BACKWARD and FORWARD parameters of the run() method of the AF_DCMotor.
void moveRight(){
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
  set_Motorspeed('R');
}
//function is used to make the robot turn left. It makes the left motors turn forward and the right motors turn backward by calling the FORWARD and BACKWARD parameters of the run() method of the AF_DCMotor.
void moveLeft(){
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
  set_Motorspeed('L');
}
//This function stops all the motors by calling the RELEASE parameter of the run() method of the AF_DCMotor library.
void Stop(){
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}


void printValues(){
    Serial.print(STATE);
    Serial.print(",");
    Serial.print(distance);
    Serial.print(",");
    Serial.print(IRLEFT);
    Serial.print(",");
    Serial.print(IRRIGHT);
    Serial.print(",");
    Serial.print(LeftDistance);
    Serial.print(",");
    Serial.println(RightDistance);
    // delay(2000);
  }