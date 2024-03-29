// Void of Creation: Team A9
// October 20, 2021
// Final Code for Temp Sensor, LCD screen, and Servo Arm Control

//Include libraries
#include <PID_v1.h> //PID library
#include <LiquidCrystal.h> //LCD library
#include <Wire.h> //LCD library
#include <Adafruit_MLX90614.h> //LCD library
#include <Servo.h> //Servo library

//define pins
Servo myservo;
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
#define servoPin 9 //servo pin
#define buttonpin 7 //button pin
#define RelayPin 8 //relay pin
#define piezopin 10 //buzzer pin

//initializing variables
int lastbuttonstate;
int currentbuttonstate;
int angle = 0; //initializing servo angle for reference
int servopos = 90; //initializing servo position

int threshold = 190; //THRESHOLD TEMP: change this for desired temp, this is in deg. F
int ServoMoveTemp = 180; //When servo moves in degrees F
int temp = 0;

//PID control stuff (got from https://www.teachmemicro.com/arduino-pid-control-tutorial/)
double Setpoint, Input, Output;
double Kp=700, Ki=1, Kd=0; //Set ki and kd to zero and then tune system where kp converges without much overshoot, then tune finer with kd and ki
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
int WindowSize = 5000;
unsigned long windowStartTime;

void setup() {
  //LCD screen stuff
  lcd.begin(16, 2);
  Serial.begin(9600);
  Serial.println("Adafruit MLX90614 test");  
  
  mlx.begin(); //start reading temps

  myservo.attach(servoPin);
  //initialize the variables we're linked to
  Input = analogRead(mlx.readObjectTempF());
  Setpoint = threshold;
  
  pinMode(RelayPin, OUTPUT);

  windowStartTime = millis();

  //initialize the variables we're linked to
  Setpoint = threshold;

  //tell the PID to range between 0 and the full window size
  myPID.SetOutputLimits(0, WindowSize);

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
}

void loop() {
  //LCD screen stuff
  lcd.setCursor(0,0);
  lcd.print("object temp [F]:");
  lcd.setCursor(5,1);
  lcd.print(mlx.readObjectTempF());
  lcd.print("         ");
  Serial.print("Ambient = "); Serial.print(mlx.readAmbientTempC());
  Serial.print("*C\tObject = "); Serial.print(mlx.readObjectTempC()); Serial.println("*C");
  Serial.print("Ambient = "); Serial.print(mlx.readAmbientTempF());
  Serial.print("*F\tObject = "); Serial.print(mlx.readObjectTempF()); Serial.println("*F");
  Serial.println();
  
  //Reading temp from the sensor
  temp = mlx.readObjectTempF();
  Input = temp;
  myPID.Compute();

  /************************************************
     turn the output pin on/off based on pid output
   ************************************************/
  unsigned long now = millis();
  if (now - windowStartTime > WindowSize)
  { //time to shift the Relay Window
    windowStartTime += WindowSize;
  }
  if (Output > now - windowStartTime) digitalWrite(RelayPin, HIGH);
  else digitalWrite(RelayPin, LOW);
  Serial.print(Output);
  
  
  //Button control
  lastbuttonstate = currentbuttonstate; //save last state
  currentbuttonstate = digitalRead(buttonpin); //reading new state
  
  if(lastbuttonstate == HIGH && currentbuttonstate == LOW){
    //if button is pressed 
   myservo.attach(9); //reattach servo
   delay(15);
   if (servopos == 90)//if the servo is at 90 move it back to zero when button is pressed
     servopos==0;  
  }
   if(temp > ServoMoveTemp){
      //if temp surpasses the threshold
      servopos = 90; //move servo out of the way to 90
      tone(piezopin,1000,2000); //sound buzzer
      myservo.write(servopos); //write new position to servo
      delay(1000);//delay to allow data to be sent to servo
      myservo.detach();//detatch servo to turn it off so it won't move back into position
   }
   else{
    servopos = 0;//if temp hasn't surpassed then keep servo at 0 to continue reading
   }
    delay(500);
    myservo.write(servopos);//write final servopos to servo--this is mostly to keep it in postion to read while temp is low
   
}
