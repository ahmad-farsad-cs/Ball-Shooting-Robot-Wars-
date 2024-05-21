#include <Servo.h> 
Servo servo1; 
Servo servo2;

int lmt1=6,lmt2=7,rmt1=8,rmt2=9,servoPin=10,servoPin2=11;     //set your motor output pins left motor terminals and right motor terminals
int pwm;
int value, pos;           // declaring variable used in serial communication
void setup()         // main function executes only once
{
 pinMode(lmt1,OUTPUT);
 pinMode(lmt2,OUTPUT);
 pinMode(rmt1,OUTPUT);
 pinMode(rmt2,OUTPUT); 
 pinMode(3,OUTPUT);
 pinMode(5,OUTPUT);
 servo1.attach(servoPin); 
 servo2.attach(servoPin2);
 pwm = 0;
 analogWrite(3, pwm);
 analogWrite(5, pwm);

 Serial.begin(9600); // serial communication begins baud rate is 9600
}
void loop()
{
   
  value=Serial.read(); //it will read the data recieved by xbee

  switch(value)   // will activate the case according to the value

  {
    

  //  for(pos=0; pos<=value; pos++)
   // Serial.println("Robot is shooting");
   // shoot();
  //  break;        // restart the loop function
  


    case 'w':     // pressing w from keyboard activates this case and robot moves forward
    {
      pwm = Serial.parseInt();
      Serial.println("Robot moves forward");
      fwd(); 
    }
    break;        // restart the loop function
    case 'd':     // pressing d from keyboard activates this case and robot turns right
    {
      pwm = Serial.parseInt();
      Serial.println("Robot turns right");
      right();
    }
    break;
    case 'a':     // pressing a from keyboard activates this case and robot turns left
    {
      pwm = Serial.parseInt();
      Serial.println("Robot turns left");
      left();
    }
    break;
    case 's':     // pressing s from keyboard activates this case and robot stops
    Serial.println("Robot stops");
    Stop();
    break;
    case 'z':     // pressing z from keyboard activates this case and robot moves back
    {
      pwm = Serial.parseInt();
      Serial.println("Robot moves back");
      back();
    }
    break;
    case 'p':     // pressing k from keyboard activates this case and robot kicking
    Serial.println("Robot prepared for shooting");
    pshoot();
    break;        // restart the loop function
    case 'k':     // pressing k from keyboard activates this case and robot kicking
    Serial.println("Robot is shooting");
    shoot();
    break;        // restart the loop function
    case 't':
      {
      int angle = Serial.parseInt();
      servo2.write(angle);
      }
      break;
  } 
}
void Stop() // function details 
{
  pwm=0;
  analogWrite(3, pwm);
  analogWrite(5, pwm);
  digitalWrite(lmt1,LOW);   // logic to stop all motor signal 0 volt
  digitalWrite(lmt2,LOW);
  digitalWrite(rmt1,LOW);
  digitalWrite(rmt2,LOW);
}
void left()
{
  analogWrite(3, pwm);
  analogWrite(5, pwm);
  digitalWrite(lmt1,LOW);   // logic to move left. Right motor moves forward and left motor backwards
  digitalWrite(lmt2,HIGH);
  digitalWrite(rmt1,HIGH);
  digitalWrite(rmt2,LOW);
}
void right()                // logic to move right. Left motor moves forward and right motor backwards
{
  analogWrite(3, pwm);
  analogWrite(5, pwm);
  digitalWrite(lmt1,HIGH);
  digitalWrite(lmt2,LOW);
  digitalWrite(rmt1,LOW);
  digitalWrite(rmt2,HIGH);
}
void fwd()                   // logic to move forward. both motor moves forward
{
  analogWrite(3, pwm);
  analogWrite(5, pwm);
  digitalWrite(lmt1,HIGH);
  digitalWrite(lmt2,LOW);
  digitalWrite(rmt1,HIGH);
  digitalWrite(rmt2,LOW);
}
void back()                 // logic to move Backwards. both motor moves backwards
{
  analogWrite(3, pwm);
  analogWrite(5, pwm);
  digitalWrite(lmt1,LOW);
  digitalWrite(lmt2,HIGH);
  digitalWrite(rmt1,LOW);
  digitalWrite(rmt2,HIGH);
}
void pshoot()
{
 //servo1.write(pos);
servo1.write(130);  
}
void shoot()
{
 //servo1.write(pos);
servo1.write(180);  
}                        

