#include <MPU6050_tockn.h>
#include <Wire.h>

#define ren 11
#define rin1 12
#define rin2 10
#define len 6
#define lin1 7
#define lin2 8

float angle;
float headAngle = 0;
float sp;
float pp;
float error;

float kp = 12;
float ki = 0.001;
float kd = 0;

float P;
float I;
float D;

float integral = 0;
float p_error = 0;
float op;
float pid_op;

float speedChange;
float speed;
float r_speed = 70;
float l_speed = 70;
float r;
float l;

float r_map;
float l_map;
float set;

int timer1 = 0;
int timer2 = 0;
int end = 5000;

MPU6050 mpu6050(Wire);

float turn(float s)
{
  integral = 0;
  p_error = 0;
  while(true)
  {

    mpu6050.update();
  angle = mpu6050.getAngleZ();
  //Serial.println(angle);

  sp = s;
  pp = angle;
  error = sp - pp;
  

  pid_op = pid_cal(error, kp, ki , kd);
  if(pid_op >= 100)
  {
    pid_op = 100;
  }
  if(pid_op<=-100)
  {
    pid_op = -100;
  }


  speed = map(pid_op, 0 , 100, 50, 255);
  speed = abs(speed);


  Serial.print(error);
  Serial.print(",");
  Serial.println(speed);
  if(error>0)
  {
    analogWrite(ren,speed);
    analogWrite(len,speed);

    digitalWrite(rin1,LOW);
    digitalWrite(rin2,HIGH);
    digitalWrite(lin1,LOW);
    digitalWrite(lin2,HIGH);

    if(error>-2 && error<2)
    {
      analogWrite(r,0);
      analogWrite(l,0);
      break;

    }

  }
  else if(error<0)
  {
    analogWrite(ren,speed);
    analogWrite(len,speed);

    digitalWrite(rin2,LOW);
    digitalWrite(rin1,HIGH);
    digitalWrite(lin2,LOW);
    digitalWrite(lin1,HIGH);

  }

  // Serial.print("pp: ");
  // Serial.print(angle);
  // Serial.print("  sp: ");
  // Serial.print(sp);
  // Serial.print("  error: ");
  // Serial.print(error);
  // Serial.print("  op: ");
  // Serial.print(pid_op);
  // Serial.print("  speed: ");
  // Serial.println(speed);

  }
  return sp;

}

void forward(int timer1, int end, int s)
{
  sp = s;
  while (timer2 < end)
  {
    timer2 = millis() - timer1;
    Serial.println(timer2);
  // put your main code here, to run repeatedly:
  mpu6050.update();
  angle = mpu6050.getAngleZ();
  //Serial.println(angle);
  sp = headAngle;
  pp = angle;
  error = pp - sp;
  //Serial.println(error);
  pid_op = pid_cal(error, kp, ki, kd);

  digitalWrite(lin1,LOW);
  digitalWrite(lin2,HIGH);
  digitalWrite(rin1,HIGH);
  digitalWrite(rin2,LOW);
  if (pid_op > 100)
  {
    pid_op = 100;
  }

  else if (pid_op < -100)
  {
    pid_op = -100;
  }

  //Serial.println(pid_op);

  speedChange = 0.5*pid_op;
  speedChange = abs(speedChange);
  if (error >= 0)
  {
    r = r_speed + speedChange;
    l = l_speed - speedChange;
    analogWrite(len,l);
    analogWrite(ren,r);

    r_map = map(r, 0, 100, 0, 170);
    l_map = map(l, 0, 100, 0, 170);

    // Serial.print("r_speed:");
    // Serial.print(r_map);
    // Serial.print(",  l_speed:");
    // Serial.println(l_map);
  }

  if (error < 0)
  {
    r = r_speed - speedChange;
    l = l_speed + speedChange;
    analogWrite(len,l);
    analogWrite(ren,r);

    r_map = map(r, 0, 100, 0, 170);
    l_map = map(l, 0, 100, 0, 170);

    // Serial.print("r_speed:");
    // Serial.print(r_map);
    // Serial.print(",  l_speed:");
    // Serial.println(l_map);
  }

  //Serial.println(speed);
  //speed = map()
  }

  analogWrite(len,0);
  analogWrite(ren,0);
}

float pid_cal(float error, float kp, float ki, float kd)
{
  //P
  P = error*kp;
  
  //I
  integral = integral + error;
  I = ki * integral;

  //D
  D = kd*(error-p_error);
  p_error = error;

  op = P+I+D;
  return op;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);

  pinMode(ren, OUTPUT);
  pinMode(len, OUTPUT);
  pinMode(rin1, OUTPUT);
  pinMode(rin2, OUTPUT);
  pinMode(lin1, OUTPUT);
  pinMode(lin2, OUTPUT);
  
}

void loop() {
  timer1 = millis();
  forward(timer1,end,0);
  set = turn(90);
  forward(timer1,2000,set);
  
}
