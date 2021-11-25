/*
   -- New project --
   
   This source code of graphical user interface 
   has been generated automatically by RemoteXY editor.
   To compile this code using RemoteXY library 2.4.3 or later version 
   download by link http://remotexy.com/en/library/
   To connect using RemoteXY mobile app by link http://remotexy.com/en/download/                   
     - for ANDROID 4.5.1 or later version;
     - for iOS 1.4.1 or later version;
    
   This source code is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public
   License as published by the Free Software Foundation; either
   version 2.1 of the License, or (at your option) any later version.    
*/

//////////////////////////////////////////////
//        RemoteXY include library          //
//////////////////////////////////////////////

// определение режима соединения и подключение библиотеки RemoteXY 
#define REMOTEXY_MODE__SOFTSERIAL
#include <SoftwareSerial.h>

#include <RemoteXY.h>

// настройки соединения 
#define REMOTEXY_SERIAL_RX 12
#define REMOTEXY_SERIAL_TX 13
#define REMOTEXY_SERIAL_SPEED 9600


// конфигурация интерфейса  
//#pragma pack(push, 1)
//uint8_t RemoteXY_CONF[] =
//  { 255,3,0,0,0,22,0,10,13,1,
//  5,1,15,63,30,30,2,26,31,1,
//  0,25,31,12,12,2,31,88,0 };
  
#pragma pack(push, 1)
uint8_t RemoteXY_CONF[] =
  { 255,3,0,0,0,20,0,10,13,1,
  3,2,45,4,12,22,2,26,5,32,
  17,58,30,30,2,26,31 };
  
// структура определяет все переменные и события вашего интерфейса управления 
struct {

    // input variables
  uint8_t select_1; // =0 if select position A, =1 if position B, =2 if position C, ... 
  int8_t joystick_1_x; // =-100..100 координата x положения джойстика 
  int8_t joystick_1_y; // =-100..100 координата y положения джойстика 
  //uint8_t button_1; // =1 if button pressed, else =0 

    // other variable
  uint8_t connect_flag;  // =1 if wire connected, else =0 

} RemoteXY;
#pragma pack(pop)

/////////////////////////////////////////////
//           END RemoteXY include          //
/////////////////////////////////////////////

#define PIN_TRIG 3
#define PIN_ECHO 2


/* defined the right motor control pins */
#define DIR_1        4
#define SPEED_1      5

/* defined the left motor control pins */
#define DIR_2        7
#define SPEED_2      6

/* defined two arrays with a list of pins for each motor */
unsigned char RightMotor[2] = 
  {DIR_1, SPEED_1};
  
unsigned char LeftMotor[2] = 
  {DIR_2, SPEED_2};

byte tick = 0;
const float k = -0.8;
long dist;
int sign = 1; 
  
void Wheel (unsigned char * motor, int v)
{
  if (v>100) v=100;
  if (v<-100) v=-100;
  if (v>0) {
    digitalWrite(motor[0], HIGH);
    analogWrite(motor[1], v*2.55);
  }
  else if (v<0) {
    digitalWrite(motor[0], LOW);
    analogWrite(motor[1], (-v)*2.55);
  }
  else {
    digitalWrite(motor[0], LOW);
    analogWrite(motor[1], 0);
  }
}

long Sonar ()
{
  //long duration;
  // Сначала генерируем короткий импульс длительностью 2-5 микросекунд.
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(5);
  digitalWrite(PIN_TRIG, HIGH);

  // Выставив высокий уровень сигнала, ждем около 10 микросекунд. В этот момент датчик будет посылать сигналы с частотой 40 КГц.
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  
  //  Время задержки акустического сигнала на эхолокаторе.
  //duration = pulseIn(PIN_ECHO, HIGH);

  // Теперь осталось преобразовать время в расстояние
  return pulseIn(PIN_ECHO, HIGH)/ 58;
}

void setup() 
{
  RemoteXY_Init ();

  //Serial.begin (9600); 

  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(DIR_1, OUTPUT);
  pinMode(SPEED_1, OUTPUT);
  pinMode(DIR_2, OUTPUT);
  pinMode(SPEED_2, OUTPUT);
  
  Serial.begin(REMOTEXY_SERIAL_SPEED);
  randomSeed(analogRead(0));
}

void loop() 
{ 
  RemoteXY_Handler ();

  tick++;
  if (tick>30)
  {
    sign = random(10);
    if (sign > 5) sign = -1;
    else sign = 1;
    tick = 0;
  }
  
  dist = Sonar();
  Serial.print(dist);
  Serial.print(" ");
  
  if (RemoteXY.select_1 == 0)
  {
    if (dist>40)
    {
    /* manage the right motor */
    Wheel (RightMotor, 100);
    /* manage the left motor */
    Wheel (LeftMotor, 100*k);
    }
    else if (dist<40 && dist>15) {
      /* manage the right motor */
      Wheel (RightMotor, 2*dist + 20);
      /* manage the left motor */
      Wheel (LeftMotor, (2*dist + 20)*k);
    }
    else {
      /* manage the right motor */
      Wheel (RightMotor, (-80)*sign);
      /* manage the left motor */
      Wheel (LeftMotor, 80*sign*k);
      delay(100);
    }
    
    // Задержка между измерениями для корректной работы
    delay(100);
  }
  else {
    Wheel (RightMotor, RemoteXY.joystick_1_y - RemoteXY.joystick_1_x);
    Wheel (LeftMotor, RemoteXY.joystick_1_y + RemoteXY.joystick_1_x);
  }
}
