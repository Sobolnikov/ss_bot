// определение режима соединения и подключение библиотеки RemoteXY 
#define REMOTEXY_MODE__SOFTSERIAL
#ifdef REMOTEXY_MODE__SOFTSERIAL
#include <SoftwareSerial.h>
#include <RemoteXY.h>

// настройки соединения 
#define REMOTEXY_SERIAL_RX A0
#define REMOTEXY_SERIAL_TX A1
#define REMOTEXY_SERIAL_SPEED 9600
  
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

    // other variable
  uint8_t connect_flag;  // =1 if wire connected, else =0 

} RemoteXY;
#pragma pack(pop)
#endif

/////////////////////////////////////////////
//           END RemoteXY include          //
/////////////////////////////////////////////

#include <AFMotor.h>   //shield L293D

enum motor_id {
  right_motor,
  left_motor
  };

enum directions {
  forward,
  backward
  };

class motor : public AF_DCMotor
{
  public:
  
    motor(uint8_t motornum):AF_DCMotor(motornum){}

    void motor_setup(float p, float i, uint8_t k)
    {
      setSpeed(255); // задаем максимальную скорость мотора
      run(RELEASE);   // останавливаем мотор 
      kp = p; 
      ki = i;
      k_sensor = k;
    }
    
    bool motor_direction = forward;
    volatile int32_t motor_position = 0;
    int32_t last_position = 0;
    double current_velocity = .0;
    uint32_t last_time = 0;
    uint8_t k_sensor = 0;  //non linear!
    
    //filter
    float filter_k = 0.2;  // коэффициент фильтрации, 0.0-1.0
    float buf[3];
    uint8_t count = 0;
    float filVal = .0;

    //PI controller
    float kp = 4.;//should be fine tuned
    float ki = 3.;//should be fine tuned
    float error_integral = .0;
     
     float PI_controller(int16_t target_velocity, uint32_t dt)
     {
        //---calculate error variables--
        float error = target_velocity - current_velocity;
        error_integral = constrain(error_integral + error*dt*0.000001, -255, 255);

//        Serial.print(error);
//        Serial.print(',');
//        Serial.println(error_integral);       
        
        return constrain(kp * error + ki*error_integral, -255, 255);
     }
     
     void GO (int16_t velocity)
     {
        //---calculate dt---
        uint32_t now = micros();
        uint32_t dt = now - last_time; //0.132s
        last_time = now;

        //---calculate current velocity---
        float k = ((float)k_sensor*2000)/dt; //250000
        current_velocity  = k*(motor_position - last_position);
        last_position  = motor_position;
        
        //---median filter---
        buf[count] = current_velocity;
        if (++count >= 3) count = 0;
        current_velocity = (max(buf[0], buf[1]) == max(buf[1], buf[2])) ? max(buf[0], buf[2]) : max(buf[1], min(buf[0], buf[2]));
      
        //---exponential moving average--- 
        filVal += (current_velocity - filVal) * filter_k;
        current_velocity = filVal;
        // Serial.print(current_velocity);
        // Serial.print(',');
        // Serial.println(velocity);
        
        if (velocity>0) {   
          velocity = (int16_t)PI_controller(velocity, dt);        
          motor_direction = forward;
          run(FORWARD);  // задаем движение вперед
          setSpeed(velocity);   // задаем скорость движения
        }
        else if (velocity<0) {
          velocity = (int16_t)PI_controller(velocity, dt);
          motor_direction = backward;
          run(BACKWARD);  // задаем движение вперед
          setSpeed(-velocity);   // задаем скорость движения
        }
        else {
          velocity = (int16_t)PI_controller(velocity, dt);
          run(RELEASE);
        }
     }
};

motor m[2] = {3,4};

/* defined the HALLs pins */
#define   PIN_HALL_SENSOR_MOTOR_L   3 
#define   PIN_HALL_SENSOR_MOTOR_R   2

void position_left_motor()
{
  if (m[left_motor].motor_direction == forward) m[left_motor].motor_position++; //if moving forwards, add counts
  else m[left_motor].motor_position--; //if moving back, subtract counts
}

void position_right_motor()
{
  if (m[right_motor].motor_direction == forward) m[right_motor].motor_position++; //if moving forwards, add counts
  else m[right_motor].motor_position--; //if moving back, subtract counts
}

/* defined the sonar control pins */
#define PIN_SONAR_TRIG A2
#define PIN_SONAR_ECHO A3

float Sonar ()
{

    static float filter_k = 0.2;  // коэффициент фильтрации, 0.0-1.0
    static float buf[3];
    static uint8_t count = 0;
    static float filVal = 0;
    float dist = 0;
  // Сначала генерируем короткий импульс длительностью 2-5 микросекунд.
  digitalWrite(PIN_SONAR_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_SONAR_TRIG, HIGH);

  // Выставив высокий уровень сигнала, ждем около 10 микросекунд. В этот момент датчик будет посылать сигналы с частотой 40 КГц.
  delayMicroseconds(10);
  digitalWrite(PIN_SONAR_TRIG, LOW);
  
  //  Время задержки акустического сигнала на эхолокаторе.
  //duration = pulseIn(PIN_ECHO, HIGH);

return pulseIn(PIN_SONAR_ECHO, HIGH)/ 58;
  // Теперь осталось преобразовать время в расстояние
//   dist = pulseIn(PIN_SONAR_ECHO, HIGH)/ 58;
//
//          //---median filter---
//        buf[count] = dist;
//        if (++count >= 3) count = 0;
//        dist = (max(buf[0], buf[1]) == max(buf[1], buf[2])) ? max(buf[0], buf[2]) : max(buf[1], min(buf[0], buf[2]));
//      
//        //---exponential moving average--- 
//        filVal += (dist - filVal) * filter_k;
//        return constrain(filVal, 0, 35);
}

/* IMU GY_85 */

#include "GY_85.h"
// Create module object
GY_85 GY85;

void setup() 
{
  #ifdef REMOTEXY_MODE__SOFTSERIAL
  RemoteXY_Init ();
  #endif
  Serial.begin (9600); 
  Serial.flush();
  //Serial.println("t,c");
  
  pinMode(PIN_SONAR_TRIG, OUTPUT);
  pinMode(PIN_SONAR_ECHO, INPUT);
  
  m[left_motor].motor_setup(1., 2., 125);
  m[right_motor].motor_setup(1.,2., 120);
  
  randomSeed(analogRead(0));
  
  pinMode(PIN_HALL_SENSOR_MOTOR_L, INPUT);
  digitalWrite(PIN_HALL_SENSOR_MOTOR_L, HIGH);//enable internal pullup resistor
  attachInterrupt(digitalPinToInterrupt(PIN_HALL_SENSOR_MOTOR_L), position_left_motor, RISING); 

  pinMode(PIN_HALL_SENSOR_MOTOR_R, INPUT_PULLUP);
  digitalWrite(PIN_HALL_SENSOR_MOTOR_R, HIGH);//enable internal pullup resistor
  attachInterrupt(digitalPinToInterrupt(PIN_HALL_SENSOR_MOTOR_R), position_right_motor, RISING);//Interrupt initialization

  // Initialize IMU module
  GY85.init();
}

void loop() 
{ 
  #ifdef REMOTEXY_MODE__SOFTSERIAL
  RemoteXY_Handler ();
  #endif

// 321 510


  static int8_t tick = 0;
  static int8_t k = -1; //get rid of it after PI controller
  static float dist = 0;
  static float prev_dist = 0;
  static int8_t sign = 1; 
  static bool escape_loop = 0;
  int16_t velocity_left = 0;
  int16_t velocity_right = 0;

  int* accelerometerReadings = GY85.readFromAccelerometer();
  int az = GY85.accelerometer_z(accelerometerReadings);
  if (az > 500) k = -1;
  if (az < 330) k = 1;

  tick++;
  if (tick>30 && (!escape_loop))
  {
    sign = random(10);
    if (sign > 5) sign = -1;
    else sign = 1;
    tick = 0;
  }
  
  prev_dist = dist;
  dist = Sonar();
 // Serial.println(dist);
  
  #ifdef REMOTEXY_MODE__SOFTSERIAL
  if (RemoteXY.select_1 == 0)
  {
  #endif
   if (escape_loop)
   {
//      /* manage the right motor */
//      m[right_motor].GO((-80)*sign);
//      /* manage the left motor */
//      m[left_motor].GO(80*sign*k);
      velocity_left = (-80)*sign*k;
      velocity_right = 80*sign*k;
      if (dist - prev_dist < 0) escape_loop = 0;
   }
   else 
      if (dist>40)
      {
//      /* manage the right motor */
//      m[right_motor].GO(250);
//      /* manage the left motor */
//      m[left_motor].GO(250*k);
        velocity_left = 200*k;
        velocity_right = 200*k;
      }
      else if (dist<40 && dist>15) {
//        /* manage the right motor */
//        m[right_motor].GO(10*dist + 20);
//        /* manage the left motor */
//        m[left_motor].GO((10*dist + 20)*k);
        velocity_left = (5*dist + 20)*k;
        velocity_right = (5*dist + 20)*k;
      }
      else {
//        /* manage the right motor */
//        m[right_motor].GO((-150)*sign);
//        /* manage the left motor */
//        m[left_motor].GO(150*sign*k);
        velocity_left = (-100)*sign*k;
        velocity_right = 100*sign*k;
        escape_loop = 1;
        //delay(100);//use without escape loop
      }
//    }
    // Задержка между измерениями для корректной работы
    //delay(100);//?

  //---motor diagnostic loop---
  static int16_t count = 0;
  count++;
//  m[left_motor].GO(250*sin((float)count/100));
//  m[right_motor].GO(250*cos((float)count/100));
//  
//  if (count >= 0 && count <= 200) {m[left_motor].GO(100);}
//  if (count > 200 && count <= 400) {m[left_motor].GO(0);}
//  if (count > 400 && count < 600) {m[left_motor].GO(-100);}
//  if (count >= 600) count = 0; 
  //----------------------------
  
  #ifdef REMOTEXY_MODE__SOFTSERIAL  
  }
  else {  
    // m[right_motor].GO(-(RemoteXY.joystick_1_y - RemoteXY.joystick_1_x));
    // m[left_motor].GO((RemoteXY.joystick_1_y + RemoteXY.joystick_1_x));
    velocity_left = 2*(RemoteXY.joystick_1_y - RemoteXY.joystick_1_x);
    velocity_right = 2*(RemoteXY.joystick_1_y + RemoteXY.joystick_1_x);
  }
  #endif

        /* manage the right motor */
   m[right_motor].GO(velocity_right);
        /* manage the left motor */
   m[left_motor].GO(velocity_left);
   
  //  Serial.print(m[left_motor].current_velocity);
  //  Serial.print(',');
  //  Serial.println(-velocity_left);
  //  Serial.println(-(RemoteXY.joystick_1_y + RemoteXY.joystick_1_x));
}
