// определение режима соединения и подключение библиотеки RemoteXY 
#define REMOTEXY_MODE__SOFTSERIAL
#ifdef REMOTEXY_MODE__SOFTSERIAL
#include <SoftwareSerial.h>
#include <RemoteXY.h>

// настройки соединения 
#define REMOTEXY_SERIAL_RX A0
#define REMOTEXY_SERIAL_TX 13//???????????????????????
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

#include <AFMotor.h>   // подключаем библиотеку для шилда L293D
AF_DCMotor motor_l(1); // подключаем мотор к клеммникам M1
AF_DCMotor motor_r(2); // подключаем мотор к клеммникам M2

enum motors_id {
  left_motor,
  right_motor
  };

AF_DCMotor motors[2] = {1, 2};

/* defined the HALLs pins */
#define   PIN_HALL_SENSOR_MOTOR_L   2 //CHECK?
#define   PIN_HALL_SENSOR_MOTOR_R   3

volatile int position_motor_l = 0;//if the interrupt will change this value, it must be volatile
volatile int position_motor_r = 0;//if the interrupt will change this value, it must be volatile
bool forwards = false;
bool backwards = false;// motor states

void position_a()
{
  if (forwards == true) position_motor_l++; //if moving forwards, add counts
  else if (backwards == true) position_motor_l--; //if moving back, subtract counts
}

void position_b()
{
  if (forwards == true) position_motor_r++; //if moving forwards, add counts
  else if (backwards == true) position_motor_r--; //if moving back, subtract counts
}

float current_velocity_l = 0;
float current_velocity_r = 0;

void calculate_velocities()
{
  static unsigned long lastTime = 0;
  unsigned long now = micros();
  unsigned long dt = now - lastTime;
  static float last_position_l = 0;  
  static float last_position_r = 0;
  current_velocity_l = (position_motor_l - last_position_l)/dt;
  current_velocity_r = (position_motor_r - last_position_r)/dt;
  last_position_l = position_motor_l;  
  last_position_r = position_motor_r;
}

int SampleTime = 5190; //1 sec?????????
byte kp = 5;
byte ki = 0;
byte kd = 0;

int PI_controller(int target_velocity, int current_velocity)
{
   /*How long since we last calculated*/
  static unsigned long lastTime = 0;
  unsigned long now = micros();
  unsigned long timeChange = now - lastTime;
  if(timeChange>=SampleTime)//Why????????
  {
    /*Compute all the working error variables*/
    int error = target_velocity - current_velocity;
    static int error_integral = 0;
    error_integral = constrain(error_integral + error*timeChange, -255, 255);

    /*Remember some variables for next time*/
    lastTime = now;

    /*Compute PI Output*/
    return constrain(kp * error + ki*error_integral, -255, 255);
  }
}
  
void Wheel (AF_DCMotor * motor, int v)
{
  v = constrain(v, -255, 255);
  if (v>0) {
    forwards = true;
    backwards = false;
    motor->run(FORWARD);  // задаем движение вперед
    motor->setSpeed(v);   // задаем скорость движения
  }
  else if (v<0) {
    forwards = false;
    backwards = true;
    motor->run(BACKWARD);  // задаем движение вперед
    motor->setSpeed(-v);   // задаем скорость движения
  }
  else {
    forwards = false;
    backwards = false;
    motor->run(RELEASE);
  }
}

/* defined the sonar control pins */
#define PIN_SONAR_TRIG A4
#define PIN_SONAR_ECHO A5

long Sonar ()
{
  //long duration;
  // Сначала генерируем короткий импульс длительностью 2-5 микросекунд.
  digitalWrite(PIN_SONAR_TRIG, LOW);
  delayMicroseconds(5);
  digitalWrite(PIN_SONAR_TRIG, HIGH);

  // Выставив высокий уровень сигнала, ждем около 10 микросекунд. В этот момент датчик будет посылать сигналы с частотой 40 КГц.
  delayMicroseconds(10);
  digitalWrite(PIN_SONAR_TRIG, LOW);
  
  //  Время задержки акустического сигнала на эхолокаторе.
  //duration = pulseIn(PIN_ECHO, HIGH);

  // Теперь осталось преобразовать время в расстояние
  return pulseIn(PIN_SONAR_ECHO, HIGH)/ 58;
}


void setup() 
{
  #ifdef REMOTEXY_MODE__SOFTSERIAL
  RemoteXY_Init ();
  #endif
  Serial.begin (9600); 

  pinMode(PIN_SONAR_TRIG, OUTPUT);
  pinMode(PIN_SONAR_ECHO, INPUT);
  motor_l.setSpeed(255); // задаем максимальную скорость мотора
  motor_l.run(RELEASE);   // останавливаем мотор
  motor_r.setSpeed(255); // задаем максимальную скорость мотора
  motor_r.run(RELEASE);   // останавливаем мотор
  
  randomSeed(analogRead(0));
  
  pinMode(PIN_HALL_SENSOR_MOTOR_L, INPUT);
  digitalWrite(PIN_HALL_SENSOR_MOTOR_L, HIGH);//enable internal pullup resistor
  attachInterrupt(digitalPinToInterrupt(PIN_HALL_SENSOR_MOTOR_L), position_a, RISING); 

  pinMode(PIN_HALL_SENSOR_MOTOR_R, INPUT_PULLUP);
  digitalWrite(PIN_HALL_SENSOR_MOTOR_R, HIGH);//enable internal pullup resistor
  attachInterrupt(digitalPinToInterrupt(PIN_HALL_SENSOR_MOTOR_R), position_b, RISING);//Interrupt initialization
}

byte tick = 0;
const float k = -0.8;
long dist = 0;
int sign = 1; 
bool escape_loop = 0;
long prev_dist;

void loop() 
{ 
  #ifdef REMOTEXY_MODE__SOFTSERIAL
  RemoteXY_Handler ();
  #endif

  tick++;
  if (tick>30)
  {
    sign = random(10);
    if (sign > 5) sign = -1;
    else sign = 1;
    tick = 0;
  }
  
  prev_dist = dist;
  dist = Sonar();
  Serial.print("a = ");
  Serial.print(position_motor_l);
  Serial.print("\n");
  Serial.print("b = ");
  Serial.print(position_motor_r);
  Serial.print("\n \n");

  calculate_velocities();
  
  #ifdef REMOTEXY_MODE__SOFTSERIAL
  if (RemoteXY.select_1 == 0)
  {
  #endif
    if (escape_loop)
    {
      /* manage the right motor */
      Wheel (&motor_r, (-80)*sign);
      /* manage the left motor */
      Wheel (&motor_l, 80*sign*k);
      if (dist - prev_dist < 0) escape_loop = 0;
    }
    else {
      if (dist>40)
      {
      /* manage the right motor */
      Wheel (&motor_r, 100);
      /* manage the left motor */
      Wheel (&motor_l, 100*k);
      }
      else if (dist<40 && dist>15) {
        /* manage the right motor */
        Wheel (&motor_r, 2*dist + 20);
        /* manage the left motor */
        Wheel (&motor_l, (2*dist + 20)*k);
      }
      else {
        /* manage the right motor */
        Wheel (&motor_r, (-80)*sign);
        /* manage the left motor */
        Wheel (&motor_l, 80*sign*k);
        escape_loop = 1;
        //delay(100);//use without escape loop
      }
    }
    // Задержка между измерениями для корректной работы
    delay(100);//?????????????????????
  #ifdef REMOTEXY_MODE__SOFTSERIAL  
  }
  else {
    Wheel (&motor_r, RemoteXY.joystick_1_y - RemoteXY.joystick_1_x);
    Wheel (&motor_l, RemoteXY.joystick_1_y + RemoteXY.joystick_1_x);
  }
  #endif
}
