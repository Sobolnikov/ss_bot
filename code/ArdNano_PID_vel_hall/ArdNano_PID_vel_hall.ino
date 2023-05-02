//const int pwm = 2 ; //initializing pin 2 as pwm
// Motor connections
#define   IN_A   5
#define   IN_B   6
#define   EN_A   10

#define   IN_C   7
#define   IN_D   8
#define   EN_B   9

#define   BUTTON_A   11
#define   BUTTON_B   12

#define   HALL_A   3
#define   HALL_B   2
//For providing logic to L298 IC to choose the direction of the DC motor

volatile int count_a = 0;//if the interrupt will change this value, it must be volatile

volatile int count_b = 0;//if the interrupt will change this value, it must be volatile

int sw[] = {1, 1}; //switch up, switch down

int currentPos = 0;//current position
int threshold = 1;
int destination = 0;

int SampleTime = 5190; //1 sec

bool forwards = false;
bool backwards = false;// motor states

bool firstRun = true;//first run of the motor once the button is pushed

/*PID variables*/
unsigned long lastTime = 0;
int lastPose = 0;
unsigned long lastErr;
double kierrSum;

double kp = 5;
double ki = 0;
double kd = 0;

void setup() {
  pinMode(EN_A, OUTPUT); //we have to set PWM pin as output
  pinMode(IN_A, OUTPUT); //Logic pins are also set as output
  pinMode(IN_B, OUTPUT);
  
  pinMode(BUTTON_A, INPUT);
  pinMode(BUTTON_B, INPUT);
  digitalWrite(BUTTON_A, HIGH);
  digitalWrite(BUTTON_B, HIGH);//set up/down switch, enable enternal relays  
  
  
  pinMode(HALL_B, INPUT);
  digitalWrite(HALL_B, HIGH);//enable internal pullup resistor
  attachInterrupt(digitalPinToInterrupt(HALL_B), angle_b, RISING);//Interrupt initialization
  
  pinMode(HALL_A, INPUT);
  digitalWrite(HALL_A, HIGH);//enable internal pullup resistor
  attachInterrupt(digitalPinToInterrupt(HALL_A), angle_a, RISING);
   
  Serial.begin (9600);
}

void angle_a()
{
  if (forwards == true) count_a++; //if moving forwards, add counts
  else if (backwards == true) count_a--; //if moving back, subtract counts
}

void angle_b()
{
 count_b = count_b+1;
}

void PID()
{
   /*How long since we last calculated*/
  unsigned long now = micros();
  unsigned long timeChange = now - lastTime;
  if(timeChange>=SampleTime)
  {
  
    /*Compute all the working error variables*/
    int error = destination - currentPos;
    kierrSum += ki*error;
    if (kierrSum>255) kierrSum=255;
    else if (kierrSum<-255) kierrSum=-255;
    int dErr = error - lastErr;

    int velocity = currentPos - lastPose;

    /*Remember some variables for next time*/
    lastErr = error;
    lastTime = now;
    lastPose = currentPos;
    
    
    if (velocity != 0)  
    {
      Serial.print(velocity);
      Serial.print(" ");
    }
    /*Compute PID Output*/
    double u = kp * error + kierrSum + kd * dErr;
    go(u);
  }
}

void go(int v)
{
  if (v>255) v=255;
  if (v<-255) v=-255;
  
  if (v>0) {
    forwards = true;
    backwards = false;
    digitalWrite(IN_A, LOW);
    digitalWrite(IN_B, HIGH);
  }
  else if (v<0) {
    forwards = false;
    backwards = true;
    digitalWrite(IN_B, LOW);
    digitalWrite(IN_A, HIGH);
  }
  else {
    forwards = false;
    backwards = false;
    digitalWrite(IN_B, LOW);
    digitalWrite(IN_A, LOW);
  }
  analogWrite(EN_A, abs(v));
}

void ReadInputs() {
  sw[0] = digitalRead(BUTTON_A), sw[1] = digitalRead(BUTTON_B); //check switches
  currentPos = count_a;     //set where you are
}//end read inputs

void loop() {

  ReadInputs();//check input button, calculate speeds

  if (sw[0] == 0 && sw[1] == 1 && backwards == false) destination = currentPos - 200;//dont change destination while moving
  else if (sw[0] == 1 && sw[1] == 0 && forwards == false) destination = currentPos + 200;//dont change destination while moving

  PID();
}
