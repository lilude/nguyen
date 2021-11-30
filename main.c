//#include <stdarg.h>
#include <signal.h>
#include <unistd.h>
#include <time.h>
#include <math.h>

#include <pigpio.h>


struct timespec tic, toc;

#define MOTOR0_EN 23
#define MOTOR1_EN 26

#define MOTOR0_PWM 18
#define MOTOR1_PWM 19

#define ENC0_A 13 
#define ENC0_B 6 

#define ENC1_A 5 
#define ENC1_B 22

int run = 1;


int GpioInit();
void DisableMotors();
void EnableMotors();
void SetValues();
void Control();
double TestInput(double t);

long ENC0_COUNT;
uint32_t ENC0_TIMER;
uint32_t ENC0_TIMER_PREV;

long ENC1_COUNT;
uint32_t ENC1_TIMER;
uint32_t ENC1_TIMER_PREV;

typedef struct motor {
  double q;
  double qPrev;
  double dq;
  double u;
}MOTOR;

MOTOR m0, m1;

void CallbackENC0(int gpio, int level, uint32_t tick)
{
  static int A, B, AB;
  static int last = -1;

  if (gpio == ENC0_A)
    A = level;
  else
    B = level;

  AB = 2*A + B;

  if(gpio != last) {

    if(gpio == ENC0_A && level != 2 ){

      if( AB == 0b10 )
        ENC0_COUNT++;
      else if ( AB == 0b01 )
        ENC0_COUNT++;
      else if ( AB == 0b11 )
        ENC0_COUNT--;
      else if ( AB == 0b00 )
        ENC0_COUNT--;

    }
    else if(gpio == ENC0_B && level != 2){

      if( AB == 0b10 )
        ENC0_COUNT--;
      else if ( AB == 0b01 )
        ENC0_COUNT--;
      else if ( AB == 0b11 )
        ENC0_COUNT++;
      else if ( AB == 0b00 )
        ENC0_COUNT++;

    }
    
    ENC0_TIMER = tick;

  }
}

void CallbackENC1(int gpio, int level, uint32_t tick)
{
  static int A, B, AB;
  static int last = -1;

  if (gpio == ENC1_A)
    A = level;
  else
    B = level;

  AB = 2*A + B;

  if(gpio != last) {

    if(gpio == ENC1_A && level != 2){

      if( AB == 0b10 )
        ENC1_COUNT++;
      else if ( AB == 0b01 )
        ENC1_COUNT++;
      else if ( AB == 0b11 )
        ENC1_COUNT--;
      else if ( AB == 0b00 )
        ENC1_COUNT--;

    }
    else if(gpio == ENC1_B && level != 2){

      if( AB == 0b10 )
        ENC1_COUNT--;
      else if ( AB == 0b01 )
        ENC1_COUNT--;
      else if ( AB == 0b11 )
        ENC1_COUNT++;
      else if ( AB == 0b00 )
        ENC1_COUNT++;

    }
    
    ENC1_TIMER = (unsigned long) tick;
  }
}


void stop(int signum)
{
  run = 0;
  DisableMotors();
}


int main()
{
  long diff;

  if (gpioInitialise()<0) return 1;

  gpioSetSignalFunc(SIGINT, stop);

  GpioInit();

  
  clock_gettime(CLOCK_MONOTONIC, &tic);
  EnableMotors();
  while(run) {
    SetValues();
    Control(); 
    clock_gettime(CLOCK_MONOTONIC, &toc);

    diff = (toc.tv_nsec - tic.tv_nsec); 
    tic = toc;

    if (diff < 0)
      diff = 1e9 + diff;

    printf("%ld %le %le %le %le\t", diff, m0.q, m0.dq, m1.q, m1.dq); 
    printf("\n");
    gpioDelay(1000);

  }
  gpioTerminate();


}


void EnableMotors()
{
  gpioHardwarePWM(MOTOR0_PWM, 1000, 1e6/2);
  gpioHardwarePWM(MOTOR1_PWM, 1000, 1e6/2);

  gpioWrite(MOTOR0_EN, 1);
  gpioWrite(MOTOR1_EN, 1);
}


void DisableMotors()
{

  gpioWrite(MOTOR0_EN, 0);
  gpioWrite(MOTOR1_EN, 0);
  gpioHardwarePWM(MOTOR0_PWM, 1000, 1e6/2);
  gpioHardwarePWM(MOTOR1_PWM, 1000, 1e6/2);

}

void SetValues()
{
  
  int diff;
  static int first;

  if (first == 0){
    first = 1;
    ENC0_TIMER_PREV = ENC0_TIMER;
    ENC1_TIMER_PREV = ENC1_TIMER;
    m0.qPrev = m0.qPrev;
    m1.qPrev = m1.qPrev;
  }

  //Set m0 values
  m0.q = 2*M_PI/(100.0*4.0)*ENC0_COUNT/14.0;

  diff = ENC0_TIMER - ENC0_TIMER_PREV;
  ENC0_TIMER_PREV = ENC0_TIMER;

  if (diff == 0)
    diff = 1;
  
  m0.dq = (m0.q - m0.qPrev)/diff*1e6;
  m0.qPrev = m0.q;


  //set m1 values
  m1.q = 2*M_PI/(100.0*4.0)*ENC1_COUNT/14.0;
  
  diff = ENC1_TIMER - ENC1_TIMER_PREV;
  ENC1_TIMER_PREV = ENC1_TIMER;

  if (diff == 0)
    diff = 1;
  
  m1.dq = (m1.q - m1.qPrev)/diff*1e6;
  m1.qPrev = m1.q;

}
  


void Control()
{

  static int first;
  static double t;
  double pwm0, pwm1;
  double qD, dqD;
  double A,w;
  
  if(first == 0) {
    first = 1;
  }

  t += 1e-3;

  A = M_PI/2;
  w = 2*M_PI*1.0;
  qD = A*sin(w*t);
  dqD = A*w*cos(w*t);
  
  m0.u = 0.4*(m0.q - qD) + 0.008*(m0.dq - dqD);

  if (m0.u > 0.4)
    m0.u = 0.4;
  else if (m0.u < -0.4)
    m0.u = -0.4;
    
  pwm0 = 1e6/2.0 + 1e6/2.0*m0.u;

  A = M_PI/2;
  w = 2*M_PI*1.0;
  qD = A* ( (sin(w*t) > 0.0) ? 1.0 : -1.0);
  dqD = A*w*cos(w*t)*0;

  m1.u = 0.4*(m1.q - qD) + 0.008*(m1.dq - dqD);

  if (m1.u > 0.4)
    m1.u = 0.4;
  else if (m1.u < -0.4)
    m1.u = -0.4;
    
  printf("%e\t",qD);
  pwm1 = 1e6/2.0 + 1e6/2.0*m1.u;

  gpioHardwarePWM(MOTOR0_PWM, 1000, pwm0);
  gpioHardwarePWM(MOTOR1_PWM, 1000, pwm1);

}


int GpioInit()
{

  gpioSetMode(MOTOR0_EN, PI_OUTPUT);
  gpioWrite(MOTOR0_EN, 0);

  gpioSetMode(MOTOR1_EN, PI_OUTPUT);
  gpioWrite(MOTOR1_EN, 0);

  gpioHardwarePWM(MOTOR0_PWM, 1000, 1e6/2);
  gpioHardwarePWM(MOTOR1_PWM, 1000, 1e6/2);

  gpioSetMode(ENC0_A, PI_INPUT);
  gpioSetPullUpDown(ENC0_A, PI_PUD_UP);

  gpioSetMode(ENC0_B, PI_INPUT);
  gpioSetPullUpDown(ENC0_B, PI_PUD_UP);

  gpioSetMode(ENC1_A, PI_INPUT);
  gpioSetPullUpDown(ENC1_A, PI_PUD_UP);

  gpioSetMode(ENC1_B, PI_INPUT);
  gpioSetPullUpDown(ENC1_B, PI_PUD_UP);

  gpioSetAlertFunc(ENC0_A, CallbackENC0);
  gpioSetAlertFunc(ENC0_B, CallbackENC0);

  gpioSetAlertFunc(ENC1_A, CallbackENC1);
  gpioSetAlertFunc(ENC1_B, CallbackENC1);

  return 0;
}


