//////////////////////
int cur_vel=0;
int ref_vel=0;
int kp=0;
int ki=0;
int kd=0;
int last_error=0;
int error_sum=0;
unsigned long time_pre;
int counter = 0;
//DEBUGGER IF TRUE LOTS OF PRINT
bool DEBUG = 1;
/*
if (DEBUG == 1){
  Serial.print("din mor");  
}
*/
//////////////////////
#include <krnl.h>
#include "pid_ctrl.h"


#define STK_SIZE 200
char s1[STK_SIZE]; // stak for t1 ... and t2 hevr 200 byte store
#define TASKPRIO 10
struct k_t *pTask1;

void task1()
{
    while (1)
    {   
        pid_ctrl(cur_vel, ref_vel, kp, ki, kd, last_error, error_sum);
        Serial.println("loop");
        k_eat_msec(1000);// IS ONLY HERE TO MAKE FUNCTION ABOVE READABLE              
    }
}

void setup()
{
    Serial.begin(115200);
    delay(2000);
    Serial.println("just bef init part");
    k_init(1, 0, 2); // 1 task, 0 semaphores, 2 messaegQueues */
    pTask1 = k_crt_task(task1, 15, s1, STK_SIZE);
    Serial.println("just bef k_start");
    pinMode(2, INPUT_PULLUP);
    pinMode(13, OUTPUT);
    k_start(); /* start krnl timer speed 1 milliseconds*/
    Serial.println("If you see this then krnl didnt start : -( ");
}

void loop() {}
