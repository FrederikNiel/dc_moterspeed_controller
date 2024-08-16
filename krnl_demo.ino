

#include <krnl.h>
#include <avr/wdt.h>
// External triggered ISR
// An Interrupt Service Routine is attached to pin 2
// So when pin2 is drived to ground (by a wire) an interrupt is generated.
// The ISR increment a counter and send it to a message Q
// naming ki_send .... "i" indicates it can be used in an ISR and demand interrupt to be disabled prio to call
// and that no task shift takes place in the call
// demonstrates ISR with  message Q and preemption(task shift) in the ISR
// NB Take a look on the ISR. For 1280 and 2560 it is INT4 but for 168,328,.. it's INTO
// It is taken care by a compile flag
// (c) beerware license JDN 2013

struct k_t *tSm1, *tSm2, *tSm3;
struct k_t *p_t1, *p_t2, *p_t3;

struct k_msg_t *pMsg, *pMsg2, *pMsg3;
char mar[10 * 2];
char mar2[10 * 2];
char mar3[10 * 2];

#define STK_SIZE 200

char s1[STK_SIZE]; // stak for t1 ... and t2
char s2[STK_SIZE];
char s3[STK_SIZE];

volatile int icnt = 0;

void doBlink(void)
{
    static char flag = 0;
    flag != flag;
    digitalWrite(13, flag);
}

void t1(void)
{
    int i;
    while (1)
    {
        delay(100);
        if (0 <= k_receive(pMsg2, &i, -1, NULL))
        {
            doBlink();
            k_sleep(100);
            DI();
            while (1)
                ;
        }
    }
}

void t2(void)
{
    int i;
    i = 0;
    while (1)
    {
        if (0 <= k_receive(pMsg, &i, 10, NULL))
        {
            Serial.println(i);
            k_send(pMsg2, &i);
        }
        else
        {
            Serial.println("-1");
        }
    }
}

void t3(void) // For reading motor information
{
    // Quad sensor uses 4 pins, GND, 3.3V, 2 Bool datapins
    // datapins are 4,5
    bool old1 = 0, old2 = 0;
#define sensorinput1 5
#define sensorinput2 5

    int i = 0;
    counter = 0;
    while (1)
    {
        if (0 <= k_receive(pMsg, &i, 10, NULL))
        {
            bool dataread1 = digitalRead(sensorinput1);
            if (old1 != dataread1)
            {
                if (dataread1 == digitalRead(sensorinput2))
                {
                    counter++;
                }
                else
                {
                    counter--;
                }
            }
            Serial.println(counter);
            k_send(pMsg2, &counter);
        }
        else
        {
            Serial.println("-1");
        }
    }
}

#if defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
ISR(INT4_vect, ISR_NAKED)
{
#else
ISR(INT0_vect, ISR_NAKED)
{
#endif
    // no local vars ?!?  ! I think
    PUSHREGS();
    if (!k_running)
        goto exitt;

    icnt++;
    ki_send(pMsg, (void *)&icnt);

    K_CHG_STAK();

exitt:

    POPREGS();
    RETI();
}

void installISR2()
{
    DI();
    pinMode(2, INPUT);     // set som input
    digitalWrite(2, HIGH); // enable pullup resistor

#if defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
    // 1280/2560 mega pin2 intr:4, pin5 intr:5
    EIMSK |= (1 << INT4);  // enable external intr
    EICRB |= (1 << ISC41); // trigger INT4 on falling edge
#else
    EIMSK |= (1 << INT0);  // enable external intr
    EICRA |= (1 << ISC01); // trigger INT0 on falling edge
#endif

    EI();
}

void setup()
{
    wdt_reset();
    wdt_disable();

    Serial.begin(9600);
    k_init(2, 5, 2); // from now you can crt task,sem etc

    tSm1 = k_crt_sem(0, 10); //
    tSm2 = k_crt_sem(0, 10); //
    p_t1 = k_crt_task(t1, 10, s1, STK_SIZE);
    p_t2 = k_crt_task(t2, 9, s2, STK_SIZE);

    pMsg = k_crt_send_Q(10, 2, mar);
    pMsg2 = k_crt_send_Q(10, 2, mar2);
    pinMode(13, OUTPUT);
    Serial.print("start ");
    Serial.println(KRNL_VRS);
    delay(2000);

    installISR2();
    Serial.println("bef gogo");

    wdt_enable(WDTO_60MS);
    k_start(10); // now we are runnning   with timer 10 msev
    Serial.println("shit - should not come here");

    // main will not come back and will sleep rest of life
}

void loop(void)
{ /* just for compilation - will never be called*/
}

/* QED :-) */

xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx NEXT xxxxxxxxxxxxxxxxxxxxxxx

#include <krnl.h>

// definerer 1 bogstavs kommandoer

#define LED13ON '1'
#define LED13OFF '2'
#define ANALOG0 '3'
#define ANALOG1 '4'
#define ANALOG2 '5'

#define DIGINPUT8 '6'
#define DIGINPUT9 '7'

    struct k_t *p_t1,
    *p_t2;

struct k_msg_t *pMsg1, *pMsg2;
char mar1[20 * 1];
char mar2[20 * 1];

#define STK_SIZE 200

char s1[STK_SIZE]; // stak for t1 ... and t2 hevr 200 byte store
char s2[STK_SIZE];

volatile int icnt = 0;

// TASK 1
void t1(void)
{
    char dataFromBuf;
    while (1)
    {
        delay(10);
        if (0 <= k_receive(pMsg1, &dataFromBuf, 0, NULL))
        {

            // modtaget en byte
            Serial.write(dataFromBuf); // send denpå seriel
            char cmd;
            while (Serial.available())
            {                        // er der karakterer
                cmd = Serial.read(); // vi laeser en karakter fra input

                switch (cmd)
                {
                case LED13ON:
                    digitalWrite(13, HIGH);
                    break;
                case LED13OFF:
                    digitalWrite(13, LOW);
                    break;
                case ANALOG0:
                    Serial.println(analogRead(0));
                    break;
                case ANALOG1:
                    Serial.println(analogRead(1));
                    break;
                case ANALOG2:
                    Serial.println(analogRead(2));
                    break;
                case DIGINPUT8:
                {
                    int level;
                    level = digitalRead(8);
                    if (level == HIGH)
                        Serial.println("+");
                    else
                        Serial.println("-");
                }
                break;

                case DIGINPUT9:
                {
                    int level;
                    level = digitalRead(9);
                    if (level == HIGH)
                        Serial.println("+");
                    else
                        Serial.println("-");
                }
                break;
                }
            }
        }
    }
}

void t2(void)
{
    char data;

    while (1)
    {
        k_sleep(10); // vent lige  10 tick

        // laes alt fra serial buffer og gem det i msg Q
        while (0 < Serial.available())
        {
            data = Serial.read();
            k_send(pMsg1, &data);

            if (data == 'a')
                digitalWrite(13, HIGH);
            else if (data == 'b')
                digitalWrite(13, LOW);
        }
    }
}
void setup()
{

    Serial.begin(9600);

    k_init(2, 0, 2); // from now you can crt task,sem etc

    p_t1 = k_crt_task(t1, 10, s1, STK_SIZE); // funktion t1 som task
    p_t2 = k_crt_task(t2, 9, s2, STK_SIZE);  // funktion t2 som task

    pMsg1 = k_crt_send_Q(20, 1, mar1); // buffer med 20 elementer hver 1 byte stor
    pMsg2 = k_crt_send_Q(20, 1, mar2); // det samme her

    pinMode(13, OUTPUT); // let there be LED
    digitalWrite(13, LOW);

    Serial.print("start ");
    Serial.println(KRNL_VRS); // kernel version

    k_start(10); // now we are runnning   with timer 10 msev
    Serial.println("shit - should not come here");

    // main will not come back and will sleep rest of life
}

void loop(void) {} /* just for compilation - will never be called*/

xxxxxxxxxxxxxxxxxxxxxxxxxxNEXT EX xxxxxxxxxxxxxxxxxxx

// Main Goal(s):
// Receive messages in ISR (from task)
// DONE -------------------------------------------  Send reply by use of another Q
// Specifications:
//   DONE -------------------------------------------  message Q shall consists of 7 int’s (from task to ISR
//   DONE -------------------------------------------  another message Q form ISR to task  of size 6 int’s
//   DONE -------------------------------------------  ISR is shall be triggered by external interrupt pulling line low
//   DONE -------------------------------------------  ISR shall try to receive a message from a krnl msgQ
//   DONE -------------------------------------------  If value 123 received, increment local counter - send this counter back to task
//   DONE -------------------------------------------  task loops every second
//   DONE -------------------------------------------  use random for interval 122,124 - both inclusive
//   DONE -------------------------------------------  Send value to ISR by Q the task->ISR Q
//   DONE -------------------------------------------  Check if anythings has arrived int the ISR-task Q
//   DONE -------------------------------------------  Print received value or print “no msg for me”

#include <krnl.h>
#define STK 200
#define TASKPRIO 10

    struct k_t *pTask1;
struct k_msg_t *msgQ;    // task to the ISR             //struct in the kernel library
struct k_msg_t *msgQISR; // ISR to task
const int BUFSZ = 7;
const int ISRBUF = 6;
int bufForQ[BUFSZ];
int ISRmsgQ[ISRBUF];
int ISRoverflow = 0;
int nrISR = 0;
volatile int counter = 0;
unsigned long last = 0;

volatile int intCnt = 0;

// ISR
ISR(INT0_vect, ISR_NAKED)
{

    // digital pin 2 for uno;  ISR_NAKED is for the compiler not to generate intro and exit codE
    PUSHREGS(); // A processor register is a quickly accessible location available to a computer's processor. Here we just order to save the data in registers.
    if (!k_running)
    {
        goto exitt;
    }

    intCnt++;

    nrISR = random(122, 125); // Iterate the nrISR variable
    ki_send(msgQ, &nrISR);    // Puts data in the ringbuffer if there is room for it. msgQ is the buffer, and &nrISR is the referece to data to be put in the buffer

    if (nrISR == 123)
    {
        counter++;
        ki_send(msgQ, &counter);
    }

    int resISR;
    int lost;

    // I MAA IKKE BRUGE K_receive
    // HVIS I VENTER I EN ISR DØR I MASKINEN
    //  if (1 == k_receive(msgQISR, &resISR, 1000, &lost) || 0 == k_receive(msgQISR, &resISR, 1000, &lost))
    if (1 == ki_receive(msgQISR, &resISR, &lost))
    {
        // Serial.print("Got msg ISR: "); // DU MAA IKK EPRINTE I EN ISR !!!!
        // Serial.print(resISR);
        // Serial.print(" - lost since last ISR ");
        // Serial.println(lost);
        // digitalWrite(13, HIGH);  // KAN I NÅ AT SE DEN TÆNDT ???
    }
    else
    {
        // digitalWrite(13, LOW);
    }

    K_CHG_STAK();

exitt:
    POPREGS(); // The popregs is often used at the exit of the subroutine to restore registers before the rts instruction
    RETI();    // The RETI instruction is used to end an interrupt service routine
}

void task1()
{
    int res = 0;
    int t = 0;
    int lost;
    while (1)
    {
        Serial.println("loop");
        if (0 > k_receive(msgQ, &res, 1000, &lost))
        { // next time nr lost is received inside msg Q system
            t++;
            Serial.print("No message - timeout for: ");
            Serial.println(t);
        }
        else
        {
            Serial.print("Got msg: ");
            Serial.print(res);
            Serial.print(" - lost since last ");
            Serial.print(lost);
            Serial.print(" int cnt ");
            Serial.println(intCnt);

            // ki_send(msgQISR, &t);   // DU MAA IKKE BRUGE kI kald fra USER SPACE da de ikke disabler interrupt det gør man i k_send
            k_send(msgQISR, &t);
            k_receive(msgQISR, &t, 10, NULL);
        }
    }
}

void setup()
{
    Serial.begin(115200);
    delay(2000);
    Serial.println("just bef init part");
    k_init(1, 0, 2); // 1 task, 0 semaphores, 2 messaegQueues */
    pTask1 = k_crt_task(task1, 15, STK);
    Serial.println("just bef k_start");
    pinMode(2, INPUT_PULLUP);
    pinMode(13, OUTPUT);
    noInterrupts();
    msgQ = k_crt_send_Q(BUFSZ, sizeof(int), bufForQ);
    msgQISR = k_crt_send_Q(ISRBUF, sizeof(int), ISRmsgQ);

    // or from krnl  DI();
    EICRA |= (1 << ISC01); // trigger INT0 on falling edge
    // https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7810-Automotive-Microcontrollers-ATmega328P_Datasheet.pdf  p 54
    EIMSK |= (1 << INT0); // enable external intr
    interrupts();
    k_start(1); /* start krnl timer speed 1 milliseconds*/
    Serial.println("If you see this then krnl didnt start : -( ");
}

void loop() {}

xxxxxxxxxxxxxxxxxxxxxxxxx NEXT EX xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
