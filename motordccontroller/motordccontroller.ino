// Declare variables and constants
float kp = 0.6;                  // Proportional gain for PID controller
float ki = 0.1;                  // Integral gain for PID controller
float kd = 0.1;                  // Derivative gain for PID controller
int last_error = 0;              // Error from previous iteration of PID controller
int error_sum = 0;               // Sum of errors for integral control
unsigned long time_pre;          // Previous time for calculating time difference in PID controller
int encdr_res = 500;             // Encoder resolution
float PWM_TO_RPS_FACTOR = 0.0118;// Factor for converting PWM signal to revolutions per second
int counter = 0;                 // Counter for encoder ticks

// Declare structures and arrays for message queue and PID setter
struct k_msg_t *pMsg1, *pMsg2, *pMsg3;
char mar1[20 * 1];
char mar2[20 * 1];
char mar3[20 * 1];
const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];
boolean newData = false;

// Declare debugging flags and pin assignments
bool DEBUG = 0;
bool TIMING = 0;
#define mot_sgn 12    // Digital pin for motor direction sign
#define mot_pwm 10    // PWM value analog pin for motor
#define tch_mtr_Q2 2  // Tachometer input pin for encoder
#define tch_mtr_Q4 9  // Tachometer direction pin for encoder

// Include kernel library
#include <krnl.h>

#define STK_SIZE 200 // Stack size for tasks
char s1[STK_SIZE], s2[STK_SIZE], s3[STK_SIZE];  // stak for t1, t2, and t3

struct k_t *pTask1, *pTask2, *pTask3;

////// Interrupt service routine for encoder //////
#if defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
ISR(INT4_vect, ISR_NAKED) {
#else
ISR(INT0_vect, ISR_NAKED) {
#endif
  // Save registers on stack
  PUSHREGS();
  if (!k_running)
    goto exitt;

  // Update encoder tick counter
  if (digitalRead(tch_mtr_Q4)) {
    counter++;
  }
  else {
    counter--;
  }

  // Print encoder count for debugging
  if (DEBUG == 1) {
    Serial.print(F("counter"));
    Serial.println(counter);
  }

  // Release stack space
  K_CHG_STAK();

exitt:
  // Restore registers and return from ISR
  POPREGS();
  RETI();
}

////// task 1: PID controller //////
void task1() {
  // This task calculates the PID control output for the motor, based on the difference between the reference velocity and the current velocity of the motor.
  while (1) {
    // Task timing for schedulability tests
    unsigned long start_time; 
    if (TIMING == 1) {
      start_time = k_millis();
    }

    // Calculate the time difference since the last loop iteration
    unsigned long time_diff = k_millis() - time_pre;
    time_pre = k_millis();

    // Calculate the current velocity of the motor using the encoder counter and the time difference
    float cur_vel = (counter / encdr_res) / (time_diff); 

    // Reset the encoder counter
    counter = 0;

    // Send the current velocity to the message queue
    k_send(pMsg2, &cur_vel);

    // Read the reference velocity from the potentiometer and map it to a range of -350 to 350
    int ref = map(analogRead(A0), 0, 700, -350, 350);

    // Convert the reference velocity to RPM and send it to the message queue
    float ref_vel = ref * PWM_TO_RPS_FACTOR;
    k_send(pMsg1, &ref_vel);

    // Calculate the error between the reference velocity and the current velocity
    float error = ref_vel - cur_vel;

    // Calculate the error sum and error change for the integral and derivative terms of the PID controller
    error_sum += error * time_diff;
    float error_chng = (error - last_error) / time_diff;
    last_error = error;

    // Calculate the PID control output
    float pid = kp * error + ki * error_sum + kd * error_chng;

    // Send the PID control output to the message queue
    k_send(pMsg3, &pid);

    // Limit the PID control output to the range of -255 to 255
    if (pid > 255) {
      pid = 255;
    } else if (pid < -255) {
      pid = -255;
    }

    // Set the direction of the motor based on the sign of the PID control output
    if (pid > 0) {
      digitalWrite(mot_sgn, HIGH);
      analogWrite(mot_pwm, abs(pid));
    } else {
      digitalWrite(mot_sgn, LOW);
      analogWrite(mot_pwm, abs(pid));
    }

    // Task timing for schedulability tests
    if (TIMING == 1) {
      unsigned long end_time = k_millis();
      unsigned long execution_time = end_time - start_time;
      Serial.print(F("Task1 PID execution time: "));
      Serial.println(execution_time);
    }
    
    // Sleep for 10 milliseconds
    k_sleep(10);
  }
}

////// task 2: Logger //////
void task2() {
  // This task logs the reference velocity, current velocity, and PWM value in the Serial Monitor every second.
  while (1) {
    // Task timing for schedulability tests
    unsigned long start_time;
    if (TIMING == 1) {
      start_time = k_millis();
    }

    // Define variables for receiving data from message queue
    float data1;  // Reference velocity
    float data2;  // Current velocity
    float data3;  // PWM value

    // Print debug message
    if (DEBUG == 1) {
      Serial.println("LOGGER!!!! Included!");
    }

    // Receive data from message queue pMsg1 (reference velocity)
    k_receive(pMsg1, &data1, 0, NULL);
    Serial.print("Reference Velocity: ");
    Serial.println(data1);

    // Receive data from message queue pMsg2 (current velocity)
    k_receive(pMsg2, &data2, 0, NULL);
    Serial.print("Current Velocity: ");
    Serial.println(data2);

    // Receive data from message queue pMsg3 (PWM value)
    k_receive(pMsg3, &data3, 0, NULL);
    Serial.print("PWM Value: ");
    Serial.println(data3);

    // Task timing for schedulability tests
    if (TIMING == 1) {
      unsigned long end_time = k_millis();
      unsigned long execution_time = end_time - start_time;
      Serial.print(F("Task2 Logger execution time: "));
      Serial.println(execution_time);
    }

    // Sleep for 1000 milliseconds (1 second)
    k_sleep(1000);
  }
}


////// task 3: change PID constants //////
void task3() {
  // Task to read serial for new PID constants
  while (1) {
    // Task timing for schedulability tests
    unsigned long start_time;
    if (TIMING == 1) {
      start_time = k_millis();
    }

    // Debugging messages to confirm the current values of the PID constants
    if (DEBUG == 1) {
      Serial.println(kp);
      Serial.println(ki);
      Serial.println(kd);
    }

    // Parse the serial input to receive new values for the PID constants
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;

    while (Serial.available() > 0 && newData == false) {
      rc = Serial.read();

      if (recvInProgress == true) {
        if (rc != endMarker) {
          receivedChars[ndx] = rc;
          ndx++;
          if (ndx >= numChars) {
            ndx = numChars - 1;
          }
        } else {
          receivedChars[ndx] = '\0';
          recvInProgress = false;
          ndx = 0;
          newData = true;
        }
      } else if (rc == startMarker) {
        recvInProgress = true;
      }
    }

    // If new data is available, update the PID constants with the new values
    if (newData == true) {
      strcpy(tempChars, receivedChars);
      kp = atof(strtok(tempChars, ","));
      ki = atof(strtok(NULL, ","));
      kd = atof(strtok(NULL, ","));
      Serial.print("kp = ");
      Serial.println(kp);
      Serial.print("ki = ");
      Serial.println(ki);
      Serial.print("kd = ");
      Serial.println(kd);
      newData = false;
    }

    // Task timing for schedulability tests
    if (TIMING == 1) {
      unsigned long end_time = k_millis();
      unsigned long execution_time = end_time - start_time;
      Serial.print(F("Task3 Serial parser execution time: "));
      Serial.println(execution_time);
    }

    // Delay for 500 milliseconds
    k_sleep(500);
  }
}


void setup() {
  Serial.begin(115200);   // initialize the serial communication at 115200 bits per second
  delay(2000);            // wait for 2 seconds
  Serial.println("Init start");

  // initialize the custom kernel library with 3 tasks, 0 semaphores, and 3 message queues
  k_init(3, 0, 3);

  // create task1 with priority 10, stack size of STK_SIZE, and semaphore s1
  pTask1 = k_crt_task(task1, 10, s1, STK_SIZE);

  // create task2 with priority 12, stack size of STK_SIZE, and semaphore s2
  pTask2 = k_crt_task(task2, 12, s2, STK_SIZE);

  // create task3 with priority 11, stack size of STK_SIZE, and semaphore s3
  pTask3 = k_crt_task(task3, 11, s3, STK_SIZE);

  // configure pins and set up message queues
  pinMode(2, INPUT_PULLUP);
  pinMode(13, OUTPUT);
  pinMode(A0, INPUT);

  // create message queues with a buffer size of 20, one message, and the corresponding message array
  pMsg1 = k_crt_send_Q(20, 1, mar1);
  pMsg2 = k_crt_send_Q(20, 1, mar2);
  pMsg3 = k_crt_send_Q(20, 1, mar3);

  noInterrupts();
  // or from krnl  DI();
  EICRA |= (1 << ISC01);  // trigger INT0 on falling edge
  // https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7810-Automotive-Microcontrollers-ATmega328P_Datasheet.pdf  p 54
  EIMSK |= (1 << INT0);  // enable external intr
  interrupts();

  Serial.println("Init complete! Starting kernel");
  k_start(); // start the kernel timer with a speed of 1 millisecond

  Serial.println("If you see this then kernel didnt start :(");
}

void loop() {}
