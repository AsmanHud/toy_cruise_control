/* Cruise control skeleton for the IL 2206 embedded lab
 *
 * Maintainers:  Rodolfo Jordao (jordao@kth.se), George Ungereanu (ugeorge@kth.se)
 *
 * Description:
 *
 *   In this file you will find the "model" for the vehicle that is being simulated on top
 *   of the RTOS and also the stub for the control task that should ideally control its
 *   velocity whenever a cruise mode is activated.
 *
 *   The missing functions and implementations in this file are left as such for
 *   the students of the IL2206 course. The goal is that they get familiriazed with
 *   the real time concepts necessary for all implemented herein and also with Sw/Hw
 *   interactions that includes HAL calls and IO interactions.
 *
 *   If the prints prove themselves too heavy for the final code, they can
 *   be exchanged for alt_printf where hexadecimals are supported and also
 *   quite readable. This modification is easily motivated and accepted by the course
 *   staff.
 */

 /*
 * Mailbox system for inputs is deprecated
 * and the input signals are now declared as global variables
 * and are updated by the IO tasks
 *
 * Vehicle and Control tasks still communicate via Mboxes
 * (velocity - from vehicle to control, and throttle - from control to vehicle)
 *
 * Cruise control system (the control task) is implemented by:
 * - Asman Hudaykulyyev and Ramazan Yestekbayev
 */
#include <stdio.h>
#include "system.h"
#include "includes.h"
#include "altera_avalon_pio_regs.h"
#include "sys/alt_irq.h"
#include "sys/alt_alarm.h"

#define DEBUG 1

#define HW_TIMER_PERIOD 100 /* 100ms */

/* Button Patterns */

#define GAS_PEDAL_FLAG      0x08
#define BRAKE_PEDAL_FLAG    0x04
#define CRUISE_CONTROL_FLAG 0x02
/* Switch Patterns */

#define TOP_GEAR_FLAG       0x00000002
#define ENGINE_FLAG         0x00000001

/* LED Patterns */

#define LED_RED_0 0x00000001 // Engine
#define LED_RED_1 0x00000002 // Top Gear

#define LED_RED_4 0x00000010 // Extra 2% load
#define LED_RED_5 0x00000020 // Extra 2% load
#define LED_RED_6 0x00000040 // Extra 2% load
#define LED_RED_7 0x00000080 // Extra 2% load
#define LED_RED_8 0x00000100 // Extra 2% load
#define LED_RED_9 0x00000200 // Extra 2% load

#define LED_RED_12 0x00001000 // LEDR17: [0m, 400m)
#define LED_RED_13 0x00002000 // LEDR16: [400m, 800m)
#define LED_RED_14 0x00004000 // LEDR15: [800m, 1200m)
#define LED_RED_15 0x00008000 // LEDR14: [1200m, 1600m)
#define LED_RED_16 0x00010000 // LEDR13: [1600m, 2000m)
#define LED_RED_17 0x00020000 // LEDR12: [2000m, 2400m]


#define LED_GREEN_1 0x0002 // Cruise Control activated
#define LED_GREEN_2 0x0004 // Cruise Control Button
#define LED_GREEN_4 0x0010 // Brake Pedal
#define LED_GREEN_6 0x0040 // Gas Pedal

/*
 * Definition of Tasks
 */

#define TASK_STACKSIZE 2048

OS_STK StartTask_Stack[TASK_STACKSIZE]; 
OS_STK ControlTask_Stack[TASK_STACKSIZE]; 
OS_STK VehicleTask_Stack[TASK_STACKSIZE];
OS_STK ButtonIO_Stack[TASK_STACKSIZE];
OS_STK SwitchIO_Stack[TASK_STACKSIZE];

OS_STK WatchdogTask_Stack[TASK_STACKSIZE]; 
OS_STK OverloadDetectionTask_Stack[TASK_STACKSIZE]; 
OS_STK ExtraLoadTask_Stack[TASK_STACKSIZE]; 


// Task Priorities

#define OVERLOADDETECT_PRIO  2
#define EXTRA_LOAD_PRIO      4

#define STARTTASK_PRIO     5
#define VEHICLETASK_PRIO  10
#define CONTROLTASK_PRIO  12
#define BUTTONIOTASK_PRIO 14
#define SWITCHIOTASK_PRIO 15

#define WATCHDOG_PRIO     17



// Task Periods

#define CONTROL_PERIOD  300
#define VEHICLE_PERIOD  300
#define IO_PERIOD       100

#define WATCHDOG_INTERVAL       300 // hyperperiod of all tasks, in ms
#define OVERLOADDETECT_INTERVAL 300
#define EXTRA_LOAD_INTERVAL     300 // extra load task interval

/*
 * Definition of Kernel Objects 
 */

// Mailboxes
OS_EVENT *Mbox_Throttle;
OS_EVENT *Mbox_Velocity;

OS_EVENT *Mbox_OKSignal;

// Semaphores
OS_EVENT *Sem_VehicleTask;
OS_EVENT *Sem_ControlTask;
OS_EVENT *Sem_ButtonIO;
OS_EVENT *Sem_SwitchIO;

OS_EVENT *Sem_Watchdog;
OS_EVENT *Sem_OverloadDetection;
OS_EVENT *Sem_ExtraLoad;

// SW-Timer
OS_TMR *ControlTask_Timer;
OS_TMR *VehicleTask_Timer;
OS_TMR *ButtonIO_Timer;
OS_TMR *SwitchIO_Timer;

OS_TMR *Watchdog_Timer;
OS_TMR *OverloadDetection_Timer;
OS_TMR *ExtraLoad_Timer;

/*
 * Types
 */
enum active {on = 2, off = 1};


/*
 * Global variables
 */
int delay; // Delay of HW-timer 
INT16U led_green = 0; // Green LEDs
INT32U led_red = 0;   // Red LEDs
INT8U extra_load = 0; // Extra load for extra load task

enum active gas_pedal = off;
enum active brake_pedal = off;
enum active top_gear = off;
enum active engine = off;
enum active cruise_control = off;

/*
 * Helper functions
 */

int buttons_pressed(void)
{
  return ~IORD_ALTERA_AVALON_PIO_DATA(D2_PIO_KEYS4_BASE);    
}

int switches_pressed(void)
{
  return IORD_ALTERA_AVALON_PIO_DATA(DE2_PIO_TOGGLES18_BASE);    
}

void cruise_control_active() {
  led_green |= LED_GREEN_1;
  IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_GREENLED9_BASE, led_green);
}

void cruise_control_inactive() {
  led_green &= ~LED_GREEN_1;
  IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_GREENLED9_BASE, led_green);
}

// Callback function for the software timers

void soft_timer_callback(void *ptmr, void *callback_arg) {
  OSSemPost((OS_EVENT *) callback_arg);
}


/*
 * ISR for HW Timer
 */
alt_u32 alarm_handler(void* context)
{
  OSTmrSignal(); /* Signals a 'tick' to the SW timers */

  return delay;
}

static int b2sLUT[] = {0x40, //0
  0x79, //1
  0x24, //2
  0x30, //3
  0x19, //4
  0x12, //5
  0x02, //6
  0x78, //7
  0x00, //8
  0x18, //9
  0x3F, //-
};

/*
 * convert int to seven segment display format
 */
int int2seven(int inval){
  return b2sLUT[inval];
}

/*
 * output current velocity on the seven segement display
 */
void show_velocity_on_sevenseg(INT8S velocity){
  int tmp = velocity;
  int out;
  INT8U out_high = 0;
  INT8U out_low = 0;
  INT8U out_sign = 0;

  if(velocity < 0){
    out_sign = int2seven(10);
    tmp *= -1;
  }else{
    out_sign = int2seven(0);
  }

  out_high = int2seven(tmp / 10);
  out_low = int2seven(tmp - (tmp/10) * 10);

  out = int2seven(0) << 21 |
    out_sign << 14 |
    out_high << 7  |
    out_low;
  IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_HEX_LOW28_BASE,out);
}

/*
 * shows the target velocity on the seven segment display (HEX5, HEX4)
 * when the cruise control is activated (0 otherwise)
 */
void show_target_velocity(INT8U target_vel)
{
  int out;
  INT8U out_high;
  INT8U out_low;
  out_high = int2seven(target_vel / 10);
  out_low = int2seven(target_vel - (target_vel/10) * 10);
  out = out_high << 7 | out_low;
  IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_HEX_HIGH28_BASE,out);
}

/*
 * indicates the position of the vehicle on the track with the four leftmost red LEDs
 * LEDR17: [0m, 400m)
 * LEDR16: [400m, 800m)
 * LEDR15: [800m, 1200m)
 * LEDR14: [1200m, 1600m)
 * LEDR13: [1600m, 2000m)
 * LEDR12: [2000m, 2400m]
 */
void show_position(INT16U position)
{
  led_red &= ~(LED_RED_12 | LED_RED_13 | LED_RED_14 | LED_RED_15 | LED_RED_16 | LED_RED_17);
  if (position < 400) {
        led_red |= LED_RED_17;
    } else if (position < 800) {
        led_red |= LED_RED_16;
    } else if (position < 1200) {
        led_red |= LED_RED_15;
    } else if (position < 1600) {
        led_red |= LED_RED_14;
    } else if (position < 2000) {
        led_red |= LED_RED_13;
    } else if (position < 2400) {
        led_red |= LED_RED_12;
    }
  IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_REDLED18_BASE, led_red);
}

/*
 * The tasks ButtonIO and SwitchIO periodically check if
 * buttons or switches, respectively, are pressed
 * They communicate through mailboxes
 */

void ButtonIOTask(void* pdata) {
  int buttons;
  INT8U err;
  while (1) {
    OSSemPend(Sem_ButtonIO, 0, &err);
    buttons = buttons_pressed();
    if (buttons & GAS_PEDAL_FLAG) {
      gas_pedal = on;
      led_green |= LED_GREEN_6;
    } else {
      gas_pedal = off;
      led_green &= ~LED_GREEN_6;
    }
    if (buttons & BRAKE_PEDAL_FLAG) {
      brake_pedal = on;
      led_green |= LED_GREEN_4;
    } else {
      brake_pedal = off;
      led_green &= ~LED_GREEN_4;
    }
    if (buttons & CRUISE_CONTROL_FLAG) {
      cruise_control = on;
      led_green |= LED_GREEN_2;
    } else {
      cruise_control = off;
      led_green &= ~LED_GREEN_2;
    }
    IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_GREENLED9_BASE, led_green);
  }
}

void SwitchIOTask(void* pdata) {
  int switches;
  INT8U err;
  while (1) {
    OSSemPend(Sem_SwitchIO, 0, &err);
    switches = switches_pressed();
    if (switches & TOP_GEAR_FLAG) {
      top_gear = on;
      led_red |= LED_RED_1;
    } else {
      top_gear = off;
      led_red &= ~LED_RED_1;
    }
    if (switches & ENGINE_FLAG) {
      engine = on;
      led_red |= LED_RED_0;
    } else {
      engine = off;
      led_red &= ~LED_RED_0;
    }

    // Extra load calculated + turn on/off LEDs
    int extractedPattern = (switches >> 4) & 0x3F; // Extract bit pattern from switch 4 to switch 9
    extra_load = extractedPattern * 2; // 2% steps
    if (extra_load > 100) extra_load = 100;

    int led_red_arr[6] = {LED_RED_4, LED_RED_5, LED_RED_6, LED_RED_7, LED_RED_8, LED_RED_9};
    int i;
    for (i = 0; i < 6; ++i) {
        if (extractedPattern & (1 << i)) {
            led_red |= led_red_arr[i];
        } else {
            led_red &= ~led_red_arr[i];
        }
    }

    IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_REDLED18_BASE, led_red);
  }
}

/*
 * The task 'VehicleTask' is the model of the vehicle being simulated. It updates variables like
 * acceleration and velocity based on the input given to the model.
 * 
 * The car model is equivalent to moving mass with linear resistances acting upon it.
 * Therefore, if left one, it will stably stop as the velocity converges to zero on a flat surface.
 * You can prove that easily via basic LTI systems methods.
 */

void VehicleTask(void* pdata)
{ 
  // constants that should not be modified
  const unsigned int wind_factor = 1;
  const unsigned int brake_factor = 4;
  const unsigned int gravity_factor = 2;
  // variables relevant to the model and its simulation on top of the RTOS
  INT8U err;  
  void* msg;
  INT8U* throttle; 
  INT16S acceleration;  
  INT16U position = 0; 
  INT16S velocity = 0;

  printf("Vehicle task created!\n");

  while(1)
  {
    err = OSMboxPost(Mbox_Velocity, (void *) &velocity);
    
    // Wait for the semaphore
    OSSemPend(Sem_VehicleTask, 0, &err);

    /* Non-blocking read of mailbox: 
       - message in mailbox: update throttle
       - no message:         use old throttle
       */
    msg = OSMboxPend(Mbox_Throttle, 1, &err); 
    if (err == OS_NO_ERR) 
      throttle = (INT8U*) msg;

    // vehichle cannot effort more than 80 units of throttle
    if (*throttle > 80) *throttle = 80;

    // brakes + wind
    if (brake_pedal == off)
    {
      // wind resistance
      acceleration = - wind_factor*velocity;
      // actuate with engines
      if (engine == on)
        acceleration += (*throttle);

      // gravity effects
      if (400 <= position && position < 800)
        acceleration -= gravity_factor; // traveling uphill
      else if (800 <= position && position < 1200)
        acceleration -= 2*gravity_factor; // traveling steep uphill
      else if (1600 <= position && position < 2000)
        acceleration += 2*gravity_factor; //traveling downhill
      else if (2000 <= position)
        acceleration += gravity_factor; // traveling steep downhill
    }
    // if the engine and the brakes are activated at the same time,
    // we assume that the brake dynamics dominates, so both cases fall
    // here.
    else 
      acceleration = - brake_factor*velocity;

    printf("Position: %d m\n", position);
    printf("Velocity: %d m/s\n", velocity);
    printf("Accell: %d m/s2\n", acceleration);
    printf("Throttle: %d V\n", *throttle);

    position = position + velocity * VEHICLE_PERIOD / 1000;
    velocity = velocity  + acceleration * VEHICLE_PERIOD / 1000.0;
    // reset the position to the beginning of the track
    if(position > 2400)
      position = 0;

    show_velocity_on_sevenseg((INT8S) velocity);
    show_position(position);
  }
} 

/*
 * The task 'ControlTask' is the main task of the application. It reacts
 * on sensors and generates responses.
 */

void ControlTask(void* pdata)
{
  INT8U err;
  INT8U throttle = 0;
  void* msg;
  INT16S* current_velocity;

  INT16S error;
  INT16S accumulated_error = 0;

  const INT16S target_velocity = 25; // 25 m/s
  const INT8U topgear_throttle = 40;
  const INT8U lowgear_throttle = 60;
  const unsigned int Kp = 2;
  const unsigned int Ki = 1;

  printf("Control Task created!\n");

  while(1)
  {
    // Wait for the soft timer
    OSSemPend(Sem_ControlTask, 0, &err);
    
    msg = OSMboxPend(Mbox_Velocity, 0, &err);
    current_velocity = (INT16S*) msg;
    show_target_velocity(0);
    cruise_control_inactive();
    if (brake_pedal == on) {
      throttle = 0;
    } else if (gas_pedal == on) {
      if (top_gear == on) {
        throttle = topgear_throttle;
      } else if (top_gear == off) {
        throttle = lowgear_throttle;
      } else {
        throttle = 0;
      }
    } else if (top_gear == on && cruise_control == on && *current_velocity >= 20) {
      cruise_control_active();
      show_target_velocity(target_velocity);
      // PI controller for cruise control, Kp and Ki were acquired experimentally
      error = target_velocity - *current_velocity;
      accumulated_error += error;
      throttle = Kp * error + Ki * accumulated_error;
    } else {
      throttle = 0;
    }

    // Sending the throttle to vehicle task
    err = OSMboxPost(Mbox_Throttle, (void *) &throttle);

  }
}

void WatchdogTask(void* pdata) {
  INT8U err;
  INT8U watchdog_err;
  while (1) {
    OSSemPend(Sem_Watchdog, 0, &err);

    OSMboxPend(Mbox_OKSignal, WATCHDOG_INTERVAL/100, &watchdog_err);
    if (watchdog_err == OS_TIMEOUT) {
      printf("Watchdog timeout! By WatchdogTask\n");
    }
  }
}

void OverloadDetectionTask(void* pdata) {
  INT8U err;
  INT8U overload_err;
  while (1) {
    OSSemPend(Sem_OverloadDetection, 0, &err);

    overload_err = OSMboxPost(Mbox_OKSignal, (void *) 1);
  }
}

void ExtraLoadTask(void* pdata) {
  INT8U err;
  INT8U x;
  unsigned int time_delay;
  while (1) {
    OSSemPend(Sem_ExtraLoad, 0, &err);
    x = 0;
    printf("Size of unsigned int: %zu bytes\n", sizeof(unsigned int));
    // finish the code
  }
}

/* 
 * The task 'StartTask' creates all other tasks kernel objects and
 * deletes itself afterwards.
 */ 

void StartTask(void* pdata)
{
  INT8U err;
  void* context;

  static alt_alarm alarm;     /* Is needed for timer ISR function */

  /* Base resolution for SW timer : HW_TIMER_PERIOD ms */
  delay = alt_ticks_per_second() * HW_TIMER_PERIOD / 1000; 
  printf("delay in ticks %d\n", delay);

  /* 
   * Create Hardware Timer with a period of 'delay' 
   */
  if (alt_alarm_start (&alarm,
        delay,
        alarm_handler,
        context) < 0)
  {
    printf("No system clock available!\n");
  }

  /*
   * Creation of Kernel Objects
   */

  // Mailboxes
  Mbox_Throttle = OSMboxCreate((void*) 0); /* Empty Mailbox - Throttle */
  Mbox_Velocity = OSMboxCreate((void*) 0); /* Empty Mailbox - Velocity */
  Mbox_OKSignal = OSMboxCreate((void*) 0); /* Empty Mailbox - OK Signal */

  // Semaphores
  Sem_VehicleTask = OSSemCreate(0); /* Binary Semaphore */
  Sem_ControlTask = OSSemCreate(1); /* Binary Semaphore */
  Sem_ButtonIO = OSSemCreate(0); /* Binary Semaphore */
  Sem_SwitchIO = OSSemCreate(0); /* Binary Semaphore */
  
  Sem_Watchdog = OSSemCreate(0); /* Binary Semaphore */
  Sem_OverloadDetection = OSSemCreate(0); /* Binary Semaphore */
  Sem_ExtraLoad = OSSemCreate(0); /* Binary Semaphore */

  /* 
   * Create and start Software Timer 
   */
  ControlTask_Timer = OSTmrCreate(0,
      CONTROL_PERIOD/100,
      OS_TMR_OPT_PERIODIC,
      soft_timer_callback,
      Sem_ControlTask,
      "Control Timer",
      &err);
  OSTmrStart(ControlTask_Timer, &err);
  VehicleTask_Timer = OSTmrCreate(0,
      VEHICLE_PERIOD/100,
      OS_TMR_OPT_PERIODIC,
      soft_timer_callback,
      Sem_VehicleTask,
      "Vehicle Timer",
      &err);
  OSTmrStart(VehicleTask_Timer, &err);
  ButtonIO_Timer = OSTmrCreate(0,
      IO_PERIOD/100,
      OS_TMR_OPT_PERIODIC,
      soft_timer_callback,
      Sem_ButtonIO,
      "ButtonIO Timer",
      &err);
  OSTmrStart(ButtonIO_Timer, &err);
  SwitchIO_Timer = OSTmrCreate(0,
      IO_PERIOD/100,
      OS_TMR_OPT_PERIODIC,
      soft_timer_callback,
      Sem_SwitchIO,
      "SwitchIO Timer",
      &err);
  OSTmrStart(SwitchIO_Timer, &err);

  Watchdog_Timer = OSTmrCreate(0,
      WATCHDOG_INTERVAL/100,
      OS_TMR_OPT_PERIODIC,
      soft_timer_callback,
      Sem_Watchdog,
      "Watchdog Timer",
      &err);
  OSTmrStart(Watchdog_Timer, &err);
  OverloadDetection_Timer = OSTmrCreate(0,
      OVERLOADDETECT_INTERVAL/100,
      OS_TMR_OPT_PERIODIC,
      soft_timer_callback,
      Sem_OverloadDetection,
      "Overload Detection Timer",
      &err);
  OSTmrStart(OverloadDetection_Timer, &err);
  ExtraLoad_Timer = OSTmrCreate(0,
      EXTRA_LOAD_INTERVAL/100,
      OS_TMR_OPT_PERIODIC,
      soft_timer_callback,
      Sem_ExtraLoad,
      "Extra Load Timer",
      &err);
  OSTmrStart(ExtraLoad_Timer, &err);

  /*
   * Create statistics task
   */

  OSStatInit();

  /* 
   * Creating Tasks in the system 
   */


  err = OSTaskCreateExt(
      ControlTask, // Pointer to task code
      NULL,        // Pointer to argument that is
      // passed to task
      &ControlTask_Stack[TASK_STACKSIZE-1], // Pointer to top
      // of task stack
      CONTROLTASK_PRIO,
      CONTROLTASK_PRIO,
      (void *)&ControlTask_Stack[0],
      TASK_STACKSIZE,
      (void *) 0,
      OS_TASK_OPT_STK_CHK);

  err = OSTaskCreateExt(
      VehicleTask, // Pointer to task code
      NULL,        // Pointer to argument that is
      // passed to task
      &VehicleTask_Stack[TASK_STACKSIZE-1], // Pointer to top
      // of task stack
      VEHICLETASK_PRIO,
      VEHICLETASK_PRIO,
      (void *)&VehicleTask_Stack[0],
      TASK_STACKSIZE,
      (void *) 0,
      OS_TASK_OPT_STK_CHK);
  
  err = OSTaskCreateExt(
      ButtonIOTask, // Pointer to task code
      NULL,        // Pointer to argument that is
      // passed to task
      &ButtonIO_Stack[TASK_STACKSIZE-1], // Pointer to top
      // of task stack
      BUTTONIOTASK_PRIO,
      BUTTONIOTASK_PRIO,
      (void *)&ButtonIO_Stack[0],
      TASK_STACKSIZE,
      (void *) 0,
      OS_TASK_OPT_STK_CHK);
  
  err = OSTaskCreateExt(
      SwitchIOTask, // Pointer to task code
      NULL,        // Pointer to argument that is
      // passed to task
      &SwitchIO_Stack[TASK_STACKSIZE-1], // Pointer to top
      // of task stack
      SWITCHIOTASK_PRIO,
      SWITCHIOTASK_PRIO,
      (void *)&SwitchIO_Stack[0],
      TASK_STACKSIZE,
      (void *) 0,
      OS_TASK_OPT_STK_CHK);
    
  err = OSTaskCreateExt(
    WatchdogTask,
    NULL,
    &WatchdogTask_Stack[TASK_STACKSIZE-1],
    WATCHDOG_PRIO,
    WATCHDOG_PRIO,
    (void *)&WatchdogTask_Stack[0],
    TASK_STACKSIZE,
    (void *) 0,
    OS_TASK_OPT_STK_CHK);
  
  err = OSTaskCreateExt(
    OverloadDetectionTask,
    NULL,
    &OverloadDetectionTask_Stack[TASK_STACKSIZE-1],
    OVERLOADDETECT_PRIO,
    OVERLOADDETECT_PRIO,
    (void *)&OverloadDetectionTask_Stack[0],
    TASK_STACKSIZE,
    (void *) 0,
    OS_TASK_OPT_STK_CHK);

err = OSTaskCreateExt(
    ExtraLoadTask,
    NULL,
    &ExtraLoadTask_Stack[TASK_STACKSIZE-1],
    EXTRA_LOAD_PRIO,
    EXTRA_LOAD_PRIO,
    (void *)&ExtraLoadTask_Stack[0],
    TASK_STACKSIZE,
    (void *) 0,
    OS_TASK_OPT_STK_CHK);


  printf("All Tasks and Kernel Objects generated!\n");

  /* Task deletes itself */

  OSTaskDel(OS_PRIO_SELF);
}

/*
 *
 * The function 'main' creates only a single task 'StartTask' and starts
 * the OS. All other tasks are started from the task 'StartTask'.
 *
 */

int main(void) {

  printf("Lab: Cruise Control\n");

  OSTaskCreateExt(
      StartTask, // Pointer to task code
      NULL,      // Pointer to argument that is
      // passed to task
      (void *)&StartTask_Stack[TASK_STACKSIZE-1], // Pointer to top
      // of task stack 
      STARTTASK_PRIO,
      STARTTASK_PRIO,
      (void *)&StartTask_Stack[0],
      TASK_STACKSIZE,
      (void *) 0,  
      OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR);

  OSStart();

  return 0;
}
