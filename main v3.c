#include "cybsp.h"
#include "cyhal.h"
#include "cy_retarget_io.h"
#include "cy_pdl.h"
/* TCP client task header file. */
// #include "tcp_client.h"

/*******************************************************************************
* Macros
*******************************************************************************/
/* PWM Frequency = 2Hz */
#define PWM_FREQUENCY (2u)
/* PWM Duty-cycle = 50% */
#define PWM_DUTY_CYCLE (50.0f)
#define DATA_BITS_8     8
#define STOP_BITS_1     1
#define BAUD_RATE       115200
#define UART_DELAY      10u
#define RX_BUF_SIZE     20
#define TX_BUF_SIZE     6
/* RTOS related macros. */
#define TCP_CLIENT_TASK_STACK_SIZE        (5 * 1024)
#define TCP_CLIENT_TASK_PRIORITY          (1)


/*******************************************************************************
* Global Variables
*******************************************************************************/
/* Variable Declarations */
cy_rslt_t rslt;
cyhal_timer_t timer_obj_2;
cyhal_uart_t uart_obj;
cy_stc_scb_uart_context_t uartContext;
uint32_t actualbaud;
uint8_t tx_buf[6] = {0xae, 0xc1, 0x20, 0x02, 0x01, 0x01};

size_t tx_length = TX_BUF_SIZE;
size_t rx_length = RX_BUF_SIZE;


#define Calibrate 0 //Change to 1 see above for details.
#define calDistance 12 //in inches 24inches or 2 foot
#define LCD 0 //Display Average Distance to Parallax 1602 serial LCD otherwise Zero will display much more information to serial console and/or LCD if connected

int calWidth = 56; //Calibrated width reading
int calHeight = 53; //Calibrated height reading
int pixelsWidth;   //read by the camera
int pixelsHeight; //read by the camera
float distanceWidth;   //calculated distance based on the width of the object
float distanceHeight;  //calculated distance based on the height of the object
float widthOfObject = 3; //inches (3.75 inches) real size of your object
float heightOfObject = 3; //inches (2.5 inches) real size of your object
int focalLengthWidth;  //calculated focal length for width
int focalLengthHeight; //calculated focal length for height
float avg;
int feet;
int inches;
int pixy_x;

double output = 0;

//Constants used in some of the functions below
#define AUTOMATIC	1
#define MANUAL	0
#define DIRECT  0
#define REVERSE  1
#define P_ON_M 0
#define P_ON_E 1

struct PID
{

	double kp;                  // * (P)roportional Tuning Parameter
    double ki;                  // * (I)ntegral Tuning Parameter
    double kd;                  // * (D)erivative Tuning Parameter

	int controllerDirection;
	int pOn;

    double *myInput;              // * Pointers to the Input, Output, and Setpoint variables
    double *myOutput;             //   This creates a hard link between the variables and the
    double *mySetpoint;           //   PID, freeing the user from having to constantly tell us
                                  //   what these values are.  with pointers we'll just know.

	unsigned long lastTime;
	double outputSum, lastInput;

	unsigned long SampleTime;
	double outMin, outMax;
	bool inAuto, pOnE;
} myPID;


/* Compute() **********************************************************************
 *     This, as they say, is where the magic happens.  this function should be called
 *   every time "void loop()" executes.  the function will decide for itself whether a new
 *   pid Output needs to be computed.  returns true when the output is computed,
 *   false when nothing has been done.
 **********************************************************************************/
bool Compute()
{
//   if(!myPID.inAuto)
//	   printf("Not AUTO!");
//	   return false;
   unsigned long now = cyhal_timer_read(&timer_obj_2)/10;
   printf("now: %lu \r\n", now);
   printf("myPID.lastTime: %lu \r\n", myPID.lastTime);
   unsigned long timeChange = (now - myPID.lastTime);
   printf("timeChange is: %lu \r\n", timeChange);
   printf("my PID sample time is: %lu \r\n", myPID.SampleTime);
   if(timeChange>=myPID.SampleTime)
   {
      /*Compute all the working error variables*/
      double input = *myPID.myInput;
      double error = *myPID.mySetpoint - input;
      if (error < 0) {
    	  error = -error;
      }

      double dInput = input - myPID.lastInput;

      double result = (myPID.ki) * error;
      // printf("result: %f \r\n", result);
      myPID.outputSum += result;

      /*Add Proportional on Measurement, if P_ON_M is specified*/
      if(!myPID.pOnE) myPID.outputSum -= myPID.kp * dInput;

      if(myPID.outputSum > myPID.outMax) myPID.outputSum= myPID.outMax;
      else if(myPID.outputSum < myPID.outMin) myPID.outputSum  = myPID.outMin;

      /*Add Proportional on Error, if P_ON_E is specified*/

      if(myPID.pOnE) output = myPID.kp * error;
      else output = 0;
	  // printf("output now is: %f \r\n", output);

      /*Compute Rest of PID Output*/
      output += myPID.outputSum - myPID.kd * dInput;
      // printf("output now is: %f \r\n", output);

	  if(output > myPID.outMax) output = myPID.outMax;
	  else if(output < myPID.outMin) output = myPID.outMin;

	  printf("output now is: %f \r\n", output);

	  myPID.myOutput = &output;
	  printf("myPID output now is: %f \r\n", *myPID.myOutput);

      /*Remember some variables for next time*/
	  myPID.lastInput = input;
	  myPID.lastTime = now;
	  return true;
   }
   else return false;
}

/* SetTunings(...)*************************************************************
 * This function allows the controller's dynamic performance to be adjusted.
 * it's called automatically from the constructor, but tunings can also
 * be adjusted on the fly during normal operation
 ******************************************************************************/
void SetTunings(double Kp, double Ki, double Kd, int POn)
{
   if (Kp<0 || Ki<0 || Kd<0) return;

   myPID.pOn = POn;
   myPID.pOnE = POn == P_ON_E;

   double SampleTimeInSec = ((double)myPID.SampleTime)/1000;
   myPID.kp = Kp;
   myPID.ki = Ki * SampleTimeInSec;
   myPID.kd = Kd / SampleTimeInSec;

  if(myPID.controllerDirection ==REVERSE)
   {
	  myPID.kp = (0 - myPID.kp);
	  myPID.ki = (0 - myPID.ki);
	  myPID.kd = (0 - myPID.kd);
   }
}

/* SetTunings(...)*************************************************************
 * Set Tunings using the last-rembered POn setting
 ******************************************************************************/
void SetTunings1(double Kp, double Ki, double Kd){
    SetTunings(Kp, Ki, Kd, myPID.pOn);
}

///* SetSampleTime(...) *********************************************************
// * sets the period, in Milliseconds, at which the calculation is performed
// ******************************************************************************/
//void SetSampleTime(int NewSampleTime)
//{
//   if (NewSampleTime > 0)
//   {
//      double ratio  = (double)NewSampleTime
//                      / (double)myPID.SampleTime;
//      myPID.ki *= ratio;
//      myPID.kd /= ratio;
//      myPID.SampleTime = (unsigned long)NewSampleTime;
//   }
//}

/* SetOutputLimits(...)****************************************************
 *     This function will be used far more often than SetInputLimits.  while
 *  the input to the controller will generally be in the 0-1023 range (which is
 *  the default already,)  the output will be a little different.  maybe they'll
 *  be doing a time window and will need 0-8000 or something.  or maybe they'll
 *  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
 *  here.
 **************************************************************************/
void SetOutputLimits(double Min, double Max)
{
   if(Min >= Max) return;
   myPID.outMin = Min;
   myPID.outMax = Max;

   if(myPID.inAuto)
   {
	   printf("Set output limits");
	   if(*myPID.myOutput > myPID.outMax) *myPID.myOutput = myPID.outMax;
	   else if(*myPID.myOutput < myPID.outMin) *myPID.myOutput = myPID.outMin;

	   if(myPID.outputSum > myPID.outMax) myPID.outputSum= myPID.outMax;
	   else if(myPID.outputSum < myPID.outMin) myPID.outputSum= myPID.outMin;
   }
}

/* Initialize()****************************************************************
 *	does all the things that need to happen to ensure a bumpless transfer
 *  from manual to automatic mode.
 ******************************************************************************/
void Initialize()
{
	myPID.outputSum = *myPID.myOutput;
	myPID.lastInput = *myPID.myInput;
    if(myPID.outputSum > myPID.outMax) myPID.outputSum = myPID.outMax;
    else if(myPID.outputSum < myPID.outMin) myPID.outputSum = myPID.outMin;
}

/* SetMode(...)****************************************************************
 * Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
 * when the transition from manual to auto occurs, the controller is
 * automatically initialized
 ******************************************************************************/
void SetMode(int Mode)
{
    bool newAuto = (Mode == AUTOMATIC);
    if (myPID.inAuto) {
		printf("newAuto yes \r\n");
	} else {
		printf("newAuto no \r\n");
	}
    if(newAuto && !myPID.inAuto)
    {  /*we just went from manual to auto*/
        Initialize();
    }
    myPID.inAuto = newAuto;
    if (myPID.inAuto) {
		printf("newAuto yes \r\n");
	} else {
		printf("newAuto no \r\n");
	}
}

void SetControllerDirection(int Direction)
{
   if(myPID.inAuto && Direction != myPID.controllerDirection)
   {
	   myPID.kp = (0 - myPID.kp);
	   myPID.ki = (0 - myPID.ki);
	   myPID.kd = (0 - myPID.kd);
   }
   myPID.controllerDirection = Direction;
}

void PID1(double* Input, double* Output, double* Setpoint,
        double Kp, double Ki, double Kd, int POn, int ControllerDirection)
{
    myPID.myOutput = Output;
    myPID.myInput = Input;
    myPID.mySetpoint = Setpoint;
    myPID.inAuto = false;

    double low = 0;
    double high = 255;
    SetOutputLimits(low, high);				//default output limit corresponds to the arduino pwm limits

    myPID.SampleTime = 100;							//default Controller Sample Time is 0.1 seconds

    SetControllerDirection(ControllerDirection);
    SetTunings(Kp, Ki, Kd, POn);

    myPID.lastTime = cyhal_timer_read(&timer_obj_2)/10 -  myPID.SampleTime;
}

/*Constructor (...)*********************************************************
 *    To allow backwards compatability for v1.1, or for people that just want
 *    to use Proportional on Error without explicitly saying so
 ***************************************************************************/

void PID(double* Input, double* Output, double* Setpoint,
        double Kp, double Ki, double Kd, int ControllerDirection)

{
	PID1(Input, Output, Setpoint, Kp, Ki, Kd, P_ON_E, ControllerDirection);
}


void UART_Isr(void)
{
	Cy_SCB_UART_Interrupt(scb_10_HW, &uartContext);
}


// PIXY TIMER ******************************************************************************

bool timer_interrupt_flag = false;
/* Timer object used */
cyhal_timer_t timer_obj_1;
static void isr_timer(void *callback_arg, cyhal_timer_event_t event)
{
    (void) callback_arg;
    (void) event;
    /* Set the interrupt flag and process it from the application */
    timer_interrupt_flag = true;
}

// UART Interrupt **************************************************************************

/* Event handler callback function */
bool user_interrupt_flag = false;
void uart_event_handler(void *handler_arg, cyhal_uart_event_t event)
{
    (void) handler_arg;
    user_interrupt_flag = true;
}

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function for the CPU. It configures the PWM and puts the CPU
* in Sleep mode to save power.
*
* Parameters:
*  void
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    /* PWM object */
    cyhal_pwm_t pwm_obj_r;
    cyhal_pwm_t pwm_obj_l;
    //cyhal_pwm_t pwm_obj_s;

    /* API return code */
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init();
    if (CY_RSLT_SUCCESS != result)
    {
        /* Halt the CPU while debugging */
        CY_ASSERT(false);
    }

    /* Enable global interrupts */
    __enable_irq();

    /* Initialize the retarget-io to use the debug UART port */
    result = cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE);
    if (CY_RSLT_SUCCESS != result)
    {
        /* Halt the CPU while debugging */
        CY_ASSERT(false);
    }
    /* The UART callback handler registration */
    cyhal_uart_register_callback(&cy_retarget_io_uart_obj, uart_event_handler, NULL);
    /* Enable required UART events */
    cyhal_uart_enable_event(&cy_retarget_io_uart_obj, (cyhal_uart_event_t)(CYHAL_UART_IRQ_RX_NOT_EMPTY), 0, true);


    /* ---------------------------------------------------------TIMER_1 PIXY INIT---------------------------------------------------------- */

	const cyhal_timer_cfg_t timer_cfg_1 =
	{
		.compare_value = 0,                 /* Timer compare value, not used */
		.period = 199,                      /* Defines the timer period */
		.direction = CYHAL_TIMER_DIR_UP,    /* Timer counts up */
		.is_compare = false,                /* Don't use compare mode */
		.is_continuous = true,              /* Run the timer indefinitely */
		.value = 0                          /* Initial value of counter */
	};
	/* Initialize the timer object. Does not use pin output ('pin' is NC) and
	 * does not use a pre-configured clock source ('clk' is NULL). */
	rslt = cyhal_timer_init(&timer_obj_1, NC, NULL);
	CY_ASSERT(CY_RSLT_SUCCESS == rslt);
	/* Apply timer configuration such as period, count direction, run mode, etc. */
	rslt = cyhal_timer_configure(&timer_obj_1, &timer_cfg_1);
	/* Set the frequency of timer to 10000 Hz */
	rslt = cyhal_timer_set_frequency(&timer_obj_1, 10000);
	/* Assign the ISR to execute on timer interrupt */
	cyhal_timer_register_callback(&timer_obj_1, isr_timer, NULL);
	/* Set the event on which timer interrupt occurs and enable it */
	cyhal_timer_enable_event(&timer_obj_1, CYHAL_TIMER_IRQ_TERMINAL_COUNT, 0, true);
	/* Start the timer with the configured settings */
	rslt = cyhal_timer_start(&timer_obj_1);



    /* ---------------------------------------------------------TIMER_2 PWM INIT---------------------------------------------------------- */
    /* Timer object used */

    const cyhal_timer_cfg_t timer_cfg =
    {
        .compare_value = 0,                 /* Timer compare value, not used */
        .period = -1,                    /* Timer period set to a large enough value
                                             * compared to event being measured */
        .direction = CYHAL_TIMER_DIR_UP,    /* Timer counts up */
        .is_compare = false,                /* Don't use compare mode */
        .is_continuous = false,             /* Do not run timer indefinitely */
        .value = 0                          /* Initial value of counter */
    };
    /* Initialize the timer object. Does not use pin output ('pin' is NC) and
     * does not use a pre-configured clock source ('clk' is NULL). */
    rslt = cyhal_timer_init(&timer_obj_2, NC, NULL);
    /* Apply timer configuration such as period, count direction, run mode, etc. */
    cyhal_timer_configure(&timer_obj_2, &timer_cfg);
    /* Set the frequency of timer to 10000 counts in a second or 10000 Hz */
    cyhal_timer_set_frequency(&timer_obj_2, 10000);
    /* Start the timer with the configured settings */
    cyhal_timer_start(&timer_obj_2);


//    for (;;) {
//    	/* Read the current timer value, which should be close to the amount of delay in ms * 10 (5000) */
//        /* Delay Function simulates the time between two events */
//        //cyhal_system_delay_ms(500);
//        uint32_t read_val = cyhal_timer_read(&timer_obj);
//    	printf("time: %u \r\n", read_val);
//    }


    /* -------------------------------------------------------PIXY UART INIT--------------------------------------------------------------- */
    printf("\x1b[2J\x1b[;H");
    printf("****************** PSoC 6 MCU: UART ******************\r\n\n");

    /* Assign UART interrupt number and priority */
    #define UART_INTR_NUM        ((IRQn_Type) scb_10_interrupt_IRQn)
    #define UART_INTR_PRIORITY   (7U)
    /* Populate configuration structure (code specific for CM4) */
    cy_stc_sysint_t uartIntrConfig =
    {
        .intrSrc      = UART_INTR_NUM,
        .intrPriority = UART_INTR_PRIORITY,
    };
    /* Hook interrupt service routine and enable interrupt */
    (void) Cy_SysInt_Init(&uartIntrConfig, &UART_Isr);
    NVIC_EnableIRQ(UART_INTR_NUM);

   /* Configure UART to operate */
   /* Allocate context for UART operation */
   cy_en_scb_uart_status_t status;

   /* Configure UART to operate */
   status = Cy_SCB_UART_Init(scb_10_HW, &scb_10_config, &uartContext);

   if (status == CY_SCB_UART_SUCCESS ) {
	   printf("init succeeds \r\n");
   }

   /* Enable UART to operate */
   Cy_SCB_UART_Enable(scb_10_HW);
   printf("tried enable \r\n");
   int counter = 0;


   /* ---------------------------------------------------------PWM INIT-------------------------------------------------------------- */

   /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
   printf("\x1b[2J\x1b[;H");
   printf("****************** PSoC 6 MCU: PWM Square Wave ******************\r\n\n");

   /* In this example, PWM output is routed to the user LED on the kit.
      See HAL API Reference document for API details. */

   /* Initialize the PWM */
   cyhal_clock_t clock_pwm;
   /* Initialize clock divider for PWMs */
   cyhal_clock_allocate(&clock_pwm, CYHAL_CLOCK_BLOCK_PERIPHERAL_16BIT);
   cyhal_clock_set_frequency(&clock_pwm, 1000000, NULL);
   if (!cyhal_clock_is_enabled(&clock_pwm))
   {
	   cyhal_clock_set_enabled(&clock_pwm, true, true);
   }


   result = cyhal_pwm_init(&pwm_obj_l, P6_5, &clock_pwm);
   result = cyhal_pwm_init(&pwm_obj_r, P6_0, &clock_pwm);
//   result = cyhal_pwm_init(&pwm_obj_s, P8_5, NULL);

//   cyhal_pwm_set_period(&pwm_obj_l, 100, 50);
//   cyhal_pwm_set_period(&pwm_obj_r, 100, 50);
   cyhal_pwm_set_duty_cycle(&pwm_obj_l, 0, 490);
   cyhal_pwm_set_duty_cycle(&pwm_obj_r, 0, 490);
//   cyhal_pwm_set_duty_cycle(&pwm_obj_s, 0, 490);

   result = cyhal_pwm_start(&pwm_obj_l);
   result = cyhal_pwm_start(&pwm_obj_r);
//   result = cyhal_pwm_start(&pwm_obj_s);

   // if(CY_RSLT_SUCCESS != result){printf("API cyhal_pwm_init failed with error code: %lu\r\n", (unsigned long) result);CY_ASSERT(false);}
   /* Set the PWM output frequency and duty cycle */

   printf("PWM started successfully. Continue...\r\n");
   result = cyhal_gpio_init(P6_3, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, true);
   result = cyhal_gpio_init(P6_4, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, false);
   result = cyhal_gpio_init(P8_2, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, true);
   result = cyhal_gpio_init(P1_0, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, false);
   result = cyhal_gpio_init(CYBSP_USER_LED, CYHAL_GPIO_DIR_OUTPUT,
                                CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);


   /* ---------------------------------------------------------PID INIT-------------------------------------------------------------- */

   //Define Variables we'll be connecting to
   double Setpoint = 18;
   double Input = 0.0;

   //Specify the links and initial tuning parameters
   double Kp=8, Ki=0.05, Kd=0;
   //calculate focal length
   double focalLengthWidth = (calWidth * calDistance) / widthOfObject;
   double focalLengthHeight = (calHeight * calDistance) / heightOfObject;
   SetMode(AUTOMATIC);

   PID(&Input, &output, &Setpoint, Kp, Ki, Kd, DIRECT);

   /* ---------------------------------------------------------WIFI INIT-------------------------------------------------------------- */
   /* Create the tasks. */
   // xTaskCreate(tcp_client_task, "Network task", TCP_CLIENT_TASK_STACK_SIZE, NULL, TCP_CLIENT_TASK_PRIORITY, NULL);


   uint8_t uart_read_value;
   int error_flag = 0;
   uint8_t rx_buf[RX_BUF_SIZE];
   /* -------------------------------------------------------BIG FOR LOOP------------------------------------------------------------- */

   for (;;) {
        /* -----------------------------------------------RECEIVE DATA FROM PIXY---------------------------------------------------------- */
        // send request bytes to Pixy camera
        /* Master: start a transfer. Slave: prepare for a transfer. */
	   	cyhal_uart_getc(&cy_retarget_io_uart_obj, &uart_read_value, 1);
	   	if (user_interrupt_flag) {
	   		user_interrupt_flag = false;
	   		if (uart_read_value == 'a') {  // if read a character 'a', then the whole program will terminates and the car will stop
	   			cyhal_gpio_toggle(CYBSP_USER_LED);
	   			break;
	   		} else if (uart_read_value >= '0' && uart_read_value <= '9') {   // the user input is in feet
	   			Setpoint = uart_read_value * 12;  // convert feet to inches
	   		}
	   	}

	   	if (timer_interrupt_flag) {
			printf("enter interrupt: \r\n");
			timer_interrupt_flag = false;
			printf("set flag: \r\n");

			printf("---------------counter: %d-------------\r\n", counter);
			counter++;

			// ("start sending \r\n");

			/* Master: start a transfer. Slave: prepare for a transfer. */
			Cy_SCB_UART_PutArrayBlocking(scb_10_HW, tx_buf, tx_length);
			/* Blocking wait for transfer completion */
			//int t = 0;
			// TODO: add a send timeout!!!

			// printf("sent \r\n");

			// receive from Pixy camera
			/* Master: start a transfer. Slave: prepare for a transfer. */
			printf("start receiving \r\n");  // due to clock discrepancy or other reasons, it is not always possible to receive 20 bytes from Pixy
			/* Start transmit operation */
			Cy_SCB_UART_GetArray(scb_10_HW, rx_buf, rx_length);
			/* Blocking wait for transmission completion */

			if (rx_buf[3] == 0 || rx_buf[3] == 1) { // check if block is detected, byte 3 length of payload will be 0 or 1 if no object detected
				error_flag = 1;
			}
			if (rx_buf[0] != 175 && rx_buf[1] != 193) { // dealing with byte mismatch
				error_flag = 1;
			}
			if (error_flag == 1) {
				printf("read error! \r\n");
				Cy_SCB_UART_ClearRxFifo(scb_10_HW);
				Cy_SysLib_Delay(100);
			} else {
	//            uint32_t one_byte;
	//            for (unsigned int i = 0; i < RX_BUF_SIZE; ++i) {
	//                one_byte = rx_buf[i];
	//                printf("byte %u: %lu \r\n", i, one_byte);
	//            }
				printf("received \r\n");

				printf("X Distance: %d%d \r\n", rx_buf[9], rx_buf[8]);
				printf("Y Distance: %d%d \r\n", rx_buf[11], rx_buf[10]);
				printf("Width: %d%d \r\n", rx_buf[13], rx_buf[12]);
				printf("Height: %d%d \r\n", rx_buf[15], rx_buf[14]);

			}
	   	}

        printf("\r\n");


        if (error_flag == 0) {

            /* ---------------------------------------CALCULATE DISTANCE------------------------------------------ */
            printf("calculate distance: \r\n");
            pixelsWidth = rx_buf[13] << 4 | rx_buf[12];
            printf("pixelsWidth: %d \r\n", pixelsWidth);
            pixelsHeight = rx_buf[15] << 4 | rx_buf[14];
            printf("pixelsHeight: %d \r\n", pixelsHeight);
            distanceWidth = (widthOfObject * focalLengthWidth) / pixelsWidth;
            printf("distanceWidth: %f \r\n", distanceWidth);
            distanceHeight = (heightOfObject * focalLengthHeight) / pixelsHeight;
            printf("distanceHeight: %f \r\n", distanceHeight);
            pixy_x = rx_buf[9] << 4 | rx_buf[8]; //PID
            printf("pixy_x: %d \r\n", pixy_x);
            avg = (distanceWidth + distanceHeight)/2;

            printf("now start pwm: \r\n");
            // if(CY_RSLT_SUCCESS != result){printf("API cyhal_pwm_start failed with error code: %lu\r\n", (unsigned long) result);CY_ASSERT(false);}


            // Cy_SysLib_Delay(100);
            printf("Output later: %f \r\n", output);
            printf("Actual Distannce: %f \r\n", avg);
            printf("\r\n");
            printf("pixy_x: %d \r\n", pixy_x);
            printf("\r\n");
            /* -----------------------------------------------PID CONTROL--------------------------------------------------------- */

            if (avg >= Setpoint) {

                //-----------------------------------------------------------------------------------------
                //forward
                //forward straight
                if (pixy_x < 178 && pixy_x > 138) {
                	printf("going straight -------- \r\n");
                    Input = avg;
                    Compute();

                    printf("pwm straight \r\n");
//                    cyhal_pwm_set_duty_cycle(&pwm_obj_l, 100, 490);
                    cyhal_pwm_set_duty_cycle(&pwm_obj_l, output*0.35, 490);
//                    cyhal_pwm_set_duty_cycle(&pwm_obj_l, 100*2.17/3.25, 490);
                    cyhal_gpio_write(P6_3, false);
                    cyhal_gpio_write(P6_4, true);

//                    cyhal_pwm_set_duty_cycle(&pwm_obj_r, 100, 490);
                    cyhal_pwm_set_duty_cycle(&pwm_obj_r, output*0.35, 490);
//                    cyhal_pwm_set_duty_cycle(&pwm_obj_r, 100, 490);
                    cyhal_gpio_write(P8_2, false);
                    cyhal_gpio_write(P1_0, true);

                }
                //turn right
                if (pixy_x >= 178) {
                	printf("going straight right-------- \r\n");
                    cyhal_pwm_set_duty_cycle(&pwm_obj_l, 55, 490);
                    cyhal_gpio_write(P6_3, false);
                    cyhal_gpio_write(P6_4, true);
                    printf("pwm right \r\n");
                    cyhal_pwm_set_duty_cycle(&pwm_obj_r, 40, 490);
                    cyhal_gpio_write(P8_2, false);
                    cyhal_gpio_write(P1_0, true);
                    printf("pwm right \r\n");

                }
                //turn left
                if (pixy_x <= 138) {
                	printf("going straight left-------- \r\n");
                    cyhal_pwm_set_duty_cycle(&pwm_obj_l, 40, 490); // 2.17/3.25
                    cyhal_gpio_write(P6_3, false);
                    cyhal_gpio_write(P6_4, true);
                    printf("pwm left \r\n");
                    cyhal_pwm_set_duty_cycle(&pwm_obj_r, 55, 490);
					cyhal_gpio_write(P8_2, false);
					cyhal_gpio_write(P1_0, true);
					printf("pwm left \r\n");

                }

                printf("output: %f \r\n", output);
            }

//            //-----------------------------------------------------------------------------------------
//            //stay stationary
            if ((Setpoint-1) < avg && avg < Setpoint) {

                //stationary straight
                if (138 < pixy_x && pixy_x < 178) {
                	printf("staying stationary-------- \r\n");

                	cyhal_pwm_set_duty_cycle(&pwm_obj_l, 0, 490);
                    cyhal_gpio_write(P6_3, false);
                    cyhal_gpio_write(P6_4, true);

                	cyhal_pwm_set_duty_cycle(&pwm_obj_r, 0, 490);
                    cyhal_gpio_write(P8_2, false);
                    cyhal_gpio_write(P1_0, true);
                }

                //turn left
                if (pixy_x <= 138) {
                	printf("staying right-------- \r\n");

                	cyhal_pwm_set_duty_cycle(&pwm_obj_l, 0, 490);
                    cyhal_gpio_write(P6_3, false);
                    cyhal_gpio_write(P6_4, true);

                	cyhal_pwm_set_duty_cycle(&pwm_obj_r, 1600/Setpoint/255*100, 490);
                    cyhal_gpio_write(P8_2, false);
                    cyhal_gpio_write(P1_0, true);


                }
                //turn right
                if (pixy_x >= 178) {
                	printf("staying left-------- \r\n");

                    cyhal_pwm_set_duty_cycle(&pwm_obj_l, 1600/Setpoint/255*100, 490);
                    cyhal_gpio_write(P6_3, false);
                    cyhal_gpio_write(P6_4, true);


                	cyhal_pwm_set_duty_cycle(&pwm_obj_r, 0, 490);
                    cyhal_gpio_write(P8_2, false);
                    cyhal_gpio_write(P1_0, true);

                    cyhal_gpio_write(CYBSP_USER_LED1, false);
                }
                printf("output: %f \r\n", output);
            }

            //-----------------------------------------------------------------------------------------
            //back up
            //back up straight
            if (avg <= (Setpoint-1)) {
            	printf("going back-------- \r\n");
                if (pixy_x < 168 && pixy_x > 148) {
                    Input = avg;
                    Compute();

                    cyhal_pwm_set_duty_cycle(&pwm_obj_l, output/255*100, 490);
                    cyhal_gpio_write(P6_3, true);
                    cyhal_gpio_write(P6_4, false);
                    cyhal_pwm_set_duty_cycle(&pwm_obj_r, output/255*100, 490);
                    cyhal_gpio_write(P8_2, true);
                    cyhal_gpio_write(P1_0, false);
                }

                //turn left
                if (pixy_x <= 148) {
                    Input = avg;
                    Compute();

                    cyhal_pwm_set_duty_cycle(&pwm_obj_l, output/255*100, 490);
                    cyhal_gpio_write(P6_3, true);
                    cyhal_gpio_write(P6_4, false);
					cyhal_pwm_set_duty_cycle(&pwm_obj_r, output/255*100/1.5, 490);
                    cyhal_gpio_write(P8_2, true);
                    cyhal_gpio_write(P1_0, false);
                }

                //turn right
                if (pixy_x >= 168) {
                    Input = avg;
                    Compute();

                    cyhal_pwm_set_duty_cycle(&pwm_obj_l, output/255*100/1.5, 490);
                    cyhal_gpio_write(P6_3, true);
                    cyhal_gpio_write(P6_4, false);
					cyhal_pwm_set_duty_cycle(&pwm_obj_r, output/255*100, 490);
                    cyhal_gpio_write(P8_2, true);
                    cyhal_gpio_write(P1_0, false);
                }
                printf("output: %f \r\n", output);
            }

        } else {
        	printf("Output last: %f \r\n", output);
        	printf("stop");
        	cyhal_pwm_set_duty_cycle(&pwm_obj_l, 0, 490);
        	cyhal_pwm_set_duty_cycle(&pwm_obj_r, 0, 490);
        }


    }
	/* -----------------------------------------------TCP DATA--------------------------------------------------------- */
}
