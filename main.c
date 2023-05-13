#include <stdint.h>
#include <string.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>
#include "hw_ints.h"
#include "hw_memmap.h"
#include "hw_sysctl.h"
#include "TM4C123GH6PM.h"
#include "Dio.h"
#include "types.h"


/*Initializations*/
void portAInit(void);
void portBInit(void);
void portDInit(void);
void PortFInit(void);

/*Functions*/
void PassengerWindowTask( void *pvParameters );
void ManualControlTask(void* parameters);
void MotorControlTask( void *pvParameters );
void JammerTask(void *pvParameters);
void MotorUp(void);
void MotorDown(void);
void MotorOff(void);
void delayMs(int n);

/*Semaphores*/
SemaphoreHandle_t xJammSemaphore;
SemaphoreHandle_t xJammTrueSemaphore;


/*Define task handles*/
xTaskHandle passenger;

/*Define Queue handles*/
xQueueHandle xMotorQueue;
xQueueHandle xPassengerQueue;


/*Motor Functions*/

void MotorUp(void){
			DIO_WritePin(&GPIO_PORTA_DATA_R,2,1);
			DIO_WritePin(&GPIO_PORTA_DATA_R,3,0);
}
void MotorDown(void){
			DIO_WritePin(&GPIO_PORTA_DATA_R,2,0);
			DIO_WritePin(&GPIO_PORTA_DATA_R,3,1);
}
void MotorOff(void){
			DIO_WritePin(&GPIO_PORTA_DATA_R,2,1);
			DIO_WritePin(&GPIO_PORTA_DATA_R,3,1);
}
void delayMs(int n){
  int i,j;
  for(i=0;i<n;i++)
  {
    for(j=0;j<3180;j++)
    {}
  }
}

/*Switch task function*/

void ManualControlTask(void* parameters) {
	portBASE_TYPE xStatus;
    while (1) {
        if ((GPIO_PORTB_DATA_R & 0x4)==0x4) { //driver manual up
					xStatus=xQueueSendToBack(xPassengerQueue,&(uint32_t){3},0);
				}
				if  ((GPIO_PORTB_DATA_R & 0x08)==0x08){ //driver manual downn
					xStatus=xQueueSendToBack(xPassengerQueue,&(uint32_t){4},0);
				}
				if (((GPIO_PORTB_DATA_R & 0x40)==0x40)&&((GPIO_PORTA_DATA_R & 0x40)!=0x40)) //passenger manual up and lock checking
				{ 
					xStatus=xQueueSendToBack(xPassengerQueue,&(uint32_t){7},0);
				}
				if (((GPIO_PORTB_DATA_R & 0x80)==0x80) &&((GPIO_PORTA_DATA_R & 0x40)!=0x40)) //passenger manual down and lock checking
				{ 
					xStatus=xQueueSendToBack(xPassengerQueue,&(uint32_t){8},0);
				}
        vTaskDelay(pdMS_TO_TICKS(10)); // debounce switch
    }
}

/*Jammer*/

void JammerTask(void *pvParameters){
	long message=(long)pvParameters;
	portBASE_TYPE xStatus;
	xSemaphoreTake(xJammSemaphore,0);
	for(;;)
	{
		  xSemaphoreTake(xJammTrueSemaphore,portMAX_DELAY);
			xSemaphoreTake(xJammSemaphore,portMAX_DELAY);
			xStatus=xQueueSendToBack(xMotorQueue,&(uint32_t){0},0);
	}
}

/*MotorController*/

void MotorControlTask(void *pvParameters){
	uint32_t message;
	for(;;)
{
	xQueueReceive(xMotorQueue,&message,portMAX_DELAY);
	if(message==0){ //Jamm Mode Motor stops then move backwards
		MotorOff();
		MotorDown();
		delayMs(500);
		MotorOff();
	}else if(message==1){ //Motor forward
		MotorUp();
	}else if(message==2){ //Motor backward
		MotorDown();
	}else if(message==3){  //Motor Stops
		MotorOff();
  }
 }
}

/*Passenger Window*/

void PassengerWindowTask(void *pvParameters){
  portBASE_TYPE xStatus;
	uint32_t message;
	for(;;)
{ 
	xQueueReceive(xPassengerQueue,&message,portMAX_DELAY);
	switch (message){
		case 1:  //Driver press passenger Automatic Up
		  xSemaphoreGive(xJammSemaphore);
      xStatus=xQueueSendToBack(xMotorQueue,&(uint32_t){1},0);
			break;
		case 2:  //Driver Press Passenger Automatic Down
			xStatus=xQueueSendToBack(xMotorQueue,&(uint32_t){2},0);
			break;
		case 3: //Driver Press Passenger Manual Up
			xStatus=xQueueSendToBack(xMotorQueue,&(uint32_t){1},0);
			break;
		case 4: //Driver Press Passenger Manual Down
			xStatus=xQueueSendToBack(xMotorQueue,&(uint32_t){2},0);
			break;
		case 5:  //Passenger Press Automatic Up
			xSemaphoreGive(xJammSemaphore);
		  xStatus=xQueueSendToBack(xMotorQueue,&(uint32_t){1},0);
			break;
		case 6:  //Passenger Press Automatic Down
			xStatus=xQueueSendToBack(xMotorQueue,&(uint32_t){2},0);
			break;
		case 7:  //Passenger Press Manual Up
			xStatus=xQueueSendToBack(xMotorQueue,&(uint32_t){1},0);
			break;
		case 8:  //Passenger Press Manual Down
			xStatus=xQueueSendToBack(xMotorQueue,&(uint32_t){2},0);
			break;
	 }
  }
 }


/*Main*/
 
int main( void )
{
	portAInit();
	portBInit();
	portDInit();
  PortFInit();
	xMotorQueue=xQueueCreate(20,sizeof(uint32_t));	
	xPassengerQueue=xQueueCreate(20,sizeof(uint32_t));
	xJammSemaphore = xSemaphoreCreateBinary();
	xJammTrueSemaphore = xSemaphoreCreateBinary();
	__ASM("CPSIE i");
	
	if( (xJammSemaphore && xJammTrueSemaphore&& xMotorQueue) != NULL )
		{
			xTaskCreate(ManualControlTask, "Switch Task", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
			xTaskCreate( PassengerWindowTask, "Passenger", configMINIMAL_STACK_SIZE, NULL, 2, passenger );
			xTaskCreate( MotorControlTask, "Motor",configMINIMAL_STACK_SIZE, NULL, 3, NULL );
			xTaskCreate (JammerTask, "Jammer", configMINIMAL_STACK_SIZE,NULL,4,NULL);
			
			vTaskStartScheduler();
		}

    for( ;; );
}

/*Initializations*/

void portAInit(void){
	SYSCTL_RCGCGPIO_R |= 0x00000001;                                     // initialize clock
	while((SYSCTL_PRGPIO_R&0x00000001) == 0){}                           // looping until clock is initiallized
	GPIO_PORTA_CR_R |= 0x000000ff;                                    // unlocking commit register for switch 1 & switch 2 
  GPIO_PORTA_DIR_R |= 0x0000000c;                            // detrmining the output pins                                   
  GPIO_PORTA_DEN_R |= 0x000000ff;              // detrmining the pins direction 
  GPIO_PORTA_DATA_R |= 0x00000000;
	GPIOA -> PDR = 0x40; 
}

void portBInit(void){

	SYSCTL->RCGCGPIO |= 0x00000002; 	// initialize clock                           // looping until clock is initiallized
	GPIOB->LOCK = 0x4C4F434B;
	GPIOB->AMSEL= 0x00;       				 // 3) disable analog function
  GPIOB->PCTL = 0x00000000;  				 // 4) GPIO clear bit PCTL 
	GPIOB->CR |= 0x000000ff;                                    // unlocking commit register for switch 1 & switch 2 
  GPIOB->DIR |= 0x00000000;                            // detrmining the output pins                                   
  GPIOB->AFSEL = 0x00; 
	GPIOB->DEN |= 0x000000ff;              // detrmining the pins direction 
  GPIOB->DATA |= 0x00000000;
	GPIOB -> PDR = 0xff; 	
	GPIOB->ICR = 0xff;     // Clear any Previous Interrupt
  GPIOB->IM |=0xff;      // Unmask the interrupts for PF0 and PF4	
	NVIC_EN0_R |= 0x02;
	NVIC_PRI0_R |= 0xe000;
}

void portDInit(void){

	SYSCTL->RCGCGPIO |= 0x00000008; 	// initialize clock                           // looping until clock is initiallized
	GPIOD->LOCK = 0x4C4F434B;
	GPIOD->AMSEL= 0x00;       				 // 3) disable analog function
  GPIOD->PCTL = 0x00000000;  				 // 4) GPIO clear bit PCTL 
	GPIOD->CR |= 0x000000ff;                                    // unlocking commit register for switch 1 & switch 2 
  GPIOD->DIR |= 0x00000000;                            // detrmining the output pins                                   
  GPIOD->AFSEL = 0x00; 
	GPIOD->DEN |= 0x000000ff;              // detrmining the pins direction 
  GPIOD->DATA |= 0x00000000;
	GPIOD -> PDR = 0xff; 	
	GPIOD->ICR = 0xff;     // Clear any Previous Interrupt
  GPIOD->IM |=0xff;      // Unmask the interrupts for PF0 and PF4	
	NVIC_EN0_R |= 0x08; 
	NVIC_PRI0_R |= 0xe000; 
}

void PortFInit(void){ 
  SYSCTL->RCGCGPIO |= 0x00000020;    // 1) F clock
  GPIOF->LOCK = 0x4C4F434B;  				 // 2) unlock PortF PF0  
  GPIOF->CR = 0x1F;          				 // allow changes to PF4-0       
  GPIOF->AMSEL= 0x00;       				 // 3) disable analog function
  GPIOF->PCTL = 0x00000000;  				 // 4) GPIO clear bit PCTL  
  GPIOF->DIR = 0x0E;         				 // 5) PF4,PF0 input, PF3,PF2,PF1 output   
  GPIOF->AFSEL = 0x00;      				 // 6) no alternate function
  GPIOF->PUR = 0x11;       				   // enable pullup resistors on PF4,PF0       
  GPIOF->DEN = 0x1F;       				   // 7) enable digital pins PF4-PF0
	GPIOF->DATA = 0x00;
	GPIOF->ICR = 0x11;     // Clear any Previous Interrupt 
	GPIOF->IM |=0x11;      // Unmask the interrupts for PF0 and PF4
	GPIOF->IS |= 0x11;     // Make bits PF0 and PF4 level sensitive
	GPIOF->IEV &= ~0x11;   // Sense on Low Level
	NVIC_EN0_R |= 0x40000000;
  NVIC_PRI7_R |= 0xe0000;  // Enable the Interrupt for PortF in NVIC
}



/*Port B Handler*/

void GPIOB_isrHandler(void){

 uint32_t pinFlags;
 pinFlags = GPIO_PORTB_MIS_R;
	/*Case Driver Window Passenger Automatic Up*/
	 if(pinFlags & (1 << 0)){
	 GPIOB->ICR = 0xff;        // clear the interrupt flag of PORTB
	 xQueueSendFromISR(xPassengerQueue,&(uint32_t){1},0);	 
  }
	/*Case Driver Window Passenger Automatic Down*/
 if(pinFlags & (1 << 1)){
	 GPIOB->ICR = 0xff;        // clear the interrupt flag of PORTB
   xQueueSendFromISR(xPassengerQueue,&(uint32_t){2},0);	
  }
 /*Case Driver Window Passenger Manual Up*/
	 if(pinFlags & (1 << 2)){	 
	 xQueueSendFromISR(xMotorQueue,&(uint32_t){3},0);	
	 GPIOB->ICR = 0xff;        // clear the interrupt flag of PORTB	
  }
	 /*Case Driver Window Passenger Manual Down*/
		 if(pinFlags & (1 << 3)){	 
	 xQueueSendFromISR(xMotorQueue,&(uint32_t){3},0);	
	 GPIOB->ICR = 0xff;        // clear the interrupt flag of PORTB
  }
	/*Case  Passenger Automatic Up*/
 if(pinFlags & (1 << 4)){
	  if((GPIO_PORTA_DATA_R & 0x40)!= 0x40){    
	 xQueueSendFromISR(xPassengerQueue,&(uint32_t){5},0);
	 portEND_SWITCHING_ISR( passenger );
   }
		GPIOB->ICR = 0xff; 
 }
	/*Case Passenger Automatic Down*/
 if(pinFlags & (1 << 5)){	 
	     if((GPIO_PORTA_DATA_R & 0x40)!= 0x40){    
	 xQueueSendFromISR(xPassengerQueue,&(uint32_t){6},0);
	 portEND_SWITCHING_ISR( passenger );
 }
	 GPIOB->ICR = 0xff; 
}
	/*Case Passenger Manual Up*/
	if(pinFlags & (1 << 6)){	 
	         // clear the interrupt flag of PORTB
   xQueueSendFromISR(xMotorQueue,&(uint32_t){3},0);	
	 GPIOB->ICR = 0xff;
}	
	/*Case Passenger Manual Down*/
	 if(pinFlags & (1 << 7)){	 
	 xQueueSendFromISR(xMotorQueue,&(uint32_t){3},0);	
	 GPIOB->ICR = 0xff; 
  }
}

/*Port D Handler*/

void GPIOD_isrHandler(void){
 uint32_t pinFlags;
 pinFlags = GPIO_PORTD_MIS_R;
	
/*Limit Switch Button*/
	
	 if(pinFlags & (1 << 0)){
	 GPIOD->ICR = 0xff;  
   xQueueSendFromISR(xMotorQueue,&(uint32_t){3},0);	
  }
}

/*Port F handler*/

void GPIOF_Handler(void){
 uint32_t pinFlags;
 pinFlags = GPIO_PORTF_MIS_R;

/*Jammer Button*/
	
	 if(pinFlags & (1 << 0)){
  GPIOF->ICR = 0xff; 
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(xJammTrueSemaphore,&xHigherPriorityTaskWoken);
	       // clear the interrupt flag of PORTF 
	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
  }
}