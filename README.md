# RTOS-Power-window-control-system
The code is written in C and uses the FreeRTOS library for task management.
The code begins with the inclusion of necessary header files and the declaration of various functions, tasks, queues, and semaphores used in the system. 
The main function initializes the necessary ports, creates tasks using the xTaskCreate() function, and starts the FreeRTOS scheduler using vTaskStartScheduler().

We defined several tasks that run concurrently and communicate with each other using queues and semaphores.

ManualControlTask: Monitors the driver's manual control inputs and sends corresponding messages to the passenger queue to run the motor.

PassengerWindowTask: Sends in motor queue based on case of which button is pressed.

MotorControlTask: Receives messages from the PassengerWindowTask and controls the motor accordingly. It can stop the motor, move it up, or move it down.

JammerTask: Monitors a button press and sends a message to the MotorControlTask through a queue to activate a "jamm" mode. This mode stops the motor and moves it backward.

![image](https://github.com/ziadmhelmy/RTOS-Power-window-control-system/assets/133164561/c9872b7a-cad0-4470-a45e-eea5f074a464)
