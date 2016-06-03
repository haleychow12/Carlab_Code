/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/

#include <device.h>
#include <stdio.h>

double ref = 3.2;
double angleRef = 0;
double inchPerPing = 1.5707963268;
double ticks = 0;
int clockPeriod = 0;
int index = 0;
double avgError = 0.0; 
double lastError = 0;
double integral = 0.0;

int numCompositeSyncs = 0;
int numFrames = 0;
double deltaT = 0;
double elapsedTime = 0;


//calculate speed
double carSpeed(int cycles)
{
    //distance traveled converting from pings
    double speedInches = inchPerPing/((double)cycles/10000.0);
    double speedFeet = speedInches/12.0;
    return speedFeet; 
}

//print to LCD function
void printDouble(double d, double e) {
    char strbuff1[9];
    char strbuff[9];
    sprintf(strbuff, "%f", d);
    LCD_Position(0,0);
    LCD_PrintString(strbuff);
    sprintf(strbuff1, "%f", e);
    LCD_Position(1,1);
    LCD_PrintString(strbuff1);

}

//calculate time of each cycle
double getSeconds(int cycles) {
    return ((double) cycles)/10000.0;
}



//do PID control based on interrupt
CY_ISR(inter)
{
    int captureTime = Speed_Timer_ReadCounter();
    int elapsedCycles = (clockPeriod-1) - captureTime;    
    
    double error = 0;
    double duty = 0;
    int MAXDUTY = 70;
    int MINDUTY = 0;
    double derivative = 0;    
    
    //control coefficients
    double kp = 5;
    double ki = 7;
    double kd = 0;
    
    //calculate input to the PWM
    double speed = carSpeed(elapsedCycles);
    error = ref - speed;
    
    ticks++;
    printDouble(ticks,0.0);
    
    
    //reset counter
    Speed_Timer_WriteCounter(clockPeriod-1);
    //derivative term
    if (lastError != 0){
        derivative = ((error - lastError)/getSeconds(elapsedCycles));
    }
    lastError = error;
    
    //integral term
    integral += error*getSeconds(elapsedCycles);


    duty = (kp*error) + (ki*integral) + (kd*derivative);
    
    if (ticks >= 1500)
        duty = MINDUTY;
        
    if (duty > MAXDUTY)
        duty = MAXDUTY;
    else if (duty < MINDUTY)
        duty = MINDUTY;
    Speed_PWM_WriteCompare((int)duty);

    //calculate avg error
    index++;
    avgError += error;
    if (index > 100){
        //printDouble(avgError/index, speed);
        avgError = 0.0;
        index = 0;
    }
    
}



//interrupt from timer after getting input from counter
//counter gets composite syncs and vertical syncs as inputs

CY_ISR(inter2){

    //get time
    double referenceTime = 28.5; //need to check if it hits next composite sync
    uint32 captureTime = Servo_Timer_1_ReadCapture();
    uint32 elapsedCycles = (Servo_Timer_1_ReadPeriod()-1) - captureTime;
    
    double dutyCycle = 0;
    double kp = 2.5;
    int MAXDUTY = 200;
    int MINDUTY = 100; //100 
    
    elapsedTime = (double)elapsedCycles; //time in microseconds
    //time - ref time and then get pos or neg error and then decide if move right or left
    deltaT = elapsedTime - referenceTime;
    
    dutyCycle = kp*deltaT + 152;
    
    if (dutyCycle > MAXDUTY)
        dutyCycle = MAXDUTY;
    else if (dutyCycle <= MINDUTY)
        dutyCycle = 0;
    
   
    Servo_PWM_WriteCompare((uint16)dutyCycle);

    
    Servo_Timer_1_ReadStatusRegister();
    


   
}

void main()
{
    /* Place your initialization/startup code here (e.g. MyInst_Start()) */

    CyGlobalIntEnable;
    
    //call hall effect interrupt
    hall_inter_Start();
    hall_inter_SetVector(inter);
    
    //start timer and call timer interrupt
    Speed_Timer_Start();
    clockPeriod = Speed_Timer_ReadPeriod();
    
    Speed_PWM_Start();
    Speed_PWM_WriteCompare(40);
    
    
    New_Sync_Counter_Start();
    
    
    steer_inter_Start();
    steer_inter_SetVector(inter2);

    
    //start servo timer
    Servo_Timer_1_Start();
    
    Servo_PWM_Start();
    //Servo_PWM_WriteCompare(30); //chose this number arbitrarily, tried to use this to check if servo was working

    LCD_Start();
    
    for(;;)
    {
        /* Place your application code here. */
    }
}

/* [] END OF FILE */
