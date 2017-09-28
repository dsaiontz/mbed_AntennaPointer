//ATENNACONTROL sends angles (in degrees) to the antenna to move towards.
//  ANTENNACONTROL can accept angles from a serial terminal or Matlab.
//
// D. Saiontz, M. Rancic, M. Kutzer 4Aug2016, USNA/SEAP
 
#include "mbed.h"
#include "mbedWSEsbc.h"
#include "Motor.h"
#include "TextLCD.h"
#include "MODSERIAL.h"
 
// Declare LCD
TextLCD lcd(p9,p10,p15,p16,p17,p18); // rs, e, d4-d7 (Most pins are already used up on the board. Don't change.
// Declare serial with interupts
MODSERIAL comp(USBTX,USBRX);
// Declare digital out for debugging
DigitalOut test[]= {LED1,LED2,LED3,LED4};
// Declare heading and elevation motors
Motor HeadingMot(p23,p22,p21);   //Heading Motor
Motor ElevationMot(p26,p25,p24);     //Elevation Motor
// Declare local file for saving debugging data
LocalFileSystem local("local");
// Declare encoder count variables
int HeadingEnc,ElevationEnc;
int HeadingEnc_bias,ElevationEnc_bias;
int HeadingEnc_pos,ElevationEnc_pos;    //Encoder Values
int HeadingEnc_corrected,ElevationEnc_corrected;
// Declare converted angle variables (degrees)
float ElevationDeg,HeadingDeg;                // Current elevation and heading
float ElevationDegDesired=0.00, HeadingDegDesired=0.00; // Desired elevation and heading
 
volatile float HeadingPWM,ElevationPWM;
volatile float ElevationDegErr,ElevationDegErr_Prev=0,ElevationDegErr_Derivative,ElevationDegErr_Integral=0,HeadingDegErr,HeadingDegErr_Prev=0,HeadingDegErr_Derivative,HeadingDegErr_Integral=0;
char calibrate=' ';
int n=0;
int counter=0;
char filename[64];
char run=' ';
char data[128];
 
// Stopping tolerance for PID
float ElevationTol = 3;
float HeadingTol = 2;
 
void SerRxInterrupt(MODSERIAL_IRQ_INFO *juan)
{
    MODSERIAL *charles = juan->serial;
    charles->move(data, 128, '\n');
    sscanf (data,"%f,%f",&ElevationDegDesired,&HeadingDegDesired);
    ElevationDegErr_Integral = 0;
    HeadingDegErr_Integral = 0;
}
 
int main()
{
    // Magical board initialization (Joe Bradshaw for help)
    mbedWSEsbcInit(9600);
    // Attach serial Rx interrupt
    comp.attach(&SerRxInterrupt, MODSERIAL::RxAutoDetect);
    // Set MODSERIAL to interupt on linefeed
    comp.autoDetectChar('\n');
    comp.baud(9600);
    wait(.2); //Give time for the LCD to set up
    //comp.format(8,SerialBase::None,1);
    /*
    //Calibration
    printf("Hit , and . to move the arm and / to stop until you have it vertical. Also, Face the dish North.\nWhen satisfied, enter m\n");
    while(calibrate != 'm') {
        calibrate=getchar();
        if(calibrate=='.') {
            ElevationMot.speed(-.4);
        } else if(calibrate==','){
            ElevationMot.speed(.4);
        }else if(calibrate=='/'){
            ElevationMot.speed(0);
            }
    }
 
    HeadingEnc_bias=LS7366_read_counter(1);   //Store the offset of each of the encoders
    ElevationEnc_bias=LS7366_read_counter(2);
 
 
    printf("If you are not entering position manually, disconnect TeraTerm and connect matlab.\n Hit any key to continue.\nHit q to quit once the program is running.\n");
    calibrate='`';
    while(calibrate=='`') {
        calibrate=getchar();
    }
    */
 
    // Allow user to do calibration manually
    lcd.printf("Please\nCalibrate");
    while(calibrate != 'k') {
        calibrate = comp.getc();
        if (calibrate == 'g') {
            ElevationMot.speed(0);
            HeadingMot.speed(0);
        } else if (calibrate == 'w') {
            ElevationMot.speed(-1);
        } else if (calibrate == 's') {
            ElevationMot.speed(1);
        } else if (calibrate == 'a') {
            HeadingMot.speed(-1);
        } else if (calibrate == 'd') {
            HeadingMot.speed(1);
        } else {
            ElevationMot.speed(0);
            HeadingMot.speed(0);
        }
        // Get current encoder count
        HeadingEnc=LS7366_read_counter(1);
        ElevationEnc=LS7366_read_counter(2);
        //LS7366_reset_counter(1);
        //LS7366_reset_counter(2);
 
        // Display to LCD
        lcd.cls();
        lcd.printf("H: %d\nE: %d",HeadingEnc,ElevationEnc);
        wait(.001);
        //printf("Heading Encoder: %d Elevation Encoder: %d\n", LS7366_read_counter(1), LS7366_read_counter(2));
    }
 
    // Get current encoder count
    HeadingEnc=LS7366_read_counter(1);
    ElevationEnc=LS7366_read_counter(2);
    // Set current encoder count to bias (zero encoders at current position
    HeadingEnc_bias=HeadingEnc;
    ElevationEnc_bias=ElevationEnc;
    // Display to LCD
    lcd.cls();
    lcd.printf("H: %d\nE: %d",HeadingEnc,ElevationEnc);
    wait(.001);
 
    /*
    // Open a new file to write to, incrementing the file number
    while(1) {
        sprintf(filename, "/local/file%03d.txt", n);    // construct the filename fileNNN.txt
        FILE *fp = fopen(filename, "r");                // try and open it
        if(fp == NULL) {                                // if not found, we're done!
            break;
        }
        fclose(fp);                                     // close the file
        n++;                                            // and try the next one
    }
    FILE *fp = fopen(filename, "w");                    //Open New file
    */
 
    // Set MBED LED for debugging
    test[0]=1;
    // Clear LCD
    lcd.cls();
    lcd.printf("Ready");
    wait(.001);
    //Manual Desired Positon entry (Testing Only)
    printf("Enter desired ElevationDeg and HeadingDeg\n");
    //scanf("%f,%f",&ElevationDegDesired,&HeadingDegDesired);
    // BEGIN PID --------------------------------------------
 
    //PID Control
    //Tune PID values
    
    //Elevation Tunings
    float dt=.2;
    float a_kp=.05;
    float a_ki=.01;
    float a_kd=.01;
 
    //Heading Tunings
    float d_kp=.04;
    float d_ki= .001;
    //float d_kd=.03;
    float d_kd= .03;
 
    // ACTUAL PID LOOP
    while(1) {
        //Read current encoder
        HeadingEnc = LS7366_read_counter(1);
        ElevationEnc = LS7366_read_counter(2);
        //Replicate variables for unknown reason
        HeadingEnc_pos=HeadingEnc;
        ElevationEnc_pos=ElevationEnc;
        //Account for encoder bias
        HeadingEnc_corrected=HeadingEnc_pos-HeadingEnc_bias;
        ElevationEnc_corrected=ElevationEnc_pos-ElevationEnc_bias;
 
        /*
                //Read from matlab
                comp.scanf("%f,%f",&ElevationDegDesired,&HeadingDegDesired);
                test[1]=!test[1];   //Flashes while receiving Data
        */
 
        //Convert Encoder values to ElevationDegs
        ElevationDeg=(ElevationEnc_corrected*(90.0/2800.0)*-1.0);
        HeadingDeg=(HeadingEnc_corrected*(180.0/3260.0)*-1.0);
 
        //ElevationDeg=(ElevationEnc*(90.0/2800.0)*-1);
        //HeadingDeg=(HeadingEnc*(180.0/3260.0)*-1);
 
        /*
        //Debug Area
                if(ElevationDeg>30) {
                    test[3]=1;
                    test[2]=0;
                } else if(ElevationDeg<30) {
                    test[3]=0;
                    test[2]=1;
                } else {
                    test[3]=0;
                    test[2]=0;
                }
        */
        /*
        //Proportional control
        ElevationPWM=-.05*(ElevationDegDesired-ElevationDeg);
        HeadingPWM=.25*(HeadingDegDesired-HeadingDeg);
        */
 
        //PID Calculation
        // Elevation motor
        ElevationDegErr=ElevationDegDesired-ElevationDeg;
        if (ElevationDegErr <= ElevationTol && -1.0*ElevationDegErr <= ElevationTol) { //If ElevationDeg is within tolerance, don't move motors
            ElevationPWM = 0;
        } else {
            ElevationDegErr_Integral=ElevationDegErr_Integral+ElevationDegErr*dt;
            ElevationDegErr_Derivative=(ElevationDegErr-ElevationDegErr_Prev)/dt;
            ElevationPWM=(a_kp*ElevationDegErr+a_ki*ElevationDegErr_Integral+a_kd*ElevationDegErr_Derivative);
        }
        ElevationDegErr_Prev=ElevationDegErr;
 
        // Heading motor      
        HeadingDegErr=HeadingDegDesired-HeadingDeg;
        if (HeadingDegErr <= HeadingTol && -1.0*HeadingDegErr <= HeadingTol) { //If HeadingDeg is within tolerance, don't move motors
            HeadingPWM = 0;
        } else {
            HeadingDegErr_Integral=HeadingDegErr_Integral+HeadingDegErr*dt;
            HeadingDegErr_Derivative=(HeadingDegErr-HeadingDegErr_Prev)/dt;
            HeadingPWM=(d_kp*HeadingDegErr+d_ki*HeadingDegErr_Integral+d_kd*HeadingDegErr_Derivative);
        }
        HeadingDegErr_Prev=HeadingDegErr;
 
 
        //Set Maximum PWM for each of the Motors
        if(ElevationPWM>1) {
            ElevationPWM=1;
        } else if(ElevationPWM<-1) {
            ElevationPWM=-1;
        }
 
        if(HeadingPWM>1) {
            HeadingPWM=1;
        } else if(HeadingPWM<-1) {
            HeadingPWM=-1;
        }
/*
        //Set Minimum PWM for each of the Motors
        if (ElevationPWM < .3 && ElevationPWM > 0)
            ElevationPWM = .3;
        else if (ElevationPWM > -.3 && ElevationPWM < 0)
            ElevationPWM = -.3;
        if (HeadingPWM < .3 && HeadingPWM > 0)
            HeadingPWM = .3;
        else if (HeadingPWM > -.3 && HeadingPWM < 0)
            HeadingPWM = -.3;
*/        
        //Set each motor to the calculated PWM
        ElevationMot.speed(ElevationPWM);    
        HeadingMot.speed(HeadingPWM);
        printf("Heading Encoder: %d\n", HeadingEnc);
        lcd.cls();
        lcd.printf("%05.2f  E  %05.2f\n%05.1f  H  %05.1f",ElevationDegDesired,ElevationDeg,HeadingDegDesired,HeadingDeg);
        wait(0.001);
        
        //counter++;
        //if (counter%1==0) {
        //    fprintf(fp,"ElevationDeg:%f Des Ang:%f Dir:%f, Des Dir:%f ElevationPWM:%f HeadingPWM:%f\r\n",ElevationDeg,ElevationDegDesired,HeadingDeg, HeadingDegDesired,ElevationPWM,HeadingPWM);   //Print to log file
        //}
 
    //run=getchar();      //check to see if it should keep running
 
        //if (counter==150) {
        //    fclose(fp); //Close File
        //    test[0]=0;
        //}
    }
}