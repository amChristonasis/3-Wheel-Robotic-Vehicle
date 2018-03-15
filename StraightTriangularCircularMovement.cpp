#include "mbed.h"
#include "MPU9150.h"
#include "math.h"
//wait ths switch
bool waitStop = true, waitStop1 = true, waitStop2 = true, waitStop3 = true;
double sumGwniaKyklou;

DigitalOut led(LED1);
DigitalIn button(PC_13);
double gwnia, vy, apostashY, prohgoumenhGwnia;

//left wheel
DigitalOut In1(PC_10);
DigitalOut In2(PC_11);
PwmOut EnA(PC_3);
double ena = 0.6;

//right wheel
DigitalOut In3(PA_13);
DigitalOut In4(PA_14);
PwmOut EnB(PA_9);
double enb = 0.6;
//ForwardMovement
void CallForward()
{
    In1=1;
    In2=0;
    ena = ena - gz*0.005;
    EnA=ena;
                        
                        
    In3=1;
    In4=0;
    enb = enb + gz*0.005;
    EnB=enb;
}

//CircularMovement
void CallCircle()
{
    In1=1;
    In2=0;
    ena = ena - (gz-7.3)*0.005;
    EnA=ena;
                        
                        
    In3=1;
    In4=0;
    enb = enb + (gz-7.3)*0.005;
    EnB=enb;
}
//pisw
void CallBackwards()
{
    In1=0;
    In2=1;
    EnA=ena;

    In3=0;
    In4=1;
    EnB=enb;
}
//de3ia
void CallRight()
{
    In1=1;
    In2=0;
    EnA=0.3;

    In3=0;
    In4=0;
    EnB=0;
}
//aristera

void CallLeft()
{
    In1=0;
    In2=0;
    EnA=0;

    In3=1;
    In4=0;
    EnB=enb;
}

//de3iaapotoma
void CallRightSwift()
{
    In1=1;
    In2=0;
    EnA=1;

    In3=0;
    In4=1;
    EnB=1;
}
//aristeraapotoma
void CallLeftSwift()
{
    In1=0;
    In2=1;
    EnA=0.5;

    In3=1;
    In4=0;
    EnB=0.5;
}
//stamata
void CallStop()
{
    In1=0;
    In2=0;
    EnA=0;

    In3=0;
    In4=0;
    EnB=0;
}

//**************************************************
//**************  Ultrasound variables  ************
//**************************************************

DigitalOut      ultraTrig(PC_6) ;
InterruptIn     ultraEcho(PC_8) ;
Timer           EchoTime ;
Ticker          UltaTickerTrig ;
float           UltraDistance ;
float           UltraEchoTime ;

//**************************************************
//**************  Ultrasound functions  ************
//**************************************************

//**************************************************
void UltraTrigStart()
{
//    led=1 ;
    ultraTrig = 1 ;
    wait_us(12) ;
    ultraTrig = 0 ;
//    led=0 ;
}

//**************************************************
void UltraEchoStart()
{
    EchoTime.reset() ;
    EchoTime.start() ;
    led=1 ;
}


//**************************************************
void UltraEchoEnd()
{
    EchoTime.stop() ;
    UltraEchoTime = EchoTime.read();
    UltraDistance = UltraEchoTime * (340.0*50) ;   // estimation in centimeters
    led=0 ;
}

/****************************************************************************
    MPU9150 Example Code
    Robotic wireless systems
    Evangelos Dermatas,
    1-4-2017

 Hardware setup:
 VDD ---------------------- 3.3V
 SDA ----------------------- PB_8
 SCL ----------------------- PB_9
 GND ---------------------- GND

*****************************************************************************/


// ********************************************************************************************************************
float sum = 0;
uint32_t sumCount = 0, mcount = 0;
char buffer[14];
Ticker metrhsh;
double gz0, ay0, vy0;
double sumgz, sumay, sumvy;
bool prwtaGzAy = true, prwtoVy = true;
MPU9150 MPU9150;
bool prohgoumemnhGwnia, dphi;

Timer t;

Serial pc(USBTX, USBRX); // tx, rx

volatile char newData=0;

void compute9150()
{
    //if(MPU9150.readByte(MPU9150_ADDRESS, INT_STATUS) & 0x01) {  // On interrupt, check if data ready interrupt
    newData=1;
    //}
}

// ********************************************************************************************************************

bool wait1=true;
bool wait2=true;
bool wait3=true;
bool arxikopoihsh1=true;
bool arxikopoihsh2=true;
bool arxikopoihsh3=true;

int main()
{
    while (button==1);

    enum STATE {k1,k2,k3,k4,k5};
    STATE st=k1;
    enum STATE2 {l1,l2,l3,l4,l5,l6,l7,l8,l9,l10,l11,l12,l13,l14,l15};
    STATE2 st2=l1;
    enum STATE3 {m1,m2,m3};
    STATE3 st3=m1;
    Timer t2;

    UltaTickerTrig.attach(&UltraTrigStart, 0.2 );   // every 200ms
    ultraEcho.rise(&UltraEchoStart) ;
    ultraEcho.fall(&UltraEchoEnd) ;

    EnA.period_ms(20); //period 20 ms

    EnB.period_ms(20);

    pc.baud(9600);

    //pc.printf( "Hello three wheel robotic car\r\n" ) ;
    //pc.printf("CPU SystemCoreClock is %d Hz\r\n", SystemCoreClock);

    //Set up I2C in fast mode: 400 kHz
    i2c.frequency(400000);

    t.start();


    // Read the WHO_AM_I register, this is a good test of communication
    uint8_t whoami = MPU9150.readByte(MPU9150_ADDRESS, WHO_AM_I_MPU9150);  // Read WHO_AM_I register for MPU-9250
    //pc.printf("I AM 0x%x\n\r", whoami);
    //pc.printf("I SHOULD BE 0x68\n\r");

    if (whoami == 0x68) { // WHO_AM_I should be 0x68
        //pc.printf("MPU9150 WHO_AM_I is 0x%x\n\r", whoami);
        //pc.printf("MPU9150 is online...\n\r");
        //wait(1);

        MPU9150.MPU9150SelfTest(SelfTest);
        //pc.printf("x-axis self test: acceleration trim within %f % of factory value\n\r", SelfTest[0]);
        //pc.printf("y-axis self test: acceleration trim within %f % of factory value\n\r", SelfTest[1]);
        /* pc.printf("z-axis self test: acceleration trim within %f % of factory value\n\r", SelfTest[2]);
         pc.printf("x-axis self test: gyration trim within %f % of factory value\n\r", SelfTest[3]);
         pc.printf("y-axis self test: gyration trim within %f % of factory value\n\r", SelfTest[4]);
         pc.printf("z-axis self test: gyration trim within %f % of factory value\n\r", SelfTest[5]);
         wait(1);*/
        MPU9150.resetMPU9150(); // Reset registers to default in preparation for device calibration
        MPU9150.calibrateMPU9150(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers
        /*pc.printf("x gyro bias = %f\n\r", gyroBias[0]);
        pc.printf("y gyro bias = %f\n\r", gyroBias[1]);
        pc.printf("z gyro bias = %f\n\r", gyroBias[2]);
        pc.printf("x accel bias = %f\n\r", accelBias[0]);
        pc.printf("y accel bias = %f\n\r", accelBias[1]);
        pc.printf("z accel bias = %f\n\r", accelBias[2]);
        wait(1);*/
        MPU9150.initMPU9150();
        //pc.printf("MPU9150 initialized for active data mode....\n\r"); // Initialize device for active mode read of acclerometer, gyroscope, and temperature
        //MPU9150.initAK8975A(magCalibration);
        //pc.printf("AK8975 initialized for active data mode....\n\r"); // Initialize device for active mode read of magnetometer
    } else {
        //pc.printf("Could not connect to MPU9150: \n\r");
        //pc.printf("%#x \n",  whoami);
        while(1) ; // Loop forever if communication doesn't happen
    }

    //uint8_t MagRate = 10; // set magnetometer read rate in Hz; 10 to 100 (max) Hz are reasonable values
    MPU9150.getAres(); // Get accelerometer sensitivity
    MPU9150.getGres(); // Get gyro sensitivity

    myled= !myled;

    metrhsh.attach(&compute9150,0.02);

    while(1) {

        if(newData) {

            MPU9150.readAccelData(accelCount);  // Read the x/y/z adc values

            ax = (double)accelCount[0]*aRes; // - accelBias[0];  // get actual g value, this depends on scale being set
            ay = - (double)accelCount[1]*aRes; // - accelBias[1];
            az = (double)accelCount[2]*aRes; // - accelBias[2];//

            MPU9150.readGyroData(gyroCount);  // Read the x/y/z adc values

            // Calculate the gyro value into actual degrees per second
            gx = (double)gyroCount[0]*gRes; // - gyroBias[0];  // get actual gyro value, this depends on scale being set
            gy = (double)gyroCount[1]*gRes; // - gyroBias[1];
            gz = (double)gyroCount[2]*gRes; // - gyroBias[2];
            
            //arxikh timh epitaxynsewn
            if(prwtaGzAy) {
                gz0 = gz;
                ay0 = ay;
                prwtaGzAy = false;
            }
            
            //ypologismos gwnias
            if(gz>10||gz<-10) {

                sumgz += gz;

                gwnia = 0.02*((gz+gz0)/2 + (sumgz-gz-gz0));
                
                dphi = gwnia - prohgoumenhGwnia;
                
                prohgoumenhGwnia = gwnia;
                
                sumGwniaKyklou+=dphi;
                
            }
            
            //ypologismos taxythtas
            if(ay>0.05) {

                sumay += ay;

                vy = 0.02*((ay+ay0)/2 + (sumay-ay-ay0));
            }
            
            //arxikopoihsh taxythtas
            if(prwtoVy) {
                vy0 = vy;
                prwtoVy = false;
            }
            
            //ypologismos apostashs
            if(vy>0.1) {

                sumvy += vy;

                apostashY = 0.02*((vy+vy0)/2 + (sumvy-vy-vy0));
            }
            
            /*//kommati_elegxou gia ef8eia

                t2.start();

                switch(st) {
                    case k1:
                        CallForward();
                        if(UltraDistance<50) {
                            CallStop();
                            st=k3;
                            break;
                        }
                        if(t2.read()>31)st=k2;
                        break;
                    case  k2:
                        t2.stop();
                        t2.reset();
                        CallStop();
                        break;
                    case  k3:
                        CallStop();
                        if( UltraDistance>50) {
                            st=k1;
                            break;
                        }
                }//telos switch ef8eias */
            
                //*//kommati elegxou trigwnou

                t2.start();

                switch(st2) {
                    case l1:
                        CallForward();
                        if(UltraDistance<50) {
                            CallStop();
                            st2=l3;
                            break;
                        }
                        if(t2.read()>32.5)st2=l2;//28.5 gia 12
                        break;
                    case  l2:
                        t2.stop();
                        t2.reset();
                        CallStop();
                        if (waitStop){
                        wait(0.3);
                        waitStop=false;
                        }
                        CallRight();
                        if (gwnia>80) {
                            CallStop();
                            if (waitStop1){
                                wait(0.3);
                                waitStop1 = false;
                                }
                            st2 = l6;}
                            break;
                    case  l3:
                        if( UltraDistance>50) {
                            st2=l1;
                            break;
                        }
                    case  l6:
                        if(arxikopoihsh1){ena=0.65,enb=0.7,arxikopoihsh1=false;}
                        CallForward();
                        if(UltraDistance<50) {
                            CallStop();
                            st2=l7;
                            break;
                        }
                        if(t2.read()>7)st2=l10;
                        break;
                    case  l7:
                        if( UltraDistance>50) {
                            st2=l6;
                            break;
                        }
                    case  l10:
                        t2.stop();
                        t2.reset(); 
                        CallStop(); 
                        if (waitStop2) { 
                            wait(0.3);
                            waitStop2 = false;
                            }
                        CallRight();
                        if (gwnia>175) {
                            CallStop();
                            if (waitStop3) {
                                wait(0.3);
                                waitStop3 = false;
                                }
                            st2 = l11;
                        }
                        break;
                    case l11:
                        if(arxikopoihsh2){ena=0.65,enb=0.7,arxikopoihsh2=false;}
                        CallForward();
                        if(UltraDistance<50) {
                            CallStop();
                            st2=l12;
                            break;
                        }
                        if(t2.read()>29)st2=l15;
                        break;
                    case  l12:
                        if( UltraDistance>50) {
                            st2=l11;
                            break;
                        }
                    case  l15:
                        t2.stop();
                        t2.reset();
                        CallStop();
                        break;

                }
                //telos switch trigwnou*/
                
                 /*//kommati elegxou kyklou
                
                t2.start();

                switch(st3) {
                    case m1:
                        //if (arxikopoihsh3){ena=0.7,enb=0.7,gz=10;}
                        //arxikopoihsh3=false;
                        CallCircle();
                        if(UltraDistance<50) {
                            CallStop();
                            st3=m3;
                            break;
                        }
                        if(t2.read()>49)st3=m2;//stamathma me xrono
                        //if(sumGwniaKyklou>390)st3=m2;
                        break;
                    case  m2:
                        t2.stop();
                        t2.reset();
                        CallStop();
                        break;
                    case  m3:
                        CallStop();
                        if( UltraDistance>50) {
                            st3=m1;
                            break;
                        }
                        }
                        //telos switch kyklou*/
                
            newData=0;
        }//telos if newData
        
        //pc.printf(" gwnia = %f  deg\n\r", gwnia);
        //pc.printf(" gz = %f deg/s\n\r", gz);

    }//telos while1

}//telos main
