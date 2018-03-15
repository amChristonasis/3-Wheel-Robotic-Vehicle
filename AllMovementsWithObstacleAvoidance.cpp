#include "mbed.h"
#include "MPU9150.h"
#include "math.h"
//wait ths switch
bool waitStop = true, waitStop1 = true, waitStop2 = true, waitStop3 = true;
double sumGwniaKyklou;
bool arxikopoihsh=false, diplhEpistrofh=false;
int pollaplhEpistrofh=1;

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
//empros
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
    
    //arxikopoihsh=false;
}

//mprosta gia mikrh apostash
void CallForwardSmall()
{
    In1=1;
    In2=0;
    ena = ena - gz*0.002;
    EnA=ena;
                        
                        
    In3=1;
    In4=0;
    enb = enb + gz*0.002;
    EnB=enb;
}

//kyklos
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
    EnA=0.25;

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
    EnB=0.3;
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

    enum STATE {k1,k2,k3,k4,k5,k6,k7,k8,k9};
    STATE st=k1;

    Timer t12m;
    Timer tApofyghs;

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
            
            //*//algori8mos g ef8eia me empodia

                switch(st) {
                    case k1:
                        if(waitStop3)wait(0.1),waitStop3=false;
                        t12m.start();
                        CallForwardSmall();
                        if(UltraDistance<50) {
                            CallStop();
                            t12m.stop();
                            st=k3;
                            break;
                        }
                        if(t12m.read()>30)st=k2;
                        break;
                    case  k2:
                        t12m.stop();
                        t12m.reset();
                        CallStop();
                        break;
                    case  k3:
                        if(waitStop){
                            wait(0.1);
                            waitStop=false;
                            }
                        CallRight();
                        if(gwnia>88){
                            CallStop();
                            st=k4;
                            break;
                            }
                        break;
                    case k4:
                        if(waitStop1)wait(0.1),waitStop1=false;
                        tApofyghs.start();
                        CallForwardSmall();
                        if(tApofyghs.read()>2)st=k5;
                        break;
                    case k5:
                        CallStop();
                        CallLeft();
                        if(gwnia<0){
                            CallStop();
                            tApofyghs.stop();
                            tApofyghs.reset();
                            st=k6;
                            break;
                            }
                        break;
                    case k6:
                        //arxikopoihsh=true;
                        if(waitStop2)wait(0.1),waitStop2=false;
                        tApofyghs.start();
                        t12m.start();
                        CallForwardSmall();
                        if(UltraDistance<50) t12m.stop(), pollaplhEpistrofh+=1, tApofyghs.stop(), tApofyghs.reset(), st=k3;
                        if(tApofyghs.read()>2) t12m.stop(), st=k7;
                        break;  
                    case k7:
                        CallStop();
                        CallLeft();
                        if(gwnia<-90){
                            CallStop();
                            tApofyghs.stop();
                            tApofyghs.reset();
                            st=k8;
                            break;
                            }
                        break;    
                    case k8:
                        CallStop();
                        if(waitStop3)wait(0.1),waitStop3=false;
                            tApofyghs.start();
                            CallForwardSmall();
                            if(tApofyghs.read()>pollaplhEpistrofh*2)tApofyghs.stop(),tApofyghs.reset(),st=k9;
                        break;              
                    case k9:
                        pollaplhEpistrofh=1;
                        CallStop();
                        CallRight();
                        if(gwnia>-5){
                            CallStop();
                            //arxikopoihsh=true;
                            waitStop3=true;
                            st=k1;
                            break;
                            }
                        break;
                }//telos switch algori8mou*/
                
            newData=0;
        }//telos if newData
        
        //pc.printf(" gwnia = %f  deg\n\r", gwnia);
        //pc.printf(" gz = %f deg/s\n\r", gz);

    }//telos while1

}//telos main
