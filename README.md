Link Youtube = https://www.youtube.com/shorts/9n-iWHo9TWY



Repository: mnugrahasadewa/sistem-kendali-mekanika--self-balancing-robot
Files analyzed: 2

Estimated tokens: 1.2k

Directory structure:
└── mnugrahasadewa-sistem-kendali-mekanika--self-balancing-robot/
    ├── README.md
    └── SelfBalancingRobot.ino


================================================
FILE: README.md
================================================
Link Youtube = https://www.youtube.com/shorts/9n-iWHo9TWY



================================================
FILE: SelfBalancingRobot.ino
================================================
#include <PID_v1.h>
#include <LMotorController.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#define MIN_ABS_SPEED 20

MPU6050 mpu;

// Variable untuk sensor MPU
bool dmpReady = true; 
uint8_t mpuIntStatus; // holds status interrupt dari MPU
uint8_t devStatus; // return status setiap inialisasi ulang
uint16_t packetSize; 
uint16_t fifoCount; 
uint8_t fifoBuffer[64];

// orientation/motion vars
Quaternion q; // container quaternion [w, x, y, z] 
VectorFloat gravity; // Vektor gravity [x, y, z] 
float ypr[3]; // yaw/pitch/roll container

//PID
double originalSetpoint = 173;
double setpoint = originalSetpoint;
double movingAngleOffset = 0.1;
double input, output;

//Value PID
double Kp = 45;   
double Ki = 7 ;
double Kd = 2.5 ;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

double motorSpeedFactorLeft = 0.3;
double motorSpeedFactorRight = 0.3;
//Pin MOTOR Driver CONTROLLER
int ENA = 5;
int IN1 = 6;
int IN2 = 7;
int IN3 = 8;
int IN4 = 9;
int ENB = 10;
LMotorController motorController(ENA, IN1, IN2, ENB, IN3, IN4, motorSpeedFactorLeft, motorSpeedFactorRight);

volatile bool mpuInterrupt = false;
void dmpDataReady()
{
    mpuInterrupt = true;
}

void setup()
{
    Serial.begin(115200);

    // Masuk ke library I2C bus (karna library I2Cdev tidak otomatis terbuka)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    TWBR = 24; 
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
    #endif

    mpu.initialize();

    devStatus = mpu.dmpInitialize();

    // setting offset gyro 
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); 

    // returns 0 jika lancar berjalan
    if (devStatus == 0)
    {
        // menyalakan DMP, now that it's ready
        mpu.setDMPEnabled(true);

        // enable interrupt detection Arduino
        attachInterrupt(digitalPinToInterrupt(2), dmpDataReady, RISING); //pin dipakai untuk interrupt
        mpuIntStatus = mpu.getIntStatus();

        dmpReady = true;

        packetSize = mpu.dmpGetFIFOPacketSize();

        //setup PID
        pid.SetMode(AUTOMATIC);
        pid.SetSampleTime(10);
        pid.SetOutputLimits(-255, 255); 
    }
    else
    {
        // status jika terjadi error
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
        while (1);
    }
}

void loop()
{
    // jika gagal akan diam
    if (!dmpReady) return;

    // menunggu interrupt MPU jika ada extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize)
    {
    
    }

    // reset interrupt flag
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // mengambil current count FIFO
    fifoCount = mpu.getFIFOCount();

    // check overflow code
    if ((mpuIntStatus & 0x10) || fifoCount == 1024)
    {
        // reset mpu jika terjadi kesalahan
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
    }
    // jika else maka, check data DMP ready interrupt
    else if (mpuIntStatus & 0x02)
    {
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // baca packet dari FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);

        // track count FIFO
        fifoCount -= packetSize;

        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        input = ypr[1] * 180/M_PI + 180;

        if (abs(input - setpoint) < 0.5) {
            output = 0;
        } else {
            pid.Compute();
            motorController.move(output, MIN_ABS_SPEED);
        }

    }
}
