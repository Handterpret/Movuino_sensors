#include "Arduino.h"
#include "MPU6050_functions.h"

//  INTERRUPT DETECTION ROUTINE

//mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high


// MPU control/status vars
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container

//config and interruption boolean
bool dmpReady = false;
volatile bool mpuInterrupt = false;

void dmpDataReady() {
    mpuInterrupt = true;
}

MPU6050 mpu_init(MPU6050 mpu) {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
    // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    
    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
    Serial.print("Dev status : ");Serial.println(devStatus);
    // supply your own gyro offsets here, scaled for min sensitivity
      /*mpu.setXGyroOffset(220);
      mpu.setYGyroOffset(76);
      mpu.setZGyroOffset(-85);
      mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
      */
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
    return mpu;
}

float * get_mpu_data(MPU6050 mpu) {
    if (check_packet(mpu)) {
        return get_metrics(mpu);
    }
    else return NULL;
}
bool check_packet(MPU6050 mpu) {
// if programming failed, don't try to do anything
    if (!dmpReady) {
      Serial.println("Error throughout MPU6250 setup");
      exit(0);
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();
	if(fifoCount < packetSize){
	        //Lets go back and wait for another interrupt. We shouldn't be here, we got an interrupt from another event
			// This is blocking so don't do it
        return false;
	}
    // check for overflow (this should never happen unless our code is too inefficient)
    else if ((mpuIntStatus & (0x01 << MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
      //  fifoCount = mpu.getFIFOCount();  // will be zero after reset no need to ask
        Serial.println(F("FIFO overflow!"));
        return false;
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & (0x01 << MPU6050_INTERRUPT_DMP_INT_BIT)) {
        return true;
    }
}
float * get_metrics(MPU6050 mpu) {
    // read a packet from FIFO
	while(fifoCount >= packetSize){ // Lets catch up to NOW, someone is using the dreaded delay()!
		mpu.getFIFOBytes(fifoBuffer, packetSize);
		// track FIFO count here in case there is > 1 packet available
		// (this lets us immediately read more without waiting for an interrupt)
		fifoCount -= packetSize;
	}
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    // get values without gravity influence
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    //get values in degrees
    mpu.dmpGetEuler(euler, &q);
    static float metrics [6];
    metrics[0] = aaReal.x;
    metrics[1] = aaReal.y;
    metrics[2] = aaReal.z;
    metrics[3] = euler[0] * 180/M_PI;
    metrics[4] = euler[1] * 180/M_PI;
    metrics[5] = euler[2] * 180/M_PI;
    /*
    Serial.print("areal\t");
    Serial.print(aaReal.x);
    Serial.print("\t");
    Serial.print(aaReal.y);
    Serial.print("\t");
    Serial.println(aaReal.z);

    Serial.print("euler\t");
    Serial.print(euler[0] * 180/M_PI);
    Serial.print("\t");
    Serial.print(euler[1] * 180/M_PI);
    Serial.print("\t");
    Serial.println(euler[2] * 180/M_PI);*/
    return metrics;
}
