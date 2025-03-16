#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/quaternion.h>

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  #include "Wire.h"
#endif

// ----- MPU6050 Definitions -----
MPU6050 mpu;
#define INTERRUPT_PIN 2   // for ESP32, choose an available interrupt-capable pin
#define LED_PIN 13        // onboard LED for activity indication
volatile bool mpuInterrupt = false;
bool dmpReady = false;
uint8_t devStatus;
uint16_t packetSize;
uint8_t fifoBuffer[64];
Quaternion q_imu;         // container for the quaternion from the MPU6050

// Interrupt callback: set flag when data is ready
void dmpDataReady() {
  mpuInterrupt = true;
}

// ----- micro-ROS Objects -----
rcl_publisher_t publisher;
geometry_msgs__msg__Quaternion q_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
// We're not using a timer callback here, so we omit the timer object.

size_t domain_id = 5; // Set your desired ROS_DOMAIN_ID here

// Macros to check return values
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ error_loop(); } }
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){} }

// Error loop (halts execution)
void error_loop() {
  while (1) {
    delay(100);
  }
}

// Initialize micro-ROS with ROS_DOMAIN_ID support
void setup_micro_ros() {
  // Set up the serial transports (Serial must already be initialized)
  set_microros_serial_transports(Serial);

  allocator = rcl_get_default_allocator();

  // --- Domain ID Initialization ---
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, allocator));   // Initialize options with the allocator
  RCCHECK(rcl_init_options_set_domain_id(&init_options, domain_id)); // Set the desired domain

  // Initialize support using the options we configured
  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

  // --- Create Node and Publisher ---
  RCCHECK(rclc_node_init_default(&node, "imu_quaternion_node", "", &support));

  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Quaternion),
    "imu_quaternion"
  ));

  // Create the executor (we're not adding a timer here because we'll poll in loop)
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  // (Optional: you could add a timer if you need periodic tasks)
}

void setup() {
  // --- MPU6050 Setup ---
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000);
  #endif

  Serial.begin(115200);  // Initialize Serial once with the desired baud rate for debugging and micro-ROS transport.
  while (!Serial);
  Serial.println("Initializing MPU6050...");
  
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  Serial.println("Testing MPU6050 connection...");
  Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  Serial.println("Initializing DMP...");
  devStatus = mpu.dmpInitialize();
  
  // Supply your own gyro offsets (adjust based on your calibration)
  mpu.setXGyroOffset(12);
  mpu.setYGyroOffset(52);
  mpu.setZGyroOffset(2);
  mpu.setZAccelOffset(1804);

  if (devStatus == 0) {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
    Serial.println("DMP ready!");
  } else {
    Serial.print("DMP Initialization failed (code ");
    Serial.print(devStatus);
    Serial.println(")");
    while (1);
  }

  pinMode(LED_PIN, OUTPUT);

  // --- micro-ROS Setup ---
  setup_micro_ros();
}

void loop() {
  // Spin the micro-ROS executor briefly to process any callbacks
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));

  // Process MPU6050 data and publish if available:
  if (dmpReady && mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpuInterrupt = false;
    mpu.dmpGetQuaternion(&q_imu, fifoBuffer);
    q_msg.w = q_imu.w;
    q_msg.x = q_imu.x;
    q_msg.y = q_imu.y;
    q_msg.z = q_imu.z;
    RCSOFTCHECK(rcl_publish(&publisher, &q_msg, NULL));
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  }
}


// // IMU position and orientation estimation
// #include <micro_ros_platformio.h>
// #include <rcl/rcl.h>
// #include <rclc/rclc.h>
// #include <rclc/executor.h>
// #include <std_msgs/msg/int32.h>

// #include "I2Cdev.h"
// #include "MPU6050_6Axis_MotionApps20.h"

// #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
//     #include "Wire.h"
// #endif

// MPU6050 mpu;

// // uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// // quaternion components in a [w, x, y, z] format (not best for parsing
// // on a remote host such as Processing or something though)
// #define OUTPUT_READABLE_QUATERNION

// // uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// // (in degrees) calculated from the quaternions coming from the FIFO.
// // Note that Euler angles suffer from gimbal lock (for more info, see
// // http://en.wikipedia.org/wiki/Gimbal_lock)
// // #define OUTPUT_READABLE_EULER

// // uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// // pitch/roll angles (in degrees) calculated from the quaternions coming
// // from the FIFO. Note this also requires gravity vector calculations.
// // Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// // more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
// // #define OUTPUT_READABLE_YAWPITCHROLL

// // uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// // components with gravity removed. This acceleration reference frame is
// // not compensated for orientation, so +X is always +X according to the
// // sensor, just without the effects of gravity. If you want acceleration
// // compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
// // #define OUTPUT_READABLE_REALACCEL

// // uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// // components with gravity removed and adjusted for the world frame of
// // reference (yaw is relative to initial orientation, since no magnetometer
// // is present in this case). Could be quite handy in some cases.
// // #define OUTPUT_READABLE_WORLDACCEL

// // uncomment "OUTPUT_TEAPOT" if you want output that matches the
// // format used for the InvenSense teapot demo
// //#define OUTPUT_TEAPOT



// #define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
// #define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
// bool blinkState = false;

// // MPU control/status vars
// bool dmpReady = false;  // set true if DMP init was successful
// uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
// uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
// uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
// uint16_t fifoCount;     // count of all bytes currently in FIFO
// uint8_t fifoBuffer[64]; // FIFO storage buffer

// // orientation/motion vars
// Quaternion q;           // [w, x, y, z]         quaternion container
// VectorInt16 aa;         // [x, y, z]            accel sensor measurements
// VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
// VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
// VectorFloat gravity;    // [x, y, z]            gravity vector
// float euler[3];         // [psi, theta, phi]    Euler angle container
// float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// // packet structure for InvenSense teapot demo
// uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };



// // ================================================================
// // ===               INTERRUPT DETECTION ROUTINE                ===
// // ================================================================

// volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
// void dmpDataReady() {
//     mpuInterrupt = true;
// }



// // ================================================================
// // ===                      INITIAL SETUP                       ===
// // ================================================================

// void setup() {
//     // join I2C bus (I2Cdev library doesn't do this automatically)
//     #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
//         Wire.begin();
//         Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
//     #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
//         Fastwire::setup(400, true);
//     #endif

//     // initialize serial communication
//     // (115200 chosen because it is required for Teapot Demo output, but it's
//     // really up to you depending on your project)

//     // Serial.begin(57600); // Arduino uno
//     Serial.begin(9600); // esp32
//     while (!Serial); // wait for Leonardo enumeration, others continue immediately

//     // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
//     // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
//     // the baud timing being too misaligned with processor ticks. You must use
//     // 38400 or slower in these cases, or use some kind of external separate
//     // crystal solution for the UART timer.

//     // initialize device
//     Serial.println(F("Initializing I2C devices..."));
//     mpu.initialize();
//     pinMode(INTERRUPT_PIN, INPUT);

//     // verify connection
//     Serial.println(F("Testing device connections..."));
//     Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

//     // wait for ready
//     Serial.println(F("\nSend any character to begin DMP programming and demo: "));
//     while (Serial.available() && Serial.read()); // empty buffer
//     while (!Serial.available());                 // wait for data
//     while (Serial.available() && Serial.read()); // empty buffer again

//     // load and configure the DMP
//     Serial.println(F("Initializing DMP..."));
//     devStatus = mpu.dmpInitialize();

//     // supply your own gyro offsets here, scaled for min sensitivity
//     mpu.setXGyroOffset(12.00000);
//     mpu.setYGyroOffset(52.00000);
//     mpu.setZGyroOffset(2.00000);
//     mpu.setZAccelOffset(1804.00000); // 1688 factory default for my test chip

//     // make sure it worked (returns 0 if so)
//     if (devStatus == 0) {
//         // Calibration Time: generate offsets and calibrate our MPU6050
//         mpu.CalibrateAccel(6);
//         mpu.CalibrateGyro(6);
//         mpu.PrintActiveOffsets();
//         // turn on the DMP, now that it's ready
//         Serial.println(F("Enabling DMP..."));
//         mpu.setDMPEnabled(true);

//         // enable Arduino interrupt detection
//         Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
//         Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
//         Serial.println(F(")..."));
//         attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
//         mpuIntStatus = mpu.getIntStatus();

//         // set our DMP Ready flag so the main loop() function knows it's okay to use it
//         Serial.println(F("DMP ready! Waiting for first interrupt..."));
//         dmpReady = true;

//         // get expected DMP packet size for later comparison
//         packetSize = mpu.dmpGetFIFOPacketSize();
//     } else {
//         // ERROR!
//         // 1 = initial memory load failed
//         // 2 = DMP configuration updates failed
//         // (if it's going to break, usually the code will be 1)
//         Serial.print(F("DMP Initialization failed (code "));
//         Serial.print(devStatus);
//         Serial.println(F(")"));
//     }

//     // configure LED for output
//     pinMode(LED_PIN, OUTPUT);
// }



// // ================================================================
// // ===                    MAIN PROGRAM LOOP                     ===
// // ================================================================

// void loop() {
//     // if programming failed, don't try to do anything
//     if (!dmpReady) return;
//     // read a packet from FIFO
//     if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
//         #ifdef OUTPUT_READABLE_QUATERNION
//             // display quaternion values in easy matrix form: w x y z
//             mpu.dmpGetQuaternion(&q, fifoBuffer);
//             Serial.print("quat\t");
//             Serial.print(q.w);
//             Serial.print("\t");
//             Serial.print(q.x);
//             Serial.print("\t");
//             Serial.print(q.y);
//             Serial.print("\t");
//             Serial.println(q.z);
//             delay(33);
//         #endif

//         #ifdef OUTPUT_READABLE_EULER
//             // display Euler angles in degrees
//             mpu.dmpGetQuaternion(&q, fifoBuffer);
//             mpu.dmpGetEuler(euler, &q);
//             Serial.print("euler\t");
//             Serial.print(euler[0] * 180/M_PI);
//             Serial.print("\t");
//             Serial.print(euler[1] * 180/M_PI);
//             Serial.print("\t");
//             Serial.println(euler[2] * 180/M_PI);
//             delay(33);
//         #endif

//         #ifdef OUTPUT_READABLE_YAWPITCHROLL
//             // display Euler angles in degrees
//             mpu.dmpGetQuaternion(&q, fifoBuffer);
//             mpu.dmpGetGravity(&gravity, &q);
//             mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
//             Serial.print("ypr\t");
//             Serial.print(ypr[0] * 180/M_PI);
//             Serial.print("\t");
//             Serial.print(ypr[1] * 180/M_PI);
//             Serial.print("\t");
//             Serial.println(ypr[2] * 180/M_PI);
//             delay(33);
//         #endif

//         #ifdef OUTPUT_READABLE_REALACCEL
//             // display real acceleration, adjusted to remove gravity
//             mpu.dmpGetQuaternion(&q, fifoBuffer);
//             mpu.dmpGetAccel(&aa, fifoBuffer);
//             mpu.dmpGetGravity(&gravity, &q);
//             mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            
//             // Serial.print("areal\t");
//             // Serial.print(aaReal.x);
//             // Serial.print("\t");
//             // Serial.print(aaReal.y);
//             // Serial.print("\t");
//             // Serial.println(aaReal.z);

//             // Serial.print("g [normalised]\t");
//             // Serial.print(gravity.x);
//             // Serial.print("\t");
//             // Serial.print(gravity.y);
//             // Serial.print("\t");
//             // Serial.println(gravity.z);

//             float g = 9.80665;
//             float a_s_real[3];
//             a_s_real[0] = (float) aaReal.x * g / 8192;
//             a_s_real[1] = (float) aaReal.y * g / 8192;
//             a_s_real[2] = (float) aaReal.z * g / 8192;

//             Serial.print("areal\t");
//             Serial.print(a_s_real[0]);
//             Serial.print("\t");
//             Serial.print(a_s_real[1]);
//             Serial.print("\t");
//             Serial.println(a_s_real[2]);

//             delay(33);
//         #endif

//         #ifdef OUTPUT_READABLE_WORLDACCEL
//             // display initial world-frame acceleration, adjusted to remove gravity
//             // and rotated based on known orientation from quaternion
//             mpu.dmpGetQuaternion(&q, fifoBuffer);
//             mpu.dmpGetAccel(&aa, fifoBuffer);
//             mpu.dmpGetGravity(&gravity, &q);
//             mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
//             mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
//             Serial.print("aworld\t");
//             Serial.print(aaWorld.x);
//             Serial.print("\t");
//             Serial.print(aaWorld.y);
//             Serial.print("\t");
//             Serial.println(aaWorld.z);
//             delay(33);
//         #endif
    
//         #ifdef OUTPUT_TEAPOT
//             // display quaternion values in InvenSense Teapot demo format:
//             teapotPacket[2] = fifoBuffer[0];
//             teapotPacket[3] = fifoBuffer[1];
//             teapotPacket[4] = fifoBuffer[4];
//             teapotPacket[5] = fifoBuffer[5];
//             teapotPacket[6] = fifoBuffer[8];
//             teapotPacket[7] = fifoBuffer[9];
//             teapotPacket[8] = fifoBuffer[12];
//             teapotPacket[9] = fifoBuffer[13];
//             Serial.write(teapotPacket, 14);
//             teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
//         #endif

//         // blink LED to indicate activity
//         blinkState = !blinkState;
//         digitalWrite(LED_PIN, blinkState);
//     }
// }





// // // Old
// // /*
// // The contents of this code and instructions are the intellectual property of Carbon Aeronautics. 
// // The text and figures in this code and instructions are licensed under a Creative Commons Attribution - Noncommercial - ShareAlike 4.0 International Public Licence. 
// // This license lets you remix, adapt, and build upon your work non-commercially, as long as you credit Carbon Aeronautics 
// // (but not in any way that suggests that we endorse you or your use of the work) and license your new creations under the identical terms.
// // This code and instruction is provided "As Is” without any further warranty. Neither Carbon Aeronautics or the author has any liability to any person or entity 
// // with respect to any loss or damage caused or declared to be caused directly or indirectly by the instructions contained in this code or by 
// // the software and hardware described in it. As Carbon Aeronautics has no control over the use, setup, assembly, modification or misuse of the hardware, 
// // software and information described in this manual, no liability shall be assumed nor accepted for any resulting damage or injury. 
// // By the act of copying, use, setup or assembly, the user accepts all resulting liability.

// // 1.0  29 December 2022 -  initial release
// // */
// // #include <Arduino.h>
// // #include <Wire.h>
// // #include <BasicLinearAlgebra.h>

// // using namespace BLA;

// // template <int Dim, typename DType = float>
// // struct DiagonalMatrix : public MatrixBase<DiagonalMatrix<Dim>, Dim, Dim, DType>
// // {
// //     Matrix<Dim, 1, DType> diagonal;

// //     // For const matrices (ones whose elements can't be modified) you just need to implement this function:
// //     DType operator()(int row, int col) const
// //     {
// //         // If it's on the diagonal and it's not larger than the matrix dimensions then return the element
// //         if (row == col)
// //             return diagonal(row);
// //         else
// //             // Otherwise return zero
// //             return 0.0f;
// //     }

// //     // If you want to declare a matrix whose elements can be modified then you'll need to define this function:
// //     // DType& operator()(int row, int col)
// // };

// // float d2r = 3.14159265359/180.0, r2d = 180.0/3.14159265359;
// // float m2mm = 1000.0, mm2m = 0.001;
// // float RateRoll, RatePitch, RateYaw;
// // float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
// // int RateCalibrationNumber;
// // int RateCalibrationTotalNumber = 1000;
// // float g = 9.80665;
// // float AccX, AccY, AccZ;
// // float AccxOffset = 0.0, AccyOffset = 0.0, AcczOffset = 0.0;
// // float AccXCompensated = 0.0, AccYCompensated = 0.0, AccZCompensated = 0.0;
// // float PosX = 0.0, PosY = 0.0, PosZ = 0.0;
// // float VelX = 0.0, VelY = 0.0, VelZ = 0.0;
// // float AngleRoll, AnglePitch;
// // float AngleRollIntegration = 0.0, AnglePitchIntegration = 0.0, AngleYawIntegration = 0.0;
// // uint32_t LoopTimer;
// // float Ts = 0.004; // [s]
// // uint32_t Ts_ms = (int) (Ts * 1000); // [ms]
// // uint32_t Ts_us = (int) (Ts * 1000000); // [us]
// // float PNstd = 4.0 * pow(10.0, 0.0);
// // float MNstd = 2.5 * pow(10.0, -1.0);
// // float KalmanAngleRoll=0, KalmanUncertaintyAngleRoll=2*2;
// // float KalmanAnglePitch=0, KalmanUncertaintyAnglePitch=2*2;
// // float Kalman1DOutput[]={0,0};

// // void imu_signals(void) {
// //   Wire.beginTransmission(0x68);
// //   Wire.write(0x1A);
// //   Wire.write(0x05);
// //   Wire.endTransmission();
// //   Wire.beginTransmission(0x68);
// //   Wire.write(0x1C);
// //   Wire.write(0x10);
// //   Wire.endTransmission();
// //   Wire.beginTransmission(0x68);
// //   Wire.write(0x3B);
// //   Wire.endTransmission(); 
// //   Wire.requestFrom(0x68,6);
// //   int16_t AccXLSB = Wire.read() << 8 | Wire.read();
// //   int16_t AccYLSB = Wire.read() << 8 | Wire.read();
// //   int16_t AccZLSB = Wire.read() << 8 | Wire.read();
// //   Wire.beginTransmission(0x68);
// //   Wire.write(0x1B); 
// //   Wire.write(0x8);
// //   Wire.endTransmission();     
// //   Wire.beginTransmission(0x68);
// //   Wire.write(0x43);
// //   Wire.endTransmission();
// //   Wire.requestFrom(0x68,6);

// //   int16_t GyroX=Wire.read()<<8 | Wire.read();
// //   int16_t GyroY=Wire.read()<<8 | Wire.read();
// //   int16_t GyroZ=Wire.read()<<8 | Wire.read();

// //   // RateRoll = (float) GyroX * (2.0 / 65.5) * d2r;
// //   // RatePitch = (float) GyroY * (2.0 / 65.5) * d2r;
// //   // RateYaw = (float) GyroZ * (2.0 / 65.5) * d2r;

// //   RateRoll = (float) GyroX * (2.0 / 65.5) * d2r;
// //   RatePitch = (float) GyroY * (2.0 / 65.5) * d2r;
// //   RateYaw = (float) GyroZ * (2.0 / 65.5) * d2r;

// //   // AccxOffset = -0.02;
// //   // AccyOffset = 0.01;
// //   // AcczOffset = 0.02;
// //   // AccX = (float) AccXLSB/4096 + AccxOffset;
// //   // AccY = (float) AccYLSB/4096 + AccyOffset;
// //   // AccZ = (float) AccZLSB/4096 + AcczOffset;
// //   // AccX *= g;
// //   // AccY *= g;
// //   // AccZ *= g;


// //   AccxOffset = -0.13665;
// //   AccyOffset = 0.08335;
// //   AcczOffset = 0.23665;
// //   AccX = (float) AccXLSB * g / 4096 + AccxOffset;
// //   AccY = (float) AccYLSB * g / 4096 + AccyOffset;
// //   AccZ = (float) AccZLSB * g / 4096 + AcczOffset;

// //   AngleRoll = atan(AccY/sqrt(AccX*AccX + AccZ*AccZ));
// //   AnglePitch = -atan(AccX/sqrt(AccY*AccY + AccZ*AccZ));
// // }

// // void remove_omega_bias() {
// //   RateRoll -= RateCalibrationRoll;
// //   RatePitch -= RateCalibrationPitch;
// //   RateYaw -= RateCalibrationYaw;
// // }

// // void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
// //   KalmanState = KalmanState + Ts*KalmanInput;
// //   KalmanUncertainty = KalmanUncertainty + Ts*Ts * PNstd*PNstd;
// //   float KalmanGain = KalmanUncertainty * 1/(1*KalmanUncertainty + MNstd*MNstd);
// //   KalmanState = KalmanState + KalmanGain * (KalmanMeasurement-KalmanState);
// //   KalmanUncertainty = (1-KalmanGain) * KalmanUncertainty;
// //   Kalman1DOutput[0] = KalmanState; 
// //   Kalman1DOutput[1] = KalmanUncertainty;
// // }

// // void compute_thx(float wx) {
// //   AngleRollIntegration += Ts * wx;
// // }

// // void compute_thy(float wy) {
// //   AnglePitchIntegration += Ts * wy;
// // }

// // void compute_thz(float wz) {
// //   AngleYawIntegration += Ts * wz;
// // }

// // void compute_px_vx(float a) {
// //   VelX += Ts * a;
// //   PosX += Ts * VelX + 0.5 * Ts * Ts * a;
// // }

// // void compute_py_vy(float a) {
// //   VelY += Ts * a;
// //   PosY += Ts * VelY + 0.5 * Ts * Ts * a;
// // }

// // void compute_pz_vz(float a) {
// //   VelZ += Ts * a;
// //   PosZ += Ts * VelZ + 0.5 * Ts * Ts * a;
// // }

// // void print_results(float valueX, float valueY, float valueZ) {
// //   Serial.print(" = [");
// //   Serial.print(valueX);
// //   Serial.print(", ");
// //   Serial.print(valueY);
// //   Serial.print(", ");
// //   Serial.print(valueZ);
// //   Serial.print("]; ");
// // }


// // // Rotation about Z axis
// // BLA::Matrix<3, 3> Rotz(float thz) {
// //   BLA::Matrix<3, 3> mat;
// //   // [ cos(thz)  -sin(thz)   0 ]
// //   // [ sin(thz)   cos(thz)   0 ]
// //   // [    0          0       1 ]
// //   mat(0,0) = cos(thz);
// //   mat(0,1) = -sin(thz);
// //   mat(0,2) = 0;
// //   mat(1,0) = sin(thz);
// //   mat(1,1) = cos(thz);
// //   mat(1,2) = 0;
// //   mat(2,0) = 0;
// //   mat(2,1) = 0;
// //   mat(2,2) = 1;
// //   return mat;
// // }

// // // Rotation about Y axis
// // BLA::Matrix<3, 3> Roty(float thy) {
// //   BLA::Matrix<3, 3> mat;
// //   // [  cos(thy)   0   sin(thy) ]
// //   // [     0       1      0     ]
// //   // [ -sin(thy)   0   cos(thy) ]
// //   mat(0,0) = cos(thy);
// //   mat(0,1) = 0;
// //   mat(0,2) = sin(thy);
// //   mat(1,0) = 0;
// //   mat(1,1) = 1;
// //   mat(1,2) = 0;
// //   mat(2,0) = -sin(thy);
// //   mat(2,1) = 0;
// //   mat(2,2) = cos(thy);
// //   return mat;
// // }

// // // Rotation about X axis
// // BLA::Matrix<3, 3> Rotx(float thx) {
// //   BLA::Matrix<3, 3> mat;
// //   // [ 1      0         0      ]
// //   // [ 0   cos(thx)  -sin(thx) ]
// //   // [ 0   sin(thx)   cos(thx) ]
// //   mat(0,0) = 1;
// //   mat(0,1) = 0;
// //   mat(0,2) = 0;
// //   mat(1,0) = 0;
// //   mat(1,1) = cos(thx);
// //   mat(1,2) = -sin(thx);
// //   mat(2,0) = 0;
// //   mat(2,1) = sin(thx);
// //   mat(2,2) = cos(thx);
// //   return mat;
// // }

// // // Rotation from ZYX Euler angles
// // BLA::Matrix<3, 3> Rotzyx(float thz, float thy, float thx) {
// //   return Rotz(thz) * Roty(thy) * Rotx(thx);
// // }


// // void setup() {
// //   Serial.begin(57600);
// //   pinMode(13, OUTPUT);
// //   digitalWrite(13, HIGH);
// //   Wire.setClock(400000);
// //   Wire.begin();
// //   delay(250);
// //   Wire.beginTransmission(0x68); 
// //   Wire.write(0x6B);
// //   Wire.write(0x00);
// //   Wire.endTransmission();
// //   for (RateCalibrationNumber = 0; RateCalibrationNumber < RateCalibrationTotalNumber; RateCalibrationNumber ++) {
// //     imu_signals();
// //     RateCalibrationRoll += RateRoll;
// //     RateCalibrationPitch += RatePitch;
// //     RateCalibrationYaw += RateYaw;
// //     Serial.print("[MPU6050 Gyro Calibration] ");
// //     Serial.print( (float) RateCalibrationNumber / RateCalibrationTotalNumber * 100);
// //     Serial.println("%");
// //     delay(1);
// //   }
// //   RateCalibrationRoll /= RateCalibrationTotalNumber;
// //   RateCalibrationPitch /= RateCalibrationTotalNumber;
// //   RateCalibrationYaw /= RateCalibrationTotalNumber;
// //   LoopTimer=micros();
// // }

// // void loop() {
// //   // Get IMU signals
// //   imu_signals();
// //   remove_omega_bias();

// //   // Orientation estimation
// //   kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
// //   KalmanAngleRoll=Kalman1DOutput[0]; 
// //   KalmanUncertaintyAngleRoll=Kalman1DOutput[1];

// //   kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
// //   KalmanAnglePitch=Kalman1DOutput[0]; 
// //   KalmanUncertaintyAnglePitch=Kalman1DOutput[1];

// //   compute_thx(RateRoll);
// //   compute_thy(RatePitch);
// //   compute_thz(RateYaw);

// //   // Position estimation
// //   // Acc requires orientation estimation to do gravity compensation!!! (e.g. use rotation matrix)
// //   // Calculate the gravity vector in the sensor frame using rotation matrix from ZYX Euler angles
// //   BLA::Matrix<3, 3> R_w_s = Rotzyx(AngleYawIntegration, KalmanAnglePitch, KalmanAngleRoll);
// //   BLA::Matrix<3, 1> g_w(0, 0, -g);
// //   BLA::Matrix<3, 1> g_s = (~R_w_s) * g_w; // transpose of R_w_s
// //   BLA::Matrix<3, 1> a_s(AccX, AccY, AccZ);
// //   BLA::Matrix<3, 1> a_s_compensated = a_s + g_s;

// //   compute_px_vx(a_s_compensated(0));
// //   compute_py_vy(a_s_compensated(1));
// //   compute_pz_vz(a_s_compensated(2));

// //   // Print results
// //   // Serial.print("[MPU6050 position and orientation estimation] ");

// //   // Serial.print("a_s [m/s^2]");
// //   // print_results(a_s(0), a_s(1), a_s(2));

// //   // Serial.print("g_s [m/s^2]");
// //   // print_results(g_s(0), g_s(1), g_s(2));

// //   // Serial.print("a_s_compensated [m/s^2]");
// //   // print_results(a_s_compensated(0), a_s_compensated(1), a_s_compensated(2));

// //   // Serial.print("Vel [mm/s] = [");
// //   // print_results(VelX * m2mm, VelY * m2mm, VelZ * m2mm);

// //   // Serial.print("Pos [mm] = [");
// //   // print_results(PosX * m2mm, PosY * m2mm, PosZ * m2mm);

// //   // Serial.print("Omega [°/s]");
// //   // print_results(RateRoll, RatePitch, RateYaw);

// //   Serial.print("Orientation [°]");
// //   print_results(KalmanAngleRoll * r2d, KalmanAnglePitch * r2d, AngleYawIntegration * r2d);
// //   // print_results(AngleRollIntegration * r2d, AnglePitchIntegration * r2d, AngleYawIntegration * r2d);

// //   Serial.println("");

// //   while (micros() - LoopTimer < Ts_us);
// //   LoopTimer=micros();
// // }