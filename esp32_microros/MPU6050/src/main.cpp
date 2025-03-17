// quat, acc
#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/bool.h>
#include <geometry_msgs/msg/quaternion.h>
#include <geometry_msgs/msg/vector3.h>

// MPU6050 libraries
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

// Variables for acceleration calculations:
VectorInt16 aa;         // raw accelerometer values from FIFO
VectorInt16 aaReal;     // linear acceleration (gravity removed) in sensor frame
VectorInt16 aaWorld;    // linear acceleration in world frame
VectorFloat gravity;    // gravity vector

// Interrupt callback: set flag when new data is ready
void dmpDataReady() {
  mpuInterrupt = true;
}

// ----- micro-ROS Objects -----
rcl_publisher_t com_pub_;
rcl_publisher_t quat_pub_;
rcl_publisher_t acc_pub_;

std_msgs__msg__Bool com_msg;
geometry_msgs__msg__Quaternion q_msg;
geometry_msgs__msg__Vector3 acc_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

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

void imu_calibration() {
  // Supply your own IMU offsets (adjust based on your calibration)
  mpu.setXGyroOffset(12);
  mpu.setYGyroOffset(52);
  mpu.setZGyroOffset(2);
  mpu.setZAccelOffset(1804);
}

// Initialize micro-ROS with ROS_DOMAIN_ID support
void setup_micro_ros() {
  // Set up the serial transports (Serial must already be initialized)
  set_microros_serial_transports(Serial);

  allocator = rcl_get_default_allocator();

  // --- Domain ID Initialization ---
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, allocator));
  RCCHECK(rcl_init_options_set_domain_id(&init_options, domain_id));

  // Initialize support using the options we configured
  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

  // --- Create Node and Publishers ---
  RCCHECK(rclc_node_init_default(&node, "mpu6050_imu", "", &support));

  RCCHECK(rclc_publisher_init_default(
      &com_pub_,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
      "/mpu6050_imu/communication_signal"
  ));

  RCCHECK(rclc_publisher_init_default(
    &quat_pub_,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Quaternion),
    "/mpu6050_imu/quat"
  ));

  RCCHECK(rclc_publisher_init_default(
    &acc_pub_,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
    "/mpu6050_imu/acc"
  ));

  // Create the executor (we're polling in the loop)
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
}

void setup() {
  // --- MPU6050 Setup ---
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000);
  #endif

  Serial.begin(115200);  // Initialize Serial once with desired baud rate
  while (!Serial);
  Serial.println("Initializing MPU6050...");
  
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  Serial.println("Initializing DMP...");
  devStatus = mpu.dmpInitialize();
  
  // Calibrate IMU offsets
  imu_calibration();

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
    // Reset interrupt flag
    mpuInterrupt = false;

    // Get the quaternion from the FIFO packet and publish it
    mpu.dmpGetQuaternion(&q_imu, fifoBuffer);
    q_msg.w = q_imu.w;
    q_msg.x = q_imu.x;
    q_msg.y = q_imu.y;
    q_msg.z = q_imu.z;
    RCSOFTCHECK(rcl_publish(&quat_pub_, &q_msg, NULL));

    // Get acceleration from the FIFO packet:
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q_imu);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q_imu);
    float g_const = 9.80665;

    // World-frame acceleration
    float a_world[3];
    a_world[0] = (float) aaWorld.x * g_const / 8192;
    a_world[1] = (float) aaWorld.y * g_const / 8192;
    a_world[2] = (float) aaWorld.z * g_const / 8192;

    // Publish the filtered acceleration
    acc_msg.x = a_world[0];
    acc_msg.y = a_world[1];
    acc_msg.z = a_world[2];
    RCSOFTCHECK(rcl_publish(&acc_pub_, &acc_msg, NULL));

    // Blink the LED as a simple indicator
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));

  }

  // Publish a communication signal to indicate that the node is alive
  com_msg.data = true;
  RCSOFTCHECK(rcl_publish(&com_pub_, &com_msg, NULL));
}







// // quat, acc, vel, pos
// #include <Arduino.h>
// #include <micro_ros_platformio.h>
// #include <rcl/rcl.h>
// #include <rcl/error_handling.h>
// #include <rclc/rclc.h>
// #include <rclc/executor.h>
// #include <std_msgs/msg/bool.h>
// #include <geometry_msgs/msg/quaternion.h>
// #include <geometry_msgs/msg/vector3.h>

// // MPU6050 libraries
// #include "I2Cdev.h"
// #include "MPU6050_6Axis_MotionApps20.h"

// // High-pass and low-pass filter libraries
// #include <filters.h>

// #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
//   #include "Wire.h"
// #endif

// // ----- MPU6050 Definitions -----
// MPU6050 mpu;
// #define INTERRUPT_PIN 2   // for ESP32, choose an available interrupt-capable pin
// #define LED_PIN 13        // onboard LED for activity indication
// volatile bool mpuInterrupt = false;
// bool dmpReady = false;
// uint8_t devStatus;
// uint16_t packetSize;
// uint8_t fifoBuffer[64];
// Quaternion q_imu;         // container for the quaternion from the MPU6050

// // Variables for acceleration calculations:
// VectorInt16 aa;         // raw accelerometer values from FIFO
// VectorInt16 aaReal;     // linear acceleration (gravity removed) in sensor frame
// VectorInt16 aaWorld;    // linear acceleration in world frame
// VectorFloat gravity;    // gravity vector

// // Integration variables for velocity and position (m/s and m, respectively)
// float velocity[3] = {0, 0, 0};
// float position[3] = {0, 0, 0};
// const float Ts = 0.05; // integration time step (in seconds)

// // High-pass filter objects to remove drift in velocity and position
// const float vel_cutoff = 0.1; // Hz
// const float pos_cutoff = 0.1; // Hz
// Filter vel_filter_x(vel_cutoff, Ts, IIR::ORDER::OD2, IIR::TYPE::HIGHPASS);
// Filter vel_filter_y(vel_cutoff, Ts, IIR::ORDER::OD2, IIR::TYPE::HIGHPASS);
// Filter vel_filter_z(vel_cutoff, Ts, IIR::ORDER::OD2, IIR::TYPE::HIGHPASS);
// Filter pos_filter_x(pos_cutoff, Ts, IIR::ORDER::OD2, IIR::TYPE::HIGHPASS);
// Filter pos_filter_y(pos_cutoff, Ts, IIR::ORDER::OD2, IIR::TYPE::HIGHPASS);
// Filter pos_filter_z(pos_cutoff, Ts, IIR::ORDER::OD2, IIR::TYPE::HIGHPASS);

// // *** New: Low-pass filters for acceleration ***
// // We use a cutoff frequency (in Hz) that suits your noise characteristics.
// // Adjust these parameters as needed.
// const float acc_lp_cutoff = 10.0; // Hz
// Filter acc_lowpass_x(acc_lp_cutoff, Ts, IIR::ORDER::OD3, IIR::TYPE::LOWPASS);
// Filter acc_lowpass_y(acc_lp_cutoff, Ts, IIR::ORDER::OD3, IIR::TYPE::LOWPASS);
// Filter acc_lowpass_z(acc_lp_cutoff, Ts, IIR::ORDER::OD3, IIR::TYPE::LOWPASS);

// // Interrupt callback: set flag when new data is ready
// void dmpDataReady() {
//   mpuInterrupt = true;
// }

// // ----- micro-ROS Objects -----
// rcl_publisher_t com_pub_;
// rcl_publisher_t quat_pub_;
// rcl_publisher_t acc_pub_;
// rcl_publisher_t vel_pub_;
// rcl_publisher_t pos_pub_;

// std_msgs__msg__Bool com_msg;
// geometry_msgs__msg__Quaternion q_msg;
// geometry_msgs__msg__Vector3 acc_msg;
// geometry_msgs__msg__Vector3 vel_msg;
// geometry_msgs__msg__Vector3 pos_msg;

// rclc_executor_t executor;
// rclc_support_t support;
// rcl_allocator_t allocator;
// rcl_node_t node;

// size_t domain_id = 5; // Set your desired ROS_DOMAIN_ID here

// // Macros to check return values
// #define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ error_loop(); } }
// #define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){} }

// // Error loop (halts execution)
// void error_loop() {
//   while (1) {
//     delay(100);
//   }
// }

// void imu_calibration() {
//   // Supply your own IMU offsets (adjust based on your calibration)
//   mpu.setXGyroOffset(12);
//   mpu.setYGyroOffset(52);
//   mpu.setZGyroOffset(2);
//   mpu.setZAccelOffset(1804);
// }

// // Initialize micro-ROS with ROS_DOMAIN_ID support
// void setup_micro_ros() {
//   // Set up the serial transports (Serial must already be initialized)
//   set_microros_serial_transports(Serial);

//   allocator = rcl_get_default_allocator();

//   // --- Domain ID Initialization ---
//   rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
//   RCCHECK(rcl_init_options_init(&init_options, allocator));
//   RCCHECK(rcl_init_options_set_domain_id(&init_options, domain_id));

//   // Initialize support using the options we configured
//   RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

//   // --- Create Node and Publishers ---
//   RCCHECK(rclc_node_init_default(&node, "mpu6050_imu", "", &support));

//   RCCHECK(rclc_publisher_init_default(
//       &com_pub_,
//       &node,
//       ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
//       "/mpu6050_imu/communication_signal"
//   ));

//   RCCHECK(rclc_publisher_init_default(
//     &quat_pub_,
//     &node,
//     ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Quaternion),
//     "/mpu6050_imu/quat"
//   ));

//   RCCHECK(rclc_publisher_init_default(
//     &acc_pub_,
//     &node,
//     ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
//     "/mpu6050_imu/acc"
//   ));

//   RCCHECK(rclc_publisher_init_default(
//     &vel_pub_,
//     &node,
//     ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
//     "/mpu6050_imu/vel"
//   ));

//   RCCHECK(rclc_publisher_init_default(
//     &pos_pub_,
//     &node,
//     ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
//     "/mpu6050_imu/pos"
//   ));

//   // Create the executor (we're polling in the loop)
//   RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
// }

// void setup() {
//   // --- MPU6050 Setup ---
//   #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
//     Wire.begin();
//     Wire.setClock(400000);
//   #endif

//   Serial.begin(115200);  // Initialize Serial once with desired baud rate
//   while (!Serial);
//   Serial.println("Initializing MPU6050...");
  
//   mpu.initialize();
//   pinMode(INTERRUPT_PIN, INPUT);
//   Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

//   Serial.println("Initializing DMP...");
//   devStatus = mpu.dmpInitialize();
  
//   // Calibrate IMU offsets
//   imu_calibration();

//   if (devStatus == 0) {
//     mpu.CalibrateAccel(6);
//     mpu.CalibrateGyro(6);
//     mpu.setDMPEnabled(true);
//     attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
//     dmpReady = true;
//     packetSize = mpu.dmpGetFIFOPacketSize();
//     Serial.println("DMP ready!");
//   } else {
//     Serial.print("DMP Initialization failed (code ");
//     Serial.print(devStatus);
//     Serial.println(")");
//     while (1);
//   }

//   pinMode(LED_PIN, OUTPUT);

//   // --- micro-ROS Setup ---
//   setup_micro_ros();
// }

// void loop() {
//   // Spin the micro-ROS executor briefly to process any callbacks
//   rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));

//   // Process MPU6050 data and publish if available:
//   if (dmpReady && mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
//     // Reset interrupt flag
//     mpuInterrupt = false;

//     // Get the quaternion from the FIFO packet and publish it
//     mpu.dmpGetQuaternion(&q_imu, fifoBuffer);
//     q_msg.w = q_imu.w;
//     q_msg.x = q_imu.x;
//     q_msg.y = q_imu.y;
//     q_msg.z = q_imu.z;
//     RCSOFTCHECK(rcl_publish(&quat_pub_, &q_msg, NULL));

//     // Get acceleration from the FIFO packet:
//     mpu.dmpGetAccel(&aa, fifoBuffer);
//     mpu.dmpGetGravity(&gravity, &q_imu);
//     mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
//     mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q_imu);
//     float g_const = 9.80665;

//     // World-frame acceleration
//     float a_world[3];
//     a_world[0] = (float) aaWorld.x * g_const / 8192;
//     a_world[1] = (float) aaWorld.y * g_const / 8192;
//     a_world[2] = (float) aaWorld.z * g_const / 8192;

//     // --- Apply Low-Pass Filter to World Acceleration ---
//     float a_world_lp[3];
//     a_world_lp[0] = acc_lowpass_x.filterIn(a_world[0]);
//     a_world_lp[1] = acc_lowpass_y.filterIn(a_world[1]);
//     a_world_lp[2] = acc_lowpass_z.filterIn(a_world[2]);

//     // Publish the filtered acceleration
//     acc_msg.x = a_world_lp[0];
//     acc_msg.y = a_world_lp[1];
//     acc_msg.z = a_world_lp[2];
//     RCSOFTCHECK(rcl_publish(&acc_pub_, &acc_msg, NULL));

//     // --- Velocity and Position Estimation ---
//     // Use the filtered acceleration for integration
//     velocity[0] += a_world_lp[0] * Ts;
//     velocity[1] += a_world_lp[1] * Ts;
//     velocity[2] += a_world_lp[2] * Ts;
//     // Apply high-pass filter to remove low-frequency drift in velocity
//     float vel_filt_x = vel_filter_x.filterIn(velocity[0]);
//     float vel_filt_y = vel_filter_y.filterIn(velocity[1]);
//     float vel_filt_z = vel_filter_z.filterIn(velocity[2]);
//     vel_msg.x = vel_filt_x;
//     vel_msg.y = vel_filt_y;
//     vel_msg.z = vel_filt_z;
//     RCSOFTCHECK(rcl_publish(&vel_pub_, &vel_msg, NULL));

//     // Integrate filtered velocity to compute position (using simple Euler integration)
//     position[0] += vel_filt_x * Ts + 0.5 * a_world_lp[0] * Ts * Ts;
//     position[1] += vel_filt_y * Ts + 0.5 * a_world_lp[1] * Ts * Ts;
//     position[2] += vel_filt_z * Ts + 0.5 * a_world_lp[2] * Ts * Ts;
//     // Apply high-pass filter to remove drift in position
//     float pos_filt_x = pos_filter_x.filterIn(position[0]);
//     float pos_filt_y = pos_filter_y.filterIn(position[1]);
//     float pos_filt_z = pos_filter_z.filterIn(position[2]);
//     pos_msg.x = pos_filt_x;
//     pos_msg.y = pos_filt_y;
//     pos_msg.z = pos_filt_z;
//     RCSOFTCHECK(rcl_publish(&pos_pub_, &pos_msg, NULL));

//     // Blink the LED as a simple indicator
//     digitalWrite(LED_PIN, !digitalRead(LED_PIN));

//   }

//   // Publish a communication signal to indicate that the node is alive
//   com_msg.data = true;
//   RCSOFTCHECK(rcl_publish(&com_pub_, &com_msg, NULL));
// }



// // MPU6050 libraries (for calibration)
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

        // #ifdef OUTPUT_READABLE_REALACCEL
        //     // display real acceleration, adjusted to remove gravity
        //     mpu.dmpGetQuaternion(&q, fifoBuffer);
        //     mpu.dmpGetAccel(&aa, fifoBuffer);
        //     mpu.dmpGetGravity(&gravity, &q);
        //     mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            
        //     // Serial.print("areal\t");
        //     // Serial.print(aaReal.x);
        //     // Serial.print("\t");
        //     // Serial.print(aaReal.y);
        //     // Serial.print("\t");
        //     // Serial.println(aaReal.z);

        //     // Serial.print("g [normalised]\t");
        //     // Serial.print(gravity.x);
        //     // Serial.print("\t");
        //     // Serial.print(gravity.y);
        //     // Serial.print("\t");
        //     // Serial.println(gravity.z);

        //     float g = 9.80665;
        //     float a_s_real[3];
        //     a_s_real[0] = (float) aaReal.x * g / 8192;
        //     a_s_real[1] = (float) aaReal.y * g / 8192;
        //     a_s_real[2] = (float) aaReal.z * g / 8192;

        //     Serial.print("areal\t");
        //     Serial.print(a_s_real[0]);
        //     Serial.print("\t");
        //     Serial.print(a_s_real[1]);
        //     Serial.print("\t");
        //     Serial.println(a_s_real[2]);

        //     delay(33);
        // #endif

        // #ifdef OUTPUT_READABLE_WORLDACCEL
        //     // display initial world-frame acceleration, adjusted to remove gravity
        //     // and rotated based on known orientation from quaternion
        //     mpu.dmpGetQuaternion(&q, fifoBuffer);
        //     mpu.dmpGetAccel(&aa, fifoBuffer);
        //     mpu.dmpGetGravity(&gravity, &q);
        //     mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        //     mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
        //     Serial.print("aworld\t");
        //     Serial.print(aaWorld.x);
        //     Serial.print("\t");
        //     Serial.print(aaWorld.y);
        //     Serial.print("\t");
        //     Serial.println(aaWorld.z);
        //     delay(33);
        // #endif
    
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