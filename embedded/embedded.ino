#include <Adafruit_BNO055.h>

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include <ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3.h>

ros::NodeHandle nh;

// Servo
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
int pins[6] = {0, 2, 4, 6, 8, 10};
int mins[6] = {124, 112, 118, 165, 89, 112}; // 102
int maxs[6] = {492, 487, 486, 520, 468, 489}; // 491
uint16_t angles[6] = {90, 80, 90, 100, 90, 100};

void setAngle(int id, int angle)
{
  pwm.setPWM(pins[id], 0, map(angle, 0, 180, mins[id], maxs[id]));
}

void servoCb(const std_msgs::Int16MultiArray& msg)
{
  for (int i = 0; i < 6; ++i)
  {
    angles[i] = msg.data[i];
  }
}

ros::Subscriber<std_msgs::Int16MultiArray> servo_sub("servo/command", servoCb);

void servoSetup()
{
  pwm.begin();
  pwm.setPWMFreq(50);
  nh.subscribe(servo_sub);
}

void servoLoop()
{
  for (int i = 0; i < 6; ++i)
  {
    setAngle(i, angles[i]);
  }
}

// Sensor
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire);
sensor_msgs::Imu imu_msg;
geometry_msgs::Vector3 euler_msg;
std_msgs::Int16MultiArray calib_msg;

ros::Publisher imu_pub("sensor/imu", &imu_msg);
ros::Publisher euler_pub("sensor/euler", &euler_msg);
ros::Publisher calib_pub("sensor/calib", &calib_msg);

void sensorSetup()
{
  bno.begin();
  bno.setExtCrystalUse(true);

  nh.advertise(imu_pub);
  nh.advertise(euler_pub);
  nh.advertise(calib_pub);

  calib_msg.data = (int16_t*)malloc(sizeof(int16_t) * 4);
  calib_msg.data_length = 4;
}

void sensorLoop()
{
  imu::Quaternion quat = bno.getQuat();
  imu_msg.orientation.x = quat.x();
  imu_msg.orientation.y = quat.y();
  imu_msg.orientation.z = quat.z();
  imu_msg.orientation.w = quat.w();

  // imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  // euler_msg.x = euler.x();
  // euler_msg.y = euler.y();
  // euler_msg.z = euler.z();

  // imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  // imu_msg.linear_acceleration.x = acc.x();
  // imu_msg.linear_acceleration.y = acc.y();
  // imu_msg.linear_acceleration.z = acc.z();
  // 
  // uint8_t system, gyro, accel, mag = 0;
  // bno.getCalibration(&system, &gyro, &accel, &mag);
  // calib_msg.data[0] = system;
  // calib_msg.data[1] = gyro;
  // calib_msg.data[2] = accel;
  // calib_msg.data[3] = mag;

  imu_pub.publish(&imu_msg);
  // euler_pub.publish(&euler_msg);
  // calib_pub.publish(&calib_msg);
}

// param
void paramLoad() {
  nh.getParam("/serial_node/pins", pins, 6);
  nh.getParam("/serial_node/mins", mins, 6);
  nh.getParam("/serial_node/maxs", maxs, 6);
}

// main
unsigned long servo_timer = 0;
unsigned long sensor_timer = 0;
unsigned long param_timer = 0;

void setup() {
  // nh.getHardware()->setBaud(9600);
  // nh.getHardware()->setBaud(115200);
  nh.initNode();

  servoSetup();
  sensorSetup();

  paramLoad();

  delay(1000);
}

void loop() {
  unsigned long now = millis();

  if ((now - servo_timer) >= 100 ) {
    servoLoop();
    servo_timer = now;
  }

  if ((now - sensor_timer) >= 100 ) {
    sensorLoop();
    sensor_timer = now;
  }

  if ((now - param_timer) >= 500 ) {
    paramLoad();
    param_timer = now;
  }
  
  nh.spinOnce();
}
