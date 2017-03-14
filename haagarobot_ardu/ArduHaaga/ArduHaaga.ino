/////////////////////////////  Includes  /////////////////////////////////////
#include <SoftwareSerial.h>
#include <Wire.h>

#include <TinyGPS.h>	//GPS http://arduiniana.org/libraries/tinygps/
#include <GY_85.h>		//IMU https://github.com/sqrtmo/GY-85-arduino

// ROS
#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Vector3.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <haagarobot_msgs/Cmd_vel.h>
#include <haagarobot_msgs/Motor.h>
#include <haagarobot_msgs/cmd_rpm.h>

/////////////////////////////  pin settings  /////////////////////////////////////
#define EN1             4                     // ENABLE pin motor1
#define PWM1            5                     // PWM pin motor1
#define EN2             6                     // ENABLE pin motor2
#define PWM2            7                     // PWM pin motor2

#define encodePinA1      18                       // encoder1 A pin  int4
#define encodePinB1      19                       // encoder1 B pin  int5
#define encodePinA2      3                       // encoder2 A pin  int1
#define encodePinB2      2                       // encoder2 B pin  int0

#define GPSTX 10
#define GPSRX 11

#define GYROSDA 20
#define GYROSCL 21

/////////////////////////////  Parameters  /////////////////////////////////////
// 1 means Right
// 2 means Left

#define Gearratio1      12                      //motor1 gear ratio
#define EncoderPPR      48*4                    // encoder's ppr setting
#define Encoderpulse1   48*4                    //motor1 encoder pulse
#define Gearratio2      12                      //motor2 gear ratio
#define Encoderpulse2   48*4                    //motor2 encoder pulse

#define LOOPTIME        50                      // PID loop time
#define NUMREADINGS     10                      // samples for Amp average
#define LOOPTIMEVEL     10
#define LOOPTIMEGPS     1000                    //GPS refresh time
#define pi 3.141592

int motor1_rpm_cmd = 0;
int motor2_rpm_cmd = 0;

unsigned long lastMilli = 0;                    // loop timing
unsigned long lastMillimonitor = 0;
unsigned long lastMilliPrint = 0;

unsigned long dtMilli = 0;                     // Communication period
unsigned long dtMillimonitor = 50;
unsigned long dtMillispeed = 0;
unsigned long lastMillispeed = 0;

float speed_term = 5.0;                          //RPM
float accel = 2.5;

float speed_req1 = 0;                            // motor1 speed (Set Point)
float speed_profile1 = 0;
float speed_act1 = 0;                            // motor1 speed (actual value)
float speed_act1_rad = 0;
float speed_act1_rpm = 0;
float speed_act1_rps = 0;
float speed_act1_filtered = 0;

float speed_req2 = 0;                            // motor1 speed (Set Point)
float speed_profile2 = 0;
float speed_act2 = 0;                            // motor1 speed (actual value)
float speed_act2_rad = 0;
float speed_act2_rpm = 0;
float speed_act2_rps = 0;
float speed_act2_filtered = 0;

int PWM_val1 = 0;                                // motor1 PWM (25% = 64; 50% = 127; 75% = 191; 100% = 255)
int PWM_val1_prev = 0;
int PWM_val1_desired = 0;
int PWM_val2 = 0;                                // motor2 PWM (25% = 64; 50% = 127; 75% = 191; 100% = 255)
int PWM_val2_prev = 0;
int PWM_val2_desired = 0;


volatile long count1 = 0;                        // motor 1 rev counter
volatile long count2 = 0;                        // motor 2 rev counter
long inc_enc_pos1 = 0;
long inc_enc_pos2 = 0;
long inc_enc_prev1 = 0;
long inc_enc_prev2= 0;
float abs_enc_pos1 = 0;
float abs_enc_pos2 = 0;

long newpre1 = 0;
long newpre2 = 0;

float Kp1 =   0.57;                               // motor1 PID proportional control Gain
float Ki1 =   2.77;                               // motor1 PID Integral control Gain
float Kd1 =   0.88;                               // motor1 PID Derivitave control Gain
float Ka1 =   3.25;

float Kp2 =   0.57;                               // motor2 PID proportional control Gain
float Ki2 =   2.77;                               // motor2 PID Derivitave control Gain
float Kd2 =   0.88;                               // motor2 PID Integral control Gain
float Ka2 =   3.25; 

float controll_inc = 1;
float controll_inc_i = 0.01;

float pid_i1 = .0;
float pid_d1 = .0;
float pid_a1 = 0;
float pidTerm_err1 = 0;

float pid_i2 = .0;
float pid_d2 = .0;
float pid_a2 = 0;
float pidTerm_err2 = 0;

int lastspeed1 = 0;
int lastspeed2 = 0;

int manual = 0;
float beta = 1.0;
float error1 = 0.0;
float error2 = 0.0;

float last_error1 = 0;
float last_error2 = 0;

float pwm_prev1 = 0;
float pwm_prev2 = 0;
float enc_prev1 = 0;
float enc_prev2 = 0;

String inputString = "";
int stringComplete = 0;

/////////////////////////////  GPS  /////////////////////////////////////
SoftwareSerial gpsSerial(GPSTX, GPSRX); // TX, RX        //GPS functions
TinyGPS gps;
int gpsdata = 0;
float flat, flon;
unsigned long lastMilliGPS = 0;               // loop timing
unsigned long dtMilliGPS = 0;
uint8_t gps_config_change[63] = {
  0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xFA, 0x00, 0x01, 0x00, 0x01, 0x00,
  0x10, 0x96,
  //Baud change
  0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08,
  0x00, 0x00, 0x00, 0xC2, 0x01, 0x00, 0x07, 0x00, 0x02, 0x00, 0x00, 0x00,
  0x00, 0x00, 0xBF, 0x78,

  //Save config
  0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0x00, 0x00,
  0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x1D, 0xAB
};

void gpsdump(TinyGPS &gps);
void printFloat(double f, int digits = 2);

/////////////////////////////  GYRO  /////////////////////////////////////
GY_85 GY85;     //create the object
int ax = 0, ay = 0, az = 0;
int cx = 0, cy = 0, cz = 0;
float gx = 0, gy = 0, gz = 0, gt = 0;

/////////////////////////////  ROS  /////////////////////////////////////
ros::NodeHandle  nh;

// publisher for motor status
haagarobot_msgs::Motor msg_motor1;
ros::Publisher pub_motor1("haagarobot/motorR", &msg_motor1);
haagarobot_msgs::Motor msg_motor2;
ros::Publisher pub_motor2("haagarobot/motorL", &msg_motor2);

// publishers for GY-85
geometry_msgs::Vector3 msg_accel;
ros::Publisher pub_accel("haagarobot/gy85/accel", &msg_accel);
geometry_msgs::Vector3 msg_magnet;
ros::Publisher pub_magnet("haagarobot/gy85/magnet", &msg_magnet);
geometry_msgs::Vector3 msg_gyro;
ros::Publisher pub_gyro("haagarobot/gy85/gyro", &msg_gyro);

// publisher for odom topic
#define WheelBase  0.62
#define WheelDiameter  0.28

// subscriber for motor velocity command
void callback(const haagarobot_msgs::cmd_rpm& msg)
{
  motor1_rpm_cmd = msg.motorR_rpm;
  motor2_rpm_cmd = msg.motorL_rpm;
}
ros::Subscriber<haagarobot_msgs::cmd_rpm> cmdRPMSub("haagarobot/cmd_rpm", &callback);

// Odometry publish

geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;
double x = 0.0;
double y = 0.0;
double theta = 0.0;
char base_link[] = "/base_link";
char odom[] = "/odom";

char message[256] = "";
char tempBuffer[256];


void setup()
{
  Wire.begin();
  delay(10);
  analogReference(DEFAULT);                            //Reference 5V (Mega condition)

  Serial.begin(115200);                               // Monitoring & command port
  delay(10);

  //mySerial.begin(57600);
  // set the data rate for the SoftwareSerial port
  gpsSerial.begin(9600);                              // Set GPS baudrate 115200 from 9600
  gpsSerial.write(gps_config_change, sizeof(gps_config_change));
  gpsSerial.end();
  gpsSerial.begin(115200);

  pinMode(EN1, OUTPUT);                                //OUTPUT Pinmodes
  pinMode(EN2, OUTPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(PWM2, OUTPUT);

  pinMode(encodePinA1, INPUT);                          //Encoder Pinmodes
  pinMode(encodePinB1, INPUT);
  pinMode(encodePinA2, INPUT);
  pinMode(encodePinB2, INPUT);

  digitalWrite(encodePinA1, HIGH);                      // turn on Endocer pin pullup resistor
  digitalWrite(encodePinB1, HIGH);
  digitalWrite(encodePinA2, HIGH);
  digitalWrite(encodePinB2, HIGH);

  attachInterrupt(5, rencoder1, CHANGE);               //Encoder pin interrupt setting
  attachInterrupt(4, rencoder1, CHANGE);
  attachInterrupt(1, rencoder2, CHANGE);
  attachInterrupt(0, rencoder2, CHANGE);

  analogWrite(PWM1, PWM_val1);
  analogWrite(PWM2, PWM_val2);
  digitalWrite(EN1, LOW);
  digitalWrite(EN2, LOW);

  GY85.init();
  delay(10);

  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(pub_accel);
  nh.advertise(pub_magnet);
  nh.advertise(pub_gyro);
  nh.advertise(pub_motor1);
  nh.advertise(pub_motor2);
  nh.subscribe(cmdRPMSub);
  broadcaster.init(nh);
}

// LPF filter setup
float LPFVAR = 0.08;
float lpFilter(float value, float prev_value, float beta)
{
  float lpf = 0;
  lpf = (beta * value) + (1 - beta) * prev_value;
  return lpf;
}

// Kalman filter setup
float Pk = 1.0;
float varP = pow(0.01, 2);  // pow(0.01, 2)
float R = pow(0.5, 2);
float Kk = 1.0;
float Xk = 20.0;
float kalmanFilter(float value)
{
  Pk = Pk + varP;
  Kk = Pk / (Pk + R);
  Xk = (Kk * value) + (1 - Kk) * Xk;
  Pk = (1 - Kk) * Pk;
  return Xk;
}
int term = 0;

/////////////////////////////  Loop  /////////////////////////////////////
void loop()
{
  //checkEvent();
  
  dtMilli = millis() - lastMilli;                          // calculate dt
  dtMillispeed = millis() - lastMillispeed;
  dtMilliGPS = millis() - lastMilliGPS;


  if ( motor1_rpm_cmd > 320)
    speed_req1 = 320;
  else if ( motor1_rpm_cmd < -320)
    speed_req1 = -320;
  else
    speed_req1 = motor1_rpm_cmd;

  if ( motor2_rpm_cmd > 320)
    speed_req2 = 320;
  else if ( motor2_rpm_cmd < -320)
    speed_req2 = -320;
  else
    speed_req2 = motor2_rpm_cmd;


  if (dtMillispeed >= LOOPTIMEVEL)
  {

    speedcalculation(dtMillispeed);                          // calculate speed, volts and Amps

    speed_act1_filtered = lpFilter(speed_act1_rpm, enc_prev1, LPFVAR);
    enc_prev1 = speed_act1_filtered;
    newpre1 = count1;

    speed_act2_filtered = lpFilter(speed_act2_rpm, enc_prev2, LPFVAR);
    enc_prev2 = speed_act2_filtered;
    newpre2 = count2;

    lastMillispeed = millis();
  }

  if (dtMilli >= LOOPTIME)                                  // time loop check, soft real time
  {

    buildVelProfile(speed_req1, speed_act1_filtered, &speed_profile1);
    buildVelProfile(speed_req2, speed_act2_filtered, &speed_profile2);
	 // Motor 2 PWM calculation								
    PWM_val1 = updatePid1(speed_profile1, speed_act1_filtered, dtMilli, Kp1, Ki1, Kd1);
    PWM_val2 = updatePid2(speed_profile2, speed_act2_filtered, dtMilli, Kp2, Ki2, Kd2);  


    if (PWM_val1 > 0)
    {
      digitalWrite(EN1, HIGH);
    }
    else
    {
      digitalWrite(EN1, LOW);
    }

    if (PWM_val2 < 0)
    {
      digitalWrite(EN2, HIGH);
    }
    else
    {
      digitalWrite(EN2, LOW);
    }
    if (speed_req1 == 0 && abs(error1) == 0)
      PWM_val1 = 0;
    if (speed_req2 == 0 && abs(error2) == 0)
      PWM_val2 = 0;

    analogWrite(PWM1, abs(PWM_val1));                                               // send PWM to motor
    analogWrite(PWM2, abs(PWM_val2));                                               // send PWM to motor

    lastMilli = millis();
  }
//////////////////////////////// TODO : fix and add gps publisher /////////////////////////
  // GPS update
  bool newdata = false;
  unsigned long start = millis();
  while (millis() - start < 10) {
    if (gpsSerial.available())
    {
      char c = gpsSerial.read();
      //Serial.print(c);  // uncomment to see raw GPS data
      if (gps.encode(c))
      {
        newdata = true;
        // break;  // uncomment to print new data immediately!
      }
    }
  }
  if (newdata)
  {
    gpsdump(gps);
  }
  else
    gpsdata = 0;

  // Compass Update
  publishGy85();

  // Data publish through ROS
  publishMotor();

  // drive in a circle
  double dx = 0.0;
  double dtheta = 0.0;
  long d_left=0,d_right=0;
  d_left = inc_enc_pos2 -inc_enc_prev2;
  d_right = inc_enc_pos1 -inc_enc_prev1;
  dx = (TicksToMeters(d_left)+TicksToMeters(d_right))/2;
  dtheta = (TicksToMeters(d_left)-TicksToMeters(d_right))*WheelBase;
  
  x += cos(theta)*dx*0.1;
  y += sin(theta)*dx*0.1;
  theta += dtheta*0.1;
  if(theta > 3.14)
    theta=-3.14;
    
  // tf odom->base_link
  t.header.frame_id = odom;
  t.child_frame_id = base_link;
  
  t.transform.translation.x = x;
  t.transform.translation.y = y;
  
  t.transform.rotation = tf::createQuaternionFromYaw(theta);
  t.header.stamp = nh.now();
  
  broadcaster.sendTransform(t);

  inc_enc_prev2 = inc_enc_pos2;
  inc_enc_prev1=inc_enc_pos1;
  
  nh.spinOnce();
}
//Publish GY85 motor & encoder data
void publishMotor()
{
  msg_motor1.motor_rpm = speed_req1;
  msg_motor2.motor_rpm = speed_req2;
   
  msg_motor1.enc_tick = inc_enc_pos1;
  msg_motor2.enc_tick = inc_enc_pos2;
  
  pub_motor1.publish(&msg_motor1);
  pub_motor2.publish(&msg_motor2);
}

//Publish GY85 sensor data
void publishGy85()
{
  ax = GY85.accelerometer_x( GY85.readFromAccelerometer() );
  ay = GY85.accelerometer_y( GY85.readFromAccelerometer() );
  az = GY85.accelerometer_z( GY85.readFromAccelerometer() );

  cx = GY85.compass_x( GY85.readFromCompass() );
  cy = GY85.compass_y( GY85.readFromCompass() );
  cz = GY85.compass_z( GY85.readFromCompass() );

  gx = GY85.gyro_x( GY85.readGyro() );
  gy = GY85.gyro_y( GY85.readGyro() );
  gz = GY85.gyro_z( GY85.readGyro() );

  msg_accel.x = ax;
  msg_accel.y = ay;
  msg_accel.z = az;

  msg_magnet.x = cx;
  msg_magnet.y = cy;
  msg_magnet.z = cz;

  msg_gyro.x = gx;
  msg_gyro.y = gy;
  msg_gyro.z = gz;

  pub_accel.publish(&msg_accel);
  pub_gyro.publish(&msg_gyro);
  pub_magnet.publish(&msg_magnet);

}



void speedcalculation(unsigned long dt)  // calculate speed, volts and Amps
{
  //speed_act1 = (((float)count1 - (float)newpre1) * 1000) / dt / (Encoderpulse1)  / Gearratio1 *3.141592*2;          // ((count difference) / (dt) / 1000) / (pulses * gear ratio) * 2* 3.141592 = rad/sec
  //speed_act2 = (((float)count2 - (float)newpre2) * 1000) / dt / (Encoderpulse2)  / Gearratio2 *3.141592*2;          // ((count difference) / (dt) / 1000) / (pulses * gear ratio) * 2* 3.141592 = rad/sec

  speed_act1_rps = (((float)count1 - (float)newpre1) * 1000) / dt / (Encoderpulse1)  / Gearratio1 ;
  speed_act1_rpm = (((float)count1 - (float)newpre1) * 1000) / dt / (Encoderpulse1)  / Gearratio1 * 60;
  speed_act1_rad = (((float)count1 - (float)newpre1) * 1000) / dt / (Encoderpulse1)  / Gearratio1 * 3.141592 * 2;
  speed_act1 = speed_act1_rpm;
  inc_enc_pos1 = count1 / ((float)Encoderpulse1 * (float)Gearratio1) * 360.0; //accumulated motor's rotational angle(deg)

  speed_act2_rps = (((float)count2 - (float)newpre2) * 1000) / dt / (Encoderpulse2)  / Gearratio2 ;
  speed_act2_rpm = (((float)count2 - (float)newpre2) * 1000) / dt / (Encoderpulse2)  / Gearratio2 * 60;
  speed_act2_rad = (((float)count2 - (float)newpre2) * 1000) / dt / (Encoderpulse2)  / Gearratio2 * 3.141592 * 2;
  speed_act2 = speed_act2_rpm;
  inc_enc_pos2 = count2 / ((float)Encoderpulse2 * (float)Gearratio2) * 360.0;
}

void buildVelProfile(float desired_vel, float current_vel, float *speed_profile)
{
  float accel = 5.0;
  float vel_diff = desired_vel - current_vel;
  float time_interval = 100; //ms

  if (abs(desired_vel - *speed_profile) < 10)
    accel = abs(desired_vel - *speed_profile);

  if (desired_vel > *speed_profile)
    *speed_profile = *speed_profile + accel;
  else if (desired_vel < *speed_profile)
    *speed_profile = *speed_profile - accel;
}
double TicksToMeters(double Ticks)
{
  return (Ticks * 3.14 * WheelDiameter) / EncoderPPR;
}
float updatePid1( float targetValue, float currentValue, unsigned long dt, float Kp, float Ki, float Kd)    // compute PWM value
{

  float pidTerm = 0;                                                            // PID correction
  float pidTerm_sat = 0;
  //float error=0;                                                                  // error initiallization
  //static float last_error=0;                                                      // For D control, static last_error
  error1 = targetValue - currentValue;                                          // error calculation

  pid_a1 = pidTerm_err1 * Ka1;
  pid_i1 = pid_i1 + (error1 + pid_a1) * dt / 1000;                             // Error Integral
  pid_d1 = (error1 - last_error1) ;

  pidTerm = (Kp * error1) + Ki * pid_i1 + (Kd * (pid_d1));

  last_error1 = error1;                                                   //for D control

  if (pidTerm > 250)
    pidTerm_sat = 250;
  else if (pidTerm < -250)
    pidTerm_sat = -250;
  else
    pidTerm_sat = pidTerm;

  pidTerm_err1 = pidTerm_sat - pidTerm;

  return pidTerm_sat;
}

float updatePid2( float targetValue, float currentValue, unsigned long dt, float Kp, float Ki, float Kd)    // compute PWM value
{

  float pidTerm = 0;                                                            // PID correction
  float pidTerm_sat = 0;
  //float error=0;                                                                  // error initiallization
  //static float last_error=0;                                                      // For D control, static last_error
  error2 = targetValue - currentValue;                                          // error calculation

  pid_a1 = pidTerm_err2 * Ka2;
  pid_i2 = pid_i2 + (error2 + pid_a1) * dt / 1000;                           // Error Integral
  pid_d2 = (error2 - last_error2);

  pidTerm = (Kp * error2) + Ki * pid_i2 + (Kd * (pid_d2)) ;

  last_error2 = error2;                                                          //for D control
  if (pidTerm > 250)
    pidTerm_sat = 250;
  else if (pidTerm < -250)
    pidTerm_sat = -250;
  else
    pidTerm_sat = pidTerm;

  pidTerm_err2 = pidTerm_sat - pidTerm;

  return pidTerm_sat;
}



void gpsdump(TinyGPS & gps)
{
  long lat, lon;
  // float flat, flon;
  unsigned long age, date, time, chars;
  int year;
  byte month, day, hour, minute, second, hundredths;
  unsigned short sentences, failed;

  gps.f_get_position(&flat, &flon, &age);
}

//count from quad encoder

void rencoder1()   // pulse and direction, direct port reading to save cycles
{
  static long oldencodePinB1 = 0;

  if (digitalRead(encodePinA1) ^ oldencodePinB1)
  {
    count1++;
  }
  else
  {
    count1--;
  }
  oldencodePinB1 = digitalRead(encodePinB1);
}

void rencoder2()  // pulse and direction, direct port reading to save cycles
{
  static long oldencodePinB2 = 0;

  if (digitalRead(encodePinA2) ^ oldencodePinB2)
  {
    count2++;
  }
  else
  {
    count2--;
  }
  oldencodePinB2 = digitalRead(encodePinB2);
}
void printFloat(double number, int digits)
{
  // Handle negative numbers
  if (number < 0.0) {
    Serial.print('-');
    number = -number;
  }

  // Round correctly so that print(1.999, 2) prints as "2.00"
  double rounding = 0.5;
  for (uint8_t i = 0; i < digits; ++i)
    rounding /= 10.0;

  number += rounding;

  // Extract the integer part of the number and print it
  unsigned long int_part = (unsigned long)number;
  double remainder = number - (double)int_part;
  Serial.print(int_part);

  // Print the decimal point, but only if there are digits beyond
  if (digits > 0)
    Serial.print(".");

  // Extract digits from the remainder one at a time
  while (digits-- > 0) {
    remainder *= 10.0;
    int toPrint = int(remainder);
    Serial.print(toPrint);
    remainder -= toPrint;
  }
}


