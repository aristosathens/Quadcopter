/* Quadcopter Microcontroller Code
 * Aristos Athens
 * Began March 2018
 * The gyro+accelerometer math in this code is taken from Joop Brokking (http://www.brokking.net/imu.html - https://youtu.be/4BoIE8YQwM8)
 * 
 * This code begins an I2C bus with the Arduino Uno as master and MPU6050 IMU as slave. Reads gyro/accelerometer data at 250 Hz

 * Arduino Uno Connections
 *  Gyro:
 *    VCC  -  5V
 *    GND  -  GND
 *    SDA  -  A4
 *    SCL  -  A5
 *  Wireless Receiver:
 *    VCC - 5V
 *    GND - GND
 *    Data - Pin 11
 *  LED:
 *    LED - Pin 13
 *  ESCs:
 *    ESC1 - Pin 4 (front-right - CCW)
 *    ESC2 - Pin 5 (rear-right - CW)
 *    ESC3 - Pin 6 (rear-left - CCW)
 *    ESC4 - Pin 7 (front-left - CW)
 *    
*/

/* -----------------------------------------------------------------------------------------------------------------------
 * Includes
 * -----------------------------------------------------------------------------------------------------------------------*/

//Includes
#include <Wire.h>                        //This is a library for I2C communication. Used to communicate between MPU6050 IMU and the Arduino Uno on the quadcopter
#include <VirtualWire.h>                 //This is a library that handles details of wireless RF communication.
#include <SPI.h>
#include <stdlib.h>                      //Use this for string/int conversion functions


/* -----------------------------------------------------------------------------------------------------------------------
 * Variables, Structs
 * -----------------------------------------------------------------------------------------------------------------------*/

//Structs - Use this to easily pass the data around and to avoid using an unecessary number of global variables
struct data_struct {
  bool calibrate_flag = false;
  boolean set_gyro_angles;
  int gyro_x, gyro_y, gyro_z;
  long gyro_x_cal, gyro_y_cal, gyro_z_cal;
  double gyro_pitch, gyro_roll, gyro_yaw;
  long acc_x, acc_y, acc_z;
  long acc_total_vector;
  float current_pitch, current_roll, current_yaw;
  float current_roll_acc, current_pitch_acc;
  float current_pitch_output, current_roll_output;
};

struct command_struct{
  bool calibrate_flag = false;
  int controller_throttle_offset, controller_pitch_offset, controller_roll_offset;
  uint8_t raw_controller_data[4];   //size is in bytes
  int controller_data[4];           //size is number of ints
  float pitch_desired, roll_desired, yaw_desired;
  float pid_pitch_integrator, pid_roll_integrator, pid_yaw_integrator;
  float pid_pitch_effort, pid_roll_effort, pid_yaw_effort;
  long esc_loop_timer;
  int esc_1, esc_2, esc_3, esc_4;
  int throttle;
};

//Global Variables
int temperature;
long loop_timer;


/* -----------------------------------------------------------------------------------------------------------------------
 * Defines
 * -----------------------------------------------------------------------------------------------------------------------*/

//Defines
#define RECEIVER_PIN 11
#define LED_ON PORTB |= B00100000; //Turn LED on.
#define LED_OFF PORTB &= B11011111; //Turn LED off.

#define p_gain_roll 1.0
#define i_gain_roll 1.0 
#define d_gain_roll 1.0
#define p_gain_pitch 1.0
#define i_gain_pitch 1.0
#define d_gain_pitch 1.0
#define p_gain_yaw 1.0
#define i_gain_yaw 1.0
#define d_gain_yaw 1.0

#define ESC_MIN 125
#define ESC_MAX 250
#define ESC_ZERO 188
#define ESC_1_ON  PORTD |= B00010000
#define ESC_1_OFF PORTD &= B11101111
#define ESC_2_ON  PORTD |= B00100000
#define ESC_2_OFF PORTD &= B11011111
#define ESC_3_ON  PORTD |= B01000000
#define ESC_3_OFF PORTD &= B10111111
#define ESC_4_ON  PORTD |= B10000000
#define ESC_4_OFF PORTD &= B01111111


/* -----------------------------------------------------------------------------------------------------------------------
 * setup() and loop()
 * -----------------------------------------------------------------------------------------------------------------------*/

void setup() {
  Serial.begin(9600);                                                  //Use only for debugging
  Serial.println("Beginning setup...");
  Wire.begin();                                                        //Start I2C as master
  init_pins();
  init_MPU6050();                                                      //Setup the registers of the MPU-6050 (500dfs / +/-8g) and start the gyro
  init_receiver();
  Serial.println("Exiting setup...");
  loop_timer = micros();                                               //Reset the loop timer
}

void loop(){
  static data_struct data;
  static command_struct com;
  if(!data.calibrate_flag) calibrate_MPU6050(data);
  if(!com.calibrate_flag) calibrate_receiver(com);
  
  read_MPU6050(data);                                                  //Read the raw acc and gyro data from the MPU-6050
  process_MPU6050_data(data);
  read_receiver(data, com);
  PID_control(data, com);
  process_ESC_commands(com);
  write_ESCs(com);
  wait();                                                              //Wait until the loop_timer reaches 4000us (250Hz) before starting the next loop
}

/* -----------------------------------------------------------------------------------------------------------------------
 * Init Pins
 * -----------------------------------------------------------------------------------------------------------------------*/

void init_pins(){
  //Pins default as inputs. No action necessary for those
  DDRB |= B00100000;                                                   //Set pin 13 as an output
}

/* -----------------------------------------------------------------------------------------------------------------------
 * MPU 6050
 * -----------------------------------------------------------------------------------------------------------------------*/

//Setup the necessary registers on the MPU6050
void init_MPU6050(){
  //Activate the MPU-6050
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x6B);                                                    //Send the requested starting register
  Wire.write(0x00);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
  //Configure the accelerometer (+/-8g)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1C);                                                    //Send the requested starting register
  Wire.write(0x10);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
  //Configure the gyro (500dps full scale)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1B);                                                    //Send the requested starting register
  Wire.write(0x08);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
}


//Get data from MPU6050 2000 times in order to find any drift
void calibrate_MPU6050(data_struct &data){
  Serial.println("Beginning MPU6050 calibration...");
  for (int cal_int = 0; cal_int < 2000 ; cal_int ++){                       //Run this code 2000 times
    read_MPU6050(data);                                                     //Read the raw acc and gyro data from the MPU-6050
    data.gyro_x_cal += data.gyro_x;                                         //Add the gyro x-axis offset to the gyro_x_cal variable
    data.gyro_y_cal += data.gyro_y;                                         //Add the gyro y-axis offset to the gyro_y_cal variable
    data.gyro_z_cal += data.gyro_z;                                         //Add the gyro z-axis offset to the gyro_z_cal variable
    delay(3);                                                               //Delay 3us to simulate the 250Hz program loop
  }
  data.gyro_x_cal /= 2000;                                                  //Divide the gyro_x_cal variable by 2000 to get the avarage offset
  data.gyro_y_cal /= 2000;                                                  //Divide the gyro_y_cal variable by 2000 to get the avarage offset
  data.gyro_z_cal /= 2000;                                                  //Divide the gyro_z_cal variable by 2000 to get the avarage offset
  data.calibrate_flag = true;
  Serial.println("Exiting MPU6050 calibration...");
}


void read_MPU6050(data_struct &data){                                     //Subroutine for reading the raw gyro and accelerometer data
//  Serial.println("Beginning MPU6050 read...");
  Wire.beginTransmission(0x68);                                             //Start communicating with the MPU-6050
  Wire.write(0x3B);                                                         //Send the requested starting register
  Wire.endTransmission(false);                                              //End the transmission
  Wire.requestFrom(0x68,14);                                                //Request 14 bytes from the MPU-6050
//  Serial.println("Beginning MPU6050 read while loop...");
  while(Wire.available() < 14);                                             //Wait until all the bytes are received
//  Serial.println("Exited MPU6050 read while loop...");
  data.acc_x = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_x variable
  data.acc_y = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_y variable
  data.acc_z = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_z variable
  temperature = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the temperature variable
  data.gyro_x = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_x variable
  data.gyro_y = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_y variable
  data.gyro_z = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_z variable
//  Serial.println("Exiting MPU6050 read...");
  Wire.endTransmission();
}


void process_MPU6050_data(data_struct &data){
  data.gyro_x -= data.gyro_x_cal;                                                         //Subtract the offset calibration value from the raw gyro_x value
  data.gyro_y -= data.gyro_y_cal;                                                         //Subtract the offset calibration value from the raw gyro_y value
  data.gyro_z -= data.gyro_z_cal;                                                         //Subtract the offset calibration value from the raw gyro_z value
  
  //Gyro angle calculations
  //00000611 = 1 / (250Hz / 65.5)
  data.current_pitch += data.gyro_x * 0.0000611;                                          //Calculate the traveled pitch angle and add this to the current_pitch variable
  data.current_roll += data.gyro_y * 0.0000611;                                           //Calculate the traveled roll angle and add this to the current_roll variable
  
  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
  data.current_pitch += data.current_roll * sin(data.gyro_z * 0.000001066);               //If the IMU has yawed transfer the roll angle to the pitch angel
  data.current_roll -= data.current_pitch * sin(data.gyro_z * 0.000001066);               //If the IMU has yawed transfer the pitch angle to the roll angel
  
  //Accelerometer angle calculations
  data.acc_total_vector = sqrt((data.acc_x*data.acc_x)+(data.acc_y*data.acc_y)+(data.acc_z*data.acc_z));  //Calculate the total accelerometer vector
  //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
  data.current_pitch_acc = asin((float)data.acc_y/data.acc_total_vector)* 57.296;         //Calculate the pitch angle
  data.current_roll_acc = asin((float)data.acc_x/data.acc_total_vector)* -57.296;         //Calculate the roll angle
  
  //Place the MPU-6050 spirit level and note the values in the following two lines for calibration
  data.current_pitch_acc -= 0.0;                                                          //Accelerometer calibration value for pitch
  data.current_roll_acc -= 0.0;                                                           //Accelerometer calibration value for roll

  if(data.set_gyro_angles){                                                               //If the IMU is already started
    data.current_pitch = data.current_pitch * 0.9996 + data.current_pitch_acc * 0.0004;   //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
    data.current_roll = data.current_roll * 0.9996 + data.current_roll_acc * 0.0004;      //Correct the drift of the gyro roll angle with the accelerometer roll angle
  }
  else{                                                                                   //At first start
    data.current_pitch = data.current_pitch_acc;                                          //Set the gyro pitch angle equal to the accelerometer pitch angle 
    data.current_roll = data.current_roll_acc;                                            //Set the gyro roll angle equal to the accelerometer roll angle 
    data.set_gyro_angles = true;                                                          //Set the IMU started flag
  }
  
  //To dampen the pitch and roll angles a complementary filter is used
  data.current_pitch_output = data.current_pitch_output * 0.9 + data.current_pitch * 0.1;   //Take 90% of the output pitch value and add 10% of the raw pitch value
  data.current_roll_output = data.current_roll_output * 0.9 + data.current_roll * 0.1;      //Take 90% of the output roll value and add 10% of the raw roll value
}


/* -----------------------------------------------------------------------------------------------------------------------
 * Receiver
 * -----------------------------------------------------------------------------------------------------------------------*/

void init_receiver(){
    vw_set_rx_pin(RECEIVER_PIN);
    vw_set_ptt_inverted(true);      // Required for DR3100
    vw_setup(2000);                 // Bits per sec
    vw_rx_start();                  // Start the receiver PLL running
}

void calibrate_receiver(command_struct &com){
  Serial.println("Beginning controller calibration...");
  int throttle_offset, pitch_offset, roll_offset = 0;
  
  for (int i = 0; i < 2000 ; i++){                                          //Run this code 2000 times
   vw_get_message((uint8_t) com.controller_data, (uint8_t)sizeof(com.controller_data));
   throttle_offset += com.controller_data[0];
   pitch_offset += com.controller_data[1];
   roll_offset += com.controller_data[2];
   delay(3);                                                               //Delay 3us to simulate the 250Hz program loop
  }
  
  com.controller_throttle_offset = throttle_offset /= 2000;   
  com.controller_pitch_offset = pitch_offset /= 2000;
  com.controller_roll_offset = roll_offset /= 2000;             
  com.calibrate_flag = true;
  Serial.println("Exiting controller calibration...");
}

//These joysticks are used https://solarbotics.com/product/29114/
void read_receiver(data_struct &data, command_struct &com){
  LED_ON;
  uint8_t array_size = (uint8_t)sizeof(com.controller_data);
  vw_get_message(com.raw_controller_data, &array_size);
  
//  for (int i = 0; i < sizeof(com.raw_controller_data); i++){
//    Serial.println(com.raw_controller_data[i]);
//  }
//    else { Serial.println("NO READING");}

//  Serial.print("data[0] is: ");
//  Serial.println(com.controller_data[0]);
//  Serial.print("data[1] is: ");
//  Serial.println(com.controller_data[1]);
//  Serial.print("data[2] is: ");
//  Serial.println(com.controller_data[2]);
//  Serial.print("data[3] is: ");
//  Serial.println(com.controller_data[3]);
  com.throttle = ((ESC_MAX - ESC_MIN)/2)*(com.controller_data[0] - com.controller_throttle_offset)/(798 - 206);
  com.yaw_desired = 0;
  com.pitch_desired = data.current_pitch - 20*((com.controller_data[1] - com.controller_pitch_offset)/(798 - 206));
  com.roll_desired = data.current_roll + 20*((com.controller_data[2] - com.controller_roll_offset)/(797 - 203));
  LED_OFF;
}


 /* -----------------------------------------------------------------------------------------------------------------------
 * PID Controller
 * -----------------------------------------------------------------------------------------------------------------------*/

void PID_control(data_struct &data,command_struct &com){
  static int last_pitch_error = 0;
  static int last_roll_error = 0;
  static int last_yaw_error = 0;
  
  int error = data.current_pitch - com.pitch_desired;
  com.pid_pitch_integrator += i_gain_pitch*error;
  clamp_float(com.pid_pitch_integrator, -100, 100);
  com.pid_pitch_effort = p_gain_pitch*error + d_gain_pitch*(error - last_pitch_error) + com.pid_pitch_integrator;
  clamp_float(com.pid_pitch_effort, -100, 100);
  last_pitch_error = error;

  error = data.current_roll - com.roll_desired;
  com.pid_roll_integrator += i_gain_roll*error;
  clamp_float(com.pid_roll_integrator, -100, 100);
  com.pid_roll_effort = p_gain_roll*error + d_gain_roll*(error - last_roll_error) + com.pid_roll_integrator;
  clamp_float(com.pid_roll_effort, -100, 100);
  last_roll_error = error;

  error = data.current_yaw - com.yaw_desired;
  com.pid_yaw_integrator += i_gain_yaw*error;
  clamp_float(com.pid_yaw_integrator, -100, 100);
  com.pid_yaw_effort = p_gain_yaw*error + d_gain_yaw*(error - last_yaw_error) + com.pid_yaw_integrator;
  clamp_float(com.pid_yaw_effort, -100, 100);
  last_yaw_error = error;  
}


/* -----------------------------------------------------------------------------------------------------------------------
 * ESCs
 * -----------------------------------------------------------------------------------------------------------------------*/


void init_ESCs(){
  DDRD |= B11110000;                        //Set Digital Pins 4, 5, 6, 7 as outpus
}

//I use the oneshot125 ESC protocol. This means that each signal will be between 125 and 250 us.
//The ESCs I use support faster protocols like OneShot42 and MultiShot. However, I use an overall refresh rate of 250 Hz (4 ms, 4000 us), so there is really no need
//to use the faster protocols.
void process_ESC_commands(command_struct &com){
  com.throttle += ESC_ZERO;
  if(com.throttle > 0.85*ESC_MAX) com.throttle = 0.85*ESC_MAX;
  com.esc_1 = (int) (com.throttle - com.pid_pitch_effort + com.pid_roll_effort - com.pid_yaw_effort) + ESC_MIN; //Calculate the pulse for esc 1 (front-right - CCW)
  com.esc_2 = (int) (com.throttle + com.pid_pitch_effort + com.pid_roll_effort + com.pid_yaw_effort) + ESC_MIN; //Calculate the pulse for esc 2 (rear-right - CW)
  com.esc_3 = (int) (com.throttle + com.pid_pitch_effort - com.pid_roll_effort - com.pid_yaw_effort) + ESC_MIN; //Calculate the pulse for esc 3 (rear-left - CCW)
  com.esc_4 = (int) (com.throttle - com.pid_pitch_effort - com.pid_roll_effort + com.pid_yaw_effort) + ESC_MIN; //Calculate the pulse for esc 4 (front-left - CW)
  //Clamp minimum ESC time to 135 to stop motors from turning off.
  clamp_int(com.esc_1, 1.1*ESC_MIN, 0.9*ESC_MAX);
  clamp_int(com.esc_2, 1.1*ESC_MIN, 0.9*ESC_MAX);
  clamp_int(com.esc_3, 1.1*ESC_MIN, 0.9*ESC_MAX);
  clamp_int(com.esc_4, 1.1*ESC_MIN, 0.9*ESC_MAX);
}

//Turn on all ESCs
void write_ESCs(command_struct &com){
  ESC_1_ON;
  ESC_2_ON;
  ESC_3_ON;
  ESC_4_ON;
  while(PORTD >= 16){                                                           //Stay in this loop until output 4,5,6 and 7 are low.
    com.esc_loop_timer = micros();                                              //Read the current time.
    if(com.esc_1 <= com.esc_loop_timer) ESC_1_OFF;                              //Set digital output 4 to low if the time is expired.
    if(com.esc_2 <= com.esc_loop_timer) ESC_2_OFF;                              //Set digital output 5 to low if the time is expired.
    if(com.esc_3 <= com.esc_loop_timer) ESC_3_OFF;                              //Set digital output 6 to low if the time is expired.
    if(com.esc_4 <= com.esc_loop_timer) ESC_4_OFF;                              //Set digital output 7 to low if the time is expired.
  }
}


/* -----------------------------------------------------------------------------------------------------------------------
 * Helper Functions
 * -----------------------------------------------------------------------------------------------------------------------*/

void clamp_int(int &value, int lower_val, int upper_val){
  if (value < lower_val){
    value = lower_val;
  } else if (value > upper_val){
    value = upper_val;
  }
}

void clamp_float(float &value, float lower_val, float upper_val){
  if (value < lower_val){
    value = lower_val;
  } else if (value > upper_val){
    value = upper_val;
  }
}

void wait(){
  while(micros() - loop_timer < 4000);                                 //Wait until the loop_timer reaches 4000us (250Hz) before starting the next loop
  loop_timer = micros();                                               //Reset the loop timer
}


