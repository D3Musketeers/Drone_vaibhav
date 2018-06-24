
#include <Wire.h>
#include <Servo.h>

#define MIN_PULSE_LENGTH 1000 // Minimum pulse length in µs
#define MAX_PULSE_LENGTH 2000 // Maximum pulse length in µs
Servo motA, motB, motC, motD;

int16_t GyroX, GyroY, GyroZ;
int gyro_error=0,initial = 1;  
int16_t rGyroX, rGyroY, rGyroZ;
float AccX, AccY, AccZ, rAccX, rAccY, rAccZ;
float elapsedTime, time, prev_time, acc_total_vector;
int16_t Gyro_angle_X, Gyro_angle_Y, Gyro_angle_Z, Angle_pitch, Angle_pitch_acc, Angle_roll_acc, Angle_roll, Angle_yaw, yaw_to_pitch, yaw_to_roll, Initial_pitch, Initial_roll, Initial_yaw;
int16_t rGyroX_err,rGyroY_err,rGyroZ_err;

void MPU_setup () {
   Wire.begin();// Select MPU address to program
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00); //Come out of sleep mode
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  //Gyroscope table for MPU6040
  //Degree/sec, MPU value, register value
  //±250 º/s       131      0b00000000
  // ±500 º/s      65.5     0b00001000
  // ±1000 º/s     32.8     0b00010000
  // ±2000 º/s     16.4     0b00010000
  Wire.write(0b00001000); // 500 Full scale Gyro setting
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  //Accelerometer table for MPU6040
  //Acceleration, MPU value, register value
  //±2 g           16,384      0b00000000
  // ±4 g          8,192       0b00001000
  // ±8 g          4,096       0b00010000
  //±16 g          2,048       0b00011000
  Wire.write(0b00010000);
  Wire.endTransmission();

time = millis();
  int error_iteration = 200;
  // Start averaging Error values to minimize error...........
if(gyro_error==0)
  {
  Serial.print("Start Averaging eror");
  for (int i = 1; i < error_iteration; i++) {
    if (i % 20 == 0) {
      Serial.print("\nError averaging.... ");
      Serial.println(i / 20);
    }
    Wire.beginTransmission(0x68);
    Wire.write(0x43); // Gyro register strats from this address. 6 registers from this add as 2 registers for X,2 for Y and 2 for Z,
    Wire.endTransmission();
    Wire.requestFrom(0x68, 6);
    while (Wire.available() < 6);
    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();
    GyroZ = Wire.read() << 8 | Wire.read();
    rGyroX_err = rGyroX_err + GyroX / 65.5;
    rGyroY_err = rGyroY_err + GyroY / 65.5;
    rGyroZ_err = rGyroZ_err + GyroZ / 65.5;
  

  if(i== error_iteration - 1 )
      {
  rGyroX_err = rGyroX_err / error_iteration;
  rGyroY_err = rGyroY_err / error_iteration;
  rGyroZ_err = rGyroZ_err / error_iteration;
  gyro_error=1;
      }
  }
  }
}


void esc_setup_pins_set () {
  // Set ESC data input pins
  pinMode(4,OUTPUT);
  pinMode(5,OUTPUT);
  pinMode(6,OUTPUT);
  pinMode(7,OUTPUT);

  // Caliberate ESC on startup
   for (int cal_int = 0; cal_int < 1250 ; cal_int ++){                           //Wait 5 seconds before continuing.
    digitalWrite(4, HIGH);                                                 //Set digital poort 4, 5, 6 and 7 high.
    digitalWrite(5, HIGH);
    digitalWrite(6, HIGH);
    digitalWrite(7, HIGH); 
    delayMicroseconds(1000);                                                //Wait 1000us.
     digitalWrite(4, LOW);                                                 //Set digital poort 4, 5, 6 and 7 low.
    digitalWrite(5, LOW);
    digitalWrite(6, LOW);
    digitalWrite(7, LOW);                                                  
    delayMicroseconds(3000);                                                //Wait 3000us.
  }
}

void setup() {
  Serial.begin(9600);
   motA.attach(4, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
    motB.attach(5, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
    motC.attach(6, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
    motD.attach(7, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
  // put your setup code here, to run once:
  Serial.print("\nESC caliberate on startup\n");
  esc_setup_pins_set () ;
  Serial.print("\nESC caliberattion DONE!!!\n");
  Serial.print("\nBeginning MPU setup\n");
  MPU_setup ();
  Serial.print("MPU setup done!!!!!\n");
}

void loop () {

set initial = 1;
  // Adding below as the final rGyroX, rGyroY, rGyroZ are in degree/sec. We need to get the angle i.e degree.
  //So we need to find elapsed time for each Gyro reading. This is also called as integrating.
  prev_time = time;
  time = millis();
  elapsedTime = (time - prev_time) / 1000;
  
  // Done.............
  // Gyro values from MPU are in degree/sec. For the FS_SEL value check the conversion from data sheet.
  Wire.beginTransmission(0x68);
  Wire.write(0x43); // Gyro register strats from this address. 6 registers from this add as 2 registers for X,2 for Y and 2 for Z,
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  while (Wire.available() < 6);
  GyroX = Wire.read() << 8 | Wire.read();
  GyroY = Wire.read() << 8 | Wire.read();
  GyroZ = Wire.read() << 8 | Wire.read();

  GyroX = GyroX / 65.5 - rGyroX_err;
  GyroY = GyroY / 65.5 - rGyroY_err;
  GyroZ = GyroZ / 65.5 - rGyroZ_err;


  //X axis
  Angle_roll += GyroX * elapsedTime;
  //Y axis
  Angle_pitch += GyroY * elapsedTime;
  //Z axis
  Angle_yaw += GyroZ * elapsedTime;


  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  while (Wire.available() < 6);
  AccX = Wire.read() << 8 | Wire.read();
  AccY = Wire.read() << 8 | Wire.read();
  AccZ = Wire.read() << 8 | Wire.read();

  rAccX = AccX / 4096;
  rAccY = AccY / 4096;
  rAccZ = AccZ / 4096;




  ///Addtitional from YMFC-AL FLight controller utube video.
  //Gyro angle calculations
  //0.0000611 = 1 / (250Hz / 65.5)
  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
      yaw_to_roll =(elapsedTime/65.5 )* (3.142/180);
      Angle_pitch -= Angle_roll * sin(rGyroZ * yaw_to_roll);                  //If the IMU has yawed transfer the roll angle to the pitch angel.
      Angle_roll += Angle_pitch * sin(rGyroZ * yaw_to_roll);                  //If the IMU has yawed transfer the pitch angle to the roll angel.
  ////
  //      //Accelerometer angle calculations
      acc_total_vector = sqrt((rAccX*rAccX)+(rAccY*rAccY)+(rAccZ*rAccZ));       //Calculate the total accelerometer vector.
  //
      if(abs(rAccY) < acc_total_vector){                                        //Prevent the asin function to produce a NaN
         Angle_pitch_acc = asin((float)rAccY/acc_total_vector)* 57.296;          //Calculate the pitch angle.
        }
       if(abs(rAccX) < acc_total_vector){                                        //Prevent the asin function to produce a NaN
          Angle_roll_acc = asin((float)rAccX/acc_total_vector)* -57.296;          //Calculate the roll angle.
        }
  //
       Angle_pitch = Angle_pitch * 0.9996 + Angle_pitch_acc * 0.0004;            //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
        Angle_roll = Angle_roll * 0.9996 + Angle_roll_acc * 0.0004;               //Correct the drift of the gyro roll angle with the accelerometer roll angle.

  //      pitch_level_adjust = Angle_pitch * 15;                                    //Calculate the pitch angle correction
  //      roll_level_adjust = Angle_roll * 15;                                      //Calculate the roll angle correction
  //////////

if (initial == 1) {
Initial_pitch = Angle_pitch;
Initial_roll = Angle_roll;
Initial_yaw = Angle_yaw;

}
initial ++;

////// Working till this point

while (Initial_pitch != Angle_pitch) {
        motA.writeMicroseconds(i);
        motB.writeMicroseconds(i);
        motC.writeMicroseconds(i);
        motD.writeMicroseconds(i);
}




//  Serial.print("Elapsed time");
//  Serial.print(elapsedTime);
//  Serial.print(" Raw values ");
//  Serial.print(" Raw values X ");
//  Serial.print(rGyroX);
//  Serial.print(" Raw values y ");
//  Serial.print(rGyroY);
//  Serial.print(" Raw values Z ");
//  Serial.print(rGyroZ);
//
//  Serial.print(" Gyro: Gx: ");
//  Serial.print(Angle_roll);
//  Serial.print(" Gy: ");
//  Serial.print(Angle_pitch);
//  Serial.print(" Gz: ");
//  Serial.print(Angle_yaw);
//  Serial.print(" Acc: Ax: ");
//  Serial.print(rAccX);
//  Serial.print(" Ay: ");
//  Serial.print(rAccY);
//  Serial.print(" Az: ");
//  Serial.println(rAccZ);
//  Serial.print(" Total Acc: ");
//  Serial.print(acc_total_vector);

}




