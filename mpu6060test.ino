#include <Servo.h> 
//I2C library
#include <Wire.h>
Servo myservo;

// ================= Parameter MPU6050 ============================== 
//Declaring some global variables
int gyro_x, gyro_y, gyro_z;
long acc_x, acc_y, acc_z, acc_total_vector;
int temperature;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
long loop_timer;
float angle_pitch, angle_roll;
int angle_pitch_buffer, angle_roll_buffer;
boolean set_gyro_angles;
float angle_roll_acc, angle_pitch_acc;
float offset_angle_pitch_acc = 0;
float offset_angle_roll_acc = 0;
float angle_pitch_output, angle_roll_output;
int i = 1;
int N = 2500;//10000;
float sum_angle_pitch_acc, sum_angle_roll_acc;

// ================= Parameter Fuzzy ================================
// derajat keanggotaan x nilai pelayanan
//float x1 = 25;  // error
//float x2 = 34;  // derror
// parameter input
/*
float paramsNB[] = {0,0,12.5};        //error, derror
float paramsNS[] = {6.25, 18.75, 25};
float paramsZ[]  = {12.5, 25, 37.5};
float paramsPS[] = {25, 31.25, 43.75};
float paramsPB[] = {37.5, 50, 50};
*/
const float divider = 1;
float paramsNB[] = {0/divider,0/divider,12.5/divider};        //error, derror
float paramsNS[] = {6.25/divider, 18.75/divider, 25/divider};
float paramsZ[]  = {12.5/divider, 25/divider, 37.5/divider};
float paramsPS[] = {25/divider, 31.25/divider, 43.75/divider};
float paramsPB[] = {37.5/divider, 50/divider, 50/divider};

// parameter output
const float alfa_NB = 0;
const float alfa_NS = 2.5;
const float alfa_Z  = 5;
const float alfa_PS = 7.5;
const float alfa_PB = 10;
const float beta_NB = 0;
const float beta_NS = 1.25;
const float beta_Z  = 2.5;
const float beta_PS = 3.75;
const float beta_PB = 5;

float setpoint_x = 0;
float last_error_x = 0;
float error_x, derror_x;
float out_MPU_x = 0;        //pembacaan sensor;
float hip_servo = 90;
float buff_hip = 0;
unsigned long currentTime = 0;
unsigned long previousTime = 0;
char userInput;

struct mf
{
  float mu;
  byte sign;
};

struct out_fuzzy
{
  float alfa;
  float beta;
};

struct mf mf_tri(float x, float params[])
{
  struct mf mf_;
  float a = params[0];
  float b = params[1];
  float c = params[2];

  if(x<a)
  {
    mf_.mu = 0;
    mf_.sign = 1;
  }
  else
  {
    if(x<=b)
    {
      if(b==a)
      {
        mf_.mu = 1;
        mf_.sign = 1;
      }
      else
      {
        mf_.mu = (x-a)/(b-a);
        mf_.sign = 1;
      }
    }
    else
    {
      if(x<=c)
      {
        if(c==b)
        {
          mf_.mu = 1;
          mf_.sign = 1; 
        }
        else
        {
          mf_.mu = (c-x)/(c-b);
          mf_.sign = 1;
        }
      }
      else
      {
        mf_.mu = 0;
        mf_.sign = 1;
      } 
    }
  }

  return mf_;
}

 

void setup() {
  Wire.begin();                                                        //Start I2C as master
  Serial.begin(57600);                                               //Use only for debugging
  //myservo.attach(10);
  pinMode(13, OUTPUT);                                                 //Set output 13 (LED) as output
  
  setup_mpu_6050_registers();                                          //Setup the registers of the MPU-6050 (500dfs / +/-8g) and start the gyro
 
  digitalWrite(13, HIGH);                                              //Set digital output 13 high to indicate startup
  Serial.println("  MPU-6050 IMU");
  Serial.println("     V1.0");
  delay(1500);                                                         //Delay 1.5 second to display the text
  Serial.println("Calibrating gyro");
   
  for (int cal_int = 0; cal_int < 2000 ; cal_int ++){                  //Run this code 2000 times
    if(cal_int % 125 == 0)Serial.print(".");                              //Print a dot on the LCD every 125 readings
    read_mpu_6050_data();                                              //Read the raw acc and gyro data from the MPU-6050
    gyro_x_cal += gyro_x;                                              //Add the gyro x-axis offset to the gyro_x_cal variable
    gyro_y_cal += gyro_y;                                              //Add the gyro y-axis offset to the gyro_y_cal variable
    gyro_z_cal += gyro_z;                                              //Add the gyro z-axis offset to the gyro_z_cal variable
    delay(3);                                                          //Delay 3us to simulate the 250Hz program loop
  }
  gyro_x_cal /= 2000;                                                  //Divide the gyro_x_cal variable by 2000 to get the avarage offset
  gyro_y_cal /= 2000;                                                  //Divide the gyro_y_cal variable by 2000 to get the avarage offset
  gyro_z_cal /= 2000;                                                  //Divide the gyro_z_cal variable by 2000 to get the avarage offset
 
  Serial.print("Pitch:");                                                 //Print text to screen
  Serial.print("Roll :");                                                 //Print text to screen
  Serial.println();
  Serial.println("READYY");
  digitalWrite(13, LOW);                                               //All done, turn the LED off
   
  loop_timer = micros();                                               //Reset the loop timer

  
}
 
void loop(){
  struct out_fuzzy out;
  float tanda_er, tanda_der, tanda_mpu_x, tanda_mpu;
  int ready = 1;
  if(Serial.available()>0 && ready==1) // jika ada user input, nerima 'c', maka baca sensor trus kirim serialprint ke python !
  {
    userInput = Serial.read();        // membaca user input
    
    if(userInput == 'c')
    {
    
      previousTime = currentTime;
      currentTime = millis();
      // ====================================== Baca sensor MPU ===============================================================================
     
      read_mpu_6050_data();                                                //Read the raw acc and gyro data from the MPU-6050
     
      gyro_x -= gyro_x_cal;                                                //Subtract the offset calibration value from the raw gyro_x value
      gyro_y -= gyro_y_cal;                                                //Subtract the offset calibration value from the raw gyro_y value
      gyro_z -= gyro_z_cal;                                                //Subtract the offset calibration value from the raw gyro_z value
       
      //Gyro angle calculations
      //0.0000611 = 1 / (250Hz / 65.5)
      angle_pitch += gyro_x * 0.0000611;                                   //Calculate the traveled pitch angle and add this to the angle_pitch variable
      angle_roll += gyro_y * 0.0000611;                                    //Calculate the traveled roll angle and add this to the angle_roll variable
       
      //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
      angle_pitch += angle_roll * sin(gyro_z * 0.000001066);               //If the IMU has yawed transfer the roll angle to the pitch angel
      angle_roll -= angle_pitch * sin(gyro_z * 0.000001066);               //If the IMU has yawed transfer the pitch angle to the roll angel
       
      //Accelerometer angle calculations
      acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));  //Calculate the total accelerometer vector
      //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
      angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296;       //Calculate the pitch angle
      angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;       //Calculate the roll angle
  
      // Sebelum menjalankan program ini, jalankan terlebih dahulu program mpu6050_kalibrasi_offset.ino dan tempatkan mpu6050 di tempat datar untuk mendapatkan nilai offset
      // Masukkan nilai offset yang didapat dari program mpu6050_kalibrasi_offset.ino ke angle_pitch_acc dan angle_roll_acc
      angle_pitch_acc -= offset_angle_pitch_acc; //-2.32;                                              //Accelerometer calibration value for pitch
      angle_roll_acc -= offset_angle_roll_acc;   //-0.52;                                               //Accelerometer calibration value for roll
  
       
      if(set_gyro_angles){                                                 //If the IMU is already started
        angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;     //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
        angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;        //Correct the drift of the gyro roll angle with the accelerometer roll angle
      }
      else{                                                                //At first start
        angle_pitch = angle_pitch_acc;                                     //Set the gyro pitch angle equal to the accelerometer pitch angle 
        angle_roll = angle_roll_acc;                                       //Set the gyro roll angle equal to the accelerometer roll angle 
        set_gyro_angles = true;                                            //Set the IMU started flag
      }
       
      //To dampen the pitch and roll angles a complementary filter is used
      angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;   //Take 90% of the output pitch value and add 10% of the raw pitch value
      angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1;      //Take 90% of the output roll value and add 10% of the raw roll value
    
      // ====================================== Hitung Fuzzy Logic ============================================================================
      // sudut pitch MPU diolah menjadi error dan derror
      // input fuzzy  : error dan derror
      // output fuzzy : alfa dan beta
      // NOTE: alfa = offset untuk sudut pitch hip. beta = offset untuk sumbu x (artinya perlu IK untuk convert posisi ke sudut)
      
      // pitch angle
      out_MPU_x = angle_pitch_output;   //random(-50, 50);  // pembacaan sensor
      
      if (out_MPU_x>0) {tanda_mpu_x = 1;tanda_mpu = 1;}
      else {tanda_mpu_x = -1; tanda_mpu = 2;}
      
      //out_MPU_x = random(-50, 50);  // pembacaan sensor

      if(out_MPU_x != setpoint_x)
      {
        error_x = setpoint_x - out_MPU_x;
        //last_error_x = error_x;
        
        if (error_x>0)tanda_er=1; //>=
        else tanda_er=-1;
        error_x = abs(error_x);     // input fuzzy harus positif
        
        derror_x = error_x - last_error_x;
        if (derror_x>0)tanda_der=1; //>=
        else tanda_der=-1;
        derror_x = abs(derror_x);   // input fuzzy harus positif
    
        if(error_x  >50/divider) error_x  = 50/divider;
        else if(error_x<=0) error_x = 0;
        else error_x = error_x;
        if(derror_x >50/divider) derror_x = 50/divider;
        else if(derror_x<=0) derror_x = 0;
        else derror_x = derror_x;
      
        out = fuzzy_sugeno(error_x,derror_x); // Fuzzy Logic Controller
    
        //hip_servo = hip_servo + tanda_er*out.alfa;
        hip_servo = hip_servo + tanda_mpu_x*out.alfa;
        last_error_x = tanda_er*out.alfa;
      }
      else {}
    
      if (hip_servo >= 180) {hip_servo = 180;}
      else if (hip_servo <=0  ) {hip_servo = 0;}
      else {hip_servo = hip_servo;}
        
      int alfa_converted = out.alfa/360*1024;
      int beta_converted = out.beta/360*1024; // HARUS dikonversi pake inverse kinematik!
      
      Serial.print(alfa_converted);Serial.print("a");
      Serial.print("-");
      Serial.print(beta_converted);Serial.print("b");
      Serial.print("-");
      Serial.print(currentTime-previousTime);Serial.print("ms");
      Serial.print("-");
      Serial.print(tanda_mpu);Serial.print("d");
      Serial.print("-");
      Serial.print(abs(out_MPU_x));Serial.println("e");
    
      // ====================================== Kirim Serial ke Raspi =========================================================================
      // misal dibikin 100Hz, berati variabel dibawah diganti jadi 10000
      //while(micros() - loop_timer < 10000);                                 //Wait until the loop_timer reaches 4000us (250Hz) before starting the next loop

      while(micros() - loop_timer < 4000);                                 //Wait until the loop_timer reaches 4000us (250Hz) before starting the next loop
      loop_timer = micros();                                               //Reset the loop timer
   
    } 
    else if(userInput == 'k') // Masuk mode Kalibrasi MPU
    { 
      // ==================== MODE KALIBRASI MPU =====================
      int kalibrasi;
      kalibrasi = 1;
      i = 1;
      offset_angle_pitch_acc = 0;
      offset_angle_roll_acc = 0;
      sum_angle_pitch_acc = 0;
      sum_angle_roll_acc = 0;
      while(kalibrasi==1)
      {
        previousTime = currentTime;
        currentTime = millis();
        // ====================================== Mode Kalibrasi MPU ===============================================================================
        read_mpu_6050_data();                                                //Read the raw acc and gyro data from the MPU-6050
     
        gyro_x -= gyro_x_cal;                                                //Subtract the offset calibration value from the raw gyro_x value
        gyro_y -= gyro_y_cal;                                                //Subtract the offset calibration value from the raw gyro_y value
        gyro_z -= gyro_z_cal;                                                //Subtract the offset calibration value from the raw gyro_z value
         
        //Gyro angle calculations
        //0.0000611 = 1 / (250Hz / 65.5)
        angle_pitch += gyro_x * 0.0000611;                                   //Calculate the traveled pitch angle and add this to the angle_pitch variable
        angle_roll += gyro_y * 0.0000611;                                    //Calculate the traveled roll angle and add this to the angle_roll variable
         
        //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
        angle_pitch += angle_roll * sin(gyro_z * 0.000001066);               //If the IMU has yawed transfer the roll angle to the pitch angel
        angle_roll -= angle_pitch * sin(gyro_z * 0.000001066);               //If the IMU has yawed transfer the pitch angle to the roll angel
         
        //Accelerometer angle calculations
        acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));  //Calculate the total accelerometer vector
        //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
        angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296;       //Calculate the pitch angle
        angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;       //Calculate the roll angle
         
        while(micros() - loop_timer < 4000);                                 //Wait until the loop_timer reaches 4000us (250Hz) before starting the next loop
        loop_timer = micros();                                               //Reset the loop timer

        sum_angle_pitch_acc = sum_angle_pitch_acc + angle_pitch_acc;
        sum_angle_roll_acc = sum_angle_roll_acc + angle_roll_acc;
       
        if(i==N)
        {
          int angle_pitch_acc_buff = angle_pitch_acc;
          int angle_roll_acc_buff = angle_roll_acc; // HARUS dikonversi pake inverse kinematik!

          // hitung nilai offset baru MPU
          offset_angle_pitch_acc = sum_angle_pitch_acc/N;
          offset_angle_roll_acc = sum_angle_roll_acc/N;
          
          // Offset baru sudah didapatkan, kirim ke raspi
          
          Serial.print(abs(offset_angle_pitch_acc));Serial.print("p");
          Serial.print("-");
          Serial.print(abs(offset_angle_roll_acc));Serial.print("r");
          Serial.print("-");
          Serial.print(i);Serial.print("i");
          Serial.print("-");
          Serial.print(currentTime-previousTime);Serial.println("ms");
          // Reset parameter untuk kalibrasi
          kalibrasi = 2;
          i = 0;
          sum_angle_pitch_acc = 0;
          sum_angle_roll_acc = 0;
        }
        else
        {
          int angle_pitch_acc_buff = angle_pitch_acc;
          int angle_roll_acc_buff = angle_roll_acc; // HARUS dikonversi pake inverse kinematik!
          
          Serial.print(abs(angle_pitch_acc));Serial.print("g");
          Serial.print("-");
          Serial.print(abs(angle_roll_acc));Serial.print("h");
          Serial.print("-");
          Serial.print(i);Serial.print("i");
          Serial.print("-");
          Serial.print(currentTime-previousTime);Serial.println("ms");
         }
        i++;
      }
    }
    
  }
  else  // jika tidak ada user input, sensor tetep baca terus, tapi gak kirim serialprint ke python!
  {
      previousTime = currentTime;
      currentTime = millis();
      // ====================================== Baca sensor MPU ===============================================================================
     
      read_mpu_6050_data();                                                //Read the raw acc and gyro data from the MPU-6050
     
      gyro_x -= gyro_x_cal;                                                //Subtract the offset calibration value from the raw gyro_x value
      gyro_y -= gyro_y_cal;                                                //Subtract the offset calibration value from the raw gyro_y value
      gyro_z -= gyro_z_cal;                                                //Subtract the offset calibration value from the raw gyro_z value
       
      //Gyro angle calculations
      //0.0000611 = 1 / (250Hz / 65.5)
      angle_pitch += gyro_x * 0.0000611;                                   //Calculate the traveled pitch angle and add this to the angle_pitch variable
      angle_roll += gyro_y * 0.0000611;                                    //Calculate the traveled roll angle and add this to the angle_roll variable
       
      //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
      angle_pitch += angle_roll * sin(gyro_z * 0.000001066);               //If the IMU has yawed transfer the roll angle to the pitch angel
      angle_roll -= angle_pitch * sin(gyro_z * 0.000001066);               //If the IMU has yawed transfer the pitch angle to the roll angel
       
      //Accelerometer angle calculations
      acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));  //Calculate the total accelerometer vector
      //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
      angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296;       //Calculate the pitch angle
      angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;       //Calculate the roll angle
  
      // Sebelum menjalankan program ini, jalankan terlebih dahulu program mpu6050_kalibrasi_offset.ino dan tempatkan mpu6050 di tempat datar untuk mendapatkan nilai offset
      // Masukkan nilai offset yang didapat dari program mpu6050_kalibrasi_offset.ino ke angle_pitch_acc dan angle_roll_acc
      angle_pitch_acc -= offset_angle_pitch_acc; //-2.32;                                              //Accelerometer calibration value for pitch
      angle_roll_acc -= offset_angle_roll_acc;   //-0.52;                                               //Accelerometer calibration value for roll
  
       
      if(set_gyro_angles){                                                 //If the IMU is already started
        angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;     //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
        angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;        //Correct the drift of the gyro roll angle with the accelerometer roll angle
      }
      else{                                                                //At first start
        angle_pitch = angle_pitch_acc;                                     //Set the gyro pitch angle equal to the accelerometer pitch angle 
        angle_roll = angle_roll_acc;                                       //Set the gyro roll angle equal to the accelerometer roll angle 
        set_gyro_angles = true;                                            //Set the IMU started flag
      }
       
      //To dampen the pitch and roll angles a complementary filter is used
      angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;   //Take 90% of the output pitch value and add 10% of the raw pitch value
      angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1;      //Take 90% of the output roll value and add 10% of the raw roll value
    
      // ====================================== Hitung Fuzzy Logic ============================================================================
      // sudut pitch MPU diolah menjadi error dan derror
      // input fuzzy  : error dan derror
      // output fuzzy : alfa dan beta
      // NOTE: alfa = offset untuk sudut pitch hip. beta = offset untuk sumbu x (artinya perlu IK untuk convert posisi ke sudut)
      
      // pitch angle
      out_MPU_x = angle_pitch_output;   //random(-50, 50);  // pembacaan sensor
      
      if (out_MPU_x>0) {tanda_mpu_x = 1;tanda_mpu = 1;}
      else {tanda_mpu_x = -1; tanda_mpu = 2;}
      
      //out_MPU_x = random(-50, 50);  // pembacaan sensor

      if(out_MPU_x != setpoint_x)
      {
        error_x = setpoint_x - out_MPU_x;
        //last_error_x = error_x;
        
        if (error_x>0)tanda_er=1; //>=
        else tanda_er=-1;
        error_x = abs(error_x);     // input fuzzy harus positif
        
        derror_x = error_x - last_error_x;
        if (derror_x>0)tanda_der=1; //>=
        else tanda_der=-1;
        derror_x = abs(derror_x);   // input fuzzy harus positif
    
        if(error_x  >50/divider) error_x  = 50/divider;
        else if(error_x<=0) error_x = 0;
        else error_x = error_x;
        if(derror_x >50/divider) derror_x = 50/divider;
        else if(derror_x<=0) derror_x = 0;
        else derror_x = derror_x;
      
        out = fuzzy_sugeno(error_x,derror_x); // Fuzzy Logic Controller
    
        //hip_servo = hip_servo + tanda_er*out.alfa;
        hip_servo = hip_servo + tanda_mpu_x*out.alfa;
        last_error_x = tanda_er*out.alfa;
      }
      else {}
    
      if (hip_servo >= 180) {hip_servo = 180;}
      else if (hip_servo <=0  ) {hip_servo = 0;}
      else {hip_servo = hip_servo;}      
    
      int alfa_converted = out.alfa/360*1024;
      int beta_converted = out.beta/360*1024; // HARUS dikonversi pake inverse kinematik!
      
      // ====================================== Kirim Serial ke Raspi =========================================================================
      // misal dibikin 100Hz, berati variabel dibawah diganti jadi 10000
      //while(micros() - loop_timer < 10000);                                 //Wait until the loop_timer reaches 4000us (250Hz) before starting the next loop

      while(micros() - loop_timer < 4000);                                 //Wait until the loop_timer reaches 4000us (250Hz) before starting the next loop
      loop_timer = micros(); 
    
    
   }
}
 
 
void read_mpu_6050_data(){                                             //Subroutine for reading the raw gyro and accelerometer data
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x3B);                                                    //Send the requested starting register
  Wire.endTransmission();                                              //End the transmission
  Wire.requestFrom(0x68,14);                                           //Request 14 bytes from the MPU-6050
  while(Wire.available() < 14);                                        //Wait until all the bytes are received
  acc_x = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_x variable
  acc_y = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_y variable
  acc_z = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_z variable
  temperature = Wire.read()<<8|Wire.read();                            //Add the low and high byte to the temperature variable
  gyro_x = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_x variable
  gyro_y = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_y variable
  gyro_z = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_z variable
 
}
 
void setup_mpu_6050_registers(){
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

struct out_fuzzy fuzzy_sugeno(float x1, float x2)
{
  // NOTE PENTING!: x1 = error, x2 = derror
  struct out_fuzzy out_fuzzy_;
  struct mf mf_errNB,mf_errNS,mf_errZ,mf_errPS,mf_errPB,mf_derrNB,mf_derrNS,mf_derrZ,mf_derrPS,mf_derrPB;

  // STEP 1 : Membuat himpunan fuzzy dan input fuzzy
  // Membership Function atau Fungsi Keanggotaan
  // error
  mf_errNB = mf_tri(x1,paramsNB);
  mf_errNS = mf_tri(x1,paramsNS);
  mf_errZ  = mf_tri(x1,paramsZ );
  mf_errPS = mf_tri(x1,paramsPS);
  mf_errPB = mf_tri(x1,paramsPB);
  // derror
  mf_derrNB = mf_tri(x2,paramsNB);
  mf_derrNS = mf_tri(x2,paramsNS);
  mf_derrZ  = mf_tri(x2,paramsZ );
  mf_derrPS = mf_tri(x2,paramsPS);
  mf_derrPB = mf_tri(x2,paramsPB);

  float mf_errall[] = {mf_errNB.mu, mf_errNS.mu, mf_errZ.mu, mf_errPS.mu, mf_errPB.mu};
  float mf_derrall[] = {mf_derrNB.mu, mf_derrNS.mu, mf_derrZ.mu, mf_derrPS.mu, mf_derrPB.mu};

   /* 
  // Tampilkan nilai mf error & derror
  for (byte i=0;i<5;i++) 
  {
    Serial.print(mf_errall[i]);
    Serial.print("\t");
  }
  Serial.println();
  for (byte i=0;i<5;i++) 
  {
    Serial.print(mf_derrall[i]);
    Serial.print("\t");
  }
  Serial.println();
  Serial.println();
*/

  float alfa[25];
  float za[25];
  float zb[25];

  //  STEP 2 : Menerapkan operator Fuzzy dan Fungsi Implikasi
  //  Aturan ke-1
  if (mf_errNB.sign==1 || mf_derrNB.sign==1)
  {
    alfa[0] = min(mf_errNB.mu,mf_derrNB.mu);
    za[0] = alfa_NB;
    zb[0] = beta_NB;
  }
  // Aturan ke-2
  if (mf_errNB.sign==1 || mf_derrNS.sign==1)
  {
    alfa[1] = min(mf_errNB.mu,mf_derrNS.mu);
    za[1] = alfa_NB;
    zb[1] = beta_NB;
  }
  // Aturan ke-3
  if (mf_errNB.sign==1 || mf_derrZ.sign==1)
  {
    alfa[2] = min(mf_errNB.mu,mf_derrZ.mu);
    za[2] = alfa_NS;
    zb[2] = beta_NS;
  }
  // Aturan ke-4
  if (mf_errNB.sign==1 || mf_derrPS.sign==1)
  {
    alfa[3] = min(mf_errNB.mu,mf_derrPS.mu);
    za[3] = alfa_NS;
    zb[3] = beta_NS;
  }
  // Aturan ke-5
  if (mf_errNB.sign==1 || mf_derrPB.sign==1)
  {
    alfa[4] = min(mf_errNB.mu,mf_derrPB.mu);
    za[4] = alfa_Z;
    zb[4] = beta_Z;
  }
  // Aturan ke-6
  if (mf_errNS.sign==1 || mf_derrNB.sign==1)
  {
    alfa[5] = min(mf_errNS.mu,mf_derrNB.mu);
    za[5] = alfa_NB;
    zb[5] = beta_NB;
  }
  // Aturan ke-7
  if (mf_errNS.sign==1 || mf_derrNS.sign==1)
  {
    alfa[6] = min(mf_errNS.mu,mf_derrNS.mu);
    za[6] = alfa_NS;
    zb[6] = beta_NS;
  }
  // Aturan ke-8
  if (mf_errNS.sign==1 || mf_derrZ.sign==1)
  {
    alfa[7] = min(mf_errNS.mu,mf_derrZ.mu);
    za[7] = alfa_NS;
    zb[7] = beta_NS;
  }
  // Aturan ke-9
  if (mf_errNS.sign==1 || mf_derrPS.sign==1)
  {
    alfa[8] = min(mf_errNS.mu,mf_derrPS.mu);
    za[8] = alfa_Z;
    zb[8] = beta_Z;
  }
  // Aturan ke-10
  if (mf_errNS.sign==1 || mf_derrPB.sign==1)
  {    
    alfa[9] = min(mf_errNS.mu,mf_derrPB.mu);
    za[9] = alfa_PS;
    zb[9] = beta_PS;
  }
  // Aturan ke-11
  if (mf_errZ.sign==1 || mf_derrNB.sign==1)
  {
    alfa[10] = min(mf_errZ.mu,mf_derrNB.mu);
    za[10] = alfa_NS;
    zb[10] = beta_NS;
  }
  // Aturan ke-12
  if (mf_errZ.sign==1 || mf_derrNS.sign==1)
  {
    alfa[11] = min(mf_errZ.mu,mf_derrNS.mu);
    za[11] = alfa_NS;
    zb[11] = beta_NS;
  }
  // Aturan ke-13
  if (mf_errZ.sign==1 || mf_derrZ.sign==1)
  {
    alfa[12] = min(mf_errZ.mu,mf_derrZ.mu);
    za[12] = alfa_Z;
    zb[12] = beta_Z;
  }
  // Aturan ke-14
  if (mf_errZ.sign==1 || mf_derrPS.sign==1)
  {
    alfa[13] = min(mf_errZ.mu,mf_derrPS.mu);
    za[13] = alfa_PS;
    zb[13] = beta_PS;
  }
  // Aturan ke-15
  if (mf_errZ.sign==1 || mf_derrPB.sign==1)
  {
    alfa[14] = min(mf_errZ.mu,mf_derrPB.mu);
    za[14] = alfa_PS;
    zb[14] = beta_PS;
  }
  // Aturan ke-16
  if (mf_errPS.sign==1 || mf_derrNB.sign==1)
  {
    alfa[15] = min(mf_errPS.mu,mf_derrNB.mu);
    za[15] = alfa_NS;
    zb[15] = beta_NS;
  }
  // Aturan ke-17
  if (mf_errPS.sign==1 || mf_derrNS.sign==1)
  {
    alfa[16] = min(mf_errPS.mu,mf_derrNS.mu);
    za[16] = alfa_Z;
    zb[16] = beta_Z;
  }
  // Aturan ke-18
  if (mf_errPS.sign==1 || mf_derrZ.sign==1)
  {    
    alfa[17] = min(mf_errPS.mu,mf_derrZ.mu);
    za[17] = alfa_PS;
    zb[17] = beta_PS;
  }
  // Aturan ke-19
  if (mf_errPS.sign==1 || mf_derrPS.sign==1)
  {    
    alfa[18] = min(mf_errPS.mu,mf_derrPS.mu);
    za[18] = alfa_PS;
    zb[18] = beta_PS;
  }
  // Aturan ke-20
  if (mf_errPS.sign==1 || mf_derrPB.sign==1)
  {    
    alfa[19] = min(mf_errPS.mu,mf_derrPB.mu);
    za[19] = alfa_PB;
    zb[19] = beta_PB;
  }
  // Aturan ke-21
  if (mf_errPB.sign==1 || mf_derrNB.sign==1)
  {    
    alfa[20] = min(mf_errPB.mu,mf_derrNB.mu);
    za[20] = alfa_Z;
    zb[20] = beta_Z;
  }
  // Aturan ke-22
  if (mf_errPB.sign==1 || mf_derrNS.sign==1)
  {    
    alfa[21] = min(mf_errPB.mu,mf_derrNS.mu);
    za[21] = alfa_PS;
    zb[21] = beta_PS;
  }
  // Aturan ke-23
  if (mf_errPB.sign==1 || mf_derrZ.sign==1)
  {    
    alfa[22] = min(mf_errPB.mu,mf_derrZ.mu);
    za[22] = alfa_PS;
    zb[22] = beta_PS;
  }
  // Aturan ke-24
  if (mf_errPB.sign==1 || mf_derrPS.sign==1)
  {    
    alfa[23] = min(mf_errPB.mu,mf_derrPS.mu);
    za[23] = alfa_PB;
    zb[23] = beta_PB;
  }
  // Aturan ke-25
  if (mf_errPB.sign==1 || mf_derrPB.sign==1)
  {    
    alfa[24] = min(mf_errPB.mu,mf_derrPB.mu);
    za[24] = alfa_PB;
    zb[24] = beta_PB;
  }

  /*
  // Tampilkan nilai alfa, za, zb
  Serial.println("alfa = ");
  for (byte i=0;i<25;i++) 
  {
    Serial.print(alfa[i]);
    Serial.print("\t");
  }
  
  Serial.println();
  Serial.println("za = ");
  for (byte i=0;i<25;i++) 
  {
    Serial.print(za[i]);
    Serial.print("\t");
  }
  Serial.println();
  
  Serial.println("zb = ");
  for (byte i=0;i<25;i++) 
  {
    Serial.print(zb[i]);
    Serial.print("\t");
  }
  Serial.println();
  delay(1000);
  */


  // ====================================================================
  // STEP 5: Defuzzyfikasi
  // Pada sugeno, lebih sederhana. Hanya menghitung center of singleton!
  // Hitung center of singleton
  float sum_singleton_a=0;
  float sum_singleton_b=0;
  float sum_alfa=0;
  for (byte i=0;i<25;i++)
  {
      sum_singleton_a=sum_singleton_a+za[i]*alfa[i];
      sum_singleton_b=sum_singleton_b+zb[i]*alfa[i];
  }
  for (byte i=0;i<25;i++)
  {
      sum_alfa=sum_alfa+alfa[i];
  }
  out_fuzzy_.alfa = sum_singleton_a/sum_alfa;
  out_fuzzy_.beta = sum_singleton_b/sum_alfa;
//  Serial.print(out_alfa);Serial.print("\t");
//  Serial.println(out_beta);
//  Serial.println(sizeof(za));
//  delay(1000);
  return out_fuzzy_;
}
