#include<Wire.h>

// Déclation des variables
const int MPU_addr=0x69; // I2C address of the MPU-6050
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
byte MAX_CALIB_LOOP = 100;
byte calibration_loop = MAX_CALIB_LOOP;
bool calibration_bit = true;
const int16_t delay_per_loop = 100;
const int16_t num_of_calibration_turn = 1000/delay_per_loop;
int16_t calibration_tab[6][num_of_calibration_turn]
int16_t calibration_index = 0;
int16_t time_elapsed_in_seconds = 0;
unsigned long StartTime;

typedef struct{
     int16_t max_drift_val;
     int16_t min_drift_val;
     int16_t val;
}metric;

// table containing variables in this order : AcX, AcY, AcZ, GyX, GyY, GyZ
metric metrics[6] = {{0, 0, 0},{0, 0, 0},{0, 0, 0},{0, 0, 0},{0, 0, 0},{0, 0, 0}};
char *sensor_names[] = {"AcX", "AcY", "AcZ", "GyX", "GyY", "GyZ"};
void init_calibration_values(metric (&values)[6]);
void calibrate_sensor_drift(metric (&metrics)[6]);
void display_metrics(metric (&metrics)[6]);
void display_drift(metric (&metrics)[6]);
void setup(){
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.begin(115200);
}

void loop(){
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true); // request a total of 14 registers
  metrics[0].val=Wire.read()<<8|Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  metrics[1].val=Wire.read()<<8|Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  metrics[2].val=Wire.read()<<8|Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  metrics[3].val=Wire.read()<<8|Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  metrics[4].val=Wire.read()<<8|Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  metrics[5].val=Wire.read()<<8|Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  if (calibration_bit) {
    if (calibration_loop <= MAX_CALIB_LOOP) {
      if (calibration_loop == MAX_CALIB_LOOP) {
        Serial.println("Calibrating standard drift, please don't move...");
        init_calibration_values(metrics);
      }
      else {
        calibrate_sensor_drift(metrics);
      }
      calibration_loop = calibration_loop - 1;
      //Serial.print("calibration_loop = ");Serial.println(calibration_loop);
    }
    else {
      display_drift(metrics);
      calibration_bit = false;
      StartTime = millis();
    }
  }
  else {
    unsigned long CurrentTime = millis();
    unsigned long ElapsedTime = CurrentTime - StartTime;
    if (ElapsedTime > (time_elapsed_in_seconds+1) * 1000) {
      time_elapsed_in_seconds ++;
      //Serial.print("And one more second 'til the end, ");Serial.print(time_elapsed_in_seconds); Serial.println(" seconds");
      display_metrics(metrics);
    }
  }
  
  delay(delay_per_loop);
}

void init_calibration_values(metric (&metrics)[6]){
  for(int i = 0; i < 6 ; i ++) {
    metrics[i].max_drift_val = metrics[i].val;
    metrics[i].min_drift_val = metrics[i].val;
  }
}

void calibrate_sensor_drift(metric (&metrics)[6]) {
  for(int i = 0; i < 6 ; i ++) {
    if ( metrics[i].max_drift_val < metrics[i].val )
      metrics[i].max_drift_val = metrics[i].val;
    if ( metrics[i].min_drift_val > metrics[i].val )
      metrics[i].min_drift_val = metrics[i].val;
  }
}
void display_drift(metric (&metrics)[6]) {
  Serial.println("Calibration Finished, order : AcX, AcY, AcZ, GyX, GyY, GyZ");
  Serial.println();
  for(int i = 0; i < 6 ; i ++) {
    Serial.print("min = ");Serial.print(metrics[i].min_drift_val);Serial.print(", max = ");Serial.println(metrics[i].max_drift_val);
  }
}
void display_metrics(metric (&metrics)[6]) {
  for(int i = 0; i < 6 ; i ++) {
    Serial.print("sensor ");Serial.print(sensor_names[i]);Serial.print(" : ");Serial.println(metrics[i].val);
  }
  Serial.println();
}
