/*
  Portenta - TestSDCARD

  The sketch shows how to mount an SDCARD and list its content.

  The circuit:
   - Portenta H7 + Vision Shield
   - Portenta H7 + Portenta Breakout

  This example code is in the public domain.
*/
#include "SDMMCBlockDevice.h"
#include "FATFileSystem.h"
#include <Wire.h>

// Basic demo for accelerometer readings from Adafruit MPU6050

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

const int adresse_du_capteur = 0x53; // Adresse I2C du capteur
const byte registre_a_lire = 0x33;   // Adresse du registre à lire (exemple)
unsigned long t = 0;

SDMMCBlockDevice block_device;
mbed::FATFileSystem fs("fs");
FILE *myFile;

void setup() {
  //Serial.begin(115200);

  //Serial.println("Mounting SDCARD...");
  int err =  fs.mount(&block_device);
  if (err) {
    // Reformat if we can't mount the filesystem
    // this should only happen on the first boot
    //Serial.println("No filesystem found, please check on computer and mually format");
   // err = fs.reformat(&block_device);  // seriously don't want to format good data
  }
  if (err) {
     //Serial.println("Error formatting SDCARD ");
     while(1);
  }
  
  DIR *dir;
  struct dirent *ent;
  int dirIndex = 0;

  //Serial.println("List SDCARD content: ");
  if ((dir = opendir("/fs")) != NULL) {
    // Print all the files and directories within directory (not recursively)
    while ((ent = readdir (dir)) != NULL) {
      //Serial.println(ent->d_name);
      dirIndex++;
    }
    closedir (dir);
  } else {
    // Could not open directory
    //Serial.println("Error opening SDCARD\n");
    while(1);
  }
  if(dirIndex == 0) {
    //Serial.println("Empty SDCARD");
  }

   //Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  while (!mpu.begin()) {
    //Serial.println("Failed to find MPU6050 chip");
    delay(100);
  }
 // Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
 /* Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }*/
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
 /* Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }*/
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  //Serial.print("Filter bandwidth set to: ");
  //Serial.println("");
  delay(100);
  //Serial.println("------------------------- Done --------------------------------");

  t=millis();
}

void loop() {

  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  char myFileName[] = "fs/test.txt";

  myFile = fopen(myFileName, "a");  // "a" for append or make it if file not there
  
  //  fprintf(myFile,"test \r\n");
    fprintf(myFile, "%d\t%f\t%f\t%f\t%f\t%f\t%f\n", t, a.acceleration.x,a.acceleration.y,a.acceleration.z,g.gyro.x,g.gyro.y,g.gyro.z);
  t = millis();
  //Serial.println(t);
  fclose(myFile);
  delay(100);

}
