#include <Wire.h>

#define ACC_ADDR       0x53
#define INT_ACT        0
#define NUM_DATA_REGS  6

#define REG_DEVID   0x00
#define REG_INTENB  0x2E
#define REG_INTMAP  0x2F
#define REG_PWRCTL  0x2D
#define REG_DATFMT  0x31
#define REG_DATAX0  0x32
#define REG_DATAX1  0x33
#define REG_DATAY0  0x34
#define REG_DATAY1  0x35
#define REG_DATAZ0  0x36
#define REG_DATAZ1  0x37

// Feature Enable
#define ARD_SERIALDEBUG 0

// data registers
int X[3] = {
  0, 0, 0};
int Y[3] = {
  0, 0, 0};
int Z[3] = {
  0, 0, 0};

char str[512];

void setup() {
  // put your setup code here, to run once:
  int dev_id = 0;

  // Init I2C library
  Wire.begin();
  Serial.begin(9600);
  Serial.println("Accessing Accelerometer Interface.");

  // Verify proper device id
  dev_id = i2c_read_control(ACC_ADDR, REG_DEVID, 1);
  Serial.print("Device ID = 0x");
  Serial.println(dev_id, HEX);


  // Configure data format +/- 2g, 13-bit mode
  i2c_write(ACC_ADDR, REG_DATFMT, 0x08);

  // Power control, enable measurements
  i2c_write(ACC_ADDR, REG_PWRCTL, 0x08);
}

int i2c_read_control(unsigned char addr, unsigned char reg, unsigned int nbytes) {

  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(addr, nbytes);
  while(Wire.available() == 0);

  return Wire.read();

}

void i2c_read_dataxyz(unsigned char addr, unsigned char reg) {

  Wire.beginTransmission(addr);
  Wire.write(reg);  
  Wire.endTransmission();

  Wire.requestFrom(addr, (unsigned int)NUM_DATA_REGS);
  while(Wire.available() == 0);

  X[0] = Wire.read();
  X[1] = Wire.read();
  Y[0] = Wire.read();
  Y[1] = Wire.read();
  Z[0] = Wire.read();
  Z[1] = Wire.read();


  X[2] = (X[1] << 8) | X[0];
  Y[2] = (Y[1] << 8) | Y[0];
  Z[2] = (Z[1] << 8) | Z[0];

}

int i2c_write(unsigned char addr, unsigned char reg, unsigned int data) {

  int verify = 0;
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();

  // verify write
  verify = i2c_read_control(addr, reg, 1);
  if(verify != data) {
    Serial.print("I2C Bus Write Error. Expected = 0x");
    Serial.print(data, HEX);
    Serial.print(" Received = 0x");
    Serial.println(verify, HEX);
  }
  else {
    Serial.print("Reg 0x");
    Serial.print(reg, HEX);
    Serial.println(" setup successfully.");
  }

}

void loop() {
  // put your main code here, to run repeatedly: 

  i2c_read_dataxyz(ACC_ADDR, REG_DATAX0);


#if ARD_SERIALDEBUG
  Serial.print("X-AXIS = ");
  Serial.print(float(X[2] * 3.9), 2);

  Serial.print("     Y-AXIS = ");
  Serial.print(Y[2] * 3.9, 2);

  Serial.print("     Z-AXIS = ");
  Serial.println(Z[2] * 3.9, 2);
#else
  // Send the data to Processing / Matlab
  sprintf(str, "%d %d %d", X[2], Y[2], Z[2]);
  Serial.print(str);
  Serial.write(10);
  delay(20);
#endif




}

