#include <SPI.h>
#include <SD.h>

#include <MPU9250.h>

// Set the pins used
#define cardSelect 4
#define SETUP 0
#define ACQ 1
#define ERRORE -1

#define LED_PIN_ROSSO 13
#define LED_PIN_STATO 8
#define TEST_1 6
#define TEST_2 9

#define CPU_HZ 48000000
#define TIMER_PRESCALER_DIV 1024


void startTimer(int frequencyHz);
void setTimerFrequency(int frequencyHz);
void TC3_Handler();

const uint16_t FSAMPLE = 32;
bool isLEDOn = false;

int state = SETUP;
File logfile;
uint16_t buffer[FSAMPLE * 4];
uint16_t accXbuf[FSAMPLE * 4];
uint16_t accYbuf[FSAMPLE * 4];
uint16_t accZbuf[FSAMPLE * 4];
uint16_t gyroXbuf[FSAMPLE * 4];
uint16_t gyroYbuf[FSAMPLE * 4];
uint16_t gyroZbuf[FSAMPLE * 4];
uint16_t magXbuf[FSAMPLE * 4];
uint16_t magYbuf[FSAMPLE * 4];
uint16_t magZbuf[FSAMPLE * 4];

uint16_t i = 0;
uint16_t k = 0;
uint16_t j = 0;
long ttime = 0;
uint8_t flagSD = 0;
uint8_t w1 = 0;
uint8_t w2 = 0;
uint8_t undersample = 0;
uint16_t cont = 0;

uint16_t rampa = 0;

MPU9250 myIMU;

void setup() {
  Serial.begin(115200);
  //while (!Serial);
  pinMode(LED_PIN_ROSSO, OUTPUT);
  pinMode(LED_PIN_STATO, OUTPUT);
  pinMode(TEST_1, OUTPUT);
  pinMode(TEST_2, OUTPUT);
  digitalWrite(LED_PIN_ROSSO, LOW);
  digitalWrite(LED_PIN_STATO, LOW);
  digitalWrite(TEST_1, LOW);
  digitalWrite(TEST_2, LOW);

  Wire.begin();
  byte c = myIMU.readByte(0x68, WHO_AM_I_MPU9250);
  myIMU.MPU9250SelfTest(myIMU.SelfTest);
  myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);
  myIMU.initMPU9250();
  byte d = myIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
  myIMU.initAK8963(myIMU.magCalibration);

  for (i = 0; i < FSAMPLE; i++)
  {
    buffer[i] = 0;
  }
  i = 0;

  startTimer(FSAMPLE);
}


void loop() {
  // put your main code here, to run repeatedly:
  switch (state)
  {
    case SETUP:
      Serial.println("SETUP");
      if (!SD.begin(cardSelect)) {
        Serial.println("Card init. failed!");
        state = ERRORE;
        return;
      }
      Serial.println("Card initialized");
      char filename[15];
      strcpy(filename, "ANALOG00.CSV");
      for (uint8_t i = 0; i < 100; i++) {
        filename[6] = '0' + i / 10;
        filename[7] = '0' + i % 10;
        // create if does not exist, do not open existing, write, sync after write
        if (! SD.exists(filename)) {
          break;
        }
      }
      Serial.println("Check filename");
      logfile = SD.open(filename, FILE_WRITE);
      if ( ! logfile ) {
        Serial.print("Couldnt create ");
        Serial.println(filename);
        state = ERRORE;
        return;
      }
      Serial.print("Writing to ");
      Serial.println(filename);

      logfile.println("ACCX, ACCY, ACCZ, GYROX, GYROY, GYROZ, MAGX, MAGY, MAGZ");
      logfile.flush();
      Serial.println("Ready!");
      state = ACQ;
      break;
    case ACQ:
      if ((flagSD == 1) && (w1 == 1))
      {
        digitalWrite(TEST_2, HIGH);
        for (j = 0; j < FSAMPLE; j++) {
          logfile.print(accXbuf[j]);
          logfile.print(",");
          logfile.print(accYbuf[j]);
          logfile.print(",");
          logfile.print(accZbuf[j]);
          logfile.print(",");
          logfile.print(gyroXbuf[j]);
          logfile.print(",");
          logfile.print(gyroYbuf[j]);
          logfile.print(",");
          logfile.print(gyroZbuf[j]);
          logfile.print(",");
          logfile.print(magXbuf[j]);
          logfile.print(",");
          logfile.print(magYbuf[j]);
          logfile.print(",");
          logfile.println(magZbuf[j]);
        }
        logfile.flush();
        digitalWrite(TEST_2, LOW);
        w1 = 0;
      }
      else if ((flagSD == 2) && (w2 == 1))
      {
        digitalWrite(TEST_2, HIGH);
        for (j = FSAMPLE; j < FSAMPLE*2; j++) {
          logfile.print(accXbuf[j]);
          logfile.print(",");
          logfile.print(accYbuf[j]);
          logfile.print(",");
          logfile.print(accZbuf[j]);
          logfile.print(",");
          logfile.print(gyroXbuf[j]);
          logfile.print(",");
          logfile.print(gyroYbuf[j]);
          logfile.print(",");
          logfile.print(gyroZbuf[j]);
          logfile.print(",");
          logfile.print(magXbuf[j]);
          logfile.print(",");
          logfile.print(magYbuf[j]);
          logfile.print(",");
          logfile.println(magZbuf[j]);
        }
        logfile.flush();
        digitalWrite(TEST_2, LOW);
        w2 = 0;
      }
      break;
    case ERRORE:
      digitalWrite(LED_PIN_ROSSO, HIGH);
  }

}

void readData() {
  //buffer[k] = rampa;//analogRead(A1);

    if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
    {
      myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values
      myIMU.getAres();

      // Now we'll calculate the accleration value into actual g's
      // This depends on scale being set
      myIMU.ax = (float)myIMU.accelCount[0];//*myIMU.aRes; // - accelBias[0];
      myIMU.ay = (float)myIMU.accelCount[1];//*myIMU.aRes; // - accelBias[1];
      myIMU.az = (float)myIMU.accelCount[2];//*myIMU.aRes; // - accelBias[2];

      myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values
      myIMU.getGres();

      // Calculate the gyro value into actual degrees per second
      // This depends on scale being set
      myIMU.gx = (float)myIMU.gyroCount[0];//*myIMU.gRes;
      myIMU.gy = (float)myIMU.gyroCount[1];//*myIMU.gRes;
      myIMU.gz = (float)myIMU.gyroCount[2];//*myIMU.gRes;

      myIMU.readMagData(myIMU.magCount);  // Read the x/y/z adc values
      myIMU.getMres();
      // User environmental x-axis correction in milliGauss, should be
      // automatically calculated
      myIMU.magbias[0] = +470.;
      // User environmental x-axis correction in milliGauss TODO axis??
      myIMU.magbias[1] = +120.;
      // User environmental x-axis correction in milliGauss
      myIMU.magbias[2] = +125.;

      // Calculate the magnetometer values in milliGauss
      // Include factory calibration per data sheet and user environmental
      // corrections
      // Get actual magnetometer value, this depends on scale being set
      myIMU.mx = (float)myIMU.magCount[0];//*myIMU.mRes*myIMU.magCalibration[0] -
      //           myIMU.magbias[0];
      myIMU.my = (float)myIMU.magCount[1];//*myIMU.mRes*myIMU.magCalibration[1] -
      //           myIMU.magbias[1];
      myIMU.mz = (float)myIMU.magCount[2];;//*myIMU.mRes*myIMU.magCalibration[2] -
      //           myIMU.magbias[2];

      accXbuf[k] = myIMU.ax;
      accYbuf[k] = myIMU.ay;
      accZbuf[k] = myIMU.az;
      gyroXbuf[k] = myIMU.gx;
      gyroYbuf[k] = myIMU.gy;
      gyroZbuf[k] = myIMU.gz;
      magXbuf[k] = myIMU.mx;
      magYbuf[k] = myIMU.my;
      magZbuf[k] = myIMU.mz;

      Serial.print(myIMU.ax);
      Serial.print(",");
      Serial.print(myIMU.ay);
      Serial.print(",");
      Serial.println(myIMU.az);

    }
    myIMU.updateTime();
    k++;


  if ((k >= FSAMPLE) && (k < FSAMPLE*2))
  {
    flagSD = 1;
    if (k == FSAMPLE)
      w1 = 1;
  }
  else if (k >= FSAMPLE)
  {
    if (k == FSAMPLE*2)
      w2 = 1;
    flagSD = 2;
  }

  if (k >= FSAMPLE)
    k = 0;

}

void setTimerFrequency(int frequencyHz) {
  int compareValue = (CPU_HZ / (TIMER_PRESCALER_DIV * frequencyHz)) - 1;
  TcCount16* TC = (TcCount16*) TC3;
  // Make sure the count is in a proportional position to where it was
  // to prevent any jitter or disconnect when changing the compare value.
  TC->COUNT.reg = map(TC->COUNT.reg, 0, TC->CC[0].reg, 0, compareValue);
  TC->CC[0].reg = compareValue;
  Serial.println(TC->COUNT.reg);
  Serial.println(TC->CC[0].reg);
  while (TC->STATUS.bit.SYNCBUSY == 1);
}

void startTimer(int frequencyHz) {
  REG_GCLK_CLKCTRL = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_TCC2_TC3) ;
  while ( GCLK->STATUS.bit.SYNCBUSY == 1 ); // wait for sync

  TcCount16* TC = (TcCount16*) TC3;

  TC->CTRLA.reg &= ~TC_CTRLA_ENABLE;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  // Use the 16-bit timer
  TC->CTRLA.reg |= TC_CTRLA_MODE_COUNT16;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  // Use match mode so that the timer counter resets when the count matches the compare register
  TC->CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  // Set prescaler to 1024
  TC->CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1024;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  setTimerFrequency(frequencyHz);

  // Enable the compare interrupt
  TC->INTENSET.reg = 0;
  TC->INTENSET.bit.MC0 = 1;

  NVIC_EnableIRQ(TC3_IRQn);

  TC->CTRLA.reg |= TC_CTRLA_ENABLE;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync
}

void TC3_Handler() {
  TcCount16* TC = (TcCount16*) TC3;
  // If this interrupt is due to the compare register matching the timer count
  // we toggle the LED.
  if (TC->INTFLAG.bit.MC0 == 1) {
    TC->INTFLAG.bit.MC0 = 1;

    if (state != ACQ)
      return;
    readData();
    cont++;

    digitalWrite(TEST_1, !digitalRead(TEST_1));

    //LED BLINK
    if (cont >= FSAMPLE)
    {
      cont = 0;
      digitalWrite(LED_PIN_STATO, !digitalRead(LED_PIN_STATO));
      //      Serial.println(millis()-ttime);
      //      ttime = millis();
    }
    //TEST
    rampa++;
    if (rampa == 65535)
      rampa = 0;
    //FINETEST
  }

}
