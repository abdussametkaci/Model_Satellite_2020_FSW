#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <Adafruit_BMP280.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Servo.h>

#include <EEPROM.h>

#define HOUR_OFFSET +3
#define VOLTAGE_PIN A0

Adafruit_BMP280 bmp; // I2C Interface

SoftwareSerial mySerial(8, 7); // rx: 8, tx: 7
Adafruit_GPS GPS(&mySerial);

//SoftwareSerial dataSerial(10, 11); // rx: 10, tx: 11

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

Servo servo;
Servo ESC;

bool hasPassword = false;
bool isBMP280Begin = false;
bool isBNO055Begin = false;
bool isGPSBegin = false;

typedef struct {
  bool isGround = false;
  bool isClimbing = false;
  bool isLimitHeight = false;
  bool isFalling = false;
  bool isLeaving = false;
} status_t;

typedef enum {
  _GROUND,
  _CLIMBING,
  _LIMIT_HEIGHT,
  _FALLING,
  _LEAVING,
  _AFTER_LEAVING,
  _WATING_RESCUE
} SATELLITE_STATUS;

typedef struct {
  const char* takim_no = "57413";
  unsigned short paket_numarasi = 0;
  String gonderme_zamani;
  float basinc;
  float yukseklik;
  float inis_hizi;
  float sicaklik;
  float pil_gerilimi;
  float gps_latitude;
  float gps_longitude;
  float gps_altitude;
  SATELLITE_STATUS uydu_statusu;
  float pitch;
  float roll;
  float yaw;
  unsigned short donus_sayisi = 0;
  String video_aktarim_bilgisi = "Hayir";
} telemetry_t;

typedef struct {
  float x;
  float y;
  float z;
} acceleration_t;

telemetry_t telemetry;
status_t statu;
acceleration_t acc;
float firstPressure = 0;
char komut;
unsigned short toplam_donus = 0;
float currentYaw = 0;
float oncekiYukseklik = 0;
float firstVelocity = 0;
uint32_t t0 = 0;
int pwm_signal = 25;
void setup() {
  Serial.begin(9600);
  //dataSerial.begin(9600);

  // GPS setup
  isGPSBegin = GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000);
  // Ask for firmware version
  mySerial.println(PMTK_Q_RELEASE);
  // END GPGS setup

  // BMP280 setup
  //Serial.println(F("BMP280 test"));
  isBMP280Begin = bmp.begin(0x76);

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_250); /* Standby time. */

  for (int i = 0; i < 5; i++) { // cop data onlemi
    bmp.readPressure();
  }

  for (int i = 0; i < 10; i++) {          // average pressure
    firstPressure += bmp.readPressure() / 100;
  }

  firstPressure /= 10;

  // END BPM280 setup

  // BNO055 setup
  isBNO055Begin = bno.begin();

  bno.setExtCrystalUse(true);
  // END BNO055 setup

  // ESC setup
  // Attach the ESC on pin 5 -> PWM pin olmali
  ESC.attach(5, 1000, 2000); // (pin, min pulse width, max pulse width in microseconds)
  motoruCalistir(0);
  // END ESC setup

  // servo setup
  servo.attach(6); // bagli oldugu pin PWM ckikisli pin olmali
  servo.write(0); // baslangic ayarlanabilir !!!
  // END servo setup

  // buzzer setup
  pinMode(9, OUTPUT); // pine bak
  //END buzzer setup

  //EEPROM setup
  EEPROM.get(0, telemetry.paket_numarasi);
  //END EEPROM

}

uint32_t timer = millis();
uint32_t t = millis() / 1000;

void loop() {
  if (hasPassword) {
    if (isGPSBegin) {
      runGPS();
    }

    if (millis() - timer > 1000) { // yaklasik 1 saniye
      timer = millis(); // reset the timer
      
      if (isBMP280Begin) {
        runBMP280();
      }

      if (isBNO055Begin) {
        runBNO055();
      }
      
      computeDonusSayisi();
      findStatus(1, 700, 400);
      computeVoltage();
      
      if (isGPSBegin) {
        GPSData();
      }

      Serial.println("OK");
      printTelemetry();
    }

    telemetry.gonderme_zamani = "";
  } else {
    Serial.print('c'); // c -> control that file exists
  }

  // control the received serial port
  if (Serial.available()) {
    komut = Serial.read();
    komutIslet(komut);
  }
}

void runGPS() {
  //GPS loop
  GPS.read();
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false

    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  // END GPS loop
}

// bu fonksiyon belli zaman araliginda calismali // -> 1 saniyede bir verimli calismaktadir
void GPSData() {
  telemetry.gonderme_zamani += String((GPS.hour + HOUR_OFFSET) % 24, DEC); telemetry.gonderme_zamani += ":";
  telemetry.gonderme_zamani += String(GPS.minute, DEC); telemetry.gonderme_zamani += ":";
  telemetry.gonderme_zamani += String(GPS.seconds, DEC); telemetry.gonderme_zamani += "-";
  telemetry.gonderme_zamani += String(GPS.day, DEC); telemetry.gonderme_zamani += "/";
  telemetry.gonderme_zamani += String(GPS.month, DEC); telemetry.gonderme_zamani += "/20";
  telemetry.gonderme_zamani += String(GPS.year, DEC);

  if (GPS.fix) {
    telemetry.gps_latitude = GPS.latitudeDegrees;
    telemetry.gps_longitude = GPS.longitudeDegrees;
    telemetry.gps_altitude = GPS.altitude;
  }
}

void runBMP280() {

  telemetry.sicaklik = bmp.readTemperature(); // celcius

  telemetry.basinc = bmp.readPressure() / 1000.0; // kPa
  //Serial.println(telemetry.basinc);

  oncekiYukseklik = telemetry.yukseklik;

  telemetry.yukseklik = bmp.readAltitude(firstPressure); // fisrtPressure hPa, metre
  if (telemetry.yukseklik < 0) {
    telemetry.yukseklik = 0;
  }

  uint32_t t1 = millis() / 1000;

  if (telemetry.uydu_statusu == _FALLING) {
    telemetry.inis_hizi = (oncekiYukseklik - telemetry.yukseklik) / (t1 - t);
  }

  t = t1;

}

void runBNO055() {
  sensors_event_t event;
  bno.getEvent(&event);

  telemetry.roll = event.orientation.z;
  telemetry.pitch = event.orientation.y;
  telemetry.yaw = event.orientation.x;

  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  acc.x = accel.x();
  acc.y = accel.y();
  acc.z = accel.z();

}

void computeDonusSayisi() {
  if ((350 < toplam_donus  && toplam_donus < 359) || (-350 > toplam_donus  && toplam_donus > -359)) { // yaklasik olarak 1 tam tur araligi belirtildi
    telemetry.donus_sayisi++;
    toplam_donus = 0;
  } else {
    float diff_yaw = abs(currentYaw - telemetry.yaw);
    if (diff_yaw > 0) { // saga donerse
      toplam_donus += diff_yaw;
    } else if (diff_yaw < 0) { // sola donerse
      toplam_donus -= diff_yaw;
    }
  }

  currentYaw = telemetry.yaw;
}

void ayrilmaMekanizmasiniCalistir(uint8_t ac = 1) {
  if (ac == 1) {
    servo.write(180);
    telemetry.uydu_statusu = _LEAVING;
  } else if (ac == 0) {
    servo.write(0);
  }

}

void buzzer(uint8_t ac = 1) {
  if (ac == 1) {
    digitalWrite(13, HIGH);
  } else if (ac == 0) {
    digitalWrite(13, LOW);
  }
}

void motoruCalistir(uint8_t esc_value) {
  // esc_value must be between 0 and 180
  if (esc_value <= 0) {
    ESC.write(0);
  } else if (0 < esc_value && esc_value <= 180) {
    ESC.write(esc_value);
  } else if (esc_value > 180) {
    ESC.write(180);
  }

}

void findStatus(float min_h, float max_h, float ayrilma_h) {

  if (telemetry.yukseklik > max_h - ((5 / 700.0)*max_h))
  {
    telemetry.uydu_statusu = _LIMIT_HEIGHT;
    statu.isLimitHeight = true;
    //Serial.print("\nlimit height: ");
    //Serial.println(telemetry.yukseklik);
  }
  //durumlar :

  if (statu.isLimitHeight == false)
  {
    //ground
    if (telemetry.yukseklik < min_h)
    {
      telemetry.uydu_statusu = _GROUND;
      statu.isGround = true;
      //Serial.print("\nground: ");
      //Serial.println(telemetry.yukseklik);
    }
    else
    {
      telemetry.uydu_statusu = _CLIMBING;
      statu.isClimbing = true;
      //Serial.print("\nclimbing: ");
      //Serial.println(telemetry.yukseklik);
    }
    //climbing
  }
  else {
    telemetry.inis_hizi = velocity(firstVelocity, acc.z);
    //limit heighte ulaştıktan sonra
    //waiting to rescue
    if (telemetry.yukseklik < min_h)
    {
      // sıfıra yakın
      telemetry.uydu_statusu = _WATING_RESCUE;
      //buzzer();
      //Serial.print("\nwaiting rescue: ");
      //Serial.println(telemetry.yukseklik);


    }
    else
    {
      //leaving
      if (telemetry.yukseklik >= (ayrilma_h - ((10 / 400.0)*ayrilma_h)) && telemetry.yukseklik <= (ayrilma_h + ((10 / 400.0)*ayrilma_h)))
      {
        //ayrilmaMekanizmasiniCalistir();
        //motoruCalistir(0); // parametreye verilecek deger hesaplanmali
        telemetry.uydu_statusu = _LEAVING;
        statu.isLeaving = true;
        //ayrilmaMekanizmasiniCalistir();
        //Serial.println("leaving");
      }
      else if (telemetry.yukseklik < max_h && statu.isLeaving == false)
      {
        //falling
        telemetry.uydu_statusu = _FALLING;
        statu.isFalling = true;

        //Serial.print("\nfalling");
        //Serial.println(telemetry.yukseklik);
      }
      else if (statu.isLeaving == true)
      {
        telemetry.uydu_statusu = _AFTER_LEAVING;
        //motoruCalistir(0); // parametreye verilecek deger hesaplanmali
      }
    }
  }
}

void printTelemetry() {
  telemetry.paket_numarasi++;
  EEPROM.put(0, telemetry.paket_numarasi);
  Serial.print(telemetry.takim_no); Serial.print(',');
  Serial.print(telemetry.paket_numarasi); Serial.print(',');
  Serial.print(telemetry.gonderme_zamani); Serial.print(',');
  Serial.print(telemetry.basinc, 3); Serial.print(',');
  Serial.print(telemetry.yukseklik); Serial.print(',');
  Serial.print(telemetry.inis_hizi); Serial.print(',');
  Serial.print(telemetry.sicaklik); Serial.print(',');
  Serial.print(telemetry.pil_gerilimi); Serial.print(',');
  Serial.print(telemetry.gps_latitude, 6); Serial.print(',');
  Serial.print(telemetry.gps_longitude, 6); Serial.print(',');
  Serial.print(telemetry.gps_altitude); Serial.print(',');
  Serial.print(telemetry.uydu_statusu); Serial.print(',');
  Serial.print(telemetry.pitch); Serial.print(',');
  Serial.print(telemetry.roll); Serial.print(',');
  Serial.print(telemetry.yaw); Serial.print(',');
  Serial.print(telemetry.donus_sayisi); Serial.print(',');
  Serial.print(telemetry.video_aktarim_bilgisi);
  Serial.println('>');
}

void computeVoltage() {
  telemetry.pil_gerilimi = (8.42 * analogRead(VOLTAGE_PIN) / 1023.0);
}

void komutIslet(unsigned char komut) { // espden gelen komutlari yerine getir

  if (komut == '1') {
    telemetry.video_aktarim_bilgisi = "Evet";
  } else if (komut == '2') { // buzzer ac
    buzzer(1);
  } else if (komut == '3') { // buzzer kapa
    buzzer(0);
  } else if (komut == '4') {
    ayrilmaMekanizmasiniCalistir(1);   // kilit ac
  } else if (komut == '5') {
    ayrilmaMekanizmasiniCalistir(0);  // kilit kapa
  } else if (komut == '6') {
    clearEEPROM();
  } else if (komut == '7') { // motor calistir
    pwm_signal += 2;
    if(pwm_signal >= 50){
      pwm_signal = 27;
    }
    motoruCalistir(pwm_signal);
  } else if (komut == '8') { // motor kapat
    motoruCalistir(0);
    pwm_signal = 27;*
  } else if (komut == 'p') {
    hasPassword = true;
  }
}

float velocity(float f_Velocity, float accel) {
  uint32_t t1 = millis();
  float v =  firstVelocity - f_Velocity + format(acc.z, 2) * ((t1 - t0) / 1000.0);
  t0 = t1;
  firstVelocity = v;
  return v;
}

float format(float f_value, int digit) {
  float basamak = pow(10, digit);
  f_value *= basamak;
  f_value = ((int)f_value) / basamak;
  return f_value;
}

void clearEEPROM() {
  for (int i = 0 ; i < EEPROM.length() ; i++) {
    EEPROM.write(i, 0);
  }
  telemetry.paket_numarasi = 0;
}
