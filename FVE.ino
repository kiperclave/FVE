#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>
#include <Adafruit_INA219.h>
#include <SoftwareSerial.h>

Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT,0x39);
Adafruit_INA219 ina219I(0x40); // Senzor intenzity
Adafruit_INA219 ina2191(0x41);
Adafruit_INA219 ina2192(0x42);
Adafruit_INA219 ina2193(0x43);
Adafruit_INA219 ina2194(0x44);


class fotorezistor {
  protected:
    int pin;

  public:
    int rezistivita;

    void ctiSenzor() {
      this->rezistivita = analogRead(pin); // Opraveno na analogRead
    }

    fotorezistor(int pin) {
      this->pin = pin;
    }
};

class motor {
  protected:
    int pin1;
    int pin2;
    int pinPWM;

  public:
    motor(int p1, int p2, int pwm) {
      pin1 = p1;
      pin2 = p2;
      pinPWM = pwm;
      pinMode(pin1, OUTPUT);
      pinMode(pin2, OUTPUT);
      pinMode(pinPWM, OUTPUT);
    }

    void jedDopredu(int rychlost = 0) {
      digitalWrite(pin1, LOW);
      digitalWrite(pin2, HIGH);
      analogWrite(pinPWM, rychlost);
    }

    void jedDozadu(int rychlost = 0) {
      digitalWrite(pin1, HIGH);
      digitalWrite(pin2, LOW);
      analogWrite(pinPWM, rychlost);
    }

    void zastav() {
      digitalWrite(pin1, LOW);
      digitalWrite(pin2, LOW);
      analogWrite(pinPWM, 0);
    }
};

// Fotorezistory (A0 - A3)
fotorezistor fotorezistorNP(A0);  // Nahoře vpravo
fotorezistor fotorezistorNL(A1);  // Nahoře vlevo
fotorezistor fotorezistorDP(A2);  // Dole vpravo
fotorezistor fotorezistorDL(A3);  // Dole vlevo

// Motory (piny pro řízení směru + PWM)
motor motorX1(5, 6, 9);
motor motorX2(7, 8, 10);
motor motorY1(11, 12, 3);

float rozdilNahore;
float rozdilDole;
float rozdilVpravo;
float rozdilVlevo;

float nepresnost = 1.2;

const byte rpiRx = 2;
const byte rpiTx = 3;

SoftwareSerial rpiKom(rpiRx, rpiTx);

void setup() {
  Serial.begin(9600);
  rpiKom.begin(9600);
  Wire.begin();

  if(!tsl.begin()){
    Serial.println("Senzor intenzity nefunkční!");
  }

  ina219I.begin();
  ina2191.begin();
  ina2192.begin();
  ina2193.begin();


  pinMode(rpiRx, INPUT);
  pinMode(rpiTx, OUTPUT);

  
}

void loop() {
  //prectiVsechnySenzory();
  //Wire.requestFrom(0x39,16);
  Serial.println("\n\nnew");
  
/*  while(Wire.available()){
    int c = Wire.read();
    Serial.println(c);
  }*/

  

  float vstupVI = ina219I.getBusVoltage_V();     // Vstupní napětí senzoru 
  float bocniVI = ina219I.getShuntVoltage_mV();  // Boční (shunt) napětí senzoru 
  float zatezVI = vstupVI + (bocniVI/1000);       // Napětí zátěže
  float proudI = ina219I.getCurrent_mA();        // Proud senzoru  

  float vstupV1 = ina2191.getBusVoltage_V();     // Vstupní napětí senzoru 
  float bocniV1 = ina2191.getShuntVoltage_mV();  // Boční (shunt) napětí senzoru 
  float zatezV1 = vstupV1 + (bocniV1/1000);       // Napětí zátěže
  float proud1 = ina2191.getCurrent_mA();        // Proud senzoru 

  float vstupV2 = ina2192.getBusVoltage_V();     // Vstupní napětí senzoru 
  float bocniV2 = ina2192.getShuntVoltage_mV();  // Boční (shunt) napětí senzoru 
  float zatezV2 = vstupV2 + (bocniV2/1000);       // Napětí zátěže
  float proud2 = ina2192.getCurrent_mA();        // Proud senzoru   

   float vstupV3 = ina2193.getBusVoltage_V();     // Vstupní napětí senzoru 
  float bocniV3 = ina2193.getShuntVoltage_mV();  // Boční (shunt) napětí senzoru 
  float zatezV3 = vstupV3 + (bocniV3/1000);       // Napětí zátěže
  float proud3 = ina2193.getCurrent_mA();        // Proud senzoru  

  Serial.print("vstupní napětí: "); Serial.print(vstupVI); Serial.println(" V");
  Serial.print("napětí na bočníku: "); Serial.print(bocniVI); Serial.println(" mV");
  Serial.print("napětí zátěže: "); Serial.print(zatezVI); Serial.println(" V");
  Serial.print("proud: "); Serial.print(proudI); Serial.println(" mA");


  sensors_event_t event;
  tsl.getEvent(&event);

  if(event.light){
    Serial.print("Senzor intenzity světla: "); Serial.print(event.light); Serial.println(" Lux");
  }else{
    Serial.println("Senzor intenzity světla přetížen!");  
  }

  rpiKom.println("new");

  if(event.light){
    rpiKom.println(event.light);
  }else{
    rpiKom.println(NULL);
  }
  rpiKom.println(vstupVI);
  rpiKom.println(bocniVI);
  rpiKom.println(zatezVI);
  rpiKom.println(proudI);

  rpiKom.println(vstupV1);
  rpiKom.println(bocniV1);
  rpiKom.println(zatezV1);
  rpiKom.println(proud1);

  rpiKom.println(vstupV2);
  rpiKom.println(bocniV2);
  rpiKom.println(zatezV2);
  rpiKom.println(proud2);

  rpiKom.println(vstupV3);
  rpiKom.println(bocniV3);
  rpiKom.println(zatezV3);
  rpiKom.println(proud3);

  delay(500);
 /* while (rozdilVpravo / rozdilVlevo > nepresnost) {
    motorX1.jedDopredu(100);
    motorX2.jedDopredu(100);
  }
    zastavVsechnyMotory();

  while (rozdilVlevo / rozdilVpravo > nepresnost) {
    motorX1.jedDozadu(100);
    motorX2.jedDozadu(100);
  }
    zastavVsechnyMotory();

  while (rozdilDole / rozdilNahore > nepresnost) {
    motorY1.jedDopredu(100);
  }
    zastavVsechnyMotory();

  while (rozdilNahore / rozdilDole > nepresnost) {
    motorY1.jedDozadu(100);
  }
    zastavVsechnyMotory();*/


}

void prectiVsechnySenzory() {
  fotorezistorNP.ctiSenzor();
  fotorezistorNL.ctiSenzor();
  fotorezistorDP.ctiSenzor();
  fotorezistorDL.ctiSenzor();

  rozdilNahore = fotorezistorNP.rezistivita - fotorezistorNL.rezistivita;
  rozdilDole = fotorezistorDP.rezistivita - fotorezistorDL.rezistivita;
  rozdilVpravo = fotorezistorNP.rezistivita - fotorezistorDP.rezistivita;
  rozdilVlevo = fotorezistorNL.rezistivita - fotorezistorDL.rezistivita;
}

void zastavVsechnyMotory() {
  motorX1.zastav();
  motorX2.zastav();
  motorY1.zastav();
}
