#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>
#include <Adafruit_INA219.h>
#include <SoftwareSerial.h>

Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT,0x39);
Adafruit_INA219 ina219I(0x40); // Senzor intenzity
Adafruit_INA219 ina2191(0x41);
Adafruit_INA219 ina2192(0x44);



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

// Fotorezistory (A1 - A4)
fotorezistor fotorezistorP(A1);  // Vpravo
fotorezistor fotorezistorN(A3);  // Nahoře 
fotorezistor fotorezistorD(A2);  // Dole 
fotorezistor fotorezistorL(A4);  // Vlevo

// Motory (piny pro řízení směru + PWM)
motor motorX1(5, 6, 9);
motor motorX2(7, 8, 10);
motor motorY1(11, 12, 3);

float rozdilX;
float rozdilY;

float nepresnost = 1.2;

void setup() {

  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);

  Serial.begin(9600);
  Wire.begin();

  if(!tsl.begin()){
    Serial.println("Senzor intenzity nefunkční!");
  }

  ina219I.begin();
  ina2191.begin();
  ina2192.begin();

  
}

void loop() {
  //prectiVsechnySenzory();
  //Wire.requestFrom(0x39,16);
  Serial.println("\n\nnew");
  
/*  while(Wire.available()){
    int c = Wire.read();
    Serial.println(c);
  }*/

  

 /* float vstupVI = ina219I.getBusVoltage_V();     // Vstupní napětí senzoru 
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

  Serial.print("vstupní napětí: "); Serial.print(vstupVI*1000); Serial.println(" mV");
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

  if(event.light){
    Serial.println(event.light);
  }else{
    Serial.println(NULL);
  }
  Serial.println(vstupVI);
  Serial.println(bocniVI);
  Serial.println(zatezVI);
  Serial.println(proudI);

  Serial.println(vstupV1);
  Serial.println(bocniV1);
  Serial.println(zatezV1);
  Serial.println(proud1);

  Serial.println(vstupV2);
  Serial.println(bocniV2);
  Serial.println(zatezV2);
  Serial.println(proud2);*/

  prectiVsechnySenzory();

  Serial.print("N: "); Serial.println(fotorezistorN.rezistivita);
  Serial.print("D: "); Serial.println(fotorezistorD.rezistivita);
  Serial.print("L: "); Serial.println(fotorezistorL.rezistivita);
  Serial.print("P: "); Serial.println(fotorezistorP.rezistivita);
  Serial.print("RX: "); Serial.println(rozdilX);
  Serial.print("RY: "); Serial.println(rozdilY);



  delay(300);
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
  fotorezistorN.ctiSenzor();
  fotorezistorD.ctiSenzor();
  fotorezistorL.ctiSenzor();
  fotorezistorP.ctiSenzor();

  rozdilX = fotorezistorL.rezistivita - fotorezistorP.rezistivita;
  rozdilY = fotorezistorN.rezistivita - fotorezistorD.rezistivita;
}

void zastavVsechnyMotory() {
  motorX1.zastav();
  motorX2.zastav();
  motorY1.zastav();
}
