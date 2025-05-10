#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>
#include <Adafruit_INA219.h>
#include <SoftwareSerial.h>
#include <SCServo.h>


Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT,0x39);
Adafruit_INA219 ina219I(0x40); // Senzor intenzity
Adafruit_INA219 ina2191(0x41);
Adafruit_INA219 ina2192(0x44);

#define S_RXD
#define S_TXD

SMS_STS st;

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
    int id
    int pozice
    int rychlost

    void prectiPozici(){
      this->pozice = st.ReadPos(id);
    }

  public:
    motor(int id) {
      this->id = id;
    }

    void zmenRychlost(int rychlost){
      this->rychlost = rychlost;
    }

    void jedDopredu() {
      st.WritePosEx(id, (pozice+100), rychlost, 50);
    }

    void jedDozadu() {
      st.WritePosEx(id, (pozice-100), rychlost, 50);
    }

    void zastav() {
      st.WritePosEx(id, pozice, 0, 50);
    }
};

// Fotorezistory (A1 - A4)
fotorezistor fotorezistorP(A1);  // Vpravo
fotorezistor fotorezistorN(A3);  // Nahoře 
fotorezistor fotorezistorD(A2);  // Dole 
fotorezistor fotorezistorL(A4);  // Vlevo

// Motory 
motor motor1Y(2);
motor motor2Y(3);
motor motorX(1);

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

  Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
  st.pSerial = &Serial1;
  delay(1000);

  
}

void loop() {
  //prectiVsechnySenzory();
  //Wire.requestFrom(0x39,16);
  Serial.println("new");
  
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

 /* Serial.print("vstupní napětí: "); Serial.print(vstupVI*1000); Serial.println(" mV");
  Serial.print("napětí na bočníku: "); Serial.print(bocniVI); Serial.println(" mV");
  Serial.print("napětí zátěže: "); Serial.print(zatezVI); Serial.println(" V");
  Serial.print("proud: "); Serial.print(proudI); Serial.println(" mA");
*/

  sensors_event_t event;
  tsl.getEvent(&event);

 /* if(event.light){
    Serial.print("Senzor intenzity světla: "); Serial.print(event.light); Serial.println(" Lux");
  }else{
    Serial.println("Senzor intenzity světla přetížen!");  
  }*/

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
  if(isnan(proud2) || isinf(proud2)){
    Serial.println(0);
  }else{
    Serial.println(proud2);
  }

  prectiVsechnySenzory();

  /*Serial.print("N: "); Serial.println(fotorezistorN.rezistivita);
  Serial.print("D: "); Serial.println(fotorezistorD.rezistivita);
  Serial.print("L: "); Serial.println(fotorezistorL.rezistivita);
  Serial.print("P: "); Serial.println(fotorezistorP.rezistivita);
  Serial.print("RX: "); Serial.println(rozdilX);
  Serial.print("RY: "); Serial.println(rozdilY);*/



  delay(1000);
 /* while (rozdilVpravo / rozdilVlevo > nepresnost) {
    motorX1.jedDopredu();
    motorX2.jedDopredu();
  }
    zastavVsechnyMotory();

  while (rozdilVlevo / rozdilVpravo > nepresnost) {
    motorX1.jedDozadu();
    motorX2.jedDozadu();
  }
    zastavVsechnyMotory();

  while (rozdilDole / rozdilNahore > nepresnost) {
    motorY1.jedDopredu();
  }
    zastavVsechnyMotory();

  while (rozdilNahore / rozdilDole > nepresnost) {
    motorY1.jedDozadu();
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
