// Include necessary libraries for sensors, serial communication, and motor control
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>
#include <Adafruit_INA219.h>
#include <SCServo.h>

#define zrychleni 50  // Acceleration of servos
#define prostred 2048 // Middle postion of servos
#define maximalniPoziceY 2500 // End position of servo
#define minimalniPoziceY 1800 // Second end position


 
// Define RX and TX pin for servo communication
#define RxPin 18
#define TxPin 19

#define A1 32
#define A2 33
#define A3 34
#define A4 35
// Initialize light sensor with its I2C address
Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 0x39);

// Initialize current and voltage sensors with their I2C addresses
Adafruit_INA219 ina219I(0x40); // Intensity sensor
Adafruit_INA219 ina2191(0x41);
Adafruit_INA219 ina2192(0x44);

// Initialize the servo motor control library
SMS_STS st;

// Class representing a photoresistor sensor
class fotorezistor {
  protected:
    int pin; // Analog pin where the sensor is connected

  public:
    float rezistivita; // Resistance value of the photoresistor

    // Reads the sensor value
    void ctiSenzor() {
      this->rezistivita = analogRead(pin); // Read analog value from the pin
    }

    // Constructor to initialize the pin
    fotorezistor(int pin) {
      this->pin = pin;
    }

    void kalibrujFotorezistor(int nasobic, int prvniPulka, int druhaPulka){
      this->rezistivita*=nasobic;
      if(this->rezistivita<2048){
        this->rezistivita = (4095-this->rezistivita) * prvniPulka;
      }else{
        this->rezistivita = (4095-this->rezistivita) * druhaPulka;
      }
    }
};

// Class representing a motor controlled via servo protocol
class motor {
  protected:
    int id;        // Motor ID
    int pozice;    // Current position of the motor
    int rychlost = 300;  // Motor speed

  public:
    // Reads the current position of the motor
    void prectiPozici() {
      this->pozice = st.ReadPos(id);
    }
    // Constructor to initialize the motor with its ID
    motor(int id) {
      this->id = id;
    }

    // Read position 
    int aktualniPozice(){
      this->pozice = st.ReadPos(id);
      return this->pozice;
    }

    // Changes the speed of the motor
    void zmenRychlost(int rychlost) {
      this->rychlost = rychlost;
    }

    // Moves the motor forward by a fixed amount
    void jedDopredu() {
      st.WritePosEx(id, (pozice + 50), rychlost, zrychleni);
      delay(100);
    }

    // Moves the motor backward by a fixed amount
    void jedDozadu() {
      st.WritePosEx(id, (pozice - 50), rychlost, zrychleni);
      delay(100);
    }

    // Returns ID of servo
    int aktualniID(){
      return this->id;
    }
    
    // Returns servo speed
    int nastavenaRychlost(){
      return this->rychlost;
    }
};

// Create photoresistor objects representing sensors at different positions
fotorezistor fotorezistorN(A4);  // Photoresistor on the top
fotorezistor fotorezistorD(A1);  // Photoresistor on the bottom
fotorezistor fotorezistorL(A3);  // Photoresistor on the left
fotorezistor fotorezistorP(A2);  // Photoresistor on the right

// Create motor objects with unique IDs
motor motorX(20);   // Motor for the X-axis
motor motor1Y(1);  // Motor for the Y-axis

// Variables for storing differences in light intensity
float rozdilX;
float rozdilY;

// Threshold for precision
int nepresnost = 30;

void setup() {
  // Configure analog pins for input
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);

  // Initialize serial communication and I2C communication
  Serial.begin(9600);
  Wire.begin();

  // Initialize the light intensity sensor
  if (!tsl.begin()) {
    Serial.println("Light intensity sensor not working!");
  }

  // Initialize current and voltage sensors
  ina219I.begin();
  ina2191.begin();
  ina2192.begin();

  // Initialize servo communication
  Serial1.begin(1000000, SERIAL_8N1, RxPin, TxPin);
  st.pSerial = &Serial1;

  // Sets up servos into middle possition 
  st.WritePosEx(motor1Y.aktualniID(), prostred, motor1Y.nastavenaRychlost(), zrychleni);
  st.WritePosEx(motorX.aktualniID(), prostred, motorX.nastavenaRychlost(), zrychleni);
  
  // Delay for servos to reach (near) middle 
  delay(2000);
}

void loop() {
  Serial.println("new");

  // Read voltage and current data from sensors
  float vstupVI = ina219I.getBusVoltage_V();     // Input voltage of sensor
  float bocniVI = ina219I.getShuntVoltage_mV();  // Shunt voltage of sensor
  float zatezVI = vstupVI + (bocniVI / 1000);    // Load voltage
  float proudI = ina219I.getCurrent_mA();        // Current from sensor

  float vstupV1 = ina2191.getBusVoltage_V();     // Input voltage of sensor
  float bocniV1 = ina2191.getShuntVoltage_mV();  // Shunt voltage of sensor
  float zatezV1 = vstupV1 + (bocniV1 / 1000);    // Load voltage
  float proud1 = ina2191.getCurrent_mA();        // Current from sensor

  float vstupV2 = ina2192.getBusVoltage_V();     // Input voltage of sensor
  float bocniV2 = ina2192.getShuntVoltage_mV();  // Shunt voltage of sensor
  float zatezV2 = vstupV2 + (bocniV2 / 1000);    // Load voltage
  float proud2 = ina2192.getCurrent_mA();        // Current from sensor

  /* Uncommented portion for debugging purposes
  Serial.print("Input voltage: "); Serial.print(vstupVI * 1000); Serial.println(" mV");
  Serial.print("Shunt voltage: "); Serial.print(bocniVI); Serial.println(" mV");
  Serial.print("Load voltage: "); Serial.print(zatezVI); Serial.println(" V");
  Serial.print("Current: "); Serial.print(proudI); Serial.println(" mA");
  */

  sensors_event_t event;
  tsl.getEvent(&event); // Fetch light intensity data

  /* Uncommented portion for light sensor debugging
  if (event.light) {
    Serial.print("Light intensity: "); Serial.print(event.light); Serial.println(" Lux");
  } else {
    Serial.println("Light intensity sensor overloaded!");
  }
  */

  
  // Send data to database on Raspberry Pi

  
  if (event.light) {
    Serial.println(event.light); // Print light intensity in Lux
  } else {
    Serial.println(NULL); // Indicate sensor overload
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

  if (isnan(proud2) || isinf(proud2)) {
    Serial.println(0); // Handle invalid current values
  } else {
    Serial.println(proud2);
  }

  // Read all photoresistor sensors
  prectiVsechnySenzory();

  // Move servos based on light differences

  while(rozdilX>nepresnost){
    motorX.jedDopredu();
    prectiVsechnySenzory();
    
  }
  
  while(rozdilY>nepresnost){
    if(motor1Y.aktualniPozice()<maximalniPoziceY){
      motor1Y.jedDopredu();
    }
    prectiVsechnySenzory();
  }
  if(rozdilX<0){
     while(rozdilX<(-nepresnost)){
      motorX.jedDozadu();
      prectiVsechnySenzory();
    }
    
  }
  if(rozdilY<0){
    while(rozdilY<(-nepresnost)){
    if(motor1Y.aktualniPozice()>minimalniPoziceY){
      motor1Y.jedDozadu();
    }
    prectiVsechnySenzory();
    }
  }
  
  prectiVsechnySenzory();
  // Uncommented portion for debugging photoresistor values
 /* Serial.print("Top: "); Serial.println(fotorezistorN.rezistivita);
  Serial.print("Bottom: "); Serial.println(fotorezistorD.rezistivita);
  Serial.print("Left: "); Serial.println(fotorezistorL.rezistivita);
  Serial.print("Right: "); Serial.println(fotorezistorP.rezistivita);
  Serial.print("Difference X: "); Serial.println(rozdilX);
  Serial.print("Difference Y: "); Serial.println(rozdilY);*/
  

  delay(10000); // Delay for stability
}

void aktualizujRozdily(){
  fotorezistorN.ctiSenzor(); // Read the top sensor
  fotorezistorD.ctiSenzor(); // Read the bottom sensor
  fotorezistorL.ctiSenzor(); // Read the left sensor
  fotorezistorP.ctiSenzor(); // Read the right sensor


  // Calibration of photorezistors
  fotorezistorN.kalibrujFotorezistor(0.9114,1,1.1);
  fotorezistorN.kalibrujFotorezistor(0.9574,1,1.2);
  fotorezistorN.kalibrujFotorezistor(1.1803,1,1);
  fotorezistorN.kalibrujFotorezistor(1,1,1.05);




  // Old version of calibration 
  /*fotorezistorN.rezistivita*=0.9114;
  fotorezistorD.rezistivita*=0.9574;
  fotorezistorL.rezistivita*=1.1803;
  fotorezistorP.rezistivita*=1;


  if(fotorezistorN.rezistivita<2048){
    fotorezistorN.rezistivita = (4095-fotorezistorN.rezistivita) * 1;
  }else{
    fotorezistorN.rezistivita = (4095-fotorezistorN.rezistivita) * 1.1;
  }

  if(fotorezistorD.rezistivita<2048){
    fotorezistorD.rezistivita = (4095-fotorezistorD.rezistivita) * 1;
  }else{
    fotorezistorD.rezistivita = (4095-fotorezistorD.rezistivita) * 1.2;
  }

  if(fotorezistorL.rezistivita<2048){
    fotorezistorL.rezistivita = (4095-fotorezistorL.rezistivita) * 1;
  }else{
    fotorezistorL.rezistivita = (4095-fotorezistorL.rezistivita) * 1;
  }

  if(fotorezistorP.rezistivita<2048){
    fotorezistorP.rezistivita = (4095-fotorezistorP.rezistivita) * 1;
  }else{
    fotorezistorP.rezistivita = (4095-fotorezistorP.rezistivita) * 1.05;
  }*/

  // Converting to int
  prevedNaInt();



  // Calculate differences in light intensity
  rozdilX = fotorezistorP.rezistivita - fotorezistorL.rezistivita;
  rozdilY = fotorezistorN.rezistivita - fotorezistorD.rezistivita;
}

void prevedNaInt(){
  fotorezistorN.rezistivita/=10;
  fotorezistorD.rezistivita/=10;
  fotorezistorL.rezistivita/=10;
  fotorezistorP.rezistivita/=10;

  fotorezistorN.rezistivita=(int)fotorezistorN.rezistivita;
  fotorezistorD.rezistivita=(int)fotorezistorD.rezistivita;
  fotorezistorL.rezistivita=(int)fotorezistorL.rezistivita;
  fotorezistorP.rezistivita=(int)fotorezistorP.rezistivita;
}

// Function to read all photoresistor sensors and calculate differences
void prectiVsechnySenzory() {
  aktualizujRozdily();
  // Read servo position
  motorX.prectiPozici();
  motor1Y.prectiPozici();
}
