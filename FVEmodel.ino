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

void setup() {
  Serial.begin(9600);
}

void loop() {
  prectiVsechnySenzory();

  while (rozdilVpravo / rozdilVlevo > nepresnost) {
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
    zastavVsechnyMotory();
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