// --- CONFIGURAZIONE PIN MOTORI (Guida Differenziale) ---
// Motori LATO DESTRO
const int IN1_DESTRA = 7;
const int IN2_DESTRA = 8;

// Motori LATO SINISTRA
const int IN1_SINISTRA = 9;
const int IN2_SINISTRA = 10;

// --- CONFIGURAZIONE SENSORE ULTRASUONI ---
const int TRIG_PIN = 2;
const int ECHO_PIN = 3;
const long DISTANZA_SICUREZZA_CM = 20;

bool allarmeInviato = false;

void setup() {
  Serial.begin(9600);
  pinMode(IN1_DESTRA, OUTPUT);
  pinMode(IN2_DESTRA, OUTPUT);
  pinMode(IN1_SINISTRA, OUTPUT);
  pinMode(IN2_SINISTRA, OUTPUT);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  stopMotori();
}

void loop() {
  if (leggiDistanza() < DISTANZA_SICUREZZA_CM) {
    stopMotori();
    if (!allarmeInviato) {
      Serial.println("OBJ_VICINO");
      allarmeInviato = true;
    }
  } else {
    allarmeInviato = false;
    if (Serial.available() > 0) {
      String comando = Serial.readStringUntil('\n');
      comando.trim();

      if (comando == "avanti") vaiAvanti();
      else if (comando == "indietro") vaiIndietro();
      else if (comando == "sinistra") giraSinistra();
      else if (comando == "destra") giraDestra();
      else if (comando == "stop") stopMotori();
    }
  }
}

void stopMotori() {
  digitalWrite(IN1_DESTRA, LOW); digitalWrite(IN2_DESTRA, LOW);
  digitalWrite(IN1_SINISTRA, LOW); digitalWrite(IN2_SINISTRA, LOW);
}

void vaiAvanti() {
  digitalWrite(IN1_DESTRA, HIGH); digitalWrite(IN2_DESTRA, LOW);
  digitalWrite(IN1_SINISTRA, HIGH); digitalWrite(IN2_SINISTRA, LOW);
}

void vaiIndietro() {
  digitalWrite(IN1_DESTRA, LOW); digitalWrite(IN2_DESTRA, HIGH);
  digitalWrite(IN1_SINISTRA, LOW); digitalWrite(IN2_SINISTRA, HIGH);
}

void giraDestra() {
  digitalWrite(IN1_DESTRA, LOW); digitalWrite(IN2_DESTRA, HIGH);
  digitalWrite(IN1_SINISTRA, HIGH); digitalWrite(IN2_SINISTRA, LOW);
}

void giraSinistra() {
  digitalWrite(IN1_DESTRA, HIGH); digitalWrite(IN2_DESTRA, LOW);
  digitalWrite(IN1_SINISTRA, LOW); digitalWrite(IN2_SINISTRA, HIGH);
}

long leggiDistanza() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long durata = pulseIn(ECHO_PIN, HIGH);
  long distanza = durata * 0.0343 / 2;
  return distanza;
}
