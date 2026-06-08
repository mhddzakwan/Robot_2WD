// ====================================================================
// PIN CONFIGURATION (TB6612FNG & ENCODER)
// ====================================================================
const int PIN_PWMA = 5;
const int PIN_AIN1 = 6;
const int PIN_AIN2 = 7;
const int ENCA = 2; 

const int PIN_PWMB = 10;
const int PIN_BIN1 = 8;
const int PIN_BIN2 = 9;
const int ENCB = 11; 

const int STBY = 4;

// ====================================================================
// CONFIGURATION INPUT
// ====================================================================
const float TARGET_JARAK_CM = 80.0;  
const float DIAMETER_RODA_CM = 6.5;   
const int   PPR_TOTAL_RODA   = 620;   

// ====================================================================
// TUNING PARAMETER (Kunci Kelurusan Robot)
// ====================================================================
const float Kp = 7;  // Proportional Gain. Naikkan jika robot lambat merespon belokan.
const int BASE_PWM = 100; // Kecepatan dasar robot (Gunakan nilai sedang agar koreksi optimal)

// ====================================================================
// VARIABLES & AUTOMATIC CALCULATION
// ====================================================================
volatile long pulsaKiri = 0;
volatile long pulsaKanan = 0;
long targetPulsaOtomatis = 0; 

int speedKiri = BASE_PWM;
int speedKanan = BASE_PWM; 

const int intervalKoreksi = 30; // Dipercepat ke 30ms agar lebih responsif
unsigned long waktuLama = 0;

void setup() {
  Serial.begin(9600);

  pinMode(PIN_PWMA, OUTPUT); pinMode(PIN_AIN1, OUTPUT); pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_PWMB, OUTPUT); pinMode(PIN_BIN1, OUTPUT); pinMode(PIN_BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);

  pinMode(ENCA, INPUT_PULLUP);
  pinMode(ENCB, INPUT_PULLUP);

  digitalWrite(STBY, HIGH);

  attachInterrupt(digitalPinToInterrupt(ENCA), countPulsaKiri, RISING);
  attachInterrupt(digitalInterrupt(ENCB), countPulsaKanan, RISING);

  // Hitung Target Pulsa
  float kelilingRoda = 3.14159265 * DIAMETER_RODA_CM;
  float jumlahPutaranRoda = TARGET_JARAK_CM / kelilingRoda;
  targetPulsaOtomatis = jumlahPutaranRoda * PPR_TOTAL_RODA;

  Serial.println("=== SISTEM PROPORTIONAL CONTROL READY ===");
  delay(2000); 
  maju();
}

void loop() {
  // 1. Cek Jarak
  if (pulsaKiri >= targetPulsaOtomatis || pulsaKanan >= targetPulsaOtomatis) {
    rem();
    Serial.println("Target Jarak Tercapai! Robot Berhenti.");
    while (1); 
  }

  // 2. Logika Koreksi Proportional (P-Control)
  unsigned long waktuSekarang = millis();
  if (waktuSekarang - waktuLama >= intervalKoreksi) {
    waktuLama = waktuSekarang;

    // Error adalah selisih ideal (Target: selisih = 0)
    long error = pulsaKiri - pulsaKanan;

    // Hitung nilai koreksi berdasarkan seberapa besar error-nya
    float koreksi = error * Kp;

    // Terapkan koreksi secara dinamis dari BASE_PWM
    speedKiri  = BASE_PWM - koreksi;
    speedKanan = BASE_PWM + koreksi;

    // Batasi nilai PWM agar tidak over-limit atau terlalu pelan hingga macet
    speedKiri  = constrain(speedKiri, 60, 230);
    speedKanan = constrain(speedKanan, 60, 230);

    // Kirim sinyal kecepatan ke Motor
    analogWrite(PIN_PWMA, speedKiri);
    analogWrite(PIN_PWMB, speedKanan);

    // Debug di Serial Plotter / Monitor
    Serial.print("Pulsa_Kiri:"); Serial.print(pulsaKiri);
    Serial.print("\tPulsa_Kanan:"); Serial.println(pulsaKanan);
  }
}

void maju() {
  digitalWrite(PIN_AIN1, HIGH);
  digitalWrite(PIN_AIN2, LOW);
  analogWrite(PIN_PWMA, speedKiri);

  digitalWrite(PIN_BIN1, LOW);
  digitalWrite(PIN_BIN2, HIGH);
  analogWrite(PIN_PWMB, speedKanan);
}

void rem() {
  digitalWrite(PIN_AIN1, HIGH);
  digitalWrite(PIN_AIN2, HIGH);
  digitalWrite(PIN_BIN1, HIGH);
  digitalWrite(PIN_BIN2, HIGH);
  analogWrite(PIN_PWMA, 0);
  analogWrite(PIN_PWMB, 0);
}

void countPulsaKiri() { pulsaKiri++; }
void countPulsaKanan() { pulsaKanan++; }
int digitalInterrupt(int pin) { return digitalPinToInterrupt(pin); }
