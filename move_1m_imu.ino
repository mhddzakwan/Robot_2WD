#include <Arduino_LSM9DS1.h>

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
// IMU TUNING PARAMETER
// ====================================================================
const float Kp_IMU = 4.5;    // Sensitivitas koreksi sudut.
const int   BASE_PWM = 100;  // Kecepatan dasar

// ====================================================================
// VARIABLES
// ====================================================================
volatile long pulsaKiri = 0;
volatile long pulsaKanan = 0;
long targetPulsaOtomatis = 0; 

int speedKiri = BASE_PWM;
int speedKanan = BASE_PWM; 

// Variabel Kalkulasi Sudut Gyro
float headingYaw = 0.0; 
float gyroZBias = 0.0; // Menyimpan nilai error bias sumbu Z
unsigned long waktuLama = 0;

void setup() {
  Serial.begin(9600);

  // Setup Pin Driver Motor
  pinMode(PIN_PWMA, OUTPUT); pinMode(PIN_AIN1, OUTPUT); pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_PWMB, OUTPUT); pinMode(PIN_BIN1, OUTPUT); pinMode(PIN_BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);

  // Setup Pin Encoder
  pinMode(ENCA, INPUT_PULLUP);
  pinMode(ENCB, INPUT_PULLUP);
  digitalWrite(STBY, HIGH);

  attachInterrupt(digitalPinToInterrupt(ENCA), countPulsaKiri, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCB), countPulsaKanan, RISING);

  // Inisialisasi Sensor IMU LSM9DS1
  if (!IMU.begin()) {
    Serial.println("Gagal menginisialisasi IMU LSM9DS1!");
    while (1); 
  }

  // Hitung Target Pulsa dari Encoder
  float kelilingRoda = 3.14159265 * DIAMETER_RODA_CM;
  float jumlahPutaranRoda = TARGET_JARAK_CM / kelilingRoda;
  targetPulsaOtomatis = jumlahPutaranRoda * PPR_TOTAL_RODA;

  // --- PROSES KALIBRASI BIAS GYRO (ROBOT HARUS DIAM) ---
  Serial.println("=== KALIBRASI GYRO BERJALAN... JANGAN SENTUH ROBOT ===");
  delay(1000); // Tunggu stabil
  
  float totalZ = 0;
  int jumlahSampel = 200;
  int sampelTerhitung = 0;
  
  while (sampelTerhitung < jumlahSampel) {
    if (IMU.gyroscopeAvailable()) {
      float x, y, z;
      IMU.readGyroscope(x, y, z);
      totalZ += z;
      sampelTerhitung++;
      delay(5);
    }
  }
  gyroZBias = totalZ / jumlahSampel; // Menghitung rata-rata noise saat diam
  
  Serial.print("Kalibrasi Selesai! Nilai Bias Z: "); Serial.println(gyroZBias);
  Serial.println("=== ROBOT BERJALAN ===");
  delay(1000);
  
  waktuLama = micros(); 
  maju();
}

void loop() {
  // 1. Cek Jarak via Encoder
  long rataRataPulsa = (pulsaKiri + pulsaKanan) / 2;
  if (rataRataPulsa >= targetPulsaOtomatis) {
    rem();
    Serial.println("Target Jarak Tercapai! Robot Berhenti.");
    while (1); 
  }

  // 2. Membaca Data Gyroscope secara berkala
  if (IMU.gyroscopeAvailable()) {
    float x, y, z;
    IMU.readGyroscope(x, y, z);

    // Hilangkan nilai bias hasil kalibrasi
    z = z - gyroZBias;

    // Hitung delta waktu (dt) dalam satuan detik
    unsigned long waktuSekarang = micros();
    float dt = (waktuSekarang - waktuLama) / 1000000.0;
    waktuLama = waktuSekarang;

    // Deadzone filter untuk mengabaikan getaran micro robot
    if (abs(z) > 0.3) { 
      headingYaw += z * dt; 
    }

    // --- LOGIKA KOREKSI DINAMIS ---
    float errorSudut = headingYaw; 
    float koreksi = errorSudut * Kp_IMU;

    // JIKA MOTOR KIRI MASIH TERLALU CEPAT, balik tanda di bawah ini:
    // Opsi A (Bawaan saat ini):
    speedKiri  = BASE_PWM - koreksi; 
    speedKanan = BASE_PWM + koreksi;

    // Opsi B (Gunakan jika Opsi A justru membuat robot makin berbelok ke kanan):
    // speedKiri  = BASE_PWM - koreksi;
    // speedKanan = BASE_PWM + koreksi;

    // Batasi nilai PWM ke range aman
    speedKiri  = constrain(speedKiri, 50, 200);
    speedKanan = constrain(speedKanan, 50, 200);

    // Kirim sinyal ke driver motor
    analogWrite(PIN_PWMA, speedKiri);
    analogWrite(PIN_PWMB, speedKanan);

    // Debugging via Serial Monitor
    Serial.print("Sudut_Yaw:"); Serial.print(headingYaw);
    Serial.print("\tPWM_Kiri:"); Serial.print(speedKiri);
    Serial.print("\tPWM_Kanan:"); Serial.println(speedKanan);
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
