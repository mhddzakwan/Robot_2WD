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
const float DIAMETER_RODA_CM = 6.5;   
const int   PPR_TOTAL_RODA   = 620;   

// ====================================================================
// IMU TUNING PARAMETER
// ====================================================================
const float Kp_IMU = 4.5;        // Sensitivitas koreksi sudut saat maju.
const float Kp_Belok = 6.5;      // Sensitivitas saat memutar badannya.
const int   BASE_PWM = 100;      // Kecepatan dasar saat maju lurus
const int   BASE_TURN_PWM = 60;  // Kecepatan MAKSIMAL dasar saat berputar (Bisa disesuaikan, misal 70-90)

// ====================================================================
// VARIABLES
// ====================================================================
volatile long pulsaKiri = 0;
volatile long pulsaKanan = 0;

int speedKiri = BASE_PWM;
int speedKanan = BASE_PWM; 

// Variabel Kalkulasi Sudut Gyro
float headingYaw = 0.0; 
float gyroZBias = 0.0; 
unsigned long waktuLama = 0;

// Variabel Pengendali Alur Langkah Robot (State Machine)
int langkahMisi = 0;
bool baruMulaiLangkah = true;

// Variabel Global khusus bantuan Debugging (JANGAN DIUBAH)
float dbgTarget = 0.0;
float dbgError = 0.0;
int dbgPwmOutKiri = 0;
int dbgPwmOutKanan = 0;

const float OFFSET_BELOK = 10;

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

  // --- PROSES KALIBRASI BIAS GYRO (ROBOT HARUS DIAM) ---
  Serial.println("=== KALIBRASI GYRO BERJALAN... JANGAN SENTUH ROBOT ===");
  delay(1000); 
  
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
  gyroZBias = totalZ / jumlahSampel; 
  
  Serial.print("Kalibrasi Selesai! Nilai Bias Z: "); Serial.println(gyroZBias);
  Serial.println("=== ROBOT READY... === ");
  delay(1000);
  
  waktuLama = micros(); 
}

void loop() {
  // Update data sudut Yaw dari IMU secara konstan di setiap siklus loop
  updateGyro();

  // --- PENYUSUNAN ALUR MISI GERAKAN ROBOT ---
  switch (langkahMisi) {
    
    case 0: // Langkah 1: Maju 80 cm
      if (jalankanMaju(80.0)) { 
        pindahLangkah(); // Jika true (selesai), lanjut ke case berikutnya
      }
      break;

    case 1: // Langkah 2: Berputar ke Kiri 90 derajat
      if (jalankanBelok(90.0)) { 
        pindahLangkah(); 
      }
      break;

    case 2: // Langkah 3: Bergerak Maju 40 cm
      if (jalankanMaju(40.0)) { 
        pindahLangkah(); 
      }
      break;

    default: // Semua sekuens di atas telah selesai dilaksanakan
      rem();
      static bool printSekali = true;
      if (printSekali) {
        Serial.println("==========================================");
        Serial.println("Misi Selesai! Seluruh rute telah dilewati.");
        Serial.println("==========================================");
        printSekali = false;
      }
      break;
  }

  // Panggil fungsi pemantau data di setiap putaran loop
  debugPergerakan();
}

// ====================================================================
// FUNCTION ACTION (MANDIRI & NON-BLOCKING)
// ====================================================================

// 1. Fungsi Gerakan Maju (Menggunakan Base-Code Kelurusan Anda)
bool jalankanMaju(float jarakCM) {
  static long targetPulsaOtomatis = 0;
  static float sudutTargetMaju = 0;

  // Set target di awal langkah
  if (baruMulaiLangkah) {
    pulsaKiri = 0;
    pulsaKanan = 0;
    float kelilingRoda = 3.14159265 * DIAMETER_RODA_CM;
    targetPulsaOtomatis = (jarakCM / kelilingRoda) * PPR_TOTAL_RODA;
    sudutTargetMaju = headingYaw; // Kunci arah hadap saat ini sebagai acuan lurus
    baruMulaiLangkah = false;
    Serial.println("\n--- [START] MANUVER MAJU ---");
  }

  // Pasok data ke variabel debug global
  dbgTarget = sudutTargetMaju;

  // Cek Jarak via Encoder
  long rataRataPulsa = (pulsaKiri + pulsaKanan) / 2;
  if (rataRataPulsa >= targetPulsaOtomatis) {
    rem();
    Serial.println("--- [STOP] MANUVER MAJU SELESAI ---");
    return true; // Selesai
  }

  // Logika Koreksi Dinamis (Sesuai Base Code Anda)
  float errorSudut = headingYaw - sudutTargetMaju; 
  float koreksi = errorSudut * Kp_IMU;

  dbgError = errorSudut; // Simpan untuk debug

  speedKiri  = BASE_PWM - koreksi; 
  speedKanan = BASE_PWM + koreksi;

  speedKiri  = constrain(speedKiri, 50, 200);
  speedKanan = constrain(speedKanan, 50, 200);

  dbgPwmOutKiri = speedKiri;   // Simpan untuk debug
  dbgPwmOutKanan = speedKanan; // Simpan untuk debug

  maju(); // Jalankan motor dengan PWM ter-update
  return false; // Belum selesai
}

// 2. Fungsi Gerakan Berputar (Positif = Kiri, Negatif = Kanan)
bool jalankanBelok(float targetSudutRelatif) {
  static float targetSudutGlobal = 0;

  if (baruMulaiLangkah) {
    // ---- LOGIKA KOMPENSASI OTOMATIS ----
    float sudutKompensasi = targetSudutRelatif;
    
    if (targetSudutRelatif > 0) {
      sudutKompensasi -= OFFSET_BELOK; // Jika ke kiri (positif), kurangi sudutnya
    } else if (targetSudutRelatif < 0) {
      sudutKompensasi += OFFSET_BELOK; // Jika ke kanan (negatif), tambah sudutnya (mendekati nol)
    }
    
    targetSudutGlobal = headingYaw + sudutKompensasi; 
    baruMulaiLangkah = false;
    Serial.print("-> Memulai Berputar ke Sudut (Telah Dikompensasi): "); Serial.println(targetSudutGlobal);
  }

  // Pasok data ke variabel debug global
  dbgTarget = targetSudutGlobal;

  float errorSudut = targetSudutGlobal - headingYaw;
  dbgError = errorSudut; // Simpan untuk debug

  // Toleransi kedekatan sudut belok (1.5 derajat)
  if (abs(errorSudut) < 1.5) {
    rem();
    Serial.print("--- [STOP] BERPUTAR SELESAI. AKHIR YAW: "); Serial.println(headingYaw);
    delay(150); // Jeda transisi agar sasis stabil
    return true; // Selesai belok
  }

  // Kontrol Kecepatan Putar Proporsional
  float koreksiSpeed = errorSudut * Kp_Belok;
  
  // Amankan torsi minimal motor saat mendekati sudut target (Agar motor tidak mendengung)
  if (koreksiSpeed > 0 && koreksiSpeed < 35)  koreksiSpeed = 35;
  if (koreksiSpeed < 0 && koreksiSpeed > -35) koreksiSpeed = -35;

  // Modifikasi: Constrain kini merujuk pada variabel independen BASE_TURN_PWM
  int pwmKiri  = constrain(koreksiSpeed, -BASE_TURN_PWM, BASE_TURN_PWM);
  int pwmKanan = constrain(-koreksiSpeed, -BASE_TURN_PWM, BASE_TURN_PWM);

  dbgPwmOutKiri = abs(pwmKiri);   // Simpan untuk debug
  dbgPwmOutKanan = abs(pwmKanan); // Simpan untuk debug

  // Pisahkan arah putaran roda (Pivot Turn / Berputar di tempat)
  digitalWrite(PIN_AIN1, (pwmKiri >= 0) ? HIGH : LOW);
  digitalWrite(PIN_AIN2, (pwmKiri >= 0) ? LOW : HIGH);
  analogWrite(PIN_PWMA, abs(pwmKiri));

  digitalWrite(PIN_BIN1, (pwmKanan >= 0) ? LOW : HIGH);
  digitalWrite(PIN_BIN2, (pwmKanan >= 0) ? HIGH : LOW);
  analogWrite(PIN_PWMB, abs(pwmKanan));

  return false; // Belum selesai
}

// Helper transisi antar case
void pindahLangkah() {
  baruMulaiLangkah = true;
  langkahMisi++;
}

// ====================================================================
// LOW LEVEL HARDWARE FUNCTIONS
// ====================================================================

void updateGyro() {
  if (IMU.gyroscopeAvailable()) {
    float x, y, z;
    IMU.readGyroscope(x, y, z);
    z = z - gyroZBias;

    unsigned long waktuSekarang = micros();
    float dt = (waktuSekarang - waktuLama) / 1000000.0;
    waktuLama = waktuSekarang;

    if (abs(z) > 0.3) { 
      headingYaw += z * dt; 
    }
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

// ====================================================================
// DEBUGGING PRINT UTILITY
// ====================================================================
void debugPergerakan() {
  static unsigned long waktuCetakLama = 0;
  // Membatasi pencetakan data setiap 80ms sekali agar Serial Monitor tidak lag
  if (millis() - waktuCetakLama > 80) {
    waktuCetakLama = millis();

    if (langkahMisi == 0 || langkahMisi == 2) { // Mode Jalan Maju
      Serial.print("[MAJU] Step:"); Serial.print(langkahMisi);
      Serial.print(" | Target_Sdt:"); Serial.print(dbgTarget, 1);
      Serial.print(" | Yaw:"); Serial.print(headingYaw, 1);
      Serial.print(" | Err:"); Serial.print(dbgError, 1);
      Serial.print(" | EncKiri:"); Serial.print(pulsaKiri);
      Serial.print(" | EncKanan:"); Serial.print(pulsaKanan);
      Serial.print(" | PWM_Ki:"); Serial.print(dbgPwmOutKiri);
      Serial.print(" | PWM_Ka:"); Serial.println(dbgPwmOutKanan);
    } 
    else if (langkahMisi == 1) { // Mode Berputar
      Serial.print("[BELOK] Step:"); Serial.print(langkahMisi);
      Serial.print(" | Target_Sdt:"); Serial.print(dbgTarget, 1);
      Serial.print(" | Yaw:"); Serial.print(headingYaw, 1);
      Serial.print(" | Err:"); Serial.print(dbgError, 1);
      Serial.print(" | PWM_Ki:"); Serial.print(dbgPwmOutKiri);
      Serial.print(" | PWM_Ka:"); Serial.println(dbgPwmOutKanan);
    }
  }
}
