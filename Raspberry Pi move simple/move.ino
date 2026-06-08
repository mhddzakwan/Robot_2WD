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

// === VARIABEL BARU UNTUK KENDALI SERIAL RASPY 5 ===
char tipePerintah = 'S'; // 'S' = Standby/Stop, 'M' = Maju, 'B' = Belok
float nilaiTarget = 0.0;
bool sedangEksekusi = false;

void setup() {
  Serial.begin(115200); // Dinaikkan ke 115200 agar komunikasi dengan Raspy 5 instan tanpa delay

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

  // Terima instruksi baru dari Raspberry Pi 5 jika robot sedang diam
  terimaPerintahRaspy();

  // --- EKSEKUSI PERINTAH DINAMIS DARI RASPY ---
  if (sedangEksekusi) {
    if (tipePerintah == 'M') {
      if (jalankanMaju(nilaiTarget)) { 
        Serial.println("DONE"); // Beritahu Raspy 5 kalau maju selesai
        sedangEksekusi = false;
        tipePerintah = 'S';
        baruMulaiLangkah = true;
      }
    } 
    else if (tipePerintah == 'B') {
      if (jalankanBelok(nilaiTarget)) { 
        Serial.println("DONE"); // Beritahu Raspy 5 kalau belok selesai
        sedangEksekusi = false;
        tipePerintah = 'S';
        baruMulaiLangkah = true;
      }
    }
  } else {
    rem(); // Jaga posisi jika tidak ada task aktif
  }

  // Panggil fungsi pemantau data di setiap putaran loop
  debugPergerakan();
}

// ====================================================================
// SERIAL PARSER FUNCTION FOR RASPBERRY PI
// ====================================================================
void terimaPerintahRaspy() {
  if (Serial.available() > 0) {
    String inputStr = Serial.readStringUntil('\n');
    inputStr.trim();
    
    // Format input: Tipe,Nilai (Contoh: M,80.0 atau B,-90.0)
    int indexKoma = inputStr.indexOf(',');
    if (indexKoma > 0) {
      String cmd = inputStr.substring(0, indexKoma);
      String val = inputStr.substring(indexKoma + 1);
      
      tipePerintah = cmd.charAt(0);
      nilaiTarget = val.toFloat();
      sedangEksekusi = true;
      baruMulaiLangkah = true; // Trigger inisialisasi parameter di fungsi gerakan asli
    }
  }
}

// ====================================================================
// FUNCTION ACTION (MANDIRI & NON-BLOCKING) - TIDAK DIUBAH
// ====================================================================

bool jalankanMaju(float jarakCM) {
  static long targetPulsaOtomatis = 0;
  static float sudutTargetMaju = 0;

  if (baruMulaiLangkah) {
    pulsaKiri = 0;
    pulsaKanan = 0;
    float kelilingRoda = 3.14159265 * DIAMETER_RODA_CM;
    targetPulsaOtomatis = (jarakCM / kelilingRoda) * PPR_TOTAL_RODA;
    sudutTargetMaju = headingYaw; 
    baruMulaiLangkah = false;
    Serial.println("\n--- [START] MANUVER MAJU ---");
  }

  dbgTarget = sudutTargetMaju;

  long rataRataPulsa = (pulsaKiri + pulsaKanan) / 2;
  if (rataRataPulsa >= targetPulsaOtomatis) {
    rem();
    Serial.println("--- [STOP] MANUVER MAJU SELESAI ---");
    return true; 
  }

  float errorSudut = headingYaw - sudutTargetMaju; 
  float koreksi = errorSudut * Kp_IMU;

  dbgError = errorSudut; 

  speedKiri  = BASE_PWM - koreksi; 
  speedKanan = BASE_PWM + koreksi;

  speedKiri  = constrain(speedKiri, 50, 200);
  speedKanan = constrain(speedKanan, 50, 200);

  dbgPwmOutKiri = speedKiri;   
  dbgPwmOutKanan = speedKanan; 

  maju(); 
  return false; 
}

bool jalankanBelok(float targetSudutRelatif) {
  static float targetSudutGlobal = 0;

  if (baruMulaiLangkah) {
    float sudutKompensasi = targetSudutRelatif;
    
    if (targetSudutRelatif > 0) {
      sudutKompensasi -= OFFSET_BELOK; 
    } else if (targetSudutRelatif < 0) {
      sudutKompensasi += OFFSET_BELOK; 
    }
    
    targetSudutGlobal = headingYaw + sudutKompensasi; 
    baruMulaiLangkah = false;
    Serial.print("-> Memulai Berputar ke Sudut (Telah Dikompensasi): "); Serial.println(targetSudutGlobal);
  }

  dbgTarget = targetSudutGlobal;

  float errorSudut = targetSudutGlobal - headingYaw;
  dbgError = errorSudut; 

  if (abs(errorSudut) < 1.5) {
    rem();
    Serial.print("--- [STOP] BERPUTAR SELESAI. AKHIR YAW: "); Serial.println(headingYaw);
    delay(150); 
    return true; 
  }

  float koreksiSpeed = errorSudut * Kp_Belok;
  
  if (koreksiSpeed > 0 && koreksiSpeed < 35)  koreksiSpeed = 35;
  if (koreksiSpeed < 0 && koreksiSpeed > -35) koreksiSpeed = -35;

  int pwmKiri  = constrain(koreksiSpeed, -BASE_TURN_PWM, BASE_TURN_PWM);
  int pwmKanan = constrain(-koreksiSpeed, -BASE_TURN_PWM, BASE_TURN_PWM);

  dbgPwmOutKiri = abs(pwmKiri);   
  dbgPwmOutKanan = abs(pwmKanan); 

  digitalWrite(PIN_AIN1, (pwmKiri >= 0) ? HIGH : LOW);
  digitalWrite(PIN_AIN2, (pwmKiri >= 0) ? LOW : HIGH);
  analogWrite(PIN_PWMA, abs(pwmKiri));

  digitalWrite(PIN_BIN1, (pwmKanan >= 0) ? LOW : HIGH);
  digitalWrite(PIN_BIN2, (pwmKanan >= 0) ? HIGH : LOW);
  analogWrite(PIN_PWMB, abs(pwmKanan));

  return false; 
}

void pindahLangkah() {
  baruMulaiLangkah = true;
  langkahMisi++;
}

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

void debugPergerakan() {
  static unsigned long waktuCetakLama = 0;
  if (millis() - waktuCetakLama > 100) {
    waktuCetakLama = millis();
    if (sedangEksekusi) {
      Serial.print("[RUNNING] Cmd:"); Serial.print(tipePerintah);
      Serial.print(" | Target:"); Serial.print(nilaiTarget);
      Serial.print(" | Current Yaw:"); Serial.println(headingYaw, 1);
    }
  }
}
