// ====================================================================
// PIN CONFIGURATION (TB6612FNG & ENCODER)
// ====================================================================

// Motor Kiri (A)
const int PIN_PWMA = 5;
const int PIN_AIN1 = 6;
const int PIN_AIN2 = 7;
const int ENCA = 2; // Pin D2 (OUT A Motor U5)

// Motor Kanan (B)
const int PIN_PWMB = 10;
const int PIN_BIN1 = 8;
const int PIN_BIN2 = 9;
const int ENCB = 11; // Pin D11 (OUT A Motor U6)

// Standby Pin Driver TB6612
const int STBY = 4;

// ====================================================================
// CONFIGURATION INPUT (INPUT DI SINI)
// ====================================================================
const float TARGET_JARAK_CM = 80.0;  // Input jarak yang diinginkan (dalam cm)
const float DIAMETER_RODA_CM = 6.5;   // Input diameter roda Anda (dalam cm)
const int   PPR_TOTAL_RODA   = 620;   // Input nilai PPR Roda hasil kalibrasi manual (PPR 18)

// ====================================================================
// VARIABLES & AUTOMATIC CALCULATION
// ====================================================================
volatile long pulsaKiri = 0;
volatile long pulsaKanan = 0;

// Variabel ini akan dihitung otomatis oleh sistem di dalam setup()
long targetPulsaOtomatis = 0; 

// Kecepatan awal motor (PWM: 0 - 255)
int speedKiri = 100;
int speedKanan = 100; 

// Interval pembacaan koreksi kelurusan robot (milidetik)
const int intervalKoreksi = 50; 
unsigned long waktuLama = 0;

void setup() {
  Serial.begin(9600);

  // Setup Pin Driver Motor sebagai Output
  pinMode(PIN_PWMA, OUTPUT); pinMode(PIN_AIN1, OUTPUT); pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_PWMB, OUTPUT); pinMode(PIN_BIN1, OUTPUT); pinMode(PIN_BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);

  // Setup Pin Encoder dengan internal Pullup
  pinMode(ENCA, INPUT_PULLUP);
  pinMode(ENCB, INPUT_PULLUP);

  // Aktifkan Driver Motor
  digitalWrite(STBY, HIGH);

  // Pasang Interrupt khusus arsitektur Nano 33 BLE Sense
  attachInterrupt(digitalPinToInterrupt(ENCA), countPulsaKiri, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCB), countPulsaKanan, RISING);

  // --- PROSES HITUNG OTOMATIS ---
  float kelilingRoda = 3.14159265 * DIAMETER_RODA_CM;
  float jumlahPutaranRoda = TARGET_JARAK_CM / kelilingRoda;
  targetPulsaOtomatis = jumlahPutaranRoda * PPR_TOTAL_RODA;

  // Menampilkan hasil kalkulasi ke Serial Monitor saat start
  Serial.println("=== SISTEM SELESAI KALKULASI ===");
  Serial.print("Target Jarak    : "); Serial.print(TARGET_JARAK_CM); Serial.println(" cm");
  Serial.print("Keliling Roda   : "); Serial.print(kelilingRoda); Serial.println(" cm");
  Serial.print("Target Pulsa    : "); Serial.println(targetPulsaOtomatis);
  Serial.println("Robot bersiap... Mulai berjalan dalam 2 detik.");
  Serial.println("==========================================");
  
  delay(2000); 
  
  // Jalankan motor pertama kali
  maju();
}

void loop() {
  // 1. Cek apakah salah satu roda sudah mencapai target jarak
  if (pulsaKiri >= targetPulsaOtomatis || pulsaKanan >= targetPulsaOtomatis) {
    rem();
    Serial.println("==========================================");
    Serial.print("Target Jarak Tercapai! Pulsa Kiri: ");
    Serial.print(pulsaKiri);
    Serial.print(" | Kanan: ");
    Serial.println(pulsaKanan);
    Serial.println("Robot Berhenti.");
    Serial.println("==========================================");
    
    // Mengunci program di sini agar robot tidak berjalan lagi
    while (1); 
  }

  // 2. Logika Penyeimbang Kecepatan (Koreksi Jalan Lurus) setiap 50 ms
  unsigned long waktuSekarang = millis();
  if (waktuSekarang - waktuLama >= intervalKoreksi) {
    waktuLama = waktuSekarang;

    // Hitung selisih pulsa antara roda kiri dan kanan
    long selisih = pulsaKiri - pulsaKanan;

    // Jika selisih positif, artinya roda kiri berputar lebih cepat dari kanan
    if (selisih > 3) {
      speedKiri--;   // Perlambat roda kiri
      speedKanan++;  // Percepat roda kanan
    } 
    // Jika selisih negatif, artinya roda kanan berputar lebih cepat dari kiri
    else if (selisih < -3) {
      speedKiri++;   // Percepat roda kiri
      speedKanan--;  // Perlambat roda kanan
    }

    // Batasi nilai PWM agar tetap stabil di range aman
    speedKiri = constrain(speedKiri, 90, 210);
    speedKanan = constrain(speedKanan, 90, 210);

    // Terapkan penyesuaian kecepatan baru ke driver motor
    analogWrite(PIN_PWMA, speedKiri);
    analogWrite(PIN_PWMB, speedKanan);

    // Debugging data secara real-time ke Serial Monitor
    Serial.print("Pulsa_Kiri:"); Serial.print(pulsaKiri);
    Serial.print("\tPulsa_Kanan:"); Serial.print(pulsaKanan);
    Serial.print("\tPWM_Kiri:"); Serial.print(speedKiri);
    Serial.print("\tPWM_Kanan:"); Serial.println(speedKanan);
  }
}

// ====================================================================
// MOTOR DIRECTIONS FUNCTIONS
// ====================================================================

void maju() {
  // Konfigurasi arah maju Motor Kiri
  digitalWrite(PIN_AIN1, HIGH);
  digitalWrite(PIN_AIN2, LOW);
  analogWrite(PIN_PWMA, speedKiri);

  // Konfigurasi arah maju Motor Kanan
  digitalWrite(PIN_BIN1, LOW);
  digitalWrite(PIN_BIN2, HIGH);
  analogWrite(PIN_PWMB, speedKanan);
}

void rem() {
  // Menggunakan fitur short-brake (mengunci motor agar berhenti instan)
  digitalWrite(PIN_AIN1, HIGH);
  digitalWrite(PIN_AIN2, HIGH);
  digitalWrite(PIN_BIN1, HIGH);
  digitalWrite(PIN_BIN2, HIGH);
  analogWrite(PIN_PWMA, 0);
  analogWrite(PIN_PWMB, 0);
}

// ====================================================================
// INTERRUPT SERVICE ROUTINES (ISR)
// ====================================================================
void countPulsaKiri() {
  pulsaKiri++;
}

void countPulsaKanan() {
  pulsaKanan++;
}
