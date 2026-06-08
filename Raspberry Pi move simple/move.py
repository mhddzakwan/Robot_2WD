import serial
import time

# Sesuaikan dengan port USB Arduino Nano BLE Anda di Raspberry Pi 5
SERIAL_PORT = '/dev/ttyACM0' 
BAUD_RATE = 115200

def kirim_perintah(ser, tipe, nilai):
    """
    Mengirim perintah gerakan ke Arduino dan memblokir alur program 
    sampai Arduino mengirimkan respon 'DONE'.
    """
    command = f"{tipe},{nilai}\n"
    print(f"[RASPY] Mengirim Perintah: {command.strip()}")
    ser.write(command.encode('utf-8'))
    
    # Tunggu feedback sampai eksekusi fisik selesai
    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').strip()
            
            # Saring data debugging jika ada log running, tampilkan di terminal Raspy
            if "[RUNNING]" in line:
                print(line)
                
            if line == "DONE":
                print(f"[RASPY] Perintah {tipe} Selesai Dieksekusi.\n")
                break
        time.sleep(0.01)

def main():
    try:
        # Inisialisasi port Serial
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print("[RASPY] Menghubungkan ke Arduino Nano BLE...")
        time.sleep(3) # Jeda waktu tunggu inisialisasi koneksi serial awal
        print("[RASPY] Koneksi Terhubung.")
        print("====================================================")
        print("FORMAT INPUT TERMINAL:")
        print("1. Maju       -> m [jarak_cm]  (Contoh: m 50)")
        print("2. Belok Kiri -> b [derajat]   (Contoh: b 90)")
        print("3. Belok Kanan-> b -[derajat]  (Contoh: b -90)")
        print("4. Keluar     -> exit")
        print("====================================================")

        while True:
            # Mengambil input dari pengguna di terminal
            userInput = input("Masukkan perintah: ").strip()
            
            # Cek jika user ingin keluar dari program
            if userInput.lower() == 'exit':
                print("[RASPY] Keluar dari program kendali.")
                break
                
            # Validasi string input tidak boleh kosong
            if not userInput:
                continue
                
            try:
                # Memisahkan berdasarkan spasi (Contoh: "m 80.0" -> ['m', '80.0'])
                parts = userInput.split()
                
                if len(parts) != 2:
                    print("[ERROR] Format salah! Gunakan spasi sebagai pemisah. Contoh: m 50")
                    continue
                    
                cmd_raw = parts[0].upper() # Ambil karakter perintah ('M' atau 'B')
                val_raw = float(parts[1])  # Ambil nilai target dan ubah ke float
                
                # Validasi karakter perintah yang diizinkan
                if cmd_raw not in ['M', 'B']:
                    print("[ERROR] Perintah tidak dikenal! Hanya gunakan huruf 'm' atau 'b'.")
                    continue
                
                # Eksekusi pengiriman data ke Arduino
                kirim_perintah(ser, cmd_raw, val_raw)
                
            except ValueError:
                print("[ERROR] Nilai jarak/sudut harus berupa angka!")
            except Exception as e:
                print(f"[ERROR] Terjadi kendala: {e}")

    except serial.SerialException as e:
        print(f"[ERROR] Gagal membuka port serial: {e}")
    except KeyboardInterrupt:
        print("\n[RASPY] Program dihentikan paksa oleh pengguna.")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("[RASPY] Port Serial Ditutup.")

if __name__ == '__main__':
    main()
