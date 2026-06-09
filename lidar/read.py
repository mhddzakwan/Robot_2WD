import serial
import time
import sys

# ====================================================================
# CONFIGURATION
# ====================================================================
# Pada Raspberry Pi 5, port UART utama biasanya /dev/ttyAMA0 atau /dev/serial0
SERIAL_PORT = '/dev/ttyAMA0' 

# Sesuaikan BAUD_RATE dengan spesifikasi Lidar Anda:
# - RPLIDAR A1 biasanya: 115200
# - YDLIDAR X4 biasanya: 128000
# - TF Mini (LiDAR Jarak) biasanya: 115200
BAUD_RATE = 115200 

def read_lidar_raw():
    try:
        # Inisialisasi koneksi Serial UART
        ser = serial.Serial(
            port=SERIAL_PORT,
            baudrate=BAUD_RATE,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1
        )
        
        print(f"[INFO] Membuka port {SERIAL_PORT} dengan Baud Rate {BAUD_RATE}...")
        time.sleep(1)
        
        # Bersihkan buffer sisa sebelum membaca
        ser.reset_input_buffer()
        print("[READY] Membaca data mentah Lidar (Tekan Ctrl+C untuk berhenti):\n")

        while True:
            if ser.in_waiting > 0:
                # Membaca data byte demi byte dari Lidar
                # Anda bisa mengubah angka 1 menjadi jumlah byte per paket jika tipe Lidar Anda spesifik (misal: 9 atau 47)
                raw_byte = ser.read(1) 
                
                if raw_byte:
                    # Mengonversi byte murni ke format Hexadecimal agar mudah dibaca manusia
                    hex_data = raw_byte.hex().upper()
                    
                    # Cetak ke terminal secara menyamping (dipisahkan spasi)
                    sys.stdout.write(hex_data + " ")
                    sys.stdout.flush()
                    
            time.sleep(0.0001) # Delay super kecil agar CPU tidak over-utilization

    except serial.SerialException as e:
        print(f"\n[ERROR] Gagal membuka atau membaca port serial: {e}")
    except KeyboardInterrupt:
        print("\n\n[INFO] Membaca data dihentikan oleh pengguna.")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("[INFO] Port Serial Ditutup.")

if __name__ == '__main__':
    read_lidar_raw()
