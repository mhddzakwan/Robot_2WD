import serial
import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator

# ====================================================================
# CONFIGURATION
# ====================================================================
SERIAL_PORT = '/dev/ttyAMA0'  # Port RX/TX Raspberry Pi 5
BAUD_RATE = 128000           # Baud rate standar YDLIDAR X4 adalah 128000

# Setup Kanvas Plotting Matplotlib
plt.ion()  # Mengaktifkan mode interaktif agar grafik bisa update real-time
fig, ax = plt.subplots(figsize=(8, 8))
scatter = ax.scatter([], [], s=10, c='red', label='Objek Terdeteksi')
robot_dot = ax.scatter([0], [0], s=100, c='blue', marker='^', label='Robot')

# Konfigurasi Grid Sesuai Request (Tiap kotak = 30 cm)
ax.set_xlim(-300, 300)  # Rentang visualisasi 3 meter ke kiri/kanan
ax.set_ylim(-300, 300)  # Rentang visualisasi 3 meter ke depan/belakang
ax.grid(True, which='both', linestyle='--', color='gray', alpha=0.7)

# Mengatur interval grid per 30 cm
ax.xaxis.set_major_locator(MultipleLocator(30))
ax.yaxis.set_major_locator(MultipleLocator(30))

ax.set_title("Visualisasi Lidar YDLIDAR - Grid Kotak 30 cm")
ax.set_xlabel("X (cm)")
ax.set_ylabel("Y (cm)")
ax.legend()

# Variabel penyimpan koordinat global
all_x = []
all_y = []

def parse_ydlidar_packet(packet):
    """
    Memparsing paket data biner mentah YDLIDAR menjadi koordinat X dan Y (cm)
    """
    x_points = []
    y_points = []
    
    if len(packet) < 10:
        return x_points, y_points

    # Membaca jumlah sample data (Byte ke-3)
    lsn = packet[3]
    
    # Membaca Sudut Awal (FSA) dan Sudut Akhir (LSA)
    fsa = (packet[5] << 8 | packet[4]) >> 1
    lsa = (packet[7] << 8 | packet[6]) >> 1
    
    fsa_angle = fsa / 64.0
    lsa_angle = lsa / 64.0
    
    # Hitung selisih sudut penyebaran sampel
    if lsa_angle >= fsa_angle:
        angle_diff = lsa_angle - fsa_angle
    else:
        angle_diff = (lsa_angle + 360.0) - fsa_angle

    # Ambil data jarak (dimulai dari byte ke-10, tiap data berukuran 2 byte)
    data_index = 10
    for i in range(lsn):
        if data_index + 1 >= len(packet):
            break
            
        # Menggabungkan 2 byte data jarak (mm)
        distance_raw = packet[data_index + 1] << 8 | packet[data_index]
        distance_mm = distance_raw / 4.0  # Formula konversi jarak YDLIDAR
        
        # Konversi ke Centimeter untuk kebutuhan Grid 30 cm Anda
        distance_cm = distance_mm / 10.0 
        
        if distance_cm > 0:  # Abaikan jika data 0 (tidak ada pantulan/blank)
            # Menghitung sudut interpolasi tiap titik sampel
            angle = (angle_diff / (lsn - 1 if lsn > 1 else 1)) * i + fsa_angle
            angle_rad = np.radians(angle)
            
            # Koreksi arah matematis sudut Lidar ke Kartesian Robot
            # Menghasilkan koordinat relatif terhadap posisi robot (0,0)
            x = distance_cm * np.sin(angle_rad)
            y = distance_cm * np.cos(angle_rad)
            
            x_points.append(x)
            y_points.append(y)
            
        data_index += 2
        
    return x_points, y_points

def main():
    global all_x, all_y
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        print(f"[INFO] Terhubung ke YDLIDAR pada {SERIAL_PORT}")
        
        buffer = bytearray()
        last_update = time.time()

        while True:
            if ser.in_waiting > 0:
                buffer.extend(ser.read(ser.in_waiting))
                
                # Cari header 'AA 55' di dalam aliran data byte
                while len(buffer) >= 10:
                    if buffer[0] == 0xAA and buffer[1] == 0x55:
                        # Dapatkan perkiraan panjang paket data berdasarkan jumlah sampel LSN
                        lsn = buffer[3]
                        packet_size = 10 + (lsn * 2)
                        
                        if len(buffer) < packet_size:
                            break # Tunggu data terkumpul lengkap
                        
                        # Potong paket utuh untuk diproses
                        packet = buffer[:packet_size]
                        del buffer[:packet_size]
                        
                        # Abaikan paket indeks nol (buffer[2] == 0x01), fokus ke paket data murni (0x00)
                        if packet[2] == 0x00:
                            x_pts, y_pts = parse_ydlidar_packet(packet)
                            all_x.extend(x_pts)
                            all_y.extend(y_pts)
                    else:
                        # Buang byte jika bukan header AA 55
                        buffer.pop(0)
            
            # Batasi proses render grafik 15 FPS agar Raspberry Pi 5 tidak lag
            if time.time() - last_update > 0.06:
                if all_x and all_y:
                    # Perbarui posisi titik data pada grafik secara dinamis
                    scatter.set_offsets(np.c_[all_x, all_y])
                    fig.canvas.draw()
                    fig.canvas.flush_events()
                    
                    # Reset array penampung untuk putaran scan berikutnya
                    all_x.clear()
                    all_y.clear()
                    
                last_update = time.time()
                
            time.sleep(0.001)

    except serial.SerialException as e:
        print(f"[ERROR] Port error: {e}")
    except KeyboardInterrupt:
        print("\n[INFO] Visualisasi dihentikan.")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
        plt.ioff()
        plt.show()

if __name__ == '__main__':
    main()
