import canopen
import time
import sys

# ===================================================================
# KONFIGURASI PORT DAN ENCODER (LINUX)
# ===================================================================

# Berdasarkan info terbaru Anda: /dev/ttyACM0 (CAN)
BUS_TYPE = 'slcan'
CHANNEL = '/dev/ttyACM0' 
BITRATE = 125000
NODE_ID_LAMA = 1
NODE_ID_BARU = 2
EDS_FILE = 'Encoder_.eds'

def ubah_id():
    print(f"--- Memulai Prosedur Setup Node ID {NODE_ID_BARU} (Linux) ---")
    network = canopen.Network()
    
    try:
        # Menghubungkan ke adapter USB-to-CAN
        # Jika error 'Permission Denied', jalankan: sudo chmod 666 /dev/ttyACM0
        network.connect(bustype=BUS_TYPE, channel=CHANNEL, bitrate=BITRATE)
        print(f"Terhubung ke {CHANNEL} pada bitrate {BITRATE}bps")
    except Exception as e:
        print(f"Gagal terhubung ke adapter CAN: {e}")
        print(f"Saran: Jalankan dengan 'sudo' atau cek izin port {CHANNEL}")
        sys.exit(1)

    try:
        # Menambahkan node dengan ID 1 (Default Pabrik)
        node = network.add_node(NODE_ID_LAMA, EDS_FILE)
        
        # Masuk ke mode Pre-Operational agar SDO lebih stabil saat ganti parameter
        print("Mengatur encoder ke state PRE-OPERATIONAL...")
        node.nmt.state = 'PRE-OPERATIONAL'
        time.sleep(0.5)

        # 1. Mengubah Node ID (Mencoba Index 0x3000 sesuai manual)
        print(f"Mengirim perintah perubahan ID ke {NODE_ID_BARU}...")
        
        berhasil_tulis = False
        
        # Strategi A: Paksa 1 Byte (Unsigned8)
        try:
            print("Mencoba kirim data 1 byte...")
            node.sdo.download(0x3000, 0, bytes([NODE_ID_BARU]))
            berhasil_tulis = True
        except Exception as e1:
            print(f"Gagal dengan 1 byte: {e1}")
            
            # Strategi B: Paksa 4 Byte (Unsigned32) - Seringkali firmware minta ini
            try:
                print("Mencoba kirim data 4 byte (alternatif)...")
                # Mengirim byte [2, 0, 0, 0]
                node.sdo.download(0x3000, 0, NODE_ID_BARU.to_bytes(4, 'little'))
                berhasil_tulis = True
            except Exception as e2:
                print(f"Gagal dengan 4 byte: {e2}")

        if not berhasil_tulis:
            print("Gagal total mengubah ID pada Index 0x3000. Cek koneksi kabel!")
            return

        time.sleep(0.5)
        
        # 2. Menyimpan pengaturan secara permanen (Save to EEPROM)
        # Menurut manual: Index 0x1010 sub 01, data 'save' (0x65766173)
        print("Menyimpan konfigurasi ke memori permanen (EEPROM)...")
        try:
            node.sdo[0x1010][1].raw = 0x65766173
            print("Penyimpanan (Save) berhasil!")
        except Exception as e:
            print(f"Gagal menyimpan via Object Dictionary: {e}")
            try:
                # Cara manual kirim string 'save'
                node.sdo.download(0x1010, 1, b'save')
                print("Penyimpanan manual 'save' berhasil!")
            except:
                print("Gagal total menyimpan ke EEPROM.")
                return

        # 3. Reset Node (Wajib agar ID baru aktif)
        print("Mengirim perintah Reset Node (NMT)...")
        network.nmt.send_command(0x81) # 0x81 = Reset All Nodes
        
        print("\n" + "="*40)
        print(f"[SUKSES] Perintah perubahan ID {NODE_ID_BARU} telah dikirim.")
        print("LANGKAH PENTING:")
        print("1. MATIKAN POWER ENCODER, lalu nyalakan lagi (Cold Reset).")
        print("2. Cek apakah encoder sekarang merespon pada ID 2.")
        print("="*40)
        
    except Exception as e:
        print(f"\n[ERROR SISTEM]: {e}")
    finally:
        network.disconnect()

if __name__ == "__main__":
    ubah_id()
