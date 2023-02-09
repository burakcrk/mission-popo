from pymavlink import mavutil

address = "udpin:localhost:14551"  ## linux için simulasyon bağlantısı
## address = "/dev/ttyACM0"  ## linux için usb bağlantısı 
## address = "com10"
## address = "tcp:127.0.0.1:5760"

## address = "/dev/ttyTSH1:115200"

vehicle = mavutil.mavlink_connection(address, baud=115200, autoreconnect=False)  ## karta bağlanma 
print("baglanti basarili-1")

vehicle.wait_heartbeat() ## bağlantıyı bekle
print("baglanti basarili-2")

"""
while True:
    message = vehicle.recv_match(blocking = True) # dronedan bilgi alır
    print(message)
"""

"""
msg = vehicle.recv_match(type="BATTERY_STATUS", blocking=True)
print(f"Batarya yüzdesi: {msg.battery_remaining}")
"""

"""
msg = vehicle.recv_match(type="VFR_HUD", blocking=True)
print(f"Airspeed: {msg.airspeed}")
"""

""" 
## uçuş modu görüntüleme
msg = vehicle.recv_match(type="HEARTBEAT", blocking=True)
mode = mavutil.mode_string_v10(msg)
print(f"Mode: {mode}")
""" 