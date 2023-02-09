from pymavlink import mavutil

address = "udpin:localhost:14551"  ## linux için simulasyon bağlantısı
## address = "/dev/ttyACM0" 
vehicle = mavutil.mavlink_connection(address, baud=115200, autoreconnect=True)
vehicle.wait_heartbeat()
print("baglanti basarili")


vehicle.set_mode("GUIDED") ## araç modunu değiştiriyor

vehicle.arducopter_arm()
## vehicle.motors_armed() ## motora arm verir arducopter simulasyonunda arm vermek için vehicle.arducopter_arm()
## vehicle.motors_armed_wait()

"""
vehicle.mav.command_long_send(vehicle.target_system, vehicle.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,0,0,0,0,0,0,0,10)
## takeof verir 10 değeri metreyi belirler
"""

"""
## takeoff verme fonksiyonunu kisaltmak için fonksiyon tanimlayip içine ekledik
def takeoff(altitude):
    vehicle.mav.command_long_send(vehicle.target_system, vehicle.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,0,0,0,0,0,0,0,altitude)
"""

"""
## anlik irtifa(altitude) değerini istenilen irtifaya ulaşana kadar ekrana yazdiran kod

## anlik irtifayi almamizi sağlayan fonksiyon
def anlik_irtifa_ver():
    msg = vehicle.recv_match(type="GLOBAL_POSITION_INT", blocking=True)
    altitude = msg.relative_alt
    altitude = altitude/1000
    return altitude

## takeoff verme fonksiyonunu kisaltmak için fonksiyon tanimlayip içine ekledik
def takeoff(altitude):
    vehicle.mav.command_long_send(vehicle.target_system, vehicle.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,0,0,0,0,0,0,0,altitude)
    while True:
        anlik_irtifa = anlik_irtifa_ver()
        if anlik_irtifa < altitude:
            print(f"anlik irtifa: {anlik_irtifa}")
        elif anlik_irtifa >= altitude:
            print(f"istenilen irtifaya ulasildi")
            break
"""

 