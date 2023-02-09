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

## anlik irtifa(altitude) değerini istenilen irtifaya ulaşana kadar ekrana yazdiran kod:
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

takeoff(10)

## dronu belirlenen konuma götürür
"""

vehicle.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
    10,                        ## timrstamp
    vehicle.target_system,     ## system ID
    vehicle.target_component,  ## component ID
    9,                         ## kordinat çerçevesi(coordinate frame) /yanliş anlamadiysam girecegimiz xyz degerleri için bir tip belirtiyor
    int(0b011111111000),       ## ???
    -20,-20,20,                ## xyz degerleri (z negatif olursa altitude artar)
    0,0,0,                     ## xyz'deki hiz degerleri
    0,0,0,                     ## xyz'deki ivme degerleri
    0,0                        ## yaw setpoint, yaw rate setpoint
))

def drone_git(coor_frame,x_konum,y_konum,z_konum,x_hiz,y_hiz,z_hiz,x_ivme,y_ivme,z_ivme):
    vehicle.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
        10,                        ## timrstamp
        vehicle.target_system,     ## system ID
        vehicle.target_component,  ## component ID
        coor_frame,                ## kordinat çerçevesi(coordinate frame) /yanliş anlamadiysam girecegimiz xyz degerleri için bir tip belirtiyor
        int(0b011111111000),       ## ???
        x_konum,y_konum,z_konum,   ## xyz degerleri (z negatif olursa altitude artar)
        x_hiz,y_hiz,z_hiz,         ## xyz'deki hiz degerleri
        x_ivme,y_ivme,z_ivme,      ## xyz'deki ivme degerleri
        0,0                        ## yaw setpoint, yaw rate setpoint
    ))

"""

## dronu istenilen konuma götürür (mission olarak ne farkı var bilmiyorum)
"""
##vehicle.mav.mission_item_send()
vehicle.mav.send(mavutil.mavlink.MAVLink_mission_item_message(
    vehicle.target_system,     ## system ID
    vehicle.target_component,  ## component ID
    0,                         ## sequence ???
    3,                         ## kordinat çerçevesi(coordinate frame) /yanliş anlamadiysam girecegimiz xyz degerleri için bir tip belirtiyor  burası 3 iken çalışıyor ama 9 iken niye çalışmıyor
    mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, ## Mav_cmd belirt komut tipi
    2,0,0,0,0,0,
    35.3621936,-149.1651106,10
))

def drone_git_mission(coor_frame, lat, lon, altitude):
    vehicle.mav.send(mavutil.mavlink.MAVLink_mission_item_message(
        vehicle.target_system,     ## system ID
        vehicle.target_component,  ## component ID
        0,                         ## sequence ???
        coor_frame,                         ## kordinat çerçevesi(coordinate frame) /yanliş anlamadiysam girecegimiz xyz degerleri için bir tip belirtiyor  burası 3 iken çalışıyor ama 9 iken niye çalışmıyor
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, ## Mav_cmd belirt komut tipi
        2,0,0,0,0,0,
        lat,lon,altitude
    ))
"""


"""
##vehicle.mav.mission_item_send()
vehicle.mav.send(mavutil.mavlink.MAVLink_mission_item_message(
    vehicle.target_system,     ## system ID
    vehicle.target_component,  ## component ID
    0,                         ## sequence ???
    3,                         ## kordinat çerçevesi(coordinate frame) /yanliş anlamadiysam girecegimiz xyz degerleri için bir tip belirtiyor
    mavutil.mavlink.MAV_CMD_NAV_LAND, ## Mav_cmd belirt komut tipi
    0,0,0,0,0,0,
    35.3621936,-149.1651106,10
))

vehicle.mav.send(mavutil.mavlink.MAVLink_mission_item_message(
    vehicle.target_system,     ## system ID
    vehicle.target_component,  ## component ID
    0,                         ## sequence ???
    3,                         ## kordinat çerçevesi(coordinate frame) /yanliş anlamadiysam girecegimiz xyz degerleri için bir tip belirtiyor
    mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, ## Mav_cmd belirt komut tipi
    2,0,0,0,0,0,
    -20,-20,10
))
"""

vehicle.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
    10,                        ## timrstamp
    vehicle.target_system,     ## system ID
    vehicle.target_component,  ## component ID
    9,                         ## kordinat çerçevesi(coordinate frame) /yanliş anlamadiysam girecegimiz xyz degerleri için bir tip belirtiyor
    int(0b011111111000),       ## ???
    -20,-20,20,                ## xyz degerleri (z negatif olursa altitude artar)
    0,0,0,                     ## xyz'deki hiz degerleri
    0,0,0,                     ## xyz'deki ivme degerleri
    0,0   ))