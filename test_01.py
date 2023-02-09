from pymavlink import mavutil, mavwp
import time

########## CONNECTION START ##########

addr = "udpin:localhost:14551"    ## for linux ardupilot connection
# address = "com10"    ## for windows usb connection
# address = "/dev/ttyACM0"    ## for linux usb connection 

drone = mavutil.mavlink_connection(addr, baud=115200, autoreconnect=True)
print("connection successful - 1")
drone.wait_heartbeat()    ## wait heartbeat
print("connection successful - 2")

########## CONNECTION END ##########

########## FUNCTION START ##########

def takeoff(altitude):
    drone.mav.command_long_send(
        drone.target_system,
        drone.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,0,0,0,0,0,0,
        altitude
        )
    while True:
        current_altitude = current_altitude_func()
        if current_altitude < altitude:
            print(f"anlik irtifa: {current_altitude}")
        elif current_altitude >= altitude:
            print(f"istenilen irtifaya ulasildi")
            break

def current_altitude_func():
    msg = drone.recv_match(type="GLOBAL_POSITION_INT", blocking=True)
    altitude = msg.relative_alt
    altitude = altitude/1000
    return altitude

def drone_move(coor_frame,x_konum,y_konum,z_konum,x_hiz,y_hiz,z_hiz,x_ivme,y_ivme,z_ivme):
    drone.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
        10,                        ## timrstamp
        drone.target_system,       ## system ID
        drone.target_component,    ## component ID
        coor_frame,                ## kordinat çerçevesi(coordinate frame) /yanliş anlamadiysam girecegimiz xyz degerleri için bir tip belirtiyor
        int(0b011111111000),       ## ???
        x_konum,y_konum,z_konum,   ## xyz degerleri (z negatif olursa altitude artar)
        x_hiz,y_hiz,z_hiz,         ## xyz'deki hiz degerleri
        x_ivme,y_ivme,z_ivme,      ## xyz'deki ivme degerleri
        0,0                        ## yaw setpoint, yaw rate setpoint
    ))

def drone_go_to(lat, lon, altitude):
    drone.mav.send(mavutil.mavlink.MAVLink_mission_item_message(
        drone.target_system,                    ## system ID
        drone.target_component,                 ## component ID
        0,                                      ## sequence ???
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,                             ## kordinat çerçevesi(coordinate frame) /yanliş anlamadiysam girecegimiz xyz degerleri için bir tip belirtiyor  burası 3 iken çalışıyor ama 9 iken niye çalışmıyor
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,   ## Mav_cmd belirt komut tipi
        2,0,0,0,0,0,
        lat,lon,altitude
    ))

wp = mavwp.MAVWPLoader()
def add_mission(seq, lat, lon, altitude):
    coor_frame = 3
    wp.add(mavutil.mavlink.MAVLink_mission_item_message(
        drone.target_system,                     ## system ID
        drone.target_component,                  ## component ID
        seq,                                     ## sequence ???
        coor_frame,                              ## kordinat çerçevesi(coordinate frame) /yanliş anlamadiysam girecegimiz xyz degerleri için bir tip belirtiyor  burası 3 iken çalışıyor ama 9 iken niye çalışmıyor
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,    ## Mav_cmd belirt komut tipi
        0,0,0,0,0,0,                             ## 2 olan yer 0dan farklı olursa sonraki waypointe geçmez
        lat,lon,altitude
    ))
    drone.waypoint_clear_all_send()            ## göderilmiş önceki waypointleri siler
    drone.waypoint_count_send(wp.count())
    for i in range (wp.count()):
        msg = drone.recv_match(type=["MISSION_REQUEST"], blocking= True)
        drone.mav.send(wp.wp(msg.seq))
        print("Sending waypoints {0}".format(msg.seq))

def set_home_current():
    drone.mav.command_long_send(
        drone.target_system,
        drone.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_HOME,
        0,
        1,
        0,0,0,
        0,0,0
    )

def set_home(x, y, z):
    drone.mav.command_long_send(
        drone.target_system,
        drone.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_HOME,
        0,
        0,
        0,0,0,
        x,y,z
    )
########## FUNCTION END ##########
drone.waypoint_clear_all_send()            ## göderilmiş önceki waypointleri siler
drone.waypoint_count_send(wp.count())

msg = drone.recv_match(type="BATTERY_STATUS", blocking=True)
print(f"Battary Level: {msg.battery_remaining}")

msg = drone.recv_match(type="HEARTBEAT", blocking=True)
mode = mavutil.mode_string_v10(msg)
print(f"Flight Mode: {mode}")

drone.set_mode("STABILIZE")
time.sleep(3)

drone.set_mode("GUIDED")
drone.arducopter_arm()    ## for arducopter sim
# drone.motors_armed()    ## for drone

first_alt = input("input takeoff altitude:  ")
takeoff(float(first_alt))

drone_go_to(-35.36319970, 149.16513470, 10)
input("aga set home")
set_home_current()
mso = drone.recv_match(type="GLOBAL_POSITION_INT", blocking=True)

print(f"x/y: {float(mso.lat)}")
print(type(mso))

"""
add_mission(0, -35.36319320, 149.16543250, 25)
add_mission(1, -35.36296790, 149.16540030, 35)
add_mission(2, -35.36292410, 149.16519640, 40)
add_mission(3, -35.36306190, 149.16500060, 20)
add_mission(4, -35.36319970, 149.16513470, 10)
drone.set_mode("AUTO")
"""

"""
input("aga home ayarla")
set_home_current()

"""

input("aga land")
drone.mav.command_long_send(
    drone.target_system,
    drone.target_component,
    mavutil.mavlink.MAV_CMD_NAV_LAND,
    0,
    0,0,0,0,
    0,0,0
)
"""
input("aga rtl")
msg = drone.recv_match(type="LOCAL_POSITION_NED", blocking=True)
print(f"aga x:{msg.x}   aga y:{msg.y}   aga z:{msg.x}")
drone.mav.command_long_send(
    drone.target_system,
    drone.target_component,
    mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
    0,
    0,0,0,0,
    0,0,0
)
"""
"""
input("aga land")
drone.mav.command_long_send(
    drone.target_system,
    drone.target_component,
    mavutil.mavlink.MAV_CMD_NAV_LAND,
    0,
    0,0,0,0,
    0,0,0
)
"""
## şimdi hocam komutlar arası bekleme olmazsa direkt son komuta atlıyor

"""
drone_move(9, -10, -10, 10, 0, 0, 0, 0, 0, 0)

drone_go_to(3, -35.36123990, 149.16496040, 10)
"""

