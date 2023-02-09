from pymavlink import mavutil, mavwp

address = "udpin:localhost:14551"  ## linux için simulasyon bağlantısı
## address = "/dev/ttyACM0" 

vehicle = mavutil.mavlink_connection(address, baud=115200, autoreconnect=True)
vehicle.wait_heartbeat()
print("baglanti basarili")

def change_ground_speed(ground_speed):
    vehicle.mav.command_long_send(
        vehicle.target_system,
        vehicle.target_component,
        mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
        0,                                      ## confirmation ?? command_long'un parametresi
        0,                                      ## speed type
        ground_speed,                           ## speed                          
        -1,                                     ## trim throttle (-1 indicates no change)
        0,0,0,0                                 ## empty
    )

change_ground_speed(5)


def change_yaw(target_heading_deg, yaw_speed, clockwise):
    vehicle.mav.command_long_send(
        vehicle.target_system,
        vehicle.target_component,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0,
        target_heading_deg,
        yaw_speed,
        clockwise,
        1, ## absolute() or relative()
        0,0,0 
    )