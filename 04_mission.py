from pymavlink import mavutil, mavwp

address = "udpin:localhost:14551"  ## linux için simulasyon bağlantısı
## address = "/dev/ttyACM0" 

vehicle = mavutil.mavlink_connection(address, baud=115200, autoreconnect=True)
vehicle.wait_heartbeat()
print("baglanti basarili")

vehicle.set_mode("GUIDED") ## araç modunu değiştiriyor

wp = mavwp.MAVWPLoader()

def takeoff(altitude):
    vehicle.mav.command_long_send(vehicle.target_system, vehicle.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,0,0,0,0,0,0,0,altitude)

def gorev_ekle(seq, lat, lon, altitude):
    coor_frame = 3
    wp.add(mavutil.mavlink.MAVLink_mission_item_message(
        vehicle.target_system,                   ## system ID
        vehicle.target_component,                ## component ID
        seq,                                     ## sequence ???
        coor_frame,                              ## kordinat çerçevesi(coordinate frame) /yanliş anlamadiysam girecegimiz xyz degerleri için bir tip belirtiyor  burası 3 iken çalışıyor ama 9 iken niye çalışmıyor
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,    ## Mav_cmd belirt komut tipi
        0,0,0,0,0,0,                             ## 2 olan yer 0dan farklı olursa sonraki waypointe geçmez
        lat,lon,altitude
    ))
    
    vehicle.waypoint_clear_all_send()            ## göderilmiş önceki waypointleri siler
    vehicle.waypoint_count_send(wp.count())
    for i in range (wp.count()):
        msg = vehicle.recv_match(type=["MISSION_REQUEST"], blocking= True)
        vehicle.mav.send(wp.wp(msg.seq))
        print("Sending waypoints {0}".format(msg.seq))


vehicle.set_mode("GUIDED")
vehicle.arducopter_arm()
takeoff(10)
gorev_ekle(0, -35.36319320, 149.16543250, 25)
gorev_ekle(1, -35.36296790, 149.16540030, 35)
gorev_ekle(2, -35.36292410, 149.16519640, 40)
gorev_ekle(3, -35.36306190, 149.16500060, 20)
gorev_ekle(4, -35.36319970, 149.16513470, 10)
vehicle.set_mode("AUTO")