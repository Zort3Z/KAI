import json
import logging
import sys
from skyros.drone import Drone

from clover import srv
from std_srvs.srv import Trigger

import rospy
from pyzbar import pyzbar
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from clover import long_callback

from clover.srv import SetLEDEffect

import math

rospy.init_node('cv')
set_effect = rospy.ServiceProxy('led/set_effect', SetLEDEffect)  # define proxy to ROS-service
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
land = rospy.ServiceProxy('land', Trigger)

bridge = CvBridge()

current_frame = None


# def wait_qr():
#     try:
#         image_sub = rospy.Subscriber('/camera/image_raw', Image, image_callback)
#
#
#         start_time = time.time()
#         timeout = 3
#
#         qr_data = None
#
#         while not rospy.is_shutdown() and time.time() - start_time < timeout:
#             if current_frame is None:
#                 continue
#
#             decoded_objects = decode(current_frame)
#             for obj in decoded_objects:
#                 qr_data = obj.data.decode('utf-8').strip()
#                 print(qr_data)
#                 logging.info(f"QR-код распознан: {qr_data}")
#         time.sleep(0.1)
#
#         image_sub.unregister()
#         return qr_data
#     except Exception as e:
#         logging.error(f"Ошибка qr кода: {e}")
#
# def image_callback(msg):
#     global current_frame
#     try:
#         current_frame = bridge.imgmsg_to_cv2(msg, 'bgr8')
#     except Exception as e:
#         logging.error(f"Ошибка преобразования изображения: {e}")
#
#
# def scan_qr():
#     wait_qr()
#     drone.navigate(x=0, y=0, z=1.20, frame_id='body')
#     wait_qr()
#     drone.wait(3)
#     drone.navigate(x=0, y=1, z=1.20, frame_id='body')
#     wait_qr()
#     drone.wait(3)
#
#     if qr_data:
#         return True
#     else:
#         return False

def navigate_wait(x=0.0, y=0.0, z=0.0, yaw=float('nan'), speed=0.5, frame_id='', auto_arm=False, tolerance=0.2):
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)


qr_data = None


@long_callback
def image_callback(msg):
    global qr_data
    img = bridge.imgmsg_to_cv2(msg, 'bgr8')
    barcodes = pyzbar.decode(img)


    for barcode in barcodes:

        b_data = barcode.data.decode('utf-8')
        qr_data = b_data
        b_type = barcode.type
        (x, y, w, h) = barcode.rect
        xc = x + w / 2
        yc = y + h / 2
        print('Found {} with data {} with center at x={}, y={}'.format(b_type, b_data, xc, yc))


def setup_logger(verbose=False, quiet=False):
    """Set up simple CLI logger"""
    level = logging.DEBUG if verbose else (logging.WARNING if quiet else logging.INFO)

    logging.basicConfig(
        level=level,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        stream=sys.stdout
    )

    return logging.getLogger()


# Create and start drone
setup_logger()

# Variables for slave drone coordinates
slave_target = None


def handle_message(msg):
    global slave_target
    print(f"Received: {msg}")
    slave_target = msg
    if slave_target == 'land':
        land()


# drone_id is set by last octet of ip address
try:
    with Drone(network_id=0x12, wifi_channel=3, tx_power=11, uart_port="/dev/ttyAMA1") as drone:
        drone.set_custom_message_callback(handle_message)

        # Send start json message to other drones
        start_message = {
            "status": "start",
            "info": {
                "drone_id": drone.drone_id,
            }
        }
        drone.broadcast_custom_message(json.dumps(start_message))
        drone.wait(0.2)
        drone.broadcast_custom_message(json.dumps(start_message))
        drone.wait(0.3)
        drone.broadcast_custom_message(json.dumps(start_message))
        drone.wait(0.4)


        # Wait for other drones to start
        if drone.wait_for_drones(n=2, timeout=30.0):
            # Get network status with detailed info
            status = drone.get_network_status()

            # Show detailed drone info
            for drone_id, details in status["drone_details"].items():
                pos = details["position"]
                logging.info(
                    f"  drone_{drone_id}: pos=({pos['x']:.1f},{pos['y']:.1f},{pos['z']:.1f}) "
                )
            drone.wait(1)


        set_effect(r=255, g=0, b=150)
        drone.takeoff(z=2)
        drone.wait(7)

        set_effect(r=255, g=255, b=255)

        # Master drone logic: drone with lowest ID becomes master
        discovered_drones = drone.get_discovered_drones()
        all_drones = discovered_drones | {drone.drone_id}
        master_drone_id = min(all_drones)

        print(all_drones)
        print(drone.drone_id)
        if drone.drone_id == master_drone_id:
            logging.info(
                f"Drone {drone.drone_id} is MASTER - controlling swarm")  # Master sends individual coordinates to each drone
            discovered_drones = drone.get_discovered_drones()

            set_effect(effect='blink', r=0, g=0, b=255)

            logging.info("Сканируем QR-код...")
            print("qr")

            image_sub = rospy.Subscriber('main_camera/image_raw_throttled', Image, image_callback, queue_size=1)

            if not qr_data:
                navigate_wait(x=1.4, y=-1.7, z=1, frame_id='aruco_map')

            if qr_data:
                set_effect(r=0, g=255, b=0)

            if not qr_data:
                navigate_wait(x=-1.1, y=-1.6, z=1, frame_id='aruco_map')

                if qr_data:
                    set_effect(r=0, g=255, b=0)

            if not qr_data:
                navigate_wait(x=-1.1, y=-0.8, z=1, frame_id='aruco_map')

                if qr_data:
                    set_effect(r=0, g=255, b=0)

            if not qr_data:
                navigate_wait(x=1.5, y=-0.7, z=1.0, frame_id='aruco_map')

                if qr_data:
                    set_effect(r=0, g=255, b=0)

            if not qr_data:
                navigate_wait(x=1.5, y=0.5, z=1.0, frame_id='aruco_map')

                if qr_data:
                    set_effect(r=0, g=255, b=0)

            if not qr_data:
                navigate_wait(x=-1.3, y=0.5, z=1.0, frame_id='aruco_map')

                if qr_data:
                    set_effect(r=0, g=255, b=0)

            if not qr_data:
                navigate_wait(x=-1.3, y=1.3, z=1, frame_id='aruco_map')

                if qr_data:
                    set_effect(r=0, g=255, b=0)

            if not qr_data:
                navigate_wait(x=1.6, y=1.5, z=1, frame_id='aruco_map')

                if qr_data:
                    set_effect(r=0, g=255, b=0)

            # Send coordinates to each drone separately

            logging.info(f"Master sending to drones land")
            drone.broadcast_custom_message('land')

            # Master flies to its position
            # set_effect(r=255, g=255, b=0)
            # drone.land()
        else:
            logging.info(f"Drone {drone.drone_id} is SLAVE - waiting for master commands")

            # Wait for master commands and execute them
            drone.wait(3)  # Wait for master to send commands

            while slave_target == 'land':
                print('це все тоже')
                land()
            # Execute received coordinates if available
            # if slave_target == 'land':
            #
            # else:
            #     logging.warning(f"Slave {drone.drone_id} no target received, flying to default")

        # Broadcast message to other drones
        drone.broadcast_custom_message(f"Hello from drone_{drone.drone_id}!")

        # Land

except Exception as e:
    print('ошибка', e)

finally:
    if qr_data:
        drone.broadcast_custom_message('land')
        drone.broadcast_custom_message('land')
        drone.broadcast_custom_message('land')
        drone.broadcast_custom_message('land')
        drone.broadcast_custom_message('land')

        set_effect(r=255, g=255, b=0)

        land()
        print('це всё')
