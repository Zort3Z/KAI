> Ирис:
import json
import logging
import sys
import time
from skyros.drone import Drone

from clover import srv
from std_srvs.srv import Trigger
import rospy
from pyzbar import pyzbar
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from clover import long_callback
from clover.srv import SetLEDEffect

import math

rospy.init_node('cv')
set_effect = rospy.ServiceProxy('led/set_effect', SetLEDEffect)
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
land = rospy.ServiceProxy('land', Trigger)

bridge = CvBridge()

GRID_SIZE = 1

RECIPES = {
    "0": {
        "name": "Алмазная кирка",
        "grid": [
            ["diamond", "diamond", "diamond"],
            [None, "stick", None],
            [None, "stick", None]
        ]
    },
    "3": {
        "name": "Алмазный мотыга",
        "grid": [
            [ "diamond", "diamond",None],
            [None, "stick", None],
            [None, "stick", None]
        ]
    },
    "2": {
        "name": "Алмазная булава",
        "grid": [
            [None, "diamond", "diamond"],
            [None, "diamond", "diamond"],
            ["stick", None, None]
        ]
    }
}


DRONE_ROLES = {
    213: "diamond",
    93: "stick",
    208: "diamond"
}

current_frame = None
qr_data = None
target_position = None

# Точки поиска QR
waypoints = [
    (1.4, -1.7, 1.0),
    (-1.1, -1.6, 1.0),
    (-1.1, -0.8, 1.0),
    (1.5, -0.7, 1.0),
    (1.5, 0.5, 1.0),
    (-1.3, 0.5, 1.0),
    (-1.3, 1.3, 1.0),
    (1.6, 1.5, 1.0)
]


def navigate_wait(x=0.0, y=0.0, z=0.0, yaw=float('nan'), speed=0.5, frame_id='', auto_arm=False, tolerance=0.2):
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x  2 + telem.y  2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)


@long_callback
def image_callback(msg):
    global qr_data, current_frame
    current_frame = bridge.imgmsg_to_cv2(msg, 'bgr8')
    barcodes = pyzbar.decode(current_frame)
    for barcode in barcodes:
        data = barcode.data.decode('utf-8')
        if len(data) != 5:
            print(f"Некорректный QR: {data}")
            continue
        print(f"QR найден: {data}")
        qr_data = data  # например, "05DSSS"



def assign_positions_from_aruco():
    recipe_id = qr_data[0]
    qr_cell_index = int(qr_data[1])  # 0–8
    # qr_cell_index = 7
    if recipe_id not in RECIPES:
        logger.error(f"Неизвестный ID рецепта: {recipe_id}")
        return False

    recipe = RECIPES[recipe_id]
    grid = recipe["grid"]

    qr_row, qr_col = qr_cell_index // 3, qr_cell_index % 3


    try:
        cx, cy = 0, 0
    except:
        logger.error("Ошибка при получении позы aruco_105")
        return False

    # Базовая точка сетки 3x3 (верхний левый угол)
    base_x = cx - GRID_SIZE
    base_y = cy + GRID_SIZE

    # Формируем целевые позиции по рецепту
    cell_assignments = {}
    for row in range(3):
        for col in range(3):
            item = grid[row][col]
            if item is None:
                continue
            x = base_x + col * GRID_SIZE
            y = base_y - row * GRID_SIZE
            cell_assignments[(row, col)] = {"item": item, "pos": (x, y)}

    # Удаляем позицию стартового куба (уже занята)
    if (qr_row, qr_col) in cell_assignments:
        del cell_assignments[(qr_row, qr_col)]

    # Распределяем дронов
    assigned_drones = {}
    for (row, col), cell in cell_assignments.items():
        needed_role = cell["item"]
        for drone_id, role in DRONE_ROLES.items():
            if role == needed_role and drone_id not in assigned_drones:
                assigned_drones[drone_id] = {
                    "role": role,
                    "position": cell["pos"],
                    "grid_pos": (row, col)
                }
                break

> Ирис:
my_role = DRONE_ROLES.get(drone.drone_id)
    for (row, col), cell in cell_assignments.items():
        if cell["item"] == my_role and (row, col) not in [(a["grid_pos"]) for a in assigned_drones.values()]:
            x, y = cell["pos"]
            logger.info(f"Дрон {drone.drone_id} ({my_role}) → позиция ({x:.2f}, {y:.2f})")
            break
    print(assigned_drones)

    for drone_id, assignment in assigned_drones.items():
        x, y = assignment["position"]
        msg = {
            "t": "fc",
            "d": drone_id,
            "x": x,
            "y": y,
            "z": 1.0
        }
        drone.broadcast_custom_message(json.dumps(msg))
        drone.wait(0.2)
        drone.broadcast_custom_message(json.dumps(msg))
        drone.wait(0.3)
        drone.broadcast_custom_message(json.dumps(msg))
        drone.wait(0.4)
        drone.broadcast_custom_message(json.dumps(msg))

        drone.wait(0.5)


        drone.broadcast_custom_message(json.dumps(msg))
        drone.wait(0.2)
        drone.broadcast_custom_message(json.dumps(msg))
        drone.wait(0.3)
        drone.broadcast_custom_message(json.dumps(msg))
        drone.wait(0.4)
        drone.broadcast_custom_message(json.dumps(msg))

        drone.wait(0.5)


        drone.broadcast_custom_message(json.dumps(msg))
        drone.wait(0.2)
        drone.broadcast_custom_message(json.dumps(msg))
        drone.wait(0.3)
        drone.broadcast_custom_message(json.dumps(msg))
        drone.wait(0.4)
        drone.broadcast_custom_message(json.dumps(msg))

        drone.broadcast_custom_message(json.dumps(msg))
        drone.wait(0.2)
        drone.broadcast_custom_message(json.dumps(msg))
        drone.wait(0.3)
        drone.broadcast_custom_message(json.dumps(msg))
        drone.wait(0.4)
        drone.broadcast_custom_message(json.dumps(msg))

        drone.wait(0.5)

        drone.broadcast_custom_message(json.dumps(msg))
        drone.wait(0.2)
        drone.broadcast_custom_message(json.dumps(msg))
        drone.wait(0.3)
        drone.broadcast_custom_message(json.dumps(msg))
        drone.wait(0.4)
        drone.broadcast_custom_message(json.dumps(msg))

        drone.wait(0.5)

        drone.broadcast_custom_message(json.dumps(msg))
        drone.wait(0.2)
        drone.broadcast_custom_message(json.dumps(msg))
        drone.wait(0.3)
        drone.broadcast_custom_message(json.dumps(msg))
        drone.wait(0.4)
        drone.broadcast_custom_message(json.dumps(msg))


        logger.info(f"Дрону {drone_id} назначена позиция ({x:.2f}, {y:.2f})")

    return True


def setup_logger(verbose=False, quiet=False):
    level = logging.DEBUG if verbose else (logging.WARNING if quiet else logging.INFO)
    logging.basicConfig(
        level=level,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        stream=sys.stdout
    )
    return logging.getLogger()


logger = setup_logger()


def handle_message(msg):
    global target_position
    print(f"Received: {msg}")
    try:
        data = json.loads(msg)
        if data.get("t") == "fc":  # flight_command
            target_drone_id = data.get("d")
            if target_drone_id == drone.drone_id:
                target_position = {
                    "x": data.get("x", 0),
                    "y": data.get("y", 0),
                    "z": data.get("z", 1)
                }
                logging.info(f"Slave {drone.drone_id} got target: ")
                print(target_drone_id, target_position)
    except json.JSONDecodeError:
        if msg == "land":
            logging.info(f"Drone {drone.drone_id} получил команду 'land'")
            target_position = {"x": 0, "y": 0, "z": 0}
            land()
        else:
            logging.warning(f"Не удалось распарсить сообщение: {msg}")

try:
    with Drone(network_id=0x12, wifi_channel=3, tx_power=11, uart_port="/dev/ttyAMA1") as drone:

        drone.set_custom_message_callback(handle_message)

> Ирис:
# Стартовое сообщение
        start_msg = {"status": "start", "info": {"drone_id": drone.drone_id}}
        drone.broadcast_custom_message(json.dumps(start_msg))
        drone.wait(0.2)
        drone.broadcast_custom_message(json.dumps(start_msg))
        drone.wait(0.3)
        drone.broadcast_custom_message(json.dumps(start_msg))
        drone.wait(0.4)
        drone.broadcast_custom_message(json.dumps(start_msg))
        drone.wait(0.1)
        drone.broadcast_custom_message(json.dumps(start_msg))
        drone.wait(0.2)
        drone.broadcast_custom_message(json.dumps(start_msg))
        drone.wait(0.3)
        drone.broadcast_custom_message(json.dumps(start_msg))
        drone.wait(0.4)
        drone.broadcast_custom_message(json.dumps(start_msg))


        # Ожидание других дронов
        if drone.wait_for_drones(n=2, timeout=60.0):
            status = drone.get_network_status()

            # for drone_id, details in status["drone_details"].items():
            #     pos = details["position"]
            #     print(
            #         f"  drone_{drone_id}: pos=({pos['x']:.1f},{pos['y']:.1f},{pos['z']:.1f}) "
            #     )
            drone.wait(5)

        # Взлёт
        set_effect(r=0, g=0, b=255)  # синий — взлёт
        navigate_wait(z=2.0, frame_id='body', auto_arm=True)
        drone.wait(5)

        # Определяем мастера
        all_drones = drone.get_discovered_drones() | {drone.drone_id}
        master_id = min(all_drones)




        if drone.drone_id == master_id:
            logging.info(f"Drone {drone.drone_id} — MASTER")
            print('это мастер')
            set_effect(effect='blink', r=0, g=0, b=255)

            # Подписка на камеру
            image_sub = rospy.Subscriber('main_camera/image_raw_throttled', Image, image_callback, queue_size=1)

            if not qr_data:
                navigate_wait(x=1.0, y=-1.2, z=1, frame_id='aruco_map')

            if qr_data:
                set_effect(r=0, g=255, b=0)

            if not qr_data:
                navigate_wait(x=-1.0, y=-1.2, z=1, frame_id='aruco_map')

                if qr_data:
                    set_effect(r=0, g=255, b=0)

            if not qr_data:
                navigate_wait(x=-1.0, y=-0.2, z=1, frame_id='aruco_map')

                if qr_data:
                    set_effect(r=0, g=255, b=0)

            if not qr_data:
                navigate_wait(x=1.0, y=-0.1, z=1.0, frame_id='aruco_map')

                if qr_data:
                    set_effect(r=0, g=255, b=0)

            if not qr_data:
                navigate_wait(x=1.0, y=0.9, z=1.0, frame_id='aruco_map')

                if qr_data:
                    set_effect(r=0, g=255, b=0)

            if not qr_data:
                navigate_wait(x=-1.0, y=0.9, z=1.0, frame_id='aruco_map')

                if qr_data:
                    set_effect(r=0, g=255, b=0)

            if not qr_data:
                navigate_wait(x=-1.0, y=1.3, z=1, frame_id='aruco_map')

                if qr_data:
                    set_effect(r=0, g=255, b=0)

            if not qr_data:
                navigate_wait(x=1.6, y=1.5, z=1, frame_id='aruco_map')

            if qr_data:
                set_effect(r=0, g=255, b=0)

            if not qr_data:
                logging.error("QR не найден")
                drone.broadcast_custom_message("land")
                sys.exit()

            logging.info(f"QR распознан: {qr_data}")
            print(qr_data)
            set_effect(r=0, g=255, b=0)  # зелёный — QR найден

            if not assign_positions_from_aruco():
                logging.error("Ошибка при назначении позиций")
                drone.broadcast_custom_message("land")
                sys.exit()

            logging.info("Позиции назначены")
            drone.wait(1)

            # Радужная индикация
            set_effect(effect='rainbow')

            while target_position is None:
                drone.wait(0.2)

> Ирис:
if target_position and "z" in target_position:
                print(f"Летим к позиции: {target_position}")
                x = round(target_position.get("x", 0.0), 1)
                y = round(target_position.get("y", 0.0), 1)
                z = round(target_position.get("z", 1.0), 1)
                navigate_wait(x=x, y=y, z=z, frame_id='aruco_map')
                print(f"Дрон {drone.drone_id} достиг цели")
                set_effect(r=255, g=255, b=255)
            else:
                logger.warning("Цель не получена, садимся")
                drone.land()

            # drone.broadcast_custom_message("land")
            # drone.land()

        else:
            print(f"Drone {drone.drone_id} — SLAVE")

            while target_position is None:
                drone.wait(0.2)

            if target_position and "z" in target_position:
                print(f"Летим к позиции: {target_position}")
                x = round(target_position.get("x", 0.0), 1)
                y = round(target_position.get("y", 0.0), 1)
                z = round(target_position.get("z", 1.0), 1)
                navigate_wait(x=x, y=y, z=z, frame_id='aruco_map')
                print(f"Дрон {drone.drone_id} достиг цели")
                set_effect(r=255, g=255, b=255)
            else:
                print("Цель не получена, садимся")


        drone.wait(5)

except Exception as e:
    logging.error(f"Ошибка: {e}")
    set_effect(r=255, g=0, b=0)  # красный при ошибке
finally:
    print('це всё')
