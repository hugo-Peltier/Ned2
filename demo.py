#!/usr/bin/env python3
import time
from pyniryo import (
    NiryoRobot,
    ConveyorID,
    ConveyorDirection,
    PinID,
    ObjectShape,
    ObjectColor,
)

ROBOT_IP = "10.10.10.10"

CONVEYOR_ID = ConveyorID.ID_1
DIRECTION = ConveyorDirection.FORWARD
CONVEYOR_SPEED = 30
IR_PIN = PinID.DI5

PICK_POSE = [0.2953, -0.1958, 0.1038, 0.3651, 1.2192, -0.3878]
PLACE_POSE = [0.2035, -0.1877, 0.1046, 2.657, 1.5329, 1.9281]

WAIT_POSE = [0.2286, 0.002, 0.2881, 2.3693, 1.4496, 2.3723]

BATCH_SIZE = 2
CONVEYOR_SPACING = 0.07

CONVEYOR_OFFSET_AXIS = "y"

WORKSPACE_CONVEYOR = "Convoyeur_final"
CAMERA_CONV_POSE = [0.1612, 0.1634, 0.264, -3.1201, 1.2636, -2.4568]

WORKSPACE_BUNDLE = "vision_set_final"
OBS_CAM_POSE = [0.007, -0.1589, 0.2846, 3.095, 1.3837, 1.5237]

DROP_SQUARE_RED = [-0.3251, -0.1508, 0.1428, -2.8131, 1.4114, 0.7274]
DROP_SQUARE_BLUE = [-0.331, -0.3236, 0.1401, 0.6263, 1.4969, -1.8895]
DROP_SQUARE_GREEN = [-0.3273, -0.2362, 0.1385, 2.6928, 1.4141, -0.0313]

DROP_CIRCLE_ORIGIN = [-0.2189, -0.3068, 0.0289, 2.6742, 1.4088, 0.8501]
CIRCLE_COLS = 2
CIRCLE_ROWS = 4
DX = 0.05
DY = 0.05

FINAL_PICK_POSE_1 = [-0.1694, 0.2627, 0.0071, -2.312, 1.4872, 0.0933]
FINAL_PICK_POSE_2 = [-0.1294, 0.3041, 0.0069, 1.3891, 1.5395, -2.9305]
FINAL_DROP_RAMP_POSE = [0.377, -0.2038, 0.1626, 0.1165, 1.196, -0.2923]

ARM_SPEED = 100
SAFE_Z = 0.18
Z_OFFSET_PICK = 0.08
Z_OFFSET_DROP = 0.04

POLL_DT = 0.02
IR_TIMEOUT_S = 15.0
VISION_RETRIES = 2

VISION_HEIGHT_OFFSET_CONVEYOR = 0.01
VISION_HEIGHT_OFFSET_BUNDLE = 0.01

TARGETS = [
    (ObjectShape.SQUARE, ObjectColor.RED),
    (ObjectShape.SQUARE, ObjectColor.BLUE),
    (ObjectShape.SQUARE, ObjectColor.GREEN),
    (ObjectShape.CIRCLE, ObjectColor.RED),
    (ObjectShape.CIRCLE, ObjectColor.BLUE),
    (ObjectShape.CIRCLE, ObjectColor.GREEN),
]


def pose_with_offset(pose, dx=0.0, dy=0.0, dz=0.0):
    p = pose.copy()
    p[0] += dx
    p[1] += dy
    p[2] += dz
    return p


def release_vacuum(robot):
    robot.push_air_vacuum_pump()
    time.sleep(0.1)


def get_grid_pose(origin, index, cols, dx, dy):
    row = index // cols
    col = index % cols
    return pose_with_offset(origin, dx=col * dx, dy=row * dy)


def move_above_pose(robot, pose, safe_z=SAFE_Z):
    robot.clear_collision_detected()
    robot.move_pose(pose[0], pose[1], safe_z, pose[3], pose[4], pose[5])


def move_to_wait_pose(robot):
    robot.clear_collision_detected()
    robot.move_pose(*WAIT_POSE)


def pick_with_vacuum(robot, pose):
    approach = pose_with_offset(pose, dz=Z_OFFSET_PICK)
    robot.clear_collision_detected()
    robot.move_pose(*approach)
    robot.move_pose(*pose)
    robot.pull_air_vacuum_pump()
    time.sleep(0.1)
    robot.move_pose(*approach)


def place_with_vacuum(robot, pose):
    above = pose_with_offset(pose, dz=Z_OFFSET_DROP)
    robot.clear_collision_detected()
    robot.move_pose(*above)
    robot.move_pose(*pose)
    release_vacuum(robot)
    robot.move_pose(*above)


def safe_move_to_drop_and_release(robot, drop_pose):
    above_safe = [drop_pose[0], drop_pose[1], SAFE_Z, drop_pose[3], drop_pose[4], drop_pose[5]]
    above_drop = [drop_pose[0], drop_pose[1], drop_pose[2] + Z_OFFSET_DROP, drop_pose[3], drop_pose[4], drop_pose[5]]

    robot.clear_collision_detected()
    robot.move_pose(*above_safe)
    robot.move_pose(*above_drop)
    robot.move_pose(*drop_pose)
    release_vacuum(robot)
    robot.move_pose(*above_drop)
    robot.move_pose(*above_safe)


def get_square_drop_pose(color):
    if color == ObjectColor.RED:
        return DROP_SQUARE_RED
    elif color == ObjectColor.BLUE:
        return DROP_SQUARE_BLUE
    elif color == ObjectColor.GREEN:
        return DROP_SQUARE_GREEN
    return None


def get_conveyor_place_pose(index):
    if CONVEYOR_OFFSET_AXIS.lower() == "x":
        return pose_with_offset(PLACE_POSE, dx=index * CONVEYOR_SPACING)
    else:
        return pose_with_offset(PLACE_POSE, dy=index * CONVEYOR_SPACING)


def load_2_parts_on_conveyor(robot):
    conv_pose = PLACE_POSE

    pick_with_vacuum(robot, PICK_POSE)

    robot.clear_collision_detected()
    robot.move_pose(*pose_with_offset(conv_pose, dz=Z_OFFSET_DROP))
    robot.move_pose(*conv_pose)
    release_vacuum(robot)
    robot.move_pose(*pose_with_offset(conv_pose, dz=Z_OFFSET_DROP))

    time.sleep(0.10)

    robot.run_conveyor(CONVEYOR_ID, CONVEYOR_SPEED, DIRECTION)

    time.sleep(0.1)

    pick_with_vacuum(robot, PICK_POSE)

    robot.clear_collision_detected()
    robot.move_pose(*pose_with_offset(conv_pose, dz=Z_OFFSET_DROP))
    robot.move_pose(*conv_pose)
    release_vacuum(robot)
    robot.move_pose(*pose_with_offset(conv_pose, dz=Z_OFFSET_DROP))

    time.sleep(0.10)

    move_to_wait_pose(robot)


def wait_ir_change_and_stop(robot):
    initial = robot.digital_read(IR_PIN)
    start = time.time()

    while True:
        state = robot.digital_read(IR_PIN)

        if state != initial:
            robot.stop_conveyor(CONVEYOR_ID)
            time.sleep(1)
            return True

        if (time.time() - start) > IR_TIMEOUT_S:
            robot.stop_conveyor(CONVEYOR_ID)
            return False

        time.sleep(POLL_DT)


def try_vision_pick_by_targets(robot, workspace_name, camera_pose, height_offset, targets):
    robot.clear_collision_detected()
    robot.move_pose(*camera_pose)
    time.sleep(0.1)

    for shape, color in targets:
        for _ in range(VISION_RETRIES):
            try:
                ret = robot.vision_pick(workspace_name, height_offset, shape, color)

                if isinstance(ret, tuple):
                    obj_found = bool(ret[0])
                else:
                    obj_found = bool(ret)

                if obj_found:
                    return shape, color

            except Exception:
                pass

            time.sleep(0.1)

    return None, None


def drop_sorted_part(robot, shape, color, circle_index):
    if shape == ObjectShape.SQUARE:
        drop_pose = get_square_drop_pose(color)
        if drop_pose is None:
            print("[WARN] Pose carree non definie")
            return circle_index, False

        safe_move_to_drop_and_release(robot, drop_pose)
        return circle_index, True

    elif shape == ObjectShape.CIRCLE:
        max_places = CIRCLE_COLS * CIRCLE_ROWS
        if circle_index >= max_places:
            print("[WARN] Palette cercle pleine")
            return circle_index, False

        drop_pose = get_grid_pose(DROP_CIRCLE_ORIGIN, circle_index, CIRCLE_COLS, DX, DY)
        safe_move_to_drop_and_release(robot, drop_pose)
        circle_index += 1
        return circle_index, True

    return circle_index, False


def sort_2_parts_from_conveyor(robot, circle_index):
    sorted_count = 0

    for i in range(BATCH_SIZE):
        print(f"[INFO] Piece convoyeur {i+1}/{BATCH_SIZE}")

        if i > 0:
            robot.run_conveyor(CONVEYOR_ID, CONVEYOR_SPEED, DIRECTION)

        move_to_wait_pose(robot)

        ok_ir = wait_ir_change_and_stop(robot)
        if not ok_ir:
            print("[WARN] Timeout IR")
            continue

        time.sleep(0.1)

        shape, color = try_vision_pick_by_targets(
            robot,
            WORKSPACE_CONVEYOR,
            CAMERA_CONV_POSE,
            VISION_HEIGHT_OFFSET_CONVEYOR,
            TARGETS
        )

        if shape is None:
            print("[WARN] Aucune piece reconnue sur le convoyeur")
            move_to_wait_pose(robot)
            continue

        print(f"[INFO] Detecte sur convoyeur : {shape} / {color}")

        circle_index, ok_drop = drop_sorted_part(robot, shape, color, circle_index)
        if ok_drop:
            sorted_count += 1

        move_to_wait_pose(robot)

    return sorted_count, circle_index


def sort_all_parts_from_bundle(robot, circle_index):
    total_sorted = 0

    while True:
        shape, color = try_vision_pick_by_targets(
            robot,
            WORKSPACE_BUNDLE,
            OBS_CAM_POSE,
            VISION_HEIGHT_OFFSET_BUNDLE,
            TARGETS
        )

        if shape is None:
            break

        print(f"[INFO] Detecte sur bundle : {shape} / {color}")

        circle_index, ok_drop = drop_sorted_part(robot, shape, color, circle_index)
        if ok_drop:
            total_sorted += 1
        else:
            break

    return total_sorted, circle_index


def final_pick_and_drop_twice(robot):
    final_pick_poses = [
        FINAL_PICK_POSE_1,
        FINAL_PICK_POSE_2,
    ]

    print("[INFO] Plus de piece dans vision_set_final.")
    print("[INFO] Prises finales x2 sur zones de pick puis depose sur la rampe...")

    for idx, pick_pose in enumerate(final_pick_poses, start=1):
        print(f"[INFO] Prise finale {idx}/2")

        move_above_pose(robot, pick_pose)
        pick_with_vacuum(robot, pick_pose)
        safe_move_to_drop_and_release(robot, FINAL_DROP_RAMP_POSE)

    print("[INFO] Double depose finale sur rampe effectuee.")


def main():
    robot = None
    circle_index = 0

    try:
        robot = NiryoRobot(ROBOT_IP)

        robot.calibrate_auto()
        robot.update_tool()
        robot.set_arm_max_velocity(ARM_SPEED)

        robot.push_air_vacuum_pump()
        time.sleep(0.1)

        print("[INFO] Chargement des 2 pieces sur le convoyeur...")
        load_2_parts_on_conveyor(robot)

        print("[INFO] Tri direct des 2 pieces depuis le convoyeur...")
        sorted_conv, circle_index = sort_2_parts_from_conveyor(robot, circle_index)
        print(f"[INFO] Pieces triees depuis convoyeur : {sorted_conv}")

        print("[INFO] Tri des pieces deposees manuellement sur bundle_workspace...")
        sorted_bundle, circle_index = sort_all_parts_from_bundle(robot, circle_index)
        print(f"[INFO] Pieces triees depuis bundle_workspace : {sorted_bundle}")

        final_pick_and_drop_twice(robot)

        move_to_wait_pose(robot)

        print("[INFO] Fin du programme.")

    except KeyboardInterrupt:
        print("\n[INFO] Arret manuel.")

    except Exception as e:
        print("[ERROR]", e)

    finally:
        if robot is not None:
            try:
                robot.stop_conveyor(CONVEYOR_ID)
            except Exception:
                pass

            try:
                robot.push_air_vacuum_pump()
            except Exception:
                pass

            try:
                robot.close_connection()
            except Exception:
                pass


if __name__ == "__main__":
    main()
