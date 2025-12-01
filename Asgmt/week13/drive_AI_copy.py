import cv2 as cv
import numpy as np
import threading, time, sys
from modul import SDcar
import tensorflow as tf
from tensorflow.keras.models import load_model

speed = 20
epsilon = 0.0001

OD_MODEL = cv.dnn.readNetFromTensorflow(
    model='modul/frozen_inference_graph.pb',
    config='modul/ssd_mobilenet_v2_coco_2018_03_29.pbtxt'
)

OD_CLASS_NAMES = []
with open('modul/object_detection_classes_coco.txt', 'r') as f: 
    OD_CLASS_NAMES = [ln.strip() for ln in f if ln.strip()]
    
    
is_running = False
enable_AIdrive = False
enable_OD = False
is_emergency_stop = False

global_frame = None
OD_frame_lock = threading.Lock()
OD_display_frame = None
od_results = []

STOP_CLASSES = ['clock', 'laptop']
NO_OBJECT_THRESHOLD = 3
no_object_counter = 0

v_x = 320
v_y = 240
v_x_grid = [int(v_x * i / 10) for i in range(1, 10)]
moment = np.array([0, 0, 0])


def func_thread():
    i = 0
    while True:
        time.sleep(1)
        i = i + 1
        if is_running is False:
            break


def key_cmd(which_key):
    print('which_key', which_key)
    is_exit = False
    global enable_AIdrive, enable_OD

    if which_key & 0xFF == 184:
        print('up')
        car.motor_go(speed)
    elif which_key & 0xFF == 178:
        print('down')
        car.motor_back(speed)
    elif which_key & 0xFF == 180:
        print('left')
        car.motor_left(30)
    elif which_key & 0xFF == 182:
        print('right')
        car.motor_right(30)
    elif which_key & 0xFF == 181:
        car.motor_stop()
        enable_AIdrive = False
        print('stop')
    elif which_key & 0xFF == ord('q'):
        car.motor_stop()
        print('exit')
        enable_AIdrive = False
        is_exit = True
        print('enable_AIdrive: ', enable_AIdrive)
    elif which_key & 0xFF == ord('e'):
        enable_AIdrive = True
        print('enable_AIdrive: ', enable_AIdrive)
    elif which_key & 0xFF == ord('w'):
        enable_AIdrive = False
        car.motor_stop()
        print('enable_AIdrive 2: ', enable_AIdrive)
    elif which_key & 0xFF == ord('t'):
        enable_OD = True
        print('OD: ON')
    elif which_key & 0xFF == ord('r'):
        enable_OD = False
        print('OD: OFF')

    return is_exit


def detect_maskY_HSV(frame):
    crop_hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    crop_hsv = cv.GaussianBlur(crop_hsv, (5, 5), cv.BORDER_DEFAULT)
    mask_Y = cv.inRange(crop_hsv, (25, 50, 100), (35, 255, 255))
    return mask_Y


def detect_maskY_BGR(frame):
    B = frame[:, :, 0]
    G = frame[:, :, 1]
    R = frame[:, :, 2]
    Y = np.zeros_like(G, np.uint8)
    Y = G * 0.5 + R * 0.5 - B * 0.7
    Y = Y.astype(np.uint8)
    Y = cv.GaussianBlur(Y, (5, 5), cv.BORDER_DEFAULT)
    _, mask_Y = cv.threshold(Y, 100, 255, cv.THRESH_BINARY)
    return mask_Y


def line_tracing(cx):
    global moment
    global v_x
    tolerance = 0.1
    diff = 0

    if moment[0] != 0 and moment[1] != 0 and moment[2] != 0:
        avg_m = np.mean(moment)
        diff = np.abs(avg_m - cx) / v_x

    if diff <= tolerance:
        moment[0] = moment[1]
        moment[1] = moment[2]
        moment[2] = cx
        print('cx : ', cx)
        if v_x_grid[2] <= cx < v_x_grid[4]:
            car.motor_go(speed)
            print('go')
        elif v_x_grid[3] >= cx:
            car.motor_left(30)
            print('turn left')
        elif v_x_grid[1] <= cx:
            car.motor_right(30)
            print('turn right')
        else:
            print("skip")
    else:
        car.motor_go(speed)
        print('go')
        moment = [0, 0, 0]


def show_grid(img):
    h, _, _ = img.shape
    for x in v_x_grid:
        cv.line(img, (x, 0), (x, h), (0, 255, 0), 1, cv.LINE_4)


def drive_AI(img):
    img = np.expand_dims(img, 0)
    res = model.predict(img)[0]
    steering_angle = np.argmax(np.array(res))
    print('steering_angle', steering_angle)
    if steering_angle == 0:
        print("go")
        speedSet = 60
        car.motor_go(speedSet)
    elif steering_angle == 1:
        print("left")
        speedSet = 20
        car.motor_left(speedSet)
    elif steering_angle == 2:
        print("right")
        speedSet = 20
        car.motor_right(speedSet)
    else:
        print("This cannot be entered")


def buzzer_beep():
    car.buzzer.alert_on()
    time.sleep(0.5)
    car.buzzer.alert_off()


def object_detection_thread():
    global car, is_emergency_stop, global_frame
    global OD_frame_lock, is_running, enable_OD, no_object_counter
    global od_results, OD_display_frame

    while is_running:
        if not enable_OD:
            time.sleep(0.1)
            with OD_frame_lock:
                od_results = []
                OD_display_frame = None
            continue

        frame_to_process = None
        with OD_frame_lock:
            if global_frame is not None:
                frame_to_process = global_frame.copy()

        if frame_to_process is None:
            time.sleep(0.01)
            continue

        with OD_frame_lock:
            OD_display_frame = frame_to_process.copy()

        od_drawing_frame = frame_to_process.copy()

        small_frame = cv.resize(frame_to_process, (160, 120))
        blob = cv.dnn.blobFromImage(image=small_frame, size=(300, 300), swapRB=True)
        OD_MODEL.setInput(blob)
        output = OD_MODEL.forward()

        h, w, _ = frame_to_process.shape
        current_detections = []
        object_is_currently_visible = False

        for detection in output[0, 0, :, :]:
            confidence = float(detection[2])
            if confidence < 0.5:
                continue
            class_id = int(detection[1])
            if 0 < class_id <= len(OD_CLASS_NAMES):
                class_name = OD_CLASS_NAMES[class_id - 1]
            else:
                continue

            x_min = int(detection[3] * w)
            y_min = int(detection[4] * h)
            x_max = int(detection[5] * w)
            y_max = int(detection[6] * h)

            is_danger = class_name in STOP_CLASSES
            current_detections.append((class_name, confidence, (x_min, y_min, x_max, y_max), is_danger))

            if is_danger:
                object_is_currently_visible = True

            color = (0, 0, 255) if is_danger else (0, 255, 0)
            cv.rectangle(od_drawing_frame, (x_min, y_min), (x_max, y_max), color, 2)
            cv.putText(
                od_drawing_frame,
                f"{class_name}:{confidence:.2f}",
                (x_min, y_min - 10),
                cv.FONT_HERSHEY_SIMPLEX,
                0.5,
                color,
                2
            )

        with OD_frame_lock:
            od_results = current_detections
            OD_display_frame = od_drawing_frame

        if object_is_currently_visible and not is_emergency_stop:
            car.motor_stop()
            threading.Thread(target=buzzer_beep, daemon=True).start()
            is_emergency_stop = True
            no_object_counter = 0
        elif not object_is_currently_visible and is_emergency_stop:
            no_object_counter += 1
            if no_object_counter >= NO_OBJECT_THRESHOLD:
                is_emergency_stop = False
                no_object_counter = 0

        time.sleep(0.01)
        
def main():
    global global_frame, is_running, enable_AIdrive, enable_OD

    camera = cv.VideoCapture(0)
    camera.set(cv.CAP_PROP_FRAME_WIDTH, v_x)
    camera.set(cv.CAP_PROP_FRAME_HEIGHT, v_y)

    try:
        while camera.isOpened() and is_running:
            ret, frame = camera.read()
            if not ret:
                break
            frame = cv.flip(frame, -1)

            display_od_frame = None
            with OD_frame_lock:
                global_frame = frame.copy()
                if OD_display_frame is not None:
                    display_od_frame = OD_display_frame

            cv.imshow('camera', frame)

            if enable_OD and display_od_frame is not None:
                cv.imshow('OD Monitor', display_od_frame)
            elif not enable_OD:
                try:
                    if cv.getWindowProperty('OD Monitor', cv.WND_PROP_VISIBLE) >= 1:
                        cv.destroyWindow('OD Monitor')
                except:
                    pass

            crop_img = frame[int(v_y / 2):, :]
            crop_img_nn = cv.resize(crop_img, (200, 66))
            cv.imshow('crop_img', cv.resize(crop_img_nn, dsize=(0, 0), fx=2, fy=2))

            if enable_AIdrive and not is_emergency_stop:
                drive_AI(crop_img_nn)

            which_key = cv.waitKey(20)
            is_exit = False
            if which_key > 0:
                is_exit = key_cmd(which_key)
            if is_exit is True:
                break

    except Exception as e:
        exception_type, exception_object, exception_traceback = sys.exc_info()
        filename = exception_traceback.tb_frame.f_code.co_filename
        line_number = exception_traceback.tb_lineno
        print("Exception type: ", exception_type)
        print("File name: ", filename)
        print("Line number: ", line_number)

        is_running = False
    finally:
        camera.release()
        cv.destroyAllWindows()


if __name__ == '__main__':
    print(v_x_grid)

    model_path = 'modul/lane_navigation_20251201_1643.h5'
    model = load_model(model_path)

    t_task1 = threading.Thread(target=func_thread, daemon=True)
    t_task1.start()

    car = SDcar.Drive()

    t_od = threading.Thread(target=object_detection_thread, daemon=True)

    is_running = True
    enable_AIdrive = False
    enable_OD = False

    t_od.start()
    main()

    is_running = False
    car.clean_GPIO()
    print('end vis')