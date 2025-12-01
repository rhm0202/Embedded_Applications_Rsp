import cv2 as cv
import numpy as np
import threading, time, sys
from tensorflow.keras.models import load_model
from modul import SDcar

speed = 10
is_emergency_stop = False 
enable_OD = False        
is_running = False 
enable_AIdrive = False

global_frame = None 
OD_frame_lock = threading.Lock()


OD_MODEL = cv.dnn.readNetFromTensorflow(
    model='modul/frozen_inference_graph.pb',
    config='modul/ssd_mobilenet_v2_coco_2018_03_29.pbtxt'
)

no_object_counter = 0
NO_OBJECT_THRESHOLD = 3

detection_interval = 1
frame_counter = 0
OD_CLASS_NAMES = []
with open('modul/object_detection_classes_coco.txt', 'r') as f: 
    OD_CLASS_NAMES = [ln.strip() for ln in f if ln.strip()]

STOP_CLASSES = ['mouse', 'cell phone']

def key_cmd(which_key):
    global enable_AIdrive, is_emergency_stop, enable_OD, car
    is_exit = False 
    if which_key & 0xFF == ord('w'):
        car.motor_go(speed)
    elif which_key & 0xFF == ord('s'):
        car.motor_back(speed)
    elif which_key & 0xFF == ord('a'):
        car.motor_left(30)   
    elif which_key & 0xFF == ord('d'):
        car.motor_right(30)            
    elif which_key & 0xFF == ord(' '):
        car.motor_stop()
        enable_AIdrive = False   
        is_emergency_stop = False  
    elif which_key & 0xFF == ord('q'):
        car.motor_stop()
        enable_AIdrive = False     
        is_exit = True    
    elif which_key & 0xFF == ord('e'):
        enable_AIdrive = True
    elif which_key & 0xFF == ord('x'):
        enable_AIdrive = False
        is_emergency_stop = False
        car.motor_stop()
    elif which_key & 0xFF == ord('t'):
        enable_OD = True
    elif which_key & 0xFF == ord('r'):
        enable_OD = False
    return is_exit  

def drive_AI(img):
    global car, model, is_emergency_stop
    img = np.expand_dims(img, 0)
    res = model.predict(img)[0]
    steering_angle = np.argmax(np.array(res))
    if is_emergency_stop:
        return
    if steering_angle == 0:
        car.motor_go(60)
    elif steering_angle == 1:
        car.motor_left(20)          
    elif steering_angle == 2:
        car.motor_right(20)

def object_detection_thread():
    global car, is_emergency_stop, global_frame
    global OD_frame_lock, is_running, enable_OD, frame_counter, no_object_counter


    while is_running:
        if not enable_OD:
            time.sleep(0.05)
            continue

        with OD_frame_lock:
            if global_frame is None:
                frame_to_process = None
            else:
                frame_to_process = global_frame.copy()

        if frame_to_process is None:
            time.sleep(0.01)
            continue

        frame_counter += 1
        if frame_counter % detection_interval != 0:
            time.sleep(0.005)
            continue
        frame_counter = 0

        blob = cv.dnn.blobFromImage(image=frame_to_process, size=(300, 300), swapRB=True)
        OD_MODEL.setInput(blob)
        output = OD_MODEL.forward()

        h, w, _ = frame_to_process.shape
        detected_texts = []
        object_is_currently_visible = False

        for detection in output[0, 0, :, :]:
            confidence = float(detection[2])
            if confidence < 0.3:
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

            detected_texts.append(class_name)
            color = (0, 255, 0)
            if class_name in STOP_CLASSES:
                object_is_currently_visible = True
                color = (0, 0, 255)

            cv.rectangle(frame_to_process, (x_min, y_min), (x_max, y_max), color, 2)
            cv.putText(frame_to_process, f"{class_name}:{confidence:.2f}", (x_min, max(0, y_min - 10)), cv.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

        if detected_texts:
            cv.putText(frame_to_process, "Detected: " + ", ".join(detected_texts),(10, 30), cv.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)

        if object_is_currently_visible and not is_emergency_stop:
            car.motor_stop()
            is_emergency_stop = True
            no_object_counter = 0
        elif not object_is_currently_visible and is_emergency_stop:
            no_object_counter += 1
            if no_object_counter >= NO_OBJECT_THRESHOLD:
                is_emergency_stop = False
                no_object_counter = 0

        with OD_frame_lock:
            global_frame = frame_to_process

        cv.imshow('OD camera', frame_to_process)
        cv.waitKey(1)
        time.sleep(0.01)

def main():
    global global_frame, OD_frame_lock, is_emergency_stop, is_running
    camera = cv.VideoCapture(0)
    camera.set(cv.CAP_PROP_FRAME_WIDTH, v_x) 
    camera.set(cv.CAP_PROP_FRAME_HEIGHT, v_y)
    try:
        while camera.isOpened() and is_running:
            ret, frame = camera.read()
            if not ret:
                break
            frame = cv.flip(frame, -1)
            with OD_frame_lock:
                global_frame = frame.copy()
                frame_to_display = global_frame.copy()
            currently_stopped = is_emergency_stop
            cv.imshow('camera', frame_to_display)
            crop_img = frame_to_display[int(v_y/2):, :]
            crop_img = cv.resize(crop_img, (200, 66))
            norm_img = cv.cvtColor(crop_img, cv.COLOR_BGR2RGB).astype(np.float32) / 255.0
            cv.imshow('crop_img', cv.resize(crop_img, dsize=(0,0), fx=2, fy=2))
            if enable_AIdrive and not currently_stopped:
                drive_AI(norm_img)
            which_key = cv.waitKey(20)
            if which_key > 0:
                is_exit = key_cmd(which_key)    
                if is_exit:
                    break
    finally:
        camera.release()
        cv.destroyAllWindows()

if __name__ == '__main__':
    v_x = 320
    v_y = 240
    v_x_grid = [int(v_x*i/10) for i in range(1, 10)]
    print(v_x_grid)
    model_path = 'modul/lane_navigation_20251127_1059.h5'
    model = load_model(model_path)
    car = SDcar.Drive()
    is_running = True
    enable_AIdrive = False
    enable_OD = False
    t_od = threading.Thread(target=object_detection_thread, daemon=True)
    t_od.start()
    main()
    is_running = False
    car.clean_GPIO()
    cv.destroyAllWindows()
    print('end vis')
