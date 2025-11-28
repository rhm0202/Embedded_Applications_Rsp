import cv2 as cv
import numpy as np
import threading, time
from modul import SDcar 
import sys
import tensorflow as tf
from tensorflow.keras.models import load_model

speed = 80
epsilon = 0.0001

is_emergency_stop = False

global_frame = None 
OD_frame_lock = threading.Lock()

OD_CLASS_NAMES = []
with open('modul/object_detection_classes_coco.txt', 'r') as f:
    OD_CLASS_NAMES = f.read().split('\n')

OD_MODEL = None 
STOP_CLASSES = ['mouse', 'cell phone']

def func_thread():
    global is_running
    i = 0
    while True:
        #print("alive!!")    
        time.sleep(1)
        i = i+1
        if is_running is False:
            break

def key_cmd(which_key):
    print('which_key', which_key)
    is_exit = False 
    
    global enable_AIdrive
    global is_emergency_stop
    global car
    
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
        is_emergency_stop = False     
        print('stop')   
    elif which_key & 0xFF == ord('q'):  
        car.motor_stop()
        print('exit')   
        enable_AIdrive = False     
        is_exit = True    
        print('enable_AIdrive: ', enable_AIdrive)          
    elif which_key & 0xFF == ord('e'):  
        enable_AIdrive = True
        is_emergency_stop = False
        print('enable_AIdrive: ', enable_AIdrive)        
    elif which_key & 0xFF == ord('w'):  
        enable_AIdrive = False
        is_emergency_stop = False
        car.motor_stop()
        print('enable_AIdrive 2: ', enable_AIdrive)   

    return is_exit  


def drive_AI(img):
    global car
    global model
    
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

def object_detection_thread():
    global OD_MODEL
    global car
    global is_emergency_stop
    global global_frame
    global OD_frame_lock
    global is_running
    
    
    while is_running:
        OD_frame_lock.acquire()
        if global_frame is None:
            OD_frame_lock.release()
            time.sleep(0.01)
            continue
            
        frame_to_process = global_frame.copy() 
        OD_frame_lock.release()
        
        blob = cv.dnn.blobFromImage(image=frame_to_process, size=(300, 300), swapRB=True)
        OD_MODEL.setInput(blob)
        output = OD_MODEL.forward()
        
        frame_height, frame_width, _ = frame_to_process.shape
        
        object_is_currently_visible = False
        
        for detection in output[0, 0, :, :]:
            confidence = detection[2]
            
            if confidence > 0.4:
                class_id = int(detection[1])
                
                if 0 < class_id <= len(OD_CLASS_NAMES):
                    class_name = OD_CLASS_NAMES[class_id - 1]
                    
                    if class_name in STOP_CLASSES:
                        object_is_currently_visible = True
                        
                        color = (0, 0, 255) 
                        x_min = detection[3] * frame_width
                        y_min = detection[4] * frame_height
                        x_max = detection[5] * frame_width
                        y_max = detection[6] * frame_height
                        cv.rectangle(frame_to_process, (int(x_min), int(y_min)), (int(x_max), int(y_max)), color, thickness=2)
                        cv.putText(frame_to_process, class_name, (int(x_min), int(y_min - 10)), cv.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
                        
                        break # 객체 하나만 감지해도 정지

        if object_is_currently_visible:
            if is_emergency_stop == False:
                car.motor_stop()
                is_emergency_stop = True
        
        elif is_emergency_stop == True:
            is_emergency_stop = False

        OD_frame_lock.acquire()
        global_frame = frame_to_process
        OD_frame_lock.release()

        time.sleep(0.01)
    
def main():
    global global_frame
    global OD_frame_lock
    global car
    global is_running
    global enable_AIdrive
    
    camera = cv.VideoCapture(0)
    camera.set(cv.CAP_PROP_FRAME_WIDTH,v_x) 
    camera.set(cv.CAP_PROP_FRAME_HEIGHT,v_y)
    
    try:
        while( camera.isOpened() ):
            ret, frame = camera.read()
            frame = cv.flip(frame,-1)
            
            OD_frame_lock.acquire()
            global_frame = frame.copy()
            
            frame_to_display = global_frame.copy() 
            OD_frame_lock.release()
            
            currently_stopped = is_emergency_stop
            
            cv.imshow('camera',frame_to_display)
            
            # image processing start here
            crop_img = frame[int(v_y/2):,:]
            crop_img = cv.resize(crop_img, (200, 66))
            cv.imshow('crop_img ', cv.resize(crop_img, dsize=(0,0), fx=2, fy=2))

            if enable_AIdrive == True and currently_stopped == False: 
                drive_AI(crop_img)

            # image processing end here
            is_exit = False
            which_key = cv.waitKey(20)
            if which_key > 0:
                is_exit = key_cmd(which_key)    
            if is_exit is True:
                cv.destroyAllWindows()
                break

    except Exception as e:
        exception_type, exception_object, exception_traceback = sys.exc_info()
        filename = exception_traceback.tb_frame.f_code.co_filename
        line_number = exception_traceback.tb_lineno

        print("Exception type: ", exception_type)
        print("File name: ", filename)
        print("Line number: ", line_number)
        is_running = False
        
    camera.release()

if __name__ == '__main__':

    v_x = 320
    v_y = 240
    v_x_grid = [int(v_x*i/10) for i in range(1, 10)]
    print(v_x_grid)
    moment = np.array([0, 0, 0])

    model_path = 'modul/lane_navigation_20251127_1059.h5'

    model = load_model(model_path)

    OD_MODEL = cv.dnn.readNetFromTensorflow(
        model='modul/frozen_inference_graph.pb',
        config='modul/ssd_mobilenet_v2_coco_2018_03_29.pbtxt'
    )

    t_task1 = threading.Thread(target = func_thread)
    t_task1.start()
    
    t_od = threading.Thread(target = object_detection_thread)
    t_od.start()

    car = SDcar.Drive()
    
    is_running = True
    enable_AIdrive = False
    main() 
    is_running = False
    car.clean_GPIO()
    print('end vis')
