#!/usr/bin/python3
import threading
import picamera
import pynmea2
import serial
import queue
import smbus
import time
import json
import sys
import os

log_queue = queue.Queue(maxsize=4096)

def logger():
  while True:
    with open("./data/mission.log", "a+") as LOG:
      while not log_queue.empty():
        try:
          LOG.write(json.dumps(log_queue.get())+"\n")
        except:
          print("Failed to log!")
          break

def log(data):
  log_queue.put(data)

def record_video():
  with picamera.PiCamera() as camera:
    camera.resolution = (1920, 1080)
    camera.framerate = 30
    num = 0
    while os.path.exists('data/vid{:03d}.h264'.format(num)):
      num += 1
    print("Recording video to data/vid{:03d}.h264".format(num))
    camera.start_recording('data/vid{:03d}.h264'.format(num))
  
    while True:
      frame = camera.frame
      data = {
        "index": frame.index,
        "frame_type": frame.frame_type,
        "frame_timestamp": frame.timestamp,
        "clock_timestamp": time.time(),
        "complete": frame.complete,
        "position": frame.position
      }
      log(data)
      camera.wait_recording(0.4/camera.framerate)
    camera.stop_recording()

def record_gps():
  pass

def record_imu():
  pass

#  bus = smbus.SMBus(1)
#  bus.write_byte_data(DEVICE_ADDRESS, DEVICE_REG_MODE1, 0x80)
#  ledout_values = [0xff, 0xff, 0xff, 0xff, 0xff, 0xff]
#  bus.write_i2c_block_data(DEVICE_ADDRESS, DEVICE_REG_LEDOUT0, ledout_values)


logger_thread = threading.Thread(target=logger)
logger_thread.start()
video_thread = threading.Thread(target=record_video)
video_thread.start()
gps_thread = threading.Thread(target=record_gps)
gps_thread.start()
imu_thread = threading.Thread(target=record_imu)
imu_thread.start()
print("Now running!")
time.sleep(10)
sys.exit("That's all folks")