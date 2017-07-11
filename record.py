#!/usr/bin/python3
import threading
import picamera
import queue
import smbus
import gpsd
import time
import json
import sys
import os

COMPASS_ADDR = 0x1e
ACCELEROMETER_ADDR = 0
GYROSCOPE_ADDR = 20

log_queue = queue.Queue(maxsize=4096)

def logger():
  while True:
    with open("./data/mission.log", "a+") as LOG:
      while not log_queue.empty():
        try:
          LOG.write(json.dumps(log_queue.get())+"\n")
        except Exception as e:
          print("Failed to log: {}".format(e))
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
        "source": "video",
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
  gpsd.connect()
  while True:
    data = gpsd.get_current()
    log({
      "source": "gps",
      "clock_timestamp": time.time(),
      "lat": data.lat,
      "lon": data.lon,
      "alt": data.alt,
      "hspeed": data.hspeed,
      "track": data.track,
      "climb": data.climb,
      "error": data.error,
      "time": data.time,
      "mode": data.mode,
      "sats": data.sats,
    })
    time.sleep(0.1)

def record_imu():
  bus = smbus.SMBus(1)
  while True:
    time.sleep(1)

logger_thread = threading.Thread(target=logger)
logger_thread.start()
video_thread = threading.Thread(target=record_video)
video_thread.start()
gps_thread = threading.Thread(target=record_gps)
gps_thread.start()
imu_thread = threading.Thread(target=record_imu)
imu_thread.start()
print("Capturing data...")
