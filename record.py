#!/usr/bin/python3
import threading
import picamera
import struct
import queue
import smbus
import gpsd
import time
import json
import sys
import os

COMPASS_ADDR = 0x1e
ACCELEROMETER_ADDR = 0x53
GYROSCOPE_ADDR = 0x68

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
    camera.start_recording('data/vid{:03d}.h264'.format(num), bitrate=25000000, motion_output='data/mot{:03}.h264'.format(num))
  
    while True:
      frame = camera.frame
      data = {
        "src": "video",
        "num": num,
        "index": frame.index,
        "frame_type": frame.frame_type,
        "frame_timestamp": frame.timestamp,
        "time": time.time(),
        "complete": frame.complete,
        "position": frame.position
      }
      log(data)
      camera.wait_recording(1/camera.framerate)
    camera.stop_recording()

def record_gps():
  gpsd.connect()
  while True:
    try:
      data = gpsd.get_current()
      log({
        "src": "gps",
        "time": time.time(),
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
    except Exception as e:
      print("{}".format(e))

def record_imu():
  bus = smbus.SMBus(1)
  # Datasheet: http://www.soc-robotics.com/pdfs/HMC5883L.pdf
  compass_config = [
    (0x00, 0b01110100), #Config reg A, 8xavg, 75Hz output rate, normal bias
    (0x01, 0b00100000), #Config reg B, +/- 1.3 Ga
    (0x02, 0b00000000), #Mode reg, continuous mode
  ]
  for i in compass_config:
    bus.write_byte_data(COMPASS_ADDR, *i)

  # Datasheet: http://www.analog.com/media/en/technical-documentation/data-sheets/ADXL345.pdf
  accelerometer_config = [
    (0x2D, 0b00001000), #Wake up, auto sleep disabled, begin measurements
    (0x31, 0b00001000), #Full res, +/- 2g, right-justify
    (0x32, 0b00001111), #Do not use FIFO
    (0x1E, 0), #Zero X offset
    (0x1F, 0), #Zero Y offset
    (0x20, 0), #Zero Z offset
    (0x2C, 0b00001000), #High-power mode, 100Hz updates
  ]
  for i in accelerometer_config:
    bus.write_byte_data(ACCELEROMETER_ADDR, *i)

  # Datasheet: https://www.sparkfun.com/datasheets/Sensors/Gyro/PS-ITG-3200-00-01.4.pdf
  gyro_config = [
    (0x15, 0x10), #Set sample rate divider to /16
    (0x16, 0b00011010), #Low-pass at 20Hz
    (0x17, 0b00000000), #I don't care about interrupts...
    (0x3E, 0b00000000), #Use internal clocking, don't sleep
  ]
  for i in gyro_config:
    bus.write_byte_data(GYROSCOPE_ADDR, *i)

  while True:
    try:
      status = bus.read_byte_data(COMPASS_ADDR, 0x09) #status reg
      if status % 2:
        #Compass reading is ready
        data = []
        for i in range(6):
          data.append(bus.read_byte_data(COMPASS_ADDR, 0x03+i)) #0x03 is start of data registers
        data = bytes(data)
        x = struct.unpack('>h', data[:2])
        y = struct.unpack('>h', data[2:4])
        z = struct.unpack('>h', data[4:6])
        log({
          "src": "compass",
          "time": time.time(),
          "x": x,
          "y": y,
          "z": z,
        })
    
      status = bus.read_byte_data(ACCELEROMETER_ADDR, 0x30) #Interrupt reg
      status &= 0b10000000 #DATA_READY
      if status:
        data = []
        for j in range(6):
          data.append(bus.read_byte_data(ACCELEROMETER_ADDR, 0x32+j))
        data = bytes(data)
        x = struct.unpack('<h', data[:2])[0]
        y = struct.unpack('<h', data[2:4])[0]
        z = struct.unpack('<h', data[4:6])[0]*-1 #Chip is upside down...
        log({
          "src": "accel",
          "time": time.time(),
          "x": x,
          "y": y,
          "z": z,
        })

      status = bus.read_byte_data(GYROSCOPE_ADDR, 0x1A) #Interrupt reg
      if status % 2:
        #gyro reading is ready
        data = []
        for j in range(8):
          data.append(bus.read_byte_data(GYROSCOPE_ADDR, 0x1B+j))
        data = bytes(data)
        t = struct.unpack('>h', data[:2])[0]
        x = struct.unpack('>h', data[2:4])[0]
        y = struct.unpack('>h', data[4:6])[0]
        z = struct.unpack('>h', data[6:8])[0]
        log({
          "src": "gyro",
          "time": time.time(),
          "temp": t,
          "x": x,
          "y": y,
          "z": z,
        })
    except Exception as e:
      print("{}".format(e))
      

logger_thread = threading.Thread(target=logger)
logger_thread.start()
video_thread = threading.Thread(target=record_video)
video_thread.start()
gps_thread = threading.Thread(target=record_gps)
gps_thread.start()
imu_thread = threading.Thread(target=record_imu)
imu_thread.start()
print("Capturing data...")
