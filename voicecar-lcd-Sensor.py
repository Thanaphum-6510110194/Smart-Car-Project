import RPi.GPIO as GPIO
import speech_recognition as sr
import time
import smbus
from threading import Thread


# กำหนดขา GPIO สำหรับ L298N ตัวที่ 1 (มอเตอร์ 1 และ 2 ฝั่งซ้าย)
IN1_A = 17
IN2_A = 27
IN3_A = 22
IN4_A = 23

# กำหนดขา GPIO สำหรับ L298N ตัวที่ 2 (มอเตอร์ 3 และ 4 ฝั่งขวา)
IN1_B = 5
IN2_B = 6
IN3_B = 13
IN4_B = 19

GPIO.setmode(GPIO.BCM)

# ตั้งค่าขา GPIO สำหรับมอเตอร์ 1 และ 2 (L298N ตัวที่ 1)
GPIO.setup(IN1_A, GPIO.OUT)
GPIO.setup(IN2_A, GPIO.OUT)
GPIO.setup(IN3_A, GPIO.OUT)
GPIO.setup(IN4_A, GPIO.OUT)

# ตั้งค่าขา GPIO สำหรับมอเตอร์ 3 และ 4 (L298N ตัวที่ 2)
GPIO.setup(IN1_B, GPIO.OUT)
GPIO.setup(IN2_B, GPIO.OUT)
GPIO.setup(IN3_B, GPIO.OUT)
GPIO.setup(IN4_B, GPIO.OUT)

# กำหนดขา GPIO สำหรับ Ultrasonic Sensor ตัวที่ 1 (หน้า)
TRIG1 = 7
ECHO1 = 8

# กำหนดขา GPIO สำหรับ Ultrasonic Sensor ตัวที่ 2 (หลัง)
TRIG2 = 25
ECHO2 = 24

# ตั้งค่าขา GPIO สำหรับ Ultrasonic Sensor ตัวที่ 1
GPIO.setup(TRIG1, GPIO.OUT)
GPIO.setup(ECHO1, GPIO.IN)

# ตั้งค่าขา GPIO สำหรับ Ultrasonic Sensor ตัวที่ 2
GPIO.setup(TRIG2, GPIO.OUT)
GPIO.setup(ECHO2, GPIO.IN)

# กำหนดพารามิเตอร์ของอุปกรณ์
I2C_ADDR  = 0x27  # ที่อยู่ I2C ของอุปกรณ์
LCD_WIDTH = 16    # จำนวนตัวอักษรสูงสุดในแต่ละบรรทัด

# กำหนดค่าคงที่ของอุปกรณ์
LCD_CHR = 1       # โหมด - ส่งข้อมูล
LCD_CMD = 0       # โหมด - ส่งคำสั่ง

LCD_LINE_1 = 0x80 # ที่อยู่ RAM ของ LCD สำหรับบรรทัดที่ 1
LCD_LINE_2 = 0xC0 # ที่อยู่ RAM ของ LCD สำหรับบรรทัดที่ 2

LCD_BACKLIGHT = 0x08   # เปิดไฟพื้นหลัง
ENABLE = 0b00000100    # บิต Enable

# ค่าคงที่สำหรับเวลา
E_PULSE = 0.0005
E_DELAY = 0.0005

# เปิดใช้งาน I2C
bus = smbus.SMBus(1)

def lcd_init():
    # การเริ่มต้นการทำงานของ LCD
    lcd_byte(0x33, LCD_CMD)  # 110011 เริ่มต้น
    lcd_byte(0x32, LCD_CMD)  # 110010 เริ่มต้น
    lcd_byte(0x06, LCD_CMD)  # 000110 ทิศทางการเคลื่อนที่ของเคอร์เซอร์
    lcd_byte(0x0C, LCD_CMD)  # 001100 เปิดการแสดงผล, ปิดเคอร์เซอร์, ปิดการกระพริบ
    lcd_byte(0x28, LCD_CMD)  # 101000 ขนาดข้อมูล, จำนวนบรรทัด, ขนาดฟอนต์
    lcd_byte(0x01, LCD_CMD)  # 000001 ล้างการแสดงผล
    time.sleep(E_DELAY)

def lcd_byte(bits, mode):
    # ส่งข้อมูลหรือคำสั่งไปยัง LCD
    bits_high = mode | (bits & 0xF0) | LCD_BACKLIGHT
    bits_low = mode | ((bits << 4) & 0xF0) | LCD_BACKLIGHT

    bus.write_byte(I2C_ADDR, bits_high)
    lcd_toggle_enable(bits_high)
    bus.write_byte(I2C_ADDR, bits_low)
    lcd_toggle_enable(bits_low)

def lcd_toggle_enable(bits):
    # สลับสถานะ Enable
    time.sleep(E_DELAY)
    bus.write_byte(I2C_ADDR, (bits | ENABLE))
    time.sleep(E_PULSE)
    bus.write_byte(I2C_ADDR, (bits & ~ENABLE))
    time.sleep(E_DELAY)

def lcd_string(message, line):
    # แสดงข้อความบน LCD
    message = message.ljust(LCD_WIDTH, " ")  # เติมช่องว่างจนกว่าจะครบความกว้างของ LCD
    lcd_byte(line, LCD_CMD)  # ตั้งตำแหน่งเริ่มต้น
    for i in range(LCD_WIDTH):
        lcd_byte(ord(message[i]), LCD_CHR)  # ส่งแต่ละตัวอักษร

def left_arrow_animation(duration=1):
    left_arrow = "<<<"
    start_pos = (LCD_WIDTH - len(left_arrow)) // 2

    # จับเวลาเริ่มต้น
    start_time = time.time()

    # ทำงานจนถึงเวลาที่กำหนด (1 วินาที)
    while time.time() - start_time < duration:
        lcd_string(" " * start_pos + left_arrow, LCD_LINE_1)
        lcd_string(" " * start_pos + left_arrow, LCD_LINE_2)
        time.sleep(0.2)
        lcd_string(" " * LCD_WIDTH, LCD_LINE_1)
        lcd_string(" " * LCD_WIDTH, LCD_LINE_2)
        time.sleep(0.2)

def right_arrow_animation(duration=1):
    right_arrow = ">>>"
    start_pos = (LCD_WIDTH - len(right_arrow)) // 2

    # จับเวลาเริ่มต้น
    start_time = time.time()

    # ทำงานจนถึงเวลาที่กำหนด (1 วินาที)
    while time.time() - start_time < duration:
        lcd_string(" " * start_pos + right_arrow, LCD_LINE_1)
        lcd_string(" " * start_pos + right_arrow, LCD_LINE_2)
        time.sleep(0.2)
        lcd_string(" " * LCD_WIDTH, LCD_LINE_1)
        lcd_string(" " * LCD_WIDTH, LCD_LINE_2)
        time.sleep(0.2)

# ฟังก์ชันคำนวณระยะทางจากเซ็นเซอร์ Ultrasonic
def get_distance(TRIG, ECHO):
    # ส่งสัญญาณ Trigger
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    start_time = time.time()
    stop_time = time.time()

    # ตรวจสอบสัญญาณ Echo
    timeout = time.time() + 1  # เพิ่ม timeout 1 วินาที
    while GPIO.input(ECHO) == 0 and time.time() < timeout:
        start_time = time.time()

    while GPIO.input(ECHO) == 1 and time.time() < timeout:
        stop_time = time.time()

    if time.time() >= timeout:
        return None  # ส่งค่า None กลับถ้า timeout

    # คำนวณระยะทาง (ในเซนติเมตร)
    pulse_duration = stop_time - start_time
    distance = pulse_duration * 17150  # ความเร็วเสียงในอากาศ = 34300 cm/s
    distance = round(distance, 2)

    #print(f"Distance: {distance} cm")
    return distance


# ฟังก์ชันหยุดมอเตอร์ทั้งหมด
def stop_motors():
    GPIO.output(IN1_A, GPIO.LOW)
    GPIO.output(IN2_A, GPIO.LOW)
    GPIO.output(IN3_A, GPIO.LOW)
    GPIO.output(IN4_A, GPIO.LOW)

    GPIO.output(IN1_B, GPIO.LOW)
    GPIO.output(IN2_B, GPIO.LOW)
    GPIO.output(IN3_B, GPIO.LOW)
    GPIO.output(IN4_B, GPIO.LOW)


def stopsensor():
    # ตรวจสอบระยะจากเซ็นเซอร์
    distance_front = get_distance(TRIG1, ECHO1)
    distance_back = get_distance(TRIG2, ECHO2)
    while True:
        distance_front = get_distance(TRIG1, ECHO1)
        distance_back = get_distance(TRIG2, ECHO2)

        if ( distance_front <= 20) or ( distance_back <= 20):
            stop_motors()
            print("stopsensor")
            break  # ออกจากลูปเมื่อเจอระยะที่ต่ำกว่าเงื่อนไข

        time.sleep(0.1)  # ตรวจสอบทุก 100 ms

r = sr.Recognizer()

# ฟังก์ชันควบคุมมอเตอร์ตามคำสั่งเสียง
def control_motor(command):

    distance_front = get_distance(TRIG1, ECHO1)
    distance_back = get_distance(TRIG2, ECHO2)

    if "forward" in command:
        if  distance_front >= 20:  # ถ้าระยะหน้ามากกว่า 20 cm
            # เดินหน้ามอเตอร์ทั้งหมด
            GPIO.output(IN1_A, GPIO.HIGH)
            GPIO.output(IN2_A, GPIO.LOW)
            GPIO.output(IN3_A, GPIO.HIGH)
            GPIO.output(IN4_A, GPIO.LOW)

            GPIO.output(IN1_B, GPIO.HIGH)
            GPIO.output(IN2_B, GPIO.LOW)
            GPIO.output(IN3_B, GPIO.HIGH)
            GPIO.output(IN4_B, GPIO.LOW)

            print("All motors moving forward")
        else:
            print("stopsensorforward")
            stop_motors()

    elif "backward" in command:
        if distance_back is not None and distance_back >= 20:  # ถ้าระยะหลังมากกว่า 20 cm
            # ถอยหลังมอเตอร์ทั้งหมด
            GPIO.output(IN1_A, GPIO.LOW)
            GPIO.output(IN2_A, GPIO.HIGH)
            GPIO.output(IN3_A, GPIO.LOW)
            GPIO.output(IN4_A, GPIO.HIGH)

            GPIO.output(IN1_B, GPIO.LOW)
            GPIO.output(IN2_B, GPIO.HIGH)
            GPIO.output(IN3_B, GPIO.LOW)
            GPIO.output(IN4_B, GPIO.HIGH)

            print("All motors moving backward")
        else:
            print("stopsensorbackward")
            stop_motors()

    elif "left" in command:
        # เลี้ยวซ้าย: มอเตอร์ฝั่งซ้ายถอยหลัง, มอเตอร์ฝั่งขวาเดินหน้า
        GPIO.output(IN1_A, GPIO.LOW)
        GPIO.output(IN2_A, GPIO.HIGH)
        GPIO.output(IN3_A, GPIO.LOW)
        GPIO.output(IN4_A, GPIO.HIGH)

        GPIO.output(IN1_B, GPIO.HIGH)
        GPIO.output(IN2_B, GPIO.LOW)
        GPIO.output(IN3_B, GPIO.HIGH)
        GPIO.output(IN4_B, GPIO.LOW)

        print("Turning left")
        left_arrow_animation(duration=1)
        lcd_byte(0x01, LCD_CMD)
        control_motor("stop")

    elif "right" in command:
        # เลี้ยวขวา: มอเตอร์ฝั่งซ้ายเดินหน้า, มอเตอร์ฝั่งขวาถอยหลัง
        GPIO.output(IN1_A, GPIO.HIGH)
        GPIO.output(IN2_A, GPIO.LOW)
        GPIO.output(IN3_A, GPIO.HIGH)
        GPIO.output(IN4_A, GPIO.LOW)

        GPIO.output(IN1_B, GPIO.LOW)
        GPIO.output(IN2_B, GPIO.HIGH)
        GPIO.output(IN3_B, GPIO.LOW)
        GPIO.output(IN4_B, GPIO.HIGH)

        print("Turning right")
        right_arrow_animation(duration=2)
        lcd_byte(0x01, LCD_CMD)
        control_motor("stop")

    elif "stop" in command:
        # หยุดมอเตอร์ทั้งหมด
        GPIO.output(IN1_A, GPIO.LOW)
        GPIO.output(IN2_A, GPIO.LOW)
        GPIO.output(IN3_A, GPIO.LOW)
        GPIO.output(IN4_A, GPIO.LOW)

        GPIO.output(IN1_B, GPIO.LOW)
        GPIO.output(IN2_B, GPIO.LOW)
        GPIO.output(IN3_B, GPIO.LOW)
        GPIO.output(IN4_B, GPIO.LOW)

        print("All motors stopped")

    elif "exit" in command:
        return False  # ใช้ในการออกจากลูปหลัก
    else:
        print("Don't understand the command")
    return True

try:
    while True:
        sensor_thread = Thread(target=stopsensor)
        sensor_thread.start()
        with sr.Microphone() as source:
            print("Recording...")

            # รอรับเสียงตลอดเวลา
            audio = r.record(source, duration=3)

            try:
                text = r.recognize_google(audio, language="en")
                print("Received:", text)
                if control_motor(text):  # หากคำสั่งถูกต้อง
                    time.sleep(3)  # รอ 3 วินาทีหลังจากดำเนินการคำสั่ง
                else:
                    break  # ออกจากลูปเมื่อเจอคำสั่ง exit

            except Exception as e:
                print("Sorry, cannot accept the command.:", e)

finally:
    GPIO.cleanup()  # ทำการ cleanup เมื่อออกจากลูป
    print("GPIO cleanup complete")
