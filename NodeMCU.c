// กำหนดค่า Blynk Template ID และชื่อ Template สำหรับโปรเจกต์
#define BLYNK_TEMPLATE_ID "TMPL6r4_krVsk"
#define BLYNK_TEMPLATE_NAME "WIFI Controller Car" 

// เปิดใช้งานการพิมพ์ข้อความผ่าน Serial Monitor เพื่อการ Debug
#define BLYNK_PRINT Serial

// รวมไลบรารีที่ใช้ในการเชื่อมต่อ WiFi และใช้งาน Blynk ผ่าน ESP8266
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>


// กำหนดสถานะการเคลื่อนที่ของรถเป็นค่าพื้นฐาน
bool forward = 0;  
bool backward = 0;  
bool left = 0;  
bool right = 0;  

// ข้อมูลการเชื่อมต่อ Blynk และ WiFi
char auth[] = "nhwT87AVyr3x8byhOHZwG_V0irptr_AS";  // ใส่โทเคน Blynk ที่ได้จากแอป
char ssid[] = "บุเรงนอง";  // ชื่อ WiFi ที่ต้องการเชื่อมต่อ
char pass[] = "thanaphum123";  // รหัสผ่านของ WiFi

// ตัวจับเวลา Blynk สำหรับเรียกฟังก์ชันตามเวลาที่กำหนด
BlynkTimer timer;

void setup() {
  // เริ่มต้นการสื่อสารผ่าน Serial ที่ความเร็ว 9600 สำหรับ UART
  Serial.begin(9600);  
  Blynk.begin(auth, ssid, pass); 
}

void sendCommand(const String& command) {
  // ส่งคำสั่งไปยัง Raspberry Pi ผ่าน Serial
  Serial.println(command);  
}

// ฟังก์ชันที่เรียกใช้เมื่อมีการกดปุ่มใน Virtual Pin V0 (สำหรับการเดินหน้า)
BLYNK_WRITE(V0) {
  forward = param.asInt();  
  if (forward == 1) {
    sendCommand("carforward");  
  } else {
    sendCommand("carstop"); 
  }
}

// ฟังก์ชันที่เรียกใช้เมื่อมีการกดปุ่มใน Virtual Pin V1 (สำหรับการถอยหลัง)
BLYNK_WRITE(V1) {
  backward = param.asInt(); 
  if (backward == 1) {
    sendCommand("carbackward");
  } else {
    sendCommand("carstop");  
  }
}

// ฟังก์ชันที่เรียกใช้เมื่อมีการกดปุ่มใน Virtual Pin V2 (สำหรับการเลี้ยวซ้าย)
BLYNK_WRITE(V2) {
  left = param.asInt();  
  if (left == 1) {
    sendCommand("carleft"); 
  } else {
    sendCommand("carstop");  
  }
}

// ฟังก์ชันที่เรียกใช้เมื่อมีการกดปุ่มใน Virtual Pin V3 (สำหรับการเลี้ยวขวา)
BLYNK_WRITE(V3) {
  right = param.asInt(); 
  if (right == 1) {
    sendCommand("carright"); 
  } else {
    sendCommand("carstop"); 
  }
}

void loop() {
  Blynk.run();  // เรียกใช้งาน Blynk เพื่อเช็คคำสั่งต่างๆ จากแอป
  timer.run();  // เรียกใช้งานตัวจับเวลา Blynk เพื่อประมวลผลตามเวลาที่ตั้งไว้
  delay(20);  // หน่วงเวลาเล็กน้อยเพื่อป้องกันการประมวลผลเร็วเกินไป
}