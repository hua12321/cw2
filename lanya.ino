// 引入库
#include <Arduino.h>

// 定义电机驱动引脚
#define IN1 A5
#define IN2 A2
#define IN3 12
#define IN4 A3
#define ENA 3
#define ENB 11

// 定义蓝牙模块引脚
#define BT_RX 1
#define BT_TX 0

void setup() {
  // 初始化电机驱动引脚
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  // 初始化蓝牙模块引脚
  pinMode(BT_RX, INPUT);
  pinMode(BT_TX, OUTPUT);

  // 设置初始状态
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  // 设置电机 PWM 引脚的初始速度
  analogWrite(ENA, 255);  // 电机 A 最大速度
  analogWrite(ENB, 255);  // 电机 B 最大速度

  Serial.begin(9600);  // 启用串口调试
}

void loop() {
  // 检查是否有蓝牙数据
  if (Serial.available() > 0) {
    char command = Serial.read(); // 读取蓝牙传来的指令
    processCommand(command); // 处理指令
  }
}

// 前进
void moveForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 255);
  analogWrite(ENB, 255);
}

// 停止电机
void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

// 处理蓝牙指令
void processCommand(char command) {
  switch (command) {
    case 'F': moveForward(); break;  // 前进
    case 'G': moveBackward(); break; // 后退
    case 'L': turnLeft(); break;     // 左转
    case 'R': turnRight(); break;    // 右转
    case 'S': stopMotors(); break;   // 停止
    default: stopMotors(); break;
  }
}

// 后退
void moveBackward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, 255);
  analogWrite(ENB, 255);
}

// 左转
void turnLeft() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, 245);
  analogWrite(ENB, 245);
}

// 右转
void turnRight() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 245);
  analogWrite(ENB, 245);
}
