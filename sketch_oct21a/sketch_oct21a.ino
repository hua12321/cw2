    #include <LiquidCrystal.h>
  #include <Wire.h>

  LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

  #define L1 10
  #define L2 2
  #define R1 12
  #define R2 13
  #define enA 3
  #define enB 11
  #define IR A3
  #define IR2 A2
  #define analogEncoder A1

  volatile int eventCount = 0;
  float distance = 0.0;
  const float wheelCircumference = 10.0;  
  const int pulsesPerRevolution = 5;     
  const float countToDistance = wheelCircumference / pulsesPerRevolution;

  unsigned long lastPulseTime = 0;
  const unsigned long debounceDelay = 10;
  unsigned long startTime = 0;
  unsigned long elapsedTime = 0;

  float pitch = -40.0; 
  bool angleControlEnabled = true; 
  bool wasd =false;
  bool startDistanceTracking = false; 

    bool nihao1234=false;
    bool nmds=false;
  // MPU6050 Variables
  const int MPU = 0x68; // MPU6050 I2C address
  float AccX, AccY, AccZ;
  float GyroX, GyroY, GyroZ;
  float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
  float roll, yaw;

  void setup() {
    pinMode(L1, OUTPUT);
    pinMode(L2, OUTPUT);
    pinMode(R1, OUTPUT);
    pinMode(R2, OUTPUT);
    pinMode(IR, INPUT);
    pinMode(IR2, INPUT);
    pinMode(enA, OUTPUT);
    pinMode(enB, OUTPUT);

    analogWrite(enA, 115);
    analogWrite(enB, 115);
    
    lcd.begin(16, 2);
    lcd.print("Initializing...");
    Serial.begin(9600);

    Wire.begin();                      // Initialize I2C communication
    Wire.beginTransmission(MPU);       
    Wire.write(0x6B);                  
    Wire.write(0x00);                  // Wake up MPU6050
    Wire.endTransmission(true);   
    
    delay(100);
    lcd.clear();
    lcd.print("Line Following");
    startTime = millis();
  }

  void readAnalogEncoder() {
    static int lastState = 0;
    int currentState = analogRead(analogEncoder);

    if (currentState > 512 && lastState <= 512) {
      unsigned long currentTime = millis();
      if (currentTime - lastPulseTime > debounceDelay) {
        eventCount++;
        if (startDistanceTracking) {
          distance = eventCount * countToDistance*2.5;
        }
        lastPulseTime = currentTime;
      }
    }
    lastState = currentState;
  }

  void readMPU6050() {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true); 

    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; 
    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; 
    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; 
    //Calculate the angle of the accelerometer
    accAngleX = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * 180 / PI; 
    accAngleY = atan(-1 * AccX / sqrt(AccY * AccY + AccZ * AccZ)) * 180 / PI;
  
    Wire.beginTransmission(MPU);
    Wire.write(0x43); 
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true); 
    
    GyroX = (Wire.read() << 8 | Wire.read()) / 131.0; 
    GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
    GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
    //Accumulated gyroscope angle
    gyroAngleX += GyroX * (millis() - lastPulseTime) / 1000.0;
    gyroAngleY += GyroY * (millis() - lastPulseTime) / 1000.0;
    yaw += GyroZ * (millis() - lastPulseTime) / 1000.0;
    //Complementary filtering fusion of accelerometer and gyroscope data
    roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
    pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;
  }

void move(int speed) {
  digitalWrite(L1, HIGH);
  digitalWrite(L2, LOW);
  digitalWrite(R1, HIGH);
  digitalWrite(R2, LOW);
  analogWrite(enA, speed);
  analogWrite(enB, speed);
}

void stop() {
  digitalWrite(L1, LOW);
  digitalWrite(L2, LOW);
  digitalWrite(R1, LOW);
  digitalWrite(R2, LOW);
  analogWrite(enA, 0);
  analogWrite(enB, 0);
}

void left(int speed) {
  digitalWrite(L1, HIGH);
  digitalWrite(L2, LOW);
  digitalWrite(R1, LOW);
  digitalWrite(R2, HIGH);
  analogWrite(enA, speed);
  analogWrite(enB, speed);
}

void right(int speed) {
  digitalWrite(L1, LOW);
  digitalWrite(L2, HIGH);
  digitalWrite(R1, HIGH);
  digitalWrite(R2, LOW);
  analogWrite(enA, speed);
  analogWrite(enB, speed);
}

void updateLCD() {
  static unsigned long lastDisplayTime = 0;
  if (millis() - lastDisplayTime > 500) {  
    lastDisplayTime = millis();
    lcd.clear();

    lcd.print("Dist: ");
    lcd.print(distance);
    lcd.print(" cm");

    lcd.setCursor(0, 1);
    lcd.print("Time: ");
    lcd.print(elapsedTime);
    lcd.print("s A: ");
    lcd.print(pitch, 1);  
  }
}

void printDebugInfo(int ir1Value, int ir2Value) {
  Serial.print("IR1: ");
  Serial.print(ir1Value);
  Serial.print("\tIR2: ");
  Serial.print(ir2Value);
  Serial.print("\tDistance: ");
  Serial.print(distance);
  Serial.print(" cm\tPitch: ");
  Serial.println(pitch);
}

void handleLineFollowing(int ir1Value, int ir2Value, int speed) {
  if (ir1Value < 102 && ir2Value < 102) {
    stop(); //  When both sensors detect a black line, the car stops
  } else if (ir1Value > 102 && ir2Value <= 102) {
    right(255);//  When the left sensor detects a white area and the right sensor detects a black line, the car turns right
  } else if (ir2Value > 102 && ir1Value <= 102) {
    left(255); //  When the right sensor detects a white area and the left sensor detects a black line, the car turns left
  } else {
    move(speed); // When both sensors detect a white area, the car moves forward
  }
}

void loop() {
  elapsedTime = millis() / 1000; 

  readAnalogEncoder();
if (wasd==false){
  readMPU6050();
}
  int ir1Value = analogRead(IR);
  int ir2Value = analogRead(IR2);
   if (nmds==false){
       
  handleLineFollowing(ir1Value, ir2Value, 155);    
    delay(2000);
    stop();
    delay(2000);
    nmds=true;
   }
    if (pitch > 20.0  ) {  
    move(255);                  
    delay(1500);              
    stop();                     
    delay(4000);               
    left(255);                 
    delay(3000);                
    stop();                     
    delay(500); 
    move(255);
    delay(2000);                
    wasd =true;
    startDistanceTracking = true; 
    pitch =0;
    eventCount = 0;            
    unsigned long startTime = millis();}
    
    
        if (distance >= 310 && nihao1234==false) {
            stop();  
            nihao1234=true;            
            delay(4000);}
        
   
  handleLineFollowing(ir1Value, ir2Value, 120);     
  updateLCD(); 
  printDebugInfo(ir1Value, ir2Value);          
}