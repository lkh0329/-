#define S1 4   // 最左感測器
#define S2 16  // 中左感測器
#define S3 17  // 正中感測器
#define S4 5   // 中右感測器
#define S5 18  // 最右感測器

#define VLS_PIN 2 // 起跑觸發開關

#define ASL_R 33  // RGB指示燈 - 紅
#define ASL_G 32  // RGB指示燈 - 綠
#define ASL_B 23  // RGB指示燈 - 藍

#define ENA 12    // 左馬達 PWM
#define IN1 13    // 左馬達 正轉
#define IN2 14    // 左馬達 反轉
#define IN3 27    // 右馬達 正轉
#define IN4 26    // 右馬達 反轉
#define ENB 25    // 右馬達 PWM

int leftBaseSpeed = 200;     // 左輪基礎速度
int rightBaseSpeed = 200;    // 右輪基礎速度

float Kp = 100.0;            // PD控制：比例係數 (轉向拉力)
float Kd = 140.0;            // PD控制：微分係數 (防止甩尾震盪)

float errorThreshold = 0.5;  // 入彎煞車門檻：誤差大於此值即開始降速
float speedDropRatio = 70.0; // 降速力道：數值越大，過彎煞車越重

float lastError = 0; // 記錄前次誤差，用於微分計算與盲區尋線
int systemState = 0; // 0: 鎖定(綠燈), 1: 準備起跑(藍燈), 2: 循跡中(紅燈)

void setup() {
  Serial.begin(115200);
  
  pinMode(S1, INPUT);
  pinMode(S2, INPUT);
  pinMode(S3, INPUT);
  pinMode(S4, INPUT);
  pinMode(S5, INPUT);
  pinMode(VLS_PIN, INPUT_PULLUP); // 啟用內部上拉電阻
  
  pinMode(ASL_R, OUTPUT);
  pinMode(ASL_G, OUTPUT);
  pinMode(ASL_B, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  // 初始確保車輛靜止並亮綠燈
  stopMotors();
  setASL(0, 1, 0); 
}

void loop() {
  // === 三階段狀態機 ===
  if (systemState == 0) { 
    // 狀態 0：安全鎖定模式
    stopMotors();
    setASL(0, 1, 0); // 綠燈
    if (digitalRead(VLS_PIN) == LOW) {
      systemState = 1; // 按下按鈕，進入準備狀態
    }
  } 
  else if (systemState == 1) { 
    // 狀態 1：起跑延遲模式
    setASL(0, 0, 1); // 藍燈
    delay(1000);     // 延遲1秒符合賽規
    systemState = 2; // 進入循跡狀態
  } 
  else if (systemState == 2) { 
    // 狀態 2：自主循跡模式
    setASL(1, 0, 0); // 紅燈
    lineTrackingLogic();
  }
}

void lineTrackingLogic() {
  // 讀取感測器，反轉邏輯使壓到黑線=1，白底=0
  int v1 = !digitalRead(S1);
  int v2 = !digitalRead(S2);
  int v3 = !digitalRead(S3);
  int v4 = !digitalRead(S4);
  int v5 = !digitalRead(S5);

  int sum = v1 + v2 + v3 + v4 + v5; 
  float error = lastError;          

  // 1. 計算誤差 (直角彎外側優先權)
  if (sum > 0) {
    if (v1 == 1) {
      error = -2.5; // 最左壓線：給予極端負值，強制猛烈左轉
    } 
    else if (v5 == 1) {
      error = 2.5;  // 最右壓線：給予極端正值，強制猛烈右轉
    } 
    else {
      // 只有內側壓線時，才進行平滑加權平均
      error = (v2 * (-1.0) + v3 * 0.0 + v4 * 1.0) / sum;
    }
  } else {
    // 2. 盲區尋線 (全白脫線狀態)
    if (lastError > 0) {
      error = 4.0;  // 剛才偏右，原地極端右轉找線
    } else {
      error = -4.0; // 剛才偏左，原地極端左轉找線
    }
  }

  // 3. 動態降速 (彎道主動煞車)
  int dropAmount = 0;
  if (abs(error) > errorThreshold) {
    dropAmount = (abs(error) - errorThreshold) * speedDropRatio;
  }
  
  // 計算當前基準速度 (扣除煞車量)
  int currentLeftBase = leftBaseSpeed - dropAmount;
  int currentRightBase = rightBaseSpeed - dropAmount;

  // 4. PD 控制計算修正量
  float P = Kp * error;
  float D = Kd * (error - lastError);
  float correction = P + D;
  
  lastError = error; // 更新誤差紀錄

  // 5. 速度合成與限制
  int leftSpeed = currentLeftBase + correction;
  int rightSpeed = currentRightBase - correction;

  leftSpeed = constrain(leftSpeed, -255, 255); // 限制PWM不超出正負255
  rightSpeed = constrain(rightSpeed, -255, 255);

  setMotorSpeed(leftSpeed, rightSpeed);
}

void setMotorSpeed(int leftPWM, int rightPWM) {
  // 左馬達方向控制：PWM為負數代表反轉
  if (leftPWM >= 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    leftPWM = -leftPWM; // 轉為正數供 analogWrite 使用
  }
  
  // 右馬達方向控制：PWM為負數代表反轉
  if (rightPWM >= 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    rightPWM = -rightPWM; // 轉為正數供 analogWrite 使用
  }

  // 輸出最終速度
  analogWrite(ENA, leftPWM);
  analogWrite(ENB, rightPWM);
}

// 停止所有馬達
void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

// 設定 RGB 指示燈
void setASL(int r, int g, int b) {
  digitalWrite(ASL_R, r);
  digitalWrite(ASL_G, g);
  digitalWrite(ASL_B, b);
}