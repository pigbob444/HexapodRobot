#include <Servo.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <SPI.h>
#include "Hexapodrobot.h"

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire);// 定义一个Adafruit_PWMServoDriver对象pwm，用于控制PCA9685 PWM驱动器。
#define LED PB5    // 将LED宏定义为PB5，这里PB5是一个引脚编号，用于指示特定的GPIO引脚。
#define SDA1 PB7   // 将SDA1宏定义为PB7，用于I2C数据线的引脚编号。
#define SCL1 PB6   // 将SCL1宏定义为PB6，用于I2C时钟线的引脚编号。
#define PCA_OE PB8 // 将PCA_OE宏定义为PB8，这通常用于控制PCA9685 PWM驱动器的输出使能引脚。

#define SERVOMIN 604.0  // 定义SERVOMIN为604.0，这是伺服电机脉冲长度的最小值（在4096的范围内）。
#define SERVOMAX 2260.0 // 定义SERVOMAX为2260.0，这是伺服电机脉冲长度的最大值（在4096的范围内）。
#define SERVO_FREQ 50   // 定义SERVO_FREQ为50，表示模拟伺服电机的更新频率大约为50Hz。

Servo servo[4]; // 定义一个包含4个Servo对象的数组，用于控制4个伺服电机。

gaitEngine Hex;
byte tripodGait[120][6]; // 定义一个二维字节数组tripodGait，用于存储三脚步态的数据，有120行6列，每个元素代表特定时间点特定腿的动作。
byte waveGait[120][6];   // 定义一个二维字节数组waveGait，用于存储波浪步态的数据，结构与tripodGait相同。

float jX = 0;   // 定义一个浮点数jX，用于存储操纵杆X轴的输入，控制机器人的前进或后退。
float jY = 0.0; // 定义一个浮点数jY，用于存储操纵杆Y轴的输入，控制机器人的左右移动。
float jR = 0.0; // 定义一个浮点数jR，用于存储操纵杆旋转的输入，控制机器人的旋转。

long frameCounter = 0; // 定义一个长整型变量frameCounter，用于计数，通常用于跟踪时间或重复的事件。

char inputBuffer[64]; // 定义一个字符数组inputBuffer，大小为64，用于存储输入的字符串数据。

float vBat = 0; // 定义一个浮点数vBat，初始化为0，用于存储电池电压的读数。

boolean IKError = false; // 定义一个布尔变量IKError，初始化为false，用于指示是否存在逆向运动学计算的错误。

boolean powerup = false; // 定义一个布尔变量powerup，初始化为false，用于指示机器人是否正在执行上电序列。

boolean powerdown = false; // 定义一个布尔变量powerdown，初始化为false，用于指示机器人是否正在执行下电序列。

boolean clawOpen = false; // 定义一个布尔变量clawOpen，初始化为false，用于指示机械爪是否处于打开状态。

boolean clawClose = false; // 定义一个布尔变量clawClose，初始化为false，用于指示机械爪是否处于关闭状态。

float clawPos = 0; // 定义一个浮点数clawPos，初始化为0，用于存储机械爪的当前位置。

long controlTimeout; // 定义一个长整型变量controlTimeout，用于存储控制超时的时间，未初始化。

//启动伺服电机
void enableServos(boolean servOn)
{                                       // 定义一个名为enableServos的函数，接受一个布尔参数servOn，用于控制伺服电机的开启或关闭。
  static boolean servosEnabled = false; // 定义一个静态布尔变量servosEnabled，用于记录伺服电机是否已经被启用，初始值为false。
  if (!servosEnabled && servOn)
  {                                           // 如果伺服电机当前未启用且servOn为true（即要求启用伺服电机）。
    digitalWrite(PCA_OE, LOW);                // 将PCA_OE引脚的电平设置为低，通常用于启用PCA9685 PWM驱动器的输出。
    servo[0].attach(PB1, SERVOMIN, SERVOMAX); // 将第一个伺服电机附加到PB1引脚，并设置其脉冲宽度范围为SERVOMIN到SERVOMAX。
    servo[1].attach(PB0, SERVOMIN, SERVOMAX); // 将第二个伺服电机附加到PB0引脚，并设置其脉冲宽度范围为SERVOMIN到SERVOMAX。
    servo[2].attach(PA7, SERVOMIN, SERVOMAX); // 将第三个伺服电机附加到PA7引脚，并设置其脉冲宽度范围为SERVOMIN到SERVOMAX。
    servo[3].attach(PA6, SERVOMIN, SERVOMAX); // 将第四个伺服电机附加到PA6引脚，并设置其脉冲宽度范围为SERVOMIN到SERVOMAX。
    servosEnabled = true;                     // 更新servosEnabled为true，表示伺服电机已启用。
  }
  else if (servosEnabled && !servOn)
  {                             // 如果伺服电机当前已启用且servOn为false（即要求关闭伺服电机）。
    digitalWrite(PCA_OE, HIGH); // 将PCA_OE引脚的电平设置为高，通常用于禁用PCA9685 PWM驱动器的输出。
    servo[0].detach();          // 断开第一个伺服电机的连接。
    servo[1].detach();          // 断开第二个伺服电机的连接。
    servo[2].detach();          // 断开第三个伺服电机的连接。
    servo[3].detach();          // 断开第四个伺服电机的连接。
    servosEnabled = false;      // 更新servosEnabled为false，表示伺服电机已关闭。
  }
}

//检查电池给伺服电机的电压
void batteryCheck()
{                              // 定义一个名为batteryCheck的函数，用于检查电池电压并根据电压高低启用或禁用伺服电机。
  static int batteryTimer = 0; // 定义一个静态整型变量batteryTimer，用于计时，静态变量的值在函数调用之间保持不变。
  if (vBat < 5)
  { // 如果电池电压vBat小于5伏特。
    if (batteryTimer > 50)
    {                      // 如果batteryTimer大于50。
      enableServos(false); // 调用enableServos函数并传入false，表示关闭伺服电机。
    }
    batteryTimer++; // batteryTimer自增1。
  }
  else if (vBat > 6)
  {                     // 如果电池电压vBat大于6伏特。
    enableServos(true); // 调用enableServos函数并传入true，表示开启伺服电机。
    batteryTimer = 0;   // 重置batteryTimer为0。
  }
}
void moveRotateAndOpenClaw(gaitEngine& Hex) {
    // 向前移动
    Hex.walkX = 10.0; // 设置前进的距离
    Hex.gaitSpeed = 1.0; // 设置步态速度为1.0
    Hex.legSpeed = 1.0; // 设置腿部速度为1.0
    Hex.gaitStep(); // 执行步态步骤
    delay(1000); // 延时一段时间，让机器人有时间向前移动

    // 原地旋转
    Hex.walkX = 0.0; // 停止前进
    Hex.walkR = 90.0; // 设置旋转角度
    Hex.gaitStep(); // 执行步态步骤
    delay(1000); // 延时一段时间，让机器人有时间旋转

    // 夹子开关
    // 这部分需要你的机器人具备夹子控制的功能，你需要根据你的代码实际情况进行修改
}

void moveRotateAndOpenClaw() {
  // 向前移动
  jX = 10.0;
  Hex.traX = 10.0;
  powerup = true;
  Hex.gaitSpeed = 1.0;
  Hex.legSpeed = 1.0;
  memcpy(Hex.gSeq, tripodGait, sizeof(Hex.gSeq));
  delay(1000); // 延时一段时间，让机器人有时间向前移动

  // 原地旋转
  jR = 90.0; // 假设jR控制机器人的旋转
  delay(1000); // 延时一段时间，让机器人有时间旋转

  // 夹子开关
  clawOpen = !clawOpen; // 假设clawOpen控制夹子的开关
}

void setup() {
  pinMode(LED, OUTPUT);       // 设置LED引脚为输出模式。
  pinMode(PCA_OE, OUTPUT);    // 设置PCA_OE引脚为输出模式。
  digitalWrite(PCA_OE, HIGH); // 将PCA_OE引脚的电平设置为高，通常用于禁用PCA9685 PWM驱动器的输出。
  Hex.gaitSpeed = 1.0;  // 设置六足机器人的步态速度为1.0，这影响整体移动速度。
  Hex.legSpeed = 1.0;   // 设置单个腿的移动速度为1.0，这影响每条腿移动的速度。
  tripodGait[0][0] = 1; // 在步态数组的第0行第0列设置值为1，表示在时间点0，腿0开始动作。
  tripodGait[0][2] = 1; // 在步态数组的第0行第2列设置值为1，表示在时间点0，腿2开始动作。
  tripodGait[0][4] = 1; // 在步态数组的第0行第4列设置值为1，表示在时间点0，腿4开始动作。

  tripodGait[30][1] = 1; // 在步态数组的第30行第1列设置值为1，表示在时间点30，腿1开始动作。
  tripodGait[30][3] = 1; // 在步态数组的第30行第3列设置值为1，表示在时间点30，腿3开始动作。
  tripodGait[30][5] = 1; // 在步态数组的第30行第5列设置值为1，表示在时间点30，腿5开始动作。

  tripodGait[60][0] = 1; // 在步态数组的第60行第0列设置值为1，表示在时间点60，腿0再次开始动作。
  tripodGait[60][2] = 1; // 在步态数组的第60行第2列设置值为1，表示在时间点60，腿2再次开始动作。
  tripodGait[60][4] = 1; // 在步态数组的第60行第4列设置值为1，表示在时间点60，腿4再次开始动作。

  tripodGait[90][1] = 1; // 在步态数组的第90行第1列设置值为1，表示在时间点90，腿1再次开始动作。
  tripodGait[90][3] = 1; // 在步态数组的第90行第3列设置值为1，表示在时间点90，腿3再次开始动作。
  tripodGait[90][5] = 1; // 在步态数组的第90行第5列设置值为1，表示在时间点90，腿5再次开始动作。

  waveGait[00][0] = 1;  // 在波浪步态数组的第0行第0列设置值为1，表示在时间点0，腿0开始动作。
  waveGait[20][4] = 1;  // 在波浪步态数组的第20行第4列设置值为1，表示在时间点20，腿4开始动作。
  waveGait[40][2] = 1;  // 在波浪步态数组的第40行第2列设置值为1，表示在时间点40，腿2开始动作。
  waveGait[60][5] = 1;  // 在波浪步态数组的第60行第5列设置值为1，表示在时间点60，腿5开始动作。
  waveGait[80][1] = 1;  // 在波浪步态数组的第80行第1列设置值为1，表示在时间点80，腿1开始动作。
  waveGait[100][3] = 1; // 在波浪步态数组的第100行第3列设置值为1，表示在时间点100，腿3开始动作。

  memcpy(Hex.gSeq, tripodGait, sizeof(Hex.gSeq)); // 使用memcpy函数将tripodGait数组的内容复制到Hex.gSeq中，用于设置当前步态序列。

  Serial1.begin(115200); // 初始化Serial1通信，设置波特率为115200，用于串行通信。
  inputBuffer[0] = '\0'; // 初始化inputBuffer字符串为空。

    // 重置I2C总线
  pinMode(SDA1, OUTPUT);    // 将SDA1引脚设置为输出模式，用于I2C数据线。
  pinMode(SCL1, OUTPUT);    // 将SCL1引脚设置为输出模式，用于I2C时钟线。
  digitalWrite(SDA1, HIGH); // 将SDA1引脚电平设置为高，确保I2C数据线处于空闲状态。
  digitalWrite(SCL1, HIGH); // 将SCL1引脚电平设置为高，确保I2C时钟线处于空闲状态。
  delay(200);               // 延迟200毫秒，等待I2C总线稳定。

  pwm.begin(); // 初始化PWM驱动器，用于控制伺服电机。
  pwm.setOscillatorFrequency(27000000); // 设置PWM伺服驱动器的振荡器频率为27MHz。
  pwm.setPWMFreq(SERVO_FREQ); // 设置PWM驱动器的更新频率为SERVO_FREQ，通常为50Hz。

  enableServos(true); // 调用enableServos函数，传入true参数，以启用伺服电机。
}

void loop() {
  if (controlTimeout < millis())
    digitalWrite(LED, millis() % 200 <= 100); // 如果控制超时，则LED以较快的频率闪烁。
  else
    digitalWrite(LED, millis() % 2000 <= 1000); // 否则，LED以较慢的频率闪烁。

  if (powerup)
  { // 如果处于上电状态。
    if (Hex.traZ > 0)
    {                                 // 如果Z轴的平移量大于0。
      Hex.legS[0].setto(-70, -85, 0); // 设置第0腿的位置。
      Hex.legS[1].setto(-90, 0, 0);   // 设置第1腿的位置。
      Hex.legS[2].setto(-70, 85, 0);  // 设置第2腿的位置。
      Hex.legS[3].setto(70, 85, 0);   // 设置第3腿的位置。
      Hex.legS[4].setto(90, 0, 0);    // 设置第4腿的位置。
      Hex.legS[5].setto(70, -85, 0);  // 设置第5腿的位置。
      Hex.traZ -= 0.3;                // 逐渐减少Z轴的平移量，使机器人降低。
    }
    else
    {
      powerup = false; // 当Z轴平移量减少到0或以下时，关闭上电状态。
    }
  }
  else if (powerdown)
 { // 如果机器人处于下电状态。
    if (Hex.traZ < 26)
    {                                 // 检查Z轴的平移量是否小于26。
      Hex.legS[0].setto(-65, -70, 0); // 设置第0腿的位置。
      Hex.legS[1].setto(-80, 0, 0);   // 设置第1腿的位置。
      Hex.legS[2].setto(-65, 70, 0);  // 设置第2腿的位置。
      Hex.legS[3].setto(65, 70, 0);   // 设置第3腿的位置。
      Hex.legS[4].setto(80, 0, 0);    // 设置第4腿的位置。
      Hex.legS[5].setto(65, -70, 0);  // 设置第5腿的位置。
      Hex.traZ += 0.3;                // 逐渐增加Z轴的平移量，使机器人升高。
    }
    else
    {
      powerdown = false; // 当Z轴平移量增加到26或以上时，关闭下电状态。
    }
  }
  clawPos = constrain(clawPos, 0, 67); // 限制爪子位置在0到67之间。
  if (clawOpen)
  { // 如果爪子处于开启状态。
    if (clawPos > 0)
      clawPos -= 2; // 如果爪子位置大于0，则逐渐减少爪子位置，以打开爪子。
    else
      clawOpen = false; // 当爪子完全打开后，关闭开启状态。
  }
  else if (clawClose)
  { // 如果爪子处于关闭状态。
    if (clawPos < 58)
      clawPos += 2; // 如果爪子位置小于58，则逐渐增加爪子位置，以关闭爪子。
    else
      clawClose = false; // 当爪子完全关闭后，关闭关闭状态。
  }
  if (controlTimeout < millis())
  {
    jX = 0; // 将X轴上的移动速度设置为0
    jY = 0; // 将Y轴上的移动速度设置为0
    jR = 0; // 将旋转速度设置为0
  }

  // Configure and run the hexapod engine
  Hex.walkX = jX; // 设置六足机器人在X轴上的行走速度。
  Hex.walkY = jY; // 设置六足机器人在Y轴上的行走速度。
  Hex.walkR = jR; // 设置六足机器人的旋转速度。
  // Check for error in previous loop
  if (IKError)
  {         // 如果上一个循环中存在逆向运动学(IK)错误。
    jX = 0; // 将X轴上的移动速度重置为0。
    jY = 0; // 将Y轴上的移动速度重置为0。
    jR = 0; // 将旋转速度重置为0。
  }
  Hex.gaitStep();                  // 执行六足机器人的步态步骤。
  Hex.runBodyIK();                 // 执行六足机器人身体的逆向运动学计算。
  IKError = (Hex.runLegIK() != 0); // 执行六足机器人腿部的逆向运动学计算，并检查是否有错误。
  // 对六足机器人每条腿的关节角度进行限制，确保它们在物理限制范围内
  float angleA = constrain(Hex.legAngle[0][0] * 180 / PI, -65, 65);    // 第1腿的第1关节角度限制在-65到65度之间
  float angleB = constrain(-Hex.legAngle[0][1] * 180 / PI, -100, 100); // 第1腿的第2关节角度限制在-100到100度之间，角度取反
  float angleC = constrain(Hex.legAngle[0][2] * 180 / PI, -100, 100);  // 第1腿的第3关节角度限制在-100到100度之间
  float angleD = constrain(Hex.legAngle[1][0] * 180 / PI, -65, 65);    // 第2腿的第1关节角度限制在-65到65度之间
  float angleE = constrain(-Hex.legAngle[1][1] * 180 / PI, -100, 100); // 第2腿的第2关节角度限制在-100到100度之间，角度取反
  float angleF = constrain(Hex.legAngle[1][2] * 180 / PI, -100, 100);  // 第2腿的第3关节角度限制在-100到100度之间
  float angleG = constrain(Hex.legAngle[2][0] * 180 / PI, -65, 65);    // 第3腿的第1关节角度限制在-65到65度之间
  float angleH = constrain(-Hex.legAngle[2][1] * 180 / PI, -100, 100); // 第3腿的第2关节角度限制在-100到100度之间，角度取反
  float angleI = constrain(Hex.legAngle[2][2] * 180 / PI, -100, 100);  // 第3腿的第3关节角度限制在-100到100度之间

  float angleJ = constrain(Hex.legAngle[3][0] * 180 / PI, -65, 65);    // 第4腿的第1关节角度限制在-65到65度之间
  float angleK = constrain(Hex.legAngle[3][1] * 180 / PI, -100, 100);  // 第4腿的第2关节角度限制在-100到100度之间
  float angleL = constrain(-Hex.legAngle[3][2] * 180 / PI, -100, 100); // 第4腿的第3关节角度限制在-100到100度之间，角度取反
  float angleM = constrain(Hex.legAngle[4][0] * 180 / PI, -65, 65);    // 第5腿的第1关节角度限制在-65到65度之间
  float angleN = constrain(Hex.legAngle[4][1] * 180 / PI, -100, 100);  // 第5腿的第2关节角度限制在-100到100度之间
  float angleO = constrain(-Hex.legAngle[4][2] * 180 / PI, -100, 100); // 第5腿的第3关节角度限制在-100到100度之间，角度取反
  float angleP = constrain(Hex.legAngle[5][0] * 180 / PI, -65, 65);    // 第6腿的第1关节角度限制在-65到65度之间
  float angleQ = constrain(Hex.legAngle[5][1] * 180 / PI, -100, 100);  // 第6腿的第2关节角度限制在-100到100度之间
  float angleR = constrain(-Hex.legAngle[5][2] * 180 / PI, -100, 100); // 第6腿的第3关节角度限制在-100到100度之间，角度取反

  float angleS = constrain(Hex.rotZ * 180 / PI * 1.7, -65, 65); // 机器人整体旋转角度限制在-65到65度之间
  float angleT = constrain(clawPos, 0, 67);                     // 爪子位置限制在0到67之间

  // 对某些角度进行微调
  angleA += -24; // 第1腿的第1关节角度微调
  angleB += 18;  // 第1腿的第2关节角度微调
  angleC += 3;   // 第1腿的第3关节角度微调

  angleD += 14; // 第2腿的第1关节角度微调
  angleE += 2;  // 第2腿的第2关节角度微调
  angleF += 0;  // 第2腿的第3关节角度不调整

  angleG += 50;  // 第3腿的第1关节角度增加50度
  angleH += -12; // 第3腿的第2关节角度减少12度
  angleI += -1;  // 第3腿的第3关节角度减少1度

  angleJ += -35; // 第4腿的第1关节角度减少35度
  angleK += 11;  // 第4腿的第2关节角度增加11度
  angleL += 6;   // 第4腿的第3关节角度增加6度

  angleM += 3;  // 第5腿的第1关节角度增加3度
  angleN += 12; // 第5腿的第2关节角度增加12度
  angleO += -6; // 第5腿的第3关节角度减少6度

  angleP += 23; // 第6腿的第1关节角度增加23度
  angleQ += 0;  // 第6腿的第2关节角度不调整
  angleR += 0;  // 第6腿的第3关节角度不调整

  angleS += -8; // 机器人整体旋转角度减少8度
  angleT += 0;  // 爪子位置不调整

  //将角度值映射到伺服电机的PWM值范围内
  int servoVALA = map(angleA * 1000, -90 * 1000, 90 * 1000, SERVOMIN * 1000, SERVOMAX * 1000) / 1000.0;   // 将angleA映射到伺服电机的PWM范围
  int servoVALB = map(angleB * 1000, -100 * 1000, 100 * 1000, SERVOMIN * 1000, SERVOMAX * 1000) / 1000.0; // 将angleB映射到伺服电机的PWM范围
  int servoVALC = map(angleC * 1000, -90 * 1000, 90 * 1000, SERVOMIN * 1000, SERVOMAX * 1000) / 1000.0;   // 将angleC映射到伺服电机的PWM范围
  int servoVALD = map(angleD * 1000, -90 * 1000, 90 * 1000, SERVOMIN * 1000, SERVOMAX * 1000) / 1000.0;   // 将angleD映射到伺服电机的PWM范围
  int servoVALE = map(angleE * 1000, -100 * 1000, 100 * 1000, SERVOMIN * 1000, SERVOMAX * 1000) / 1000.0; // 将angleE映射到伺服电机的PWM范围
  int servoVALF = map(angleF * 1000, -90 * 1000, 90 * 1000, SERVOMIN * 1000, SERVOMAX * 1000) / 1000.0;   // 将angleF映射到伺服电机的PWM范围
  int servoVALG = map(angleG * 1000, -90 * 1000, 90 * 1000, SERVOMIN * 1000, SERVOMAX * 1000) / 1000.0;   // 将angleG映射到伺服电机的PWM范围
  int servoVALH = map(angleH * 1000, -100 * 1000, 100 * 1000, SERVOMIN * 1000, SERVOMAX * 1000) / 1000.0; // 将angleH映射到伺服电机的PWM范围
  int servoVALI = map(angleI * 1000, -90 * 1000, 90 * 1000, SERVOMIN * 1000, SERVOMAX * 1000) / 1000.0;   // 将angleI映射到伺服电机的PWM范围

  int servoVALJ = map(angleJ * 1000, -90 * 1000, 90 * 1000, SERVOMIN * 1000, SERVOMAX * 1000) / 1000.0;   // 将angleJ映射到伺服电机的PWM范围
  int servoVALK = map(angleK * 1000, -100 * 1000, 100 * 1000, SERVOMIN * 1000, SERVOMAX * 1000) / 1000.0; // 将angleK映射到伺服电机的PWM范围
  int servoVALL = map(angleL * 1000, -90 * 1000, 90 * 1000, SERVOMIN * 1000, SERVOMAX * 1000) / 1000.0;   // 将angleL映射到伺服电机的PWM范围
  int servoVALM = map(angleM * 1000, -90 * 1000, 90 * 1000, SERVOMIN * 1000, SERVOMAX * 1000) / 1000.0;   // 将angleM映射到伺服电机的PWM范围
  int servoVALN = map(angleN * 1000, -100 * 1000, 100 * 1000, SERVOMIN * 1000, SERVOMAX * 1000) / 1000.0; // 将angleN映射到伺服电机的PWM范围
  int servoVALO = map(angleO * 1000, -90 * 1000, 90 * 1000, SERVOMIN * 1000, SERVOMAX * 1000) / 1000.0;   // 将angleO映射到伺服电机的PWM范围
  int servoVALP = map(angleP * 1000, -90 * 1000, 90 * 1000, SERVOMIN * 1000, SERVOMAX * 1000) / 1000.0;   // 将angleP映射到伺服电机的PWM范围
  int servoVALQ = map(angleQ * 1000, -100 * 1000, 100 * 1000, SERVOMIN * 1000, SERVOMAX * 1000) / 1000.0; // 将angleQ映射到伺服电机的PWM范围
  int servoVALR = map(angleR * 1000, -90 * 1000, 90 * 1000, SERVOMIN * 1000, SERVOMAX * 1000) / 1000.0;   // 将angleR映射到伺服电机的PWM范围

  int servoVALS = map(angleS * 1000, -90 * 1000, 90 * 1000, SERVOMIN * 1000, SERVOMAX * 1000) / 1000.0; // 将angleS映射到伺服电机的PWM范围
  int servoVALT = map(angleT * 1000, -90 * 1000, 90 * 1000, SERVOMIN * 1000, SERVOMAX * 1000) / 1000.0; // 将angleT映射到伺服电机的PWM范围

  static int testVal = SERVOMIN;
    if (testVal >= SERVOMAX)
    testVal = SERVOMIN; // 如果testVal大于或等于SERVOMAX，则将其重置为SERVOMIN

  vBat = (float(analogRead(PA3)) / 4069.0) * 10.321; // 读取PA3引脚的模拟值，通过比例换算计算出电池电压值。

  batteryCheck(); // 调用batteryCheck函数，检查电池电压是否在安全范围内。

  if (IKError)
  {                                    // 如果存在逆向运动学错误（即计算出的关节角度不可达）。
    Serial.println("IKError Occured"); // 通过串口输出错误信息，提示逆向运动学计算出错。
  }
  else
  {                                      // 如果没有逆向运动学错误，执行以下代码。
    pwm.writeMicroseconds(0, servoVALM); // 控制第0号PWM通道的伺服电机，使其转动到servoVALM指定的位置。
    pwm.writeMicroseconds(1, servoVALN); // 控制第1号PWM通道的伺服电机，使其转动到servoVALN指定的位置。
    pwm.writeMicroseconds(2, servoVALO); // 控制第2号PWM通道的伺服电机，使其转动到servoVALO指定的位置。

    pwm.writeMicroseconds(3, servoVALP); // 控制第3号PWM通道的伺服电机，使其转动到servoVALP指定的位置。
    pwm.writeMicroseconds(4, servoVALQ); // 控制第4号PWM通道的伺服电机，使其转动到servoVALQ指定的位置。
    pwm.writeMicroseconds(5, servoVALR); // 控制第5号PWM通道的伺服电机，使其转动到servoVALR指定的位置。

    pwm.writeMicroseconds(6, servoVALA); // 控制第6号PWM通道的伺服电机，使其转动到servoVALA指定的位置。
    pwm.writeMicroseconds(7, servoVALB); // 控制第7号PWM通道的伺服电机，使其转动到servoVALB指定的位置。
    pwm.writeMicroseconds(8, servoVALC); // 控制第8号PWM通道的伺服电机，使其转动到servoVALC指定的位置。

    servo[0].writeMicroseconds(servoVALD); // 控制第0号伺服对象的电机，使其转动到servoVALD指定的位置。
    servo[1].writeMicroseconds(servoVALE); // 控制第1号伺服对象的电机，使其转动到servoVALE指定的位置。
    servo[2].writeMicroseconds(servoVALF); // 控制第2号伺服对象的电机，使其转动到servoVALF指定的位置。

    servo[3].writeMicroseconds(servoVALJ); // 控制第3号伺服对象的电机，使其转动到servoVALJ指定的位置。
    pwm.writeMicroseconds(15, servoVALK);  // 控制第15号PWM通道的伺服电机，使其转动到servoVALK指定的位置。
    pwm.writeMicroseconds(14, servoVALL);  // 控制第14号PWM通道的伺服电机，使其转动到servoVALL指定的位置。

    pwm.writeMicroseconds(13, servoVALG); // 控制第13号PWM通道的伺服电机，使其转动到servoVALG指定的位置。
    pwm.writeMicroseconds(12, servoVALH); // 控制第12号PWM通道的伺服电机，使其转动到servoVALH指定的位置。
    pwm.writeMicroseconds(11, servoVALI); // 控制第11号PWM通道的伺服电机，使其转动到servoVALI指定的位置。

    pwm.writeMicroseconds(10, servoVALS); // 控制第10号PWM通道的伺服电机，使其转动到servoVALS指定的位置。
    pwm.writeMicroseconds(9, servoVALT);  // 控制第9号PWM通道的伺服电机，使其转动到servoVALT指定的位置。
  }
  gaitEngine Hex; // 创建一个gaitEngine的实例

while (true) {
    moveRotateAndOpenClaw(Hex);
}

  while (Serial1.available() > 0)
  {                              // 当串口1有数据可读时，进入循环
    char input = Serial1.read(); // 读取一个字符
    static int s_len;            // 静态变量s_len，默认初始化为0，用于记录输入缓冲区的长度
    if ((s_len >= 64) || (input != '\n' && input != '\r'))
    {                               // 如果输入缓冲区已满或读取的字符不是换行或回车
      inputBuffer[s_len++] = input; // 将读取的字符存入输入缓冲区，并增加缓冲区长度计数
    }
    else
    {                                        // 如果读取的字符是换行或回车，表示一条指令结束
      char commandChar = inputBuffer[0];     // 获取指令的第一个字符，用于判断指令类型
      char *strtokIndx;                      // 声明一个指针，用于strtok()函数的索引
      strtokIndx = strtok(inputBuffer, " "); // 使用strtok()函数分割字符串，获取指令的第一个参数
      strtokIndx = strtok(NULL, " ");        // 继续调用strtok()获取下一个参数
      if (commandChar == 'm')
      {                                 // 如果指令是移动（MOVE）
        jX = atof(strtokIndx);          // 将字符串转换为浮点数，赋值给jX
        strtokIndx = strtok(NULL, " "); // 获取下一个参数
        jY = atof(strtokIndx);          // 将字符串转换为浮点数，赋值给jY
        strtokIndx = strtok(NULL, " "); // 获取下一个参数
        jR = atof(strtokIndx);          // 将字符串转换为浮点数，赋值给jR
      }
      else if (commandChar == 't')
      {                                 // 如果指令是平移（TRANSLATE）
        Hex.traX = atof(strtokIndx);    // 将字符串转换为浮点数，赋值给Hex.traX
        strtokIndx = strtok(NULL, " "); // 获取下一个参数
        Hex.traY = atof(strtokIndx);    // 将字符串转换为浮点数，赋值给Hex.traY
        strtokIndx = strtok(NULL, " "); // 获取下一个参数
        Hex.traZ += atof(strtokIndx);   // 将字符串转换为浮点数并累加到Hex.traZ上
      }
      else if (commandChar == 'r')
      {                                 // 如果指令是旋转（ROTATE）
        Hex.rotX = atof(strtokIndx);    // 将字符串转换为浮点数，赋值给Hex.rotX
        strtokIndx = strtok(NULL, " "); // 获取下一个参数
        Hex.rotY = atof(strtokIndx);    // 将字符串转换为浮点数，赋值给Hex.rotY
        strtokIndx = strtok(NULL, " "); // 获取下一个参数
        Hex.rotZ = atof(strtokIndx);    // 将字符串转换为浮点数，赋值给Hex.rotZ
      }
      else if (commandChar == 'u')
      {                 // 如果指令是上升（UP）
        powerup = true; // 设置上升标志为真
      }
      else if (commandChar == 'd')
      {                   // 如果指令字符是'd'，表示下降指令
        powerdown = true; // 设置下降标志为真
      }
      else if (commandChar == 'a')
      {                                                 // 如果指令字符是'a'，表示使用三脚步态
        Hex.gaitSpeed = 1.0;                            // 设置步态速度为1.0
        Hex.legSpeed = 1.0;                             // 设置腿部速度为1.0
        memcpy(Hex.gSeq, tripodGait, sizeof(Hex.gSeq)); // 将三脚步态数组复制到Hex.gSeq中
      }
      else if (commandChar == 'b')
      {                                               // 如果指令字符是'b'，表示使用波浪步态
        Hex.gaitSpeed = 2.0;                          // 设置步态速度为2.0
        Hex.legSpeed = 1.3;                           // 设置腿部速度为1.3
        memcpy(Hex.gSeq, waveGait, sizeof(Hex.gSeq)); // 将波浪步态数组复制到Hex.gSeq中
      }
      else if (commandChar == 'c')
      {                   // 如果指令字符是'c'，表示关闭爪子
        clawClose = true; // 设置关闭爪子标志为真
      }
      else if (commandChar == 'o')
      {                  // 如果指令字符是'o'，表示打开爪子
        clawOpen = true; // 设置打开爪子标志为真
      }
      else if (commandChar == 'g')
      {                              // 如果指令字符是'g'，表示对爪子进行微调
        clawPos += atof(strtokIndx); // 将下一个参数转换为浮点数并累加到爪子位置上
      }
      else if (commandChar == 'h')
      {                                   // 如果指令字符是'h'，表示心跳信号
        controlTimeout = millis() + 3000; // 设置控制超时时间为当前时间加3000毫秒
      }
      // 重置输入缓冲区
      memset(inputBuffer, 0, sizeof(inputBuffer)); // 将输入缓冲区清零
      s_len = 0;                                   // 重置输入缓冲区长度为0
    }
  }

}

