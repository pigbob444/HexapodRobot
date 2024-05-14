#include "Hexapodrobot.h" // 包含ZeroBug.h头文件，可能包含类定义、函数原型等。

/*---------------------------------------------------------------*
  Class Vector
*----------------------------------------------------------------*/

vector::vector()
{          // 定义Vector类的构造函数，用于初始化Vector对象。
  x = 0.0; // 将x成员变量初始化为0.0。
  y = 0.0; // 将y成员变量初始化为0.0。
  z = 0.0; // 将z成员变量初始化为0.0。
}

void vector::reset()
{          // 定义Vector类的reset方法，用于重置Vector对象的成员变量。
  x = 0.0; // 将x成员变量重置为0.0。
  y = 0.0; // 将y成员变量重置为0.0。
  z = 0.0; // 将z成员变量重置为0.0。
}

void vector::setto(float nx, float ny, float nz)
{                   // 定义Vector类的setto方法，用于设置Vector对象的成员变量。
  xs = xt = x = nx; // 将nx赋值给x，并且可能还有其他用途的xs和xt变量。
  ys = yt = y = ny; // 将ny赋值给y，并且可能还有其他用途的ys和yt变量。
  zs = zt = z = nz; // 将nz赋值给z，并且可能还有其他用途的zs和zt变量。
}

void vector::setto(vector v)
{                    // 定义Vector类的重载setto方法，用于将一个Vector对象的值复制给另一个Vector对象。
  xs = xt = x = v.x; // 将参数v的x成员变量赋值给当前对象的x，并且可能还有其他用途的xs和xt变量。
  ys = yt = y = v.y; // 将参数v的y成员变量赋值给当前对象的y，并且可能还有其他用途的ys和yt变量。
  zs = zt = z = v.z; // 将参数v的z成员变量赋值给当前对象的z，并且可能还有其他用途的zs和zt变量。
}

bool vector::equalegS(vector v)
{                      // 定义vector类的equalegS方法，用于判断两个vector对象是否相等。
  bool isEqual = true; // 初始化isEqual变量为true。
  if (x != v.x || y != v.y || z != v.z)
    isEqual = false; // 如果当前对象与参数v的任一成员变量不相等，则设置isEqual为false。
  return isEqual;    // 返回isEqual的值。
}

float vector::move2D(float pps)
{                                                        // 定义vector类的move2D方法，用于在2D平面上移动vector对象。
  float totalDistance = sqrt(sq(xt - xs) + sq(yt - ys)); // 计算从起始点(xs, ys)到目标点(xt, yt)的总距离。
  pps = 30 / pps;                           // 将参数pps（每秒移动的像素数）转换为每步移动的距离。
  float dx = xt - x;                        // 计算当前位置到目标位置在x轴上的距离。
  float dy = yt - y;                        // 计算当前位置到目标位置在y轴上的距离。
  float distance = sqrt(dx * dx + dy * dy); // 计算当前位置到目标位置的直线距离。
  float angle = abs(atan(dy / dx));         // 计算移动方向的角度。
  if (distance > totalDistance / pps)
  { // 如果当前位置到目标位置的距离大于每步应移动的距离，则进行移动。
    if (dx > 0)
      x += cos(angle) * totalDistance / pps; // 如果dx大于0，沿正x方向移动。
    else
      x -= cos(angle) * totalDistance / pps; // 如果dx小于0，沿负x方向移动。
    if (dy > 0)
      y += sin(angle) * totalDistance / pps; // 如果dy大于0，沿正y方向移动。
    else
      y -= sin(angle) * totalDistance / pps; // 如果dy小于0，沿负y方向移动。
  }
  else
  { // 如果当前位置到目标位置的距离小于或等于每步应移动的距离，直接移动到目标位置。
    x = xt;
    y = yt;
  }
  //} // 这是被注释掉的代码块的结束。
  distance = sqrt(sq(xt - x) + sq(yt - y)); // 重新计算当前位置到目标位置的直线距离。

  float distanceLeft = distance / totalDistance; // 计算剩余距离占总距离的比例。
  if (totalDistance == 0)
    distanceLeft = 0; // 如果总距离为0，则将剩余距离比例设置为0。

  return distanceLeft; // 返回剩余距离比例。
}

/*---------------------------------------------------------------*
    Class gaitEngine
*----------------------------------------------------------------*/

gaitEngine::gaitEngine()
{ // 定义gaitEngine类的构造函数，用于初始化gaitEngine对象。

  for (int i = 0; i < 6; i++)
  {                        // 使用for循环初始化leg、legS、legT、legIK和coxaPos数组中的每个元素。
    leg[i] = vector();     // 为leg数组的每个元素创建一个vector对象。
    legS[i] = vector();    // 为legS数组的每个元素创建一个vector对象。
    legT[i] = vector();    // 为legT数组的每个元素创建一个vector对象。
    legIK[i] = vector();   // 为legIK数组的每个元素创建一个vector对象。
    coxaPos[i] = vector(); // 为coxaPos数组的每个元素创建一个vector对象。
  }
  leg[0].setto(-70, -85, 0); // 初始化第一个腿的位置。
  leg[1].setto(-90, 0, 0);   // 初始化第二个腿的位置。
  leg[2].setto(-70, 85, 0);  // 初始化第三个腿的位置。
  leg[3].setto(70, 85, 0);   // 初始化第四个腿的位置。
  leg[4].setto(90, 0, 0);    // 初始化第五个腿的位置。
  leg[5].setto(70, -85, 0);  // 初始化第六个腿的位置。

  for (int i = 0; i < 6; i++)
  {                        // 使用for循环将leg数组中的每个元素的值复制到legS和legT数组中的对应元素。
    legS[i].setto(leg[i]); // 将leg数组中的每个元素的值复制到legS数组中的对应元素。
    legT[i].setto(leg[i]); // 将leg数组中的每个元素的值复制到legT数组中的对应元素。
  }

  int bodyHeight = 40;                    // 初始化机器人的身体高度为40。
  coxaPos[0].setto(-28, -48, bodyHeight); // 设置第一个腿的coxa位置。
  coxaPos[1].setto(-35, 0, bodyHeight);   // 设置第二个腿的coxa位置。
  coxaPos[2].setto(-28, 48, bodyHeight);  // 设置第三个腿的coxa位置。
  coxaPos[3].setto(28, 48, bodyHeight);   // 设置第四个腿的coxa位置。
  coxaPos[4].setto(35, 0, bodyHeight);    // 设置第五个腿的coxa位置。
  coxaPos[5].setto(28, -48, bodyHeight);  // 设置第六个腿的coxa位置。
}

void gaitEngine::gaitStep()
{ // 定义gaitEngine类的gaitStep方法，用于控制机器人的步态。

  walkX = constrain(walkX, -0.8, 0.8); // 限制机器人X轴方向的移动速度
  walkY = constrain(walkY, -0.6, 0.6); // 限制机器人Y轴方向的移动速度
  walkR = constrain(walkR, -0.6, 0.6); // 限制机器人旋转速度

  gaitStepN += gaitSpeed;                        // 根据步态速度更新步态步骤
  int gSeqLength = sizeof gSeq / sizeof gSeq[0]; // 计算步态序列的长度
  if (gaitStepN >= gSeqLength)
    gaitStepN = 0; // 如果当前步骤超过序列长度，则重置步骤编号

  // 计算腿部相对于地面的旋转和平移
  float s = sin(-walkR / 100); // 计算旋转角度的正弦值
  float c = cos(-walkR / 100); // 计算旋转角度的余弦值

  // 根据地面移动腿部
  for (int i = 0; i < 6; i++)
  { // 遍历所有腿
    if (leg[i].z == 0)
    {                                                   // 如果腿在地面上
      leg[i].x = (leg[i].x * c - leg[i].y * s) - walkX; // 根据旋转和X轴移动更新腿的X坐标
      leg[i].y = (leg[i].y * c + leg[i].x * s) + walkY; // 根据旋转和Y轴移动更新腿的Y坐标
      // 如果腿的当前位置与起始位置的距离小于目标死区，则不需要移动
      if (leg[i].z == 0 && sqrt(sq(leg[i].x - legS[i].x) + sq(leg[i].y - legS[i].y)) < targetDeadZone)
      {
        legT[i].x = leg[i].x; // 保持腿的目标X坐标不变
        legT[i].y = leg[i].y; // 保持腿的目标Y坐标不变
      }
      else
      {                         // 如果腿离中心太远，则重置目标位置
        legT[i].setto(legS[i]); // 将腿的目标位置重置为起始位置
      }
    }
  }

  for (int legN = 0; legN < 6; legN++)
  { // 遍历所有六条腿

    // 如果轮到该腿按顺序移动，并且目标位置未达到，并且腿还没有开始移动
    // 待办事项：不要依赖于对gaitStepN的四舍五入来匹配gSeq中的序列
    if (gSeq[(int)gaitStepN][legN] == 1 && !leg[legN].equalegS(legT[legN]) && legMoving[legN] == 0)
    { // 检查步态序列以确定哪些腿应该开始移动。
      // 如果当前步态序列指示该腿应该移动，且腿的目标位置不等于起始位置，且腿当前未在移动状态，则执行以下操作
      leg[legN].xt = legS[legN].x; // 设置腿的目标X坐标为起始X坐标
      leg[legN].yt = legS[legN].y; // 设置腿的目标Y坐标为起始Y坐标
      leg[legN].xs = leg[legN].x;  // 记录腿的当前X坐标
      leg[legN].ys = leg[legN].y;  // 记录腿的当前Y坐标
      legMoving[legN] = 1;         // 将腿的移动状态设置为1，表示开始移动
    }

    // if leg ist already moving
    if (legMoving[legN] == 1)
    {
      // 如果腿已经在移动状态
      float distanceLeft = leg[legN].move2D(legSpeed); // 计算腿移动的剩余距离
      // leg[legN].z = legS[legN].z + stepHeight-stepHeight*sq(2*distanceLeft-1);//quadratic function leg lift
      leg[legN].z = legS[legN].z + stepHeight / 2 * (sin(2 * PI * (distanceLeft - 0.25)) + 1); // 使用正弦函数平滑地提升腿部
      if (leg[legN].z == 0)
        legMoving[legN] = 0; // 如果腿回到地面，将移动状态重置为0
    }
  }
}

// 机体的平移和旋转
void gaitEngine::runBodyIK()
{ // 开始定义gaitEngine类的runBodyIK方法，用于计算机体的平移和旋转对腿部逆运动学的影响

  traX = constrain(traX, -40, 40); // 限制机体X轴的平移范围在-40到40之间
  traY = constrain(traY, -40, 40); // 限制机体Y轴的平移范围在-40到40之间
  traZ = constrain(traZ, -40, 40); // 限制机体Z轴的平移范围在-40到40之间

  rotX = constrain(rotX, -0.3, 0.3);   // 限制机体绕X轴的旋转范围在-0.3到0.3之间
  rotY = constrain(rotY, -0.25, 0.25); // 限制机体绕Y轴的旋转范围在-0.25到0.25之间
  rotZ = constrain(rotZ, -0.25, 0.25); // 限制机体绕Z轴的旋转范围在-0.25到0.25之间

  for (int i = 0; i < 6; i++)
  { // 遍历所有六条腿，对每条腿执行以下计算
    // 根据机体的旋转和平移计算每条腿的新X坐标
    legIK[i].x = leg[i].x * cos(rotZ) * cos(rotX) - leg[i].z * cos(rotZ) * sin(rotX) + leg[i].y * sin(rotZ) + traX;
    // 根据机体的旋转和平移计算每条腿的新Y坐标
    legIK[i].y = leg[i].x * (sin(rotY) * sin(rotX) - cos(rotY) * sin(rotZ) * cos(rotX)) + leg[i].z * (cos(rotY) * sin(rotZ) * sin(rotX) + sin(rotY) * cos(rotX)) + leg[i].y * cos(rotY) * cos(rotZ) + traY;
    // 根据机体的旋转和平移计算每条腿的新Z坐标
    legIK[i].z = leg[i].x * (sin(rotY) * sin(rotZ) * cos(rotX) + cos(rotY) * sin(rotX)) + leg[i].z * (sin(rotY) * sin(rotZ) * sin(rotX) + cos(rotY) * cos(rotX)) - leg[i].y * sin(rotY) * cos(rotZ) + traZ;
  }
}

// 完整的腿部逆向运动学（基于https://oscarliang.com/inverse-kinematics-implementation-hexapod-robots/）
int gaitEngine::runLegIK()
{
  int mathError = 0; // 初始化数学错误标志为0，表示没有错误

  for (int i = 0; i < 6; i++) // 遍历六条腿
  {
    float deltaX = (legIK[i].x - coxaPos[i].x); // 计算腿的X方向上的位移
    float deltaY = (legIK[i].y - coxaPos[i].y); // 计算腿的Y方向上的位移
    float deltaZ = -(legIK[i].z - coxaPos[i].z); // 计算腿的Z方向上的位移，注意取反

    float legLength = sqrt(sq(deltaX) + sq(deltaY)); // 计算腿在XY平面上的长度
    float HF = sqrt(sq(legLength - coxa) + sq(deltaZ)); // 计算从coxa到脚尖的直线距离
    // 如果目标位置不可达，设置数学错误标志为1
    if ((HF > femur + tibia) || (HF < abs(femur - tibia)))
      mathError = 1;
    float AX1 = atan((legLength - coxa) / deltaZ); // 计算腿部与Z轴的夹角
    if (AX1 < 0)
      AX1 = PI + AX1; // 如果AX1为负，调整其值
    float AX2 = acos((sq(tibia) - sq(femur) - sq(HF)) / (-2 * femur * HF)); // 计算股骨与HF的夹角
    legAngle[i][1] = PI / 2 - (AX1 + AX2); // 计算股骨角度
    float BX1 = acos((sq(HF) - sq(tibia) - sq(femur)) / (-2 * femur * tibia)); // 计算胫骨角度
    legAngle[i][2] = PI / 2 - BX1;          // 胫骨角度
    legAngle[i][0] = atan(deltaY / deltaX); // 计算coxa角度

  }
  return mathError; // 返回数学错误标志
}
