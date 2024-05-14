#ifndef ZEROHEX_H // 如果没有定义ZEROHEX_H，则执行以下代码，这是为了防止重复包含同一个头文件
#define ZEROHEX_H // 定义ZEROHEX_H，标记这个头文件已经被包含
#include "Arduino.h" 

class vector { // 定义一个名为vector的类

	public: // 下面的成员变量和方法都是公开的，可以在类的外部访问
		float x; // 定义一个浮点数x，用于存储向量的x坐标
		float y; // 定义一个浮点数y，用于存储向量的y坐标
		float z; // 定义一个浮点数z，用于存储向量的z坐标
		
		float xs; // 定义一个浮点数xs，用于存储向量的起始x坐标
		float ys; // 定义一个浮点数ys，用于存储向量的起始y坐标
		float zs; // 定义一个浮点数zs，用于存储向量的起始z坐标
		
		float xt; // 定义一个浮点数xt，用于存储向量的目标x坐标
		float yt; // 定义一个浮点数yt，用于存储向量的目标y坐标
		float zt; // 定义一个浮点数zt，用于存储向量的目标z坐标
		int oldmillis; // 定义一个整数oldmillis，用于存储上一次更新向量时的时间戳
		
		vector(); // 声明一个构造函数，用于创建vector类的实例
		
		void reset(); // 声明一个方法reset，用于重置向量的所有坐标到初始状态
		
		void setto(float nx, float ny, float nz); // 声明一个方法setto，用于设置向量的新坐标
		
		void setto(vector v); // 声明一个重载的方法setto，用于将当前向量设置为另一个向量v的坐标
		
		bool equalegS(vector v); // 声明一个方法equalegS，用于判断当前向量的起始坐标是否与另一个向量v的坐标相等
		
		float move2D(float pps); // 声明一个方法move2D，用于根据给定的速度pps（每秒像素）移动向量，并返回剩余距离
};
class gaitEngine { // 定义一个名为gaitEngine的类，用于控制六足机器人的步态
	
	private:
		float gaitStepN = 0; // 步态步骤的编号，用于控制步态的进程
		
	public:
		byte gSeq[120][6]; // 步态序列，定义每个步骤中哪些腿应该移动
		byte legMoving[6]; // 记录每条腿是否正在移动的数组
		float gaitSpeed = 1.0; // 控制步态速度，通过调整gaitStepN的进程
		float legSpeed = 1.0; // 控制腿抬起函数的速度
		float stepHeight = 20; // 步高，控制腿抬起的高度
		float targetDeadZone = 3; // 允许腿的位置偏离中心的死区
		float walkX = 0; // 控制在X轴方向上行走的输入
		float walkY = 0; // 控制在Y轴方向上行走的输入
		float walkR = 0; // 控制旋转的输入
		float traX, traY, traZ; // 当前机体的平移量
		float rotX, rotY, rotZ; // 当前机体的旋转量
		// 六足机器人的常量
		int coxa  = 11.6; // 腿部的coxa（肩部）长度
		int femur = 44; // 腿部的femur（大腿）长度
		int tibia = 69; // 腿部的tibia（小腿）长度
		
		vector coxaPos[6]; // 机体的位置
		vector leg[6]; // 当前腿的位置（不包括机体的平移/旋转）
		vector legS[6]; // 腿的默认位置
		vector legT[6]; // 腿的目标位置
		vector legIK[6]; // 用于机体平移/旋转的腿的位置
		
		float legAngle[6][3]; // 记录每条腿的角度
		
		gaitEngine(); // gaitEngine的构造函数
		
		void gaitStep(); // 控制步态进程的方法
		
		// 机体的平移和旋转
		void runBodyIK(); 
		
		int runLegIK(); // 运行腿部逆向运动学的方法
		
		float fsin(float x); // 一个用于计算正弦值的函数，可能用于优化或特殊计算
		float fcos(float x); // 一个用于计算余弦值的函数，可能用于优化或特殊计算
		
};
#endif