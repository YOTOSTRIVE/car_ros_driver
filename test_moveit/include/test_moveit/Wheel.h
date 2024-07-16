#ifndef WHEEL_H
#define WHEEL_H

#include<iostream>
#include"controlcan.h"
#include "ros/ros.h"
using namespace std;
class Wheel{
	public:
		/**
		 *@brief wheel(): 
		 *初始化发布者，打开并获取设备信息
		 */
		Wheel(ros::NodeHandle nh);
		
		/*
		* @brief ahead(): 弃用
		* 
		*/
	    bool  ahead(int speed);

		/*
		* @brief step_back(): 弃用
		* 
		*/
	    bool  step_back(int distance);

		/*
		* @brief turn_around():
		* int R:单位cm；从转弯圆心到车轴中点的距离；|R|>30.5;R>0左转
		* 目前是将速度低的车轮速度设为150
		*/
	    bool  turn_around(int R);
		/*
		* @brief stopWithMode1()：通过切换工作模式停止；
		* 
		*/
	    bool  stopWithMode1();

		/*
		* @brief init():初始化波特率，收发模式，过滤
		*/
	    int init(VCI_INIT_CONFIG config);

		/*
		* @brief receiveAndPublish():创建接受can帧的进程；
		*/
	    int  receiveAndPublish();

		/*
		* @brief move():
		* speed>0前进；speed<0后退，speed单位是?
		* distance ?
		*/
	   	bool move( long long  speed,int distance);

		/*
		* @brief getPDO():设置定时上报，获取电机实际位置
		* 上报时间间隔
		*/
		bool getPDO_position();

		/*
		* @brief getPDO():设置定时上报，获取电机速度
		* 上报时间间隔
		*/
		bool getPDO_vel();

		bool stopPDO_vel();

		/*
		* @brief getHEXarray():将toHEX转化为一个大小为num的HEX数组；
		* mode = 0计算num
		* mode = 1计算速度
		* mode = 2计算加速度
		*/
		std::vector<unsigned char> intToHexArray(long long  num, int arrayLength,int mode);

		/*
		* @brief：setAcceleration():设置梯形加速度减速度
		* ratio: 1 默认加速度1，
		* 计算公式 
		*65536*10000/4000000
		*/
		bool setAcceleration(int ratio );



		/*
		* @brief :stopWithVel0():用速度0停止电机
		*/
		bool stopWithVel0();

		bool cleanSend();

		//void setWheel_vel_601();
		//void setWheel_vel_602();
		void decToRpm(VCI_CAN_OBJ& recv);


		void caculateDx_Dy_DthAndPub(double dt );

		/*
		* @brief:~wheel():关闭设备；
		*/
		~Wheel();

			

			double L = 0.61;
			double R= 0.21; 
		
	private:
		ros::NodeHandle nh_;
		double distanceBetweenWheels = 61.0;
		double radiusOfWheel = 21.0/2.0;
		ros::Publisher can_pub;
		int encoderResolution = 10000;
    	int rpmMultiplier = 512;
    	int divisor = 1875;
		double RPMratio = 15.0;//设置的RPM与实际的RPM比例
		double wheel_vel_601= 0;//601对应的是181  单位是cm/s
		double wheel_vel_602= 0;
		double pi = 3.1415;
		double now_x = 0.0;
		double now_y = 0.0;
		double now_th =0.0;

		

};

#endif

