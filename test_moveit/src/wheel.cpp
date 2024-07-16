#include "Wheel.h"
#include "controlcan.h"
#include<iostream>
#include<pthread.h>
#include<ros/ros.h>
#include <std_msgs/String.h>
#include <unistd.h> 
#include <sstream>
#include <iomanip>
#include <cmath>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/tf.h> 


using namespace std;
void *receive_func(void* arg)  
{	
	long long  count = 0;
	Wheel* instance = static_cast<Wheel*>(arg);

	int reclen=0;
	VCI_CAN_OBJ rec[30000];//接收缓存，设为3000为佳。
	int i,j;
    int ind=0;
	ros::Rate rate(20);
	usleep(100000);
	ros::Time current = ros::Time::now();
	ros::Time last = ros::Time::now();
	double T = 0.0;
	while(1)
	{	
		
		if((reclen=VCI_Receive(VCI_USBCAN2,0,ind,rec,30000,1000))>0)
		{
			for(j=0;j<reclen;j++)
			{	
				
				 current = ros::Time::now();
				double dt = (current-last).toSec();
				T+=dt;
				instance->decToRpm(rec[j]);
				instance->caculateDx_Dy_DthAndPub( dt);

				std::cout<<"T:   "<<T<<std::endl;
				std::cout<<count<<"  "<<std::endl;
				count++;
				printf("CAN%d RX ID:0x%08X", ind+1, rec[j].ID);
				if(rec[j].ExternFlag==0) printf(" Standard ");
				if(rec[j].ExternFlag==1) printf(" Extend   ");
				if(rec[j].RemoteFlag==0) printf(" Data   ");
				if(rec[j].RemoteFlag==1) printf(" Remote ");
				printf("DLC:0x%02X",rec[j].DataLen);
				printf(" data:0x");	
				for(i = 0; i < rec[j].DataLen; i++)
				{
					printf(" %02X", rec[j].Data[i]);
				}
				printf(" TimeStamp:0x%08X",rec[j].TimeStamp);
				printf("\n");
				last = current;
				rate.sleep();
        		}


		}
		
			
	}
	return nullptr;
}


Wheel::Wheel(ros::NodeHandle nh){
	this->nh_ = nh;
    	can_pub = nh.advertise<std_msgs::String>("can_messages", 10);
	int num=0;
	VCI_BOARD_INFO pInfo;//用来获取设备信息。
	VCI_BOARD_INFO pInfo1 [50];
	cout<<"open usb_can"<<endl;
	num=VCI_FindUsbDevice2(pInfo1);
	printf(">>USBCAN DEVICE NUM:");
	printf("%d", num);
	printf(" PCS");printf("\n");

		for(int i=0;i<num;i++)
		{
		printf("Device:");printf("%d", i);printf("\n");
                printf(">>Get VCI_ReadBoardInfo success!\n");
		
		printf(">>Serial_Num:%c", pInfo1[i].str_Serial_Num[0]);
		printf("%c", pInfo1[i].str_Serial_Num[1]);
		printf("%c", pInfo1[i].str_Serial_Num[2]);
		printf("%c", pInfo1[i].str_Serial_Num[3]);
		printf("%c", pInfo1[i].str_Serial_Num[4]);
		printf("%c", pInfo1[i].str_Serial_Num[5]);
		printf("%c", pInfo1[i].str_Serial_Num[6]);
		printf("%c", pInfo1[i].str_Serial_Num[7]);
		printf("%c", pInfo1[i].str_Serial_Num[8]);
		printf("%c", pInfo1[i].str_Serial_Num[9]);
		printf("%c", pInfo1[i].str_Serial_Num[10]);
		printf("%c", pInfo1[i].str_Serial_Num[11]);
		printf("%c", pInfo1[i].str_Serial_Num[12]);
		printf("%c", pInfo1[i].str_Serial_Num[13]);
		printf("%c", pInfo1[i].str_Serial_Num[14]);
		printf("%c", pInfo1[i].str_Serial_Num[15]);
		printf("%c", pInfo1[i].str_Serial_Num[16]);
		printf("%c", pInfo1[i].str_Serial_Num[17]);
		printf("%c", pInfo1[i].str_Serial_Num[18]);
		printf("%c", pInfo1[i].str_Serial_Num[19]);printf("\n");

		printf(">>hw_Type:%c", pInfo1[i].str_hw_Type[0]);
		printf("%c", pInfo1[i].str_hw_Type[1]);
		printf("%c", pInfo1[i].str_hw_Type[2]);
		printf("%c", pInfo1[i].str_hw_Type[3]);
		printf("%c", pInfo1[i].str_hw_Type[4]);
		printf("%c", pInfo1[i].str_hw_Type[5]);
		printf("%c", pInfo1[i].str_hw_Type[6]);
		printf("%c", pInfo1[i].str_hw_Type[7]);
		printf("%c", pInfo1[i].str_hw_Type[8]);
		printf("%c", pInfo1[i].str_hw_Type[9]);printf("\n");	

		printf(">>Firmware Version:V");
		printf("%x", (pInfo1[i].fw_Version&0xF00)>>8);
		printf(".");
		printf("%x", (pInfo1[i].fw_Version&0xF0)>>4);
		printf("%x", pInfo1[i].fw_Version&0xF);
		printf("\n");
	}
	printf(">>\n");
	printf(">>\n");
	printf(">>\n");
	if(VCI_OpenDevice(VCI_USBCAN2,0,0)==1)//打开设备
	{
		printf(">>open deivce success!\n");//打开设备成功
	}else
	{
		printf(">>open deivce error!\n");
		exit(1);
	}
	if(VCI_ReadBoardInfo(VCI_USBCAN2,0,&pInfo)==1)//读取设备序列号、版本等信息。
	{
                printf(">>Get VCI_ReadBoardInfo success!\n");
		
		printf(">>Serial_Num:%c", pInfo.str_Serial_Num[0]);
		printf("%c", pInfo.str_Serial_Num[1]);
		printf("%c", pInfo.str_Serial_Num[2]);
		printf("%c", pInfo.str_Serial_Num[3]);
		printf("%c", pInfo.str_Serial_Num[4]);
		printf("%c", pInfo.str_Serial_Num[5]);
		printf("%c", pInfo.str_Serial_Num[6]);
		printf("%c", pInfo.str_Serial_Num[7]);
		printf("%c", pInfo.str_Serial_Num[8]);
		printf("%c", pInfo.str_Serial_Num[9]);
		printf("%c", pInfo.str_Serial_Num[10]);
		printf("%c", pInfo.str_Serial_Num[11]);
		printf("%c", pInfo.str_Serial_Num[12]);
		printf("%c", pInfo.str_Serial_Num[13]);
		printf("%c", pInfo.str_Serial_Num[14]);
		printf("%c", pInfo.str_Serial_Num[15]);
		printf("%c", pInfo.str_Serial_Num[16]);
		printf("%c", pInfo.str_Serial_Num[17]);
		printf("%c", pInfo.str_Serial_Num[18]);
		printf("%c", pInfo.str_Serial_Num[19]);printf("\n");

		printf(">>hw_Type:%c", pInfo.str_hw_Type[0]);
		printf("%c", pInfo.str_hw_Type[1]);
		printf("%c", pInfo.str_hw_Type[2]);
		printf("%c", pInfo.str_hw_Type[3]);
		printf("%c", pInfo.str_hw_Type[4]);
		printf("%c", pInfo.str_hw_Type[5]);
		printf("%c", pInfo.str_hw_Type[6]);
		printf("%c", pInfo.str_hw_Type[7]);
		printf("%c", pInfo.str_hw_Type[8]);
		printf("%c", pInfo.str_hw_Type[9]);printf("\n");

		printf(">>Firmware Version:V");
		printf("%x", (pInfo.fw_Version&0xF00)>>8);
		printf(".");
		printf("%x", (pInfo.fw_Version&0xF0)>>4);
		printf("%x", pInfo.fw_Version&0xF);
		printf("\n");	
	}else
	{
		printf(">>Get VCI_ReadBoardInfo error!\n");
		exit(1);
	}
}
bool Wheel::getPDO_vel(){
	VCI_CAN_OBJ send1[14];
		send1[0].ID=0x601;
		send1[0].SendType=1;
		send1[0].RemoteFlag=0;
		send1[0].ExternFlag=0;
		send1[0].DataLen=8;
		send1[0].Data[0] = 0x2f;
		send1[0].Data[1] = 0x00;
		send1[0].Data[2] = 0x1a;
		send1[0].Data[3] = 00;
		send1[0].Data[4] = 0x01;
		send1[0].Data[5] = 00;
		send1[0].Data[6] = 00;
		send1[0].Data[7] = 00;

	        send1[1].ID=0x601;
                send1[1].SendType=1;
                send1[1].RemoteFlag=0;
                send1[1].ExternFlag=0;
                send1[1].DataLen=8;
                send1[1].Data[0] = 0x23;
                send1[1].Data[1] = 0x00;
                send1[1].Data[2] = 0x1a;
                send1[1].Data[3] = 0x01;
                send1[1].Data[4] = 0x20;
                send1[1].Data[5] = 00;
                send1[1].Data[6] = 0x6c;
                send1[1].Data[7] = 0x60;


		 send1[2].ID=0x601;
                send1[2].SendType=1;
                send1[2].RemoteFlag=0;
                send1[2].ExternFlag=0;
                send1[2].DataLen=8;
                send1[2].Data[0] = 0x23;
                send1[2].Data[1] = 0x00;
                send1[2].Data[2] = 0x18;
                send1[2].Data[3] = 0x01;
                send1[2].Data[4] = 0x81;
                send1[2].Data[5] = 0x01;
                send1[2].Data[6] = 0x00;
                send1[2].Data[7] = 00;

		 send1[3].ID=0x601;
                send1[3].SendType=1;
                send1[3].RemoteFlag=0;
                send1[3].ExternFlag=0;
                send1[3].DataLen=8;
                send1[3].Data[0] = 0x2f;
                send1[3].Data[1] = 0x00;
                send1[3].Data[2] = 0x18;
                send1[3].Data[3] = 0x02;
                send1[3].Data[4] = 0xff;
                send1[3].Data[5] = 0x00;
                send1[3].Data[6] = 0x00;
                send1[3].Data[7] = 0x00;
					 send1[4].ID=0x601;
                send1[4].SendType=1;
                send1[4].RemoteFlag=0;
                send1[4].ExternFlag=0;
                send1[4].DataLen=8;
                send1[4].Data[0] = 0x2b;
                send1[4].Data[1] = 0x00;
                send1[4].Data[2] = 0x18;
                send1[4].Data[3] = 0x03;
                send1[4].Data[4] = 0x00;
                send1[4].Data[5] = 0x00;
                send1[4].Data[6] = 0x00;
                send1[4].Data[7] = 0x00;
					 send1[5].ID=0x601;
                send1[5].SendType=1;
                send1[5].RemoteFlag=0;
                send1[5].ExternFlag=0;
                send1[5].DataLen=8;
                send1[5].Data[0] = 0x2b;
                send1[5].Data[1] = 0x00;
                send1[5].Data[2] = 0x18;
                send1[5].Data[3] = 0x05;
                send1[5].Data[4] = 0x2c;
                send1[5].Data[5] = 0x01;
                send1[5].Data[6] = 0x00;
                send1[5].Data[7] = 0x00;
					send1[6].ID=0x00;
                send1[6].SendType=1;
                send1[6].RemoteFlag=0;
                send1[6].ExternFlag=0;
                send1[6].DataLen=2;
                send1[6].Data[0] = 0x01;
                send1[6].Data[1] = 0x01;
                send1[6].Data[2] = 0x00;
                send1[6].Data[3] = 0x00;
                send1[6].Data[4] = 0x00;
                send1[6].Data[5] = 0x00;
                send1[6].Data[6] = 0x00;
                send1[6].Data[7] = 0x00;
					send1[7].ID=0x602;
				send1[7].SendType=1;
				send1[7].RemoteFlag=0;
				send1[7].ExternFlag=0;
				send1[7].DataLen=8;
				send1[7].Data[0] = 0x2f;
				send1[7].Data[1] = 0x00;
				send1[7].Data[2] = 0x1a;
				send1[7].Data[3] = 00;
				send1[7].Data[4] = 0x01;
				send1[7].Data[5] = 00;
				send1[7].Data[6] = 00;
				send1[7].Data[7] = 00;

	        		send1[8].ID=0x602;
                send1[8].SendType=1;
                send1[8].RemoteFlag=0;
                send1[8].ExternFlag=0;
                send1[8].DataLen=8;
                send1[8].Data[0] = 0x23;
                send1[8].Data[1] = 0x00;
                send1[8].Data[2] = 0x1a;
                send1[8].Data[3] = 0x01;
                send1[8].Data[4] = 0x20;
                send1[8].Data[5] = 00;
                send1[8].Data[6] = 0x6c;
                send1[8].Data[7] = 0x60;


		 			send1[9].ID=0x602;
                send1[9].SendType=1;
                send1[9].RemoteFlag=0;
                send1[9].ExternFlag=0;
                send1[9].DataLen=8;
                send1[9].Data[0] = 0x23;
                send1[9].Data[1] = 0x00;
                send1[9].Data[2] = 0x18;
                send1[9].Data[3] = 0x01;
                send1[9].Data[4] = 0x82;
                send1[9].Data[5] = 0x01;
                send1[9].Data[6] = 0x00;
                send1[9].Data[7] = 00;

		 			send1[10].ID=0x602;
                send1[10].SendType=1;
                send1[10].RemoteFlag=0;
                send1[10].ExternFlag=0;
                send1[10].DataLen=8;
                send1[10].Data[0] = 0x2f;
                send1[10].Data[1] = 0x00;
                send1[10].Data[2] = 0x18;
                send1[10].Data[3] = 0x02;
                send1[10].Data[4] = 0xff;
                send1[10].Data[5] = 0x00;
                send1[10].Data[6] = 0x00;
                send1[10].Data[7] = 0x00;
					 send1[11].ID=0x602;
                send1[11].SendType=1;
                send1[11].RemoteFlag=0;
                send1[11].ExternFlag=0;
                send1[11].DataLen=8;
                send1[11].Data[0] = 0x2b;
                send1[11].Data[1] = 0x00;
                send1[11].Data[2] = 0x18;
                send1[11].Data[3] = 0x03;
                send1[11].Data[4] = 0x00;
                send1[11].Data[5] = 0x00;
                send1[11].Data[6] = 0x00;
                send1[11].Data[7] = 0x00;
					send1[12].ID=0x602;
                send1[12].SendType=1;
                send1[12].RemoteFlag=0;
                send1[12].ExternFlag=0;
                send1[12].DataLen=8;
                send1[12].Data[0] = 0x2b;
                send1[12].Data[1] = 0x00;
                send1[12].Data[2] = 0x18;
                send1[12].Data[3] = 0x05;
                send1[12].Data[4] = 0x2c;
                send1[12].Data[5] = 0x01;
                send1[12].Data[6] = 0x00;
                send1[12].Data[7] = 0x00;
					send1[13].ID=0x00;
                send1[13].SendType=1;
                send1[13].RemoteFlag=0;
                send1[13].ExternFlag=0;
                send1[13].DataLen=2;
                send1[13].Data[0] = 0x01;
                send1[13].Data[1] = 0x02;
                send1[13].Data[2] = 0x00;
                send1[13].Data[3] = 0x00;
                send1[13].Data[4] = 0x00;
                send1[13].Data[5] = 0x00;
                send1[13].Data[6] = 0x00;
                send1[13].Data[7] = 0x00;

				int ret;
				int count=0;
				 for(int i = 0;i<14;i++ ){
  
                 ret = VCI_Transmit(VCI_USBCAN2, 0, 0, &send1[i],1);
                 if(ret==1) count++;
                 usleep(20000);

                }
				if(count==14)return true;
				return false;

	
}

void Wheel::decToRpm(VCI_CAN_OBJ& recv){
	//设置转速rpm 最高是3000，
	//如果rpm>5000，就认为转速为负

	long long result = 0;
	if(recv.ID == 0x00000181 || recv.ID == 0x00000182){
		result |= (static_cast<long long>(recv.Data[0]));
		result |= (static_cast<long long>(recv.Data[1]) << 8);
		result |= (static_cast<long long>(recv.Data[2]) << 16);
		result |= (static_cast<long long>(recv.Data[3]) << 24);
		std::cout<<std::hex<<recv.Data[4]<< std::endl; 
		std::cout << "Value of 4: " << "0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(recv.Data[0]) << std::endl;
		std::cout << "Value of 4: " << "0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(recv.Data[1]) << std::endl;
		std::cout << "Value of 4: " << "0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(recv.Data[2]) << std::endl;
		std::cout << "Value of 4: " << "0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(recv.Data[3]) << std::endl;
	

		

		long long rpm = result *1875 / 512 /10000 ;
		if(rpm>1500){
			result |= (static_cast<long long>(0xff) << 32);
			result |= (static_cast<long long>(0xff) << 40);
			result |= (static_cast<long long>(0xff) << 48);
			result |= (static_cast<long long>(0xff) << 56);
			rpm = result *1875 / 512 /10000;
		}
		std::cout <<"result"<<result << std::endl;
		std::cout <<"rpm"<<rpm << std::endl;
		if(recv.ID==0x181){
			this->wheel_vel_601 = rpm *2 * this->pi* this->radiusOfWheel /this->RPMratio /60.0;
			std::cout<<"wheel_vel_601:"<<this->wheel_vel_601<<std::endl;
		}
		if(recv.ID==0x182){
			this->wheel_vel_602 = rpm *2 * this->pi* this->radiusOfWheel /this->RPMratio /60.0;
			std::cout<<"wheel_vel_602:"<<this->wheel_vel_602<<std::endl;
		}

	}
	






}
bool Wheel::setAcceleration(int ratio = 1){

	VCI_CAN_OBJ send1[4];
	send1[0].ID=0x601;
		send1[0].SendType=1;
		send1[0].RemoteFlag=0;
		send1[0].ExternFlag=0;
		send1[0].DataLen=8;
		send1[0].Data[0] = 0x23;
		send1[0].Data[1] = 0x84;
		send1[0].Data[2] = 0x60;
		send1[0].Data[3] = 00;
		send1[0].Data[4] = 0xa3;
		send1[0].Data[5] = 00;
		send1[0].Data[6] = 00;
		send1[0].Data[7] = 00;

	        send1[1].ID=0x601;
                send1[1].SendType=1;
                send1[1].RemoteFlag=0;
                send1[1].ExternFlag=0;
                send1[1].DataLen=8;
                send1[1].Data[0] = 0x23;
                send1[1].Data[1] = 0x83;
                send1[1].Data[2] = 0x60;
                send1[1].Data[3] = 0x00;
                send1[1].Data[4] = 0xa3;
                send1[1].Data[5] = 00;
                send1[1].Data[6] = 00;
                send1[1].Data[7] = 00;

		 send1[2].ID=0x602;
                send1[2].SendType=1;
                send1[2].RemoteFlag=0;
                send1[2].ExternFlag=0;
                send1[2].DataLen=8;
                send1[2].Data[0] = 0x23;
                send1[2].Data[1] = 0x83;
                send1[2].Data[2] = 0x60;
                send1[2].Data[3] = 0x00;
                send1[2].Data[4] = 0xa3;
                send1[2].Data[5] = 00;
                send1[2].Data[6] = 00;
                send1[2].Data[7] = 00;

		 send1[3].ID=0x602;
                send1[3].SendType=1;
                send1[3].RemoteFlag=0;
                send1[3].ExternFlag=0;
                send1[3].DataLen=8;
                send1[3].Data[0] = 0x23;
                send1[3].Data[1] = 0x84;
                send1[3].Data[2] = 0x60;
                send1[3].Data[3] = 0x00;
                send1[3].Data[4] = 0xa3;
                send1[3].Data[5] = 0x00;
                send1[3].Data[6] = 0x00;
                send1[3].Data[7] = 0x00;

		std::vector<unsigned char> hexArray = this->intToHexArray(ratio,4,2);
		for(int i = 0;i<4;i++){
			for(int j = 4,k = 3;j<8&&k>-1;j++,k--){
				send1[i].Data[j] = hexArray[k];
			}
		}
		for(int i = 0;i<4;i++ ){
            VCI_Transmit(VCI_USBCAN2, 0, 0, &send1[i],1);
            usleep(20000);
           }


	return 0;


}
bool Wheel::stopWithVel0(){
	VCI_CAN_OBJ send1[2];
	send1[0].ID=0x601;
		send1[0].SendType=1;
		send1[0].RemoteFlag=0;
		send1[0].ExternFlag=0;
		send1[0].DataLen=8;
		send1[0].Data[0] = 0x23;
		send1[0].Data[1] = 0xff;
		send1[0].Data[2] = 0x60;
		send1[0].Data[3] = 00;
		send1[0].Data[4] = 0x00;
		send1[0].Data[5] = 00;
		send1[0].Data[6] = 00;
		send1[0].Data[7] = 00;

	send1[1].ID=0x602;
		send1[1].SendType=1;
		send1[1].RemoteFlag=0;
		send1[1].ExternFlag=0;
		send1[1].DataLen=8;
		send1[1].Data[0] = 0x23;
		send1[1].Data[1] = 0xff;
		send1[1].Data[2] = 0x60;
		send1[1].Data[3] = 0x00;
		send1[1].Data[4] = 0x00;
		send1[1].Data[5] = 00;
		send1[1].Data[6] = 0x00;
		send1[1].Data[7] = 0x00;

	for(int i = 0;i<2;i++){
		VCI_Transmit(VCI_USBCAN2, 0, 0, &send1[i],1);
		usleep(20000);
	}

	return 0;






}
bool Wheel::getPDO_position(){

	VCI_CAN_OBJ send1[14];
		send1[0].ID=0x601;
		send1[0].SendType=1;
		send1[0].RemoteFlag=0;
		send1[0].ExternFlag=0;
		send1[0].DataLen=8;
		send1[0].Data[0] = 0x2f;
		send1[0].Data[1] = 0x00;
		send1[0].Data[2] = 0x1a;
		send1[0].Data[3] = 00;
		send1[0].Data[4] = 0x01;
		send1[0].Data[5] = 00;
		send1[0].Data[6] = 00;
		send1[0].Data[7] = 00;

	        send1[1].ID=0x601;
                send1[1].SendType=1;
                send1[1].RemoteFlag=0;
                send1[1].ExternFlag=0;
                send1[1].DataLen=8;
                send1[1].Data[0] = 0x23;
                send1[1].Data[1] = 0x00;
                send1[1].Data[2] = 0x1a;
                send1[1].Data[3] = 0x01;
                send1[1].Data[4] = 0x20;
                send1[1].Data[5] = 00;
                send1[1].Data[6] = 0x63;
                send1[1].Data[7] = 0x60;


		 send1[2].ID=0x601;
                send1[2].SendType=1;
                send1[2].RemoteFlag=0;
                send1[2].ExternFlag=0;
                send1[2].DataLen=8;
                send1[2].Data[0] = 0x23;
                send1[2].Data[1] = 0x00;
                send1[2].Data[2] = 0x18;
                send1[2].Data[3] = 0x01;
                send1[2].Data[4] = 0x81;
                send1[2].Data[5] = 0x01;
                send1[2].Data[6] = 0x00;
                send1[2].Data[7] = 00;

		 send1[3].ID=0x601;
                send1[3].SendType=1;
                send1[3].RemoteFlag=0;
                send1[3].ExternFlag=0;
                send1[3].DataLen=8;
                send1[3].Data[0] = 0x2f;
                send1[3].Data[1] = 0x00;
                send1[3].Data[2] = 0x18;
                send1[3].Data[3] = 0x02;
                send1[3].Data[4] = 0xff;
                send1[3].Data[5] = 0x00;
                send1[3].Data[6] = 0x00;
                send1[3].Data[7] = 0x00;
					 send1[4].ID=0x601;
                send1[4].SendType=1;
                send1[4].RemoteFlag=0;
                send1[4].ExternFlag=0;
                send1[4].DataLen=8;
                send1[4].Data[0] = 0x2b;
                send1[4].Data[1] = 0x00;
                send1[4].Data[2] = 0x18;
                send1[4].Data[3] = 0x03;
                send1[4].Data[4] = 0x00;
                send1[4].Data[5] = 0x00;
                send1[4].Data[6] = 0x00;
                send1[4].Data[7] = 0x00;
					 send1[5].ID=0x601;
                send1[5].SendType=1;
                send1[5].RemoteFlag=0;
                send1[5].ExternFlag=0;
                send1[5].DataLen=8;
                send1[5].Data[0] = 0x2b;
                send1[5].Data[1] = 0x00;
                send1[5].Data[2] = 0x18;
                send1[5].Data[3] = 0x05;
                send1[5].Data[4] = 0xc0;
                send1[5].Data[5] = 0x12;
                send1[5].Data[6] = 0x00;
                send1[5].Data[7] = 0x00;
					send1[6].ID=0x00;
                send1[6].SendType=1;
                send1[6].RemoteFlag=0;
                send1[6].ExternFlag=0;
                send1[6].DataLen=2;
                send1[6].Data[0] = 0x01;
                send1[6].Data[1] = 0x01;
                send1[6].Data[2] = 0x00;
                send1[6].Data[3] = 0x00;
                send1[6].Data[4] = 0x00;
                send1[6].Data[5] = 0x00;
                send1[6].Data[6] = 0x00;
                send1[6].Data[7] = 0x00;
					send1[7].ID=0x602;
				send1[7].SendType=1;
				send1[7].RemoteFlag=0;
				send1[7].ExternFlag=0;
				send1[7].DataLen=8;
				send1[7].Data[0] = 0x2f;
				send1[7].Data[1] = 0x00;
				send1[7].Data[2] = 0x1a;
				send1[7].Data[3] = 00;
				send1[7].Data[4] = 0x01;
				send1[7].Data[5] = 00;
				send1[7].Data[6] = 00;
				send1[7].Data[7] = 00;

	        		send1[8].ID=0x602;
                send1[8].SendType=1;
                send1[8].RemoteFlag=0;
                send1[8].ExternFlag=0;
                send1[8].DataLen=8;
                send1[8].Data[0] = 0x23;
                send1[8].Data[1] = 0x00;
                send1[8].Data[2] = 0x1a;
                send1[8].Data[3] = 0x01;
                send1[8].Data[4] = 0x20;
                send1[8].Data[5] = 00;
                send1[8].Data[6] = 0x63;
                send1[8].Data[7] = 0x60;


		 			send1[9].ID=0x602;
                send1[9].SendType=1;
                send1[9].RemoteFlag=0;
                send1[9].ExternFlag=0;
                send1[9].DataLen=8;
                send1[9].Data[0] = 0x23;
                send1[9].Data[1] = 0x00;
                send1[9].Data[2] = 0x18;
                send1[9].Data[3] = 0x01;
                send1[9].Data[4] = 0x82;
                send1[9].Data[5] = 0x01;
                send1[9].Data[6] = 0x00;
                send1[9].Data[7] = 00;

		 			send1[10].ID=0x602;
                send1[10].SendType=1;
                send1[10].RemoteFlag=0;
                send1[10].ExternFlag=0;
                send1[10].DataLen=8;
                send1[10].Data[0] = 0x2f;
                send1[10].Data[1] = 0x00;
                send1[10].Data[2] = 0x18;
                send1[10].Data[3] = 0x02;
                send1[10].Data[4] = 0xff;
                send1[10].Data[5] = 0x00;
                send1[10].Data[6] = 0x00;
                send1[10].Data[7] = 0x00;
					 send1[11].ID=0x602;
                send1[11].SendType=1;
                send1[11].RemoteFlag=0;
                send1[11].ExternFlag=0;
                send1[11].DataLen=8;
                send1[11].Data[0] = 0x2b;
                send1[11].Data[1] = 0x00;
                send1[11].Data[2] = 0x18;
                send1[11].Data[3] = 0x03;
                send1[11].Data[4] = 0x00;
                send1[11].Data[5] = 0x00;
                send1[11].Data[6] = 0x00;
                send1[11].Data[7] = 0x00;
					send1[12].ID=0x602;
                send1[12].SendType=1;
                send1[12].RemoteFlag=0;
                send1[12].ExternFlag=0;
                send1[12].DataLen=8;
                send1[12].Data[0] = 0x2b;
                send1[12].Data[1] = 0x00;
                send1[12].Data[2] = 0x18;
                send1[12].Data[3] = 0x05;
                send1[12].Data[4] = 0xc0;
                send1[12].Data[5] = 0x12;
                send1[12].Data[6] = 0x00;
                send1[12].Data[7] = 0x00;
					send1[13].ID=0x00;
                send1[13].SendType=1;
                send1[13].RemoteFlag=0;
                send1[13].ExternFlag=0;
                send1[13].DataLen=2;
                send1[13].Data[0] = 0x01;
                send1[13].Data[1] = 0x02;
                send1[13].Data[2] = 0x00;
                send1[13].Data[3] = 0x00;
                send1[13].Data[4] = 0x00;
                send1[13].Data[5] = 0x00;
                send1[13].Data[6] = 0x00;
                send1[13].Data[7] = 0x00;

				int ret;
				int count=0;
				 for(int i = 0;i<14;i++ ){
  
                 ret = VCI_Transmit(VCI_USBCAN2, 0, 0, &send1[i],1);
                 if(ret==1) count++;
                 usleep(20000);

                }
				if(count==14)return true;
				return false;

}
void Wheel::caculateDx_Dy_DthAndPub(double dt){

	

	 double T = 0.0;
	 double Dth =  (fabs(this->wheel_vel_602) - fabs(this->wheel_vel_601))/this->distanceBetweenWheels;
	 double Dx = 0.0;
	 double Dy = 0.0;
	 double radius = radius =((this->wheel_vel_601 * dt)+(this->wheel_vel_602*dt))/(2*Dth);
	
	 if(std::isnan(radius) || radius > 100000000 || radius < -100000000 || fabs(Dth)<0.1){
		 Dx = this->wheel_vel_601 * dt;
		 Dy = 0.0;
		 Dth = 0.0;
		 std::cout << "ZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZ " <<std::endl;
	 }else{
		std::cout << "JJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJ " <<std::endl;
		 Dx = radius * sin(Dth);
		 Dy = radius * (1-cos(Dth));
	 }
	
	std::cout<<"dt:"<<dt<<std::endl;
	 std::cout<<"wheel_vel_601"<<this->wheel_vel_601<<std::endl;
	 std::cout<<"wheel_vel_602"<<this->wheel_vel_602<<std::endl;
	 	std::cout<<"radius: "<<radius<<std::endl;
		std::cout << "Dx: " << Dx << std::endl;
		std::cout << "Dy: " <<Dy << std::endl;
		std::cout << "Dth: " <<Dth<< std::endl;
	this->now_x +=Dx;
	this->now_y +=Dy;
	this->now_th +=Dth;
		std::cout<<"now_x:"<<this->now_x<<std::endl;
		std::cout<<"now_y:"<<this->now_y<<std::endl;
		std::cout<<"now_th:"<<this->now_th<<std::endl;



	 static ros::Publisher odom_pub = this->nh_.advertise<nav_msgs::Odometry>("odom", 1);
     static tf::TransformBroadcaster odom_broadcaster;
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(this->now_th);
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.stamp = ros::Time::now();
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_link_11";
	odom_trans.transform.translation.x = this->now_x;
	odom_trans.transform.translation.y = this->now_y;
	odom_trans.transform.translation.z = 0.0;
	odom_trans.transform.rotation = odom_quat;
	odom_broadcaster.sendTransform(odom_trans);
	nav_msgs::Odometry odom;
        odom.header.stamp = ros::Time::now();
        odom.header.frame_id = "odom";
		//还需要发布nav_msgs/Odometry消息，以便导航包可以从中获取速度信息。
	//将消息的header设置为current_time和“odom”坐标系。
        //set the position
        odom.pose.pose.position.x = this->now_x;
        odom.pose.pose.position.y = this->now_y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;
 
        //set the velocity
        odom.child_frame_id = "base_link_11";
        odom.twist.twist.linear.x = 0;
        odom.twist.twist.linear.y = 0;
        odom.twist.twist.angular.z = 0;
	//这将使用里程数据填充消息，并发送出去。
	//我们将消息的child_frame_id设置为“base_link”坐标系，
	//因为我们要发送速度信息到这个坐标系。
 
        //publish the message
        odom_pub.publish(odom);


}
bool Wheel::ahead(int speed){
       	VCI_CAN_OBJ send1[4];
		send1[0].ID=0x601;
		send1[0].SendType=1;
		send1[0].RemoteFlag=0;
		send1[0].ExternFlag=0;
		send1[0].DataLen=8;
		send1[0].Data[0] = 0x2f;
		send1[0].Data[1] = 0x60;
		send1[0].Data[2] = 0x60;
		send1[0].Data[3] = 00;
		send1[0].Data[4] = 0x03;
		send1[0].Data[5] = 00;
		send1[0].Data[6] = 00;
		send1[0].Data[7] = 00;

	        send1[1].ID=0x602;
                send1[1].SendType=1;
                send1[1].RemoteFlag=0;
                send1[1].ExternFlag=0;
                send1[1].DataLen=8;
                send1[1].Data[0] = 0x2f;
                send1[1].Data[1] = 0x60;
                send1[1].Data[2] = 0x60;
                send1[1].Data[3] = 00;
                send1[1].Data[4] = 0x03;
                send1[1].Data[5] = 00;
                send1[1].Data[6] = 00;
                send1[1].Data[7] = 00;


		 send1[2].ID=0x601;
                send1[2].SendType=1;
                send1[2].RemoteFlag=0;
                send1[2].ExternFlag=0;
                send1[2].DataLen=8;
                send1[2].Data[0] = 0x23;
                send1[2].Data[1] = 0xff;
                send1[2].Data[2] = 0x60;
                send1[2].Data[3] = 00;
                send1[2].Data[4] = 0x00;
                send1[2].Data[5] = 0x40;
                send1[2].Data[6] = 0x06;
                send1[2].Data[7] = 00;

		 send1[3].ID=0x602;
                send1[3].SendType=1;
                send1[3].RemoteFlag=0;
                send1[3].ExternFlag=0;
                send1[3].DataLen=8;
                send1[3].Data[0] = 0x23;
                send1[3].Data[1] = 0xff;
                send1[3].Data[2] = 0x60;
                send1[3].Data[3] = 00;
                send1[3].Data[4] = 00;
                send1[3].Data[5] = 0xc0;
                send1[3].Data[6] = 0xf9;
                send1[3].Data[7] = 0xff;
              
                int ret;
                
                for(int i = 0;i<4;i++ ){
  
                 ret = VCI_Transmit(VCI_USBCAN2, 0, 0, &send1[i],1);
                 
                 usleep(1000);
                }

	return 0;
}
bool Wheel::move(long long speed,int distance = 0){
	if(speed>2000){
		cout<<"Speed must less than 3000"<<endl;
		return 0;

	}
    int encoderResolution = 10000;
    int rpmMultiplier = 512;
    int divisor = 1875;

    long long decValue1 = ( speed* 10000 * 512) / 1875;
    long long decValue2= ( (0-speed) * 10000 * 512) / 1875;
	 decValue1 = decValue1 & 0xFFFFFFFF;
    decValue2=  decValue2 & 0xFFFFFFFF;
	std::cout<<std::dec<<decValue1<<std::endl;
	std::cout<<std::dec<<decValue2<<std::endl;
    std::ostringstream ss1,ss2;
    ss1 << "0x" << std::setfill('0') << std::setw(8) << std::hex << decValue1;
    ss2 << "0x" << std::setfill('0') << std::setw(8) << std::hex << decValue2;
    std::string result1 = ss1.str();
	std::string result2 = ss2.str();
	std::cout << "ss1: " << ss1.str() << std::endl;
	std::cout << "ss2: " << ss2.str() << std::endl;
    // 将字符串 result 存储在四个 unsigned char 数组中
    unsigned char hex_array1[4],hex_array2[4];
    for (int j = 0; j < 4; ++j) {
        std::string byte_str1 = result1.substr(2 + j * 2, 2);
        std::string byte_str2 = result2.substr(2 + j * 2, 2);
        hex_array1[j] = static_cast<unsigned char>(std::stoi(byte_str1, nullptr, 16));
        hex_array2[j] = static_cast<unsigned char>(std::stoi(byte_str2, nullptr, 16));
        std::cout << "hex_array1[" << j << "] = 0x" << std::hex << (int)hex_array1[j] << std::endl;
        std::cout << "hex_array2[" << j << "] = 0x" << std::hex << (int)hex_array2[j] << std::endl;
        
    }
    		VCI_CAN_OBJ send1[4];
		send1[0].ID=0x601;
		send1[0].SendType=1;
		send1[0].RemoteFlag=0;
		send1[0].ExternFlag=0;
		send1[0].DataLen=8;
		send1[0].Data[0] = 0x2f;
		send1[0].Data[1] = 0x60;
		send1[0].Data[2] = 0x60;
		send1[0].Data[3] = 00;
		send1[0].Data[4] = 0x03;
		send1[0].Data[5] = 00;
		send1[0].Data[6] = 00;
		send1[0].Data[7] = 00;

	        send1[1].ID=0x602;
                send1[1].SendType=1;
                send1[1].RemoteFlag=0;
                send1[1].ExternFlag=0;
                send1[1].DataLen=8;
                send1[1].Data[0] = 0x2f;
                send1[1].Data[1] = 0x60;
                send1[1].Data[2] = 0x60;
                send1[1].Data[3] = 00;
                send1[1].Data[4] = 0x03;
                send1[1].Data[5] = 00;
                send1[1].Data[6] = 00;
                send1[1].Data[7] = 00;


		 send1[2].ID=0x602;
                send1[2].SendType=1;
                send1[2].RemoteFlag=0;
                send1[2].ExternFlag=0;
                send1[2].DataLen=8;
                send1[2].Data[0] = 0x23;
                send1[2].Data[1] = 0xff;
                send1[2].Data[2] = 0x60;
                send1[2].Data[3] = 00;
                send1[2].Data[4] = 0x00;
                send1[2].Data[5] = 0x40;
                send1[2].Data[6] = 0x06;
                send1[2].Data[7] = 00;

		 send1[3].ID=0x601;
                send1[3].SendType=1;
                send1[3].RemoteFlag=0;
                send1[3].ExternFlag=0;
                send1[3].DataLen=8;
                send1[3].Data[0] = 0x23;
                send1[3].Data[1] = 0xff;
                send1[3].Data[2] = 0x60;
                send1[3].Data[3] = 00;
                send1[3].Data[4] = 00;
                send1[3].Data[5] = 0xc0;
                send1[3].Data[6] = 0xf9;
                send1[3].Data[7] = 0xff;

    if(speed!=0){
    	for(int j = 4,k = 3;k>-1&&j<8;j++,k--){
    		send1[2].Data[j] = hex_array1[k];
    		send1[3].Data[j] = hex_array2[k];
    	}
    }
    for(int i = 0;i<4;i++ ){     	
        VCI_Transmit(VCI_USBCAN2, 0, 0, &send1[i],1);
        usleep(20000);
    }
	return 0;
}

std::vector<unsigned char> Wheel::intToHexArray(long long  num, int arrayLength,int mode = 0){

	int decValue = 0;
	if(mode == 2){
		decValue = (num * this->encoderResolution * 65536) / 4000000;
		
	}
	if(mode == 1){
		decValue = (num * this->encoderResolution * this->rpmMultiplier) / this->divisor;

	}
	if(mode == 0){
		decValue = num;
	}
	decValue = decValue & 0xFFFFFFFF;

    
    std::ostringstream ss;
    ss << "0x" << std::setfill('0') << std::setw(arrayLength * 2) << std::hex << decValue;
    std::string result = ss.str();

    std::vector<unsigned char> hexArray(arrayLength, 0);

    for (int j = 0; j < arrayLength; ++j) {
        std::string byteStr = result.substr(2 + j * 2, 2);
        hexArray[j] = static_cast<unsigned char>(std::stoi(byteStr, nullptr, 16));
    }

    return hexArray;
}


bool Wheel::turn_around(int R){
	//统一单位cm 
	//1左转 -1右
	double wheel_L = this->L * 100;
	long long  speed_low = 150;
	double ratio = 1- wheel_L / (R+(wheel_L/2));
	long long  speed_high = 0;
	//R应该绝对值大于30.5   
	if(R>0){
		speed_high =speed_low / ratio;
	}else{
		speed_high =speed_low * ratio;
	}
	

	VCI_CAN_OBJ send1[4];
			send1[0].ID=0x601;
		send1[0].SendType=1;
		send1[0].RemoteFlag=0;
		send1[0].ExternFlag=0;
		send1[0].DataLen=8;
		send1[0].Data[0] = 0x2f;
		send1[0].Data[1] = 0x60;
		send1[0].Data[2] = 0x60;
		send1[0].Data[3] = 00;
		send1[0].Data[4] = 0x03;
		send1[0].Data[5] = 00;
		send1[0].Data[6] = 00;
		send1[0].Data[7] = 00;

	        send1[1].ID=0x602;
                send1[1].SendType=1;
                send1[1].RemoteFlag=0;
                send1[1].ExternFlag=0;
                send1[1].DataLen=8;
                send1[1].Data[0] = 0x2f;
                send1[1].Data[1] = 0x60;
                send1[1].Data[2] = 0x60;
                send1[1].Data[3] = 00;
                send1[1].Data[4] = 0x03;
                send1[1].Data[5] = 00;
                send1[1].Data[6] = 00;
                send1[1].Data[7] = 00;


		 send1[2].ID=0x602;
                send1[2].SendType=1;
                send1[2].RemoteFlag=0;
                send1[2].ExternFlag=0;
                send1[2].DataLen=8;
                send1[2].Data[0] = 0x23;
                send1[2].Data[1] = 0xff;
                send1[2].Data[2] = 0x60;
                send1[2].Data[3] = 00;
                send1[2].Data[4] = 0x00;
                send1[2].Data[5] = 0x40;
                send1[2].Data[6] = 0x06;
                send1[2].Data[7] = 00;

		 send1[3].ID=0x601;
                send1[3].SendType=1;
                send1[3].RemoteFlag=0;
                send1[3].ExternFlag=0;
                send1[3].DataLen=8;
                send1[3].Data[0] = 0x23;
                send1[3].Data[1] = 0xff;
                send1[3].Data[2] = 0x60;
                send1[3].Data[3] = 00;
                send1[3].Data[4] = 00;
                send1[3].Data[5] = 0xc0;
                send1[3].Data[6] = 0xf9;
                send1[3].Data[7] = 0xff;

	if(R>0){
		 	int encoderResolution = 10000;
    		int rpmMultiplier = 512;
    		int divisor = 1875;

    		long long  decValue1 = ( speed_low* encoderResolution * rpmMultiplier) / divisor;
   			long long  decValue2= ( (0-speed_high) * encoderResolution * rpmMultiplier) / divisor;
			 decValue1 = decValue1 & 0xFFFFFFFF;
    		decValue2=  decValue2 & 0xFFFFFFFF;
		    std::ostringstream ss1,ss2;
			ss1 << "0x" << std::setfill('0') << std::setw(8) << std::hex << decValue1 ;
			ss2 << "0x" << std::setfill('0') << std::setw(8) << std::hex << decValue2;
			std::string result1 = ss1.str();
			std::string result2 = ss2.str();
			// 将字符串 result 存储在四个 unsigned char 数组中
			unsigned char hex_array1[4],hex_array2[4];
			for (int j = 0; j < 4; ++j) {
				std::string byte_str1 = result1.substr(2 + j * 2, 2);
				std::string byte_str2 = result2.substr(2 + j * 2, 2);
				hex_array1[j] = static_cast<unsigned char>(std::stoi(byte_str1, nullptr, 16));
				hex_array2[j] = static_cast<unsigned char>(std::stoi(byte_str2, nullptr, 16));
				
			}
			for(int j = 4,k = 3;k>-1&&j<8;j++,k--){
    			send1[2].Data[j] = hex_array2[k];
    			send1[3].Data[j] = hex_array1[k];
    	}
		      int ret;
                for(int i = 0;i<4;i++ ){
                 	ret = VCI_Transmit(VCI_USBCAN2, 0, 0, &send1[i],1);              
                 	usleep(20000);
                }
	



	}
		if(R<0){
			 long long  encoderResolution = 10000;
    		long long  rpmMultiplier = 512;
    		long long  divisor = 1875;

    		long long  decValue1 = ( (0-speed_low)* encoderResolution * rpmMultiplier) / divisor;
   			 long long  decValue2= ( (speed_high) * encoderResolution * rpmMultiplier) / divisor;
			  decValue1 = decValue1 & 0xFFFFFFFF;
   			 decValue2=  decValue2 & 0xFFFFFFFF;
		    std::ostringstream ss1,ss2;
			ss1 << "0x" << std::setfill('0') << std::setw(8) << std::hex << decValue1;
			ss2 << "0x" << std::setfill('0') << std::setw(8) << std::hex << decValue2;
			std::string result1 = ss1.str();
			std::string result2 = ss2.str();
			// 将字符串 result 存储在四个 unsigned char 数组中
			unsigned char hex_array1[4],hex_array2[4];
			for (int j = 0; j < 4; ++j) {
				std::string byte_str1 = result1.substr(2 + j * 2, 2);
				std::string byte_str2 = result2.substr(2 + j * 2, 2);
				hex_array1[j] = static_cast<unsigned char>(std::stoi(byte_str1, nullptr, 16));
				hex_array2[j] = static_cast<unsigned char>(std::stoi(byte_str2, nullptr, 16));
				std::cout << "hex_array1[" << j << "] = 0x" << std::hex << (int)hex_array1[j] << std::endl;
				std::cout << "hex_array2[" << j << "] = 0x" << std::hex << (int)hex_array2[j] << std::endl;
				
			}
			for(int j = 4,k = 3;k>-1&&j<8;j++,k--){
    			send1[2].Data[j] = hex_array1[k];
    			send1[3].Data[j] = hex_array2[k];
    	}
		      int ret;
                for(int i = 0;i<4;i++ ){
                 	ret = VCI_Transmit(VCI_USBCAN2, 0, 0, &send1[i],1);              
                 	usleep(20000);
                }
	

	}
          

	return 0;}


int Wheel::init(VCI_INIT_CONFIG xx){

	VCI_INIT_CONFIG config = xx ;
	//config.AccCode=0;
	//config.AccMask=0xFFFFFFFF;
	//config.Filter=1;//接收所有帧
	//config.Timing0=0x09;/*波特率500 Kbps  0x09  0x1C*/
	//config.Timing1=0x1C;
	//config.Mode=0;//正常模式		
	
	if(VCI_InitCAN(VCI_USBCAN2,0,0,&config)!=1)
	{
		printf(">>Init CAN1 error\n");
		VCI_CloseDevice(VCI_USBCAN2,0);
	}

	if(VCI_StartCAN(VCI_USBCAN2,0,0)!=1)
	{
		printf(">>Start CAN1 error\n");
		VCI_CloseDevice(VCI_USBCAN2,0);

	}

	if(VCI_InitCAN(VCI_USBCAN2,0,1,&config)!=1)
	{
		printf(">>Init can2 error\n");
		VCI_CloseDevice(VCI_USBCAN2,0);

	}
	if(VCI_StartCAN(VCI_USBCAN2,0,1)!=1)
	{
		printf(">>Start can2 error\n");
		VCI_CloseDevice(VCI_USBCAN2,0);

	}
	return 1;
}
int Wheel::receiveAndPublish(){
	pthread_t pthreadpid;
	if(pthread_create(&pthreadpid,NULL,receive_func,this)==0){
		return 1;
		}else return 0;
	
}

bool Wheel::stopWithMode1(){

		VCI_CAN_OBJ send1[2];
		send1[0].ID=0x601;
		send1[0].SendType=1;
		send1[0].RemoteFlag=0;
		send1[0].ExternFlag=0;
		send1[0].DataLen=8;
		send1[0].Data[0] = 0x2f;
		send1[0].Data[1] = 0x60;
		send1[0].Data[2] = 0x60;
		send1[0].Data[3] = 00;
		send1[0].Data[4] = 0x01;
		send1[0].Data[5] = 00;
		send1[0].Data[6] = 00;
		send1[0].Data[7] = 00;
		
		
		send1[1].ID=0x602;
		send1[1].SendType=1;
		send1[1].RemoteFlag=0;
		send1[1].ExternFlag=0;
		send1[1].DataLen=8;
		send1[1].Data[0] = 0x2f;
		send1[1].Data[1] = 0x60;
		send1[1].Data[2] = 0x60;
		send1[1].Data[3] = 00;
		send1[1].Data[4] = 0x01;
		send1[1].Data[5] = 00;
		send1[1].Data[6] = 00;
		send1[1].Data[7] = 00;
		int ret;
 
		for(int i = 0; i < 2; i++) {
    		ret = VCI_Transmit(VCI_USBCAN2, 0, 0, &send1[i], 1);
    		usleep(20000);	
		}
		return 1;
   
}
bool Wheel::cleanSend(){
	VCI_ClearBuffer(4,0,0);
}
bool Wheel::stopPDO_vel(){
	VCI_CAN_OBJ send1[2];
		send1[0].ID=0x00;
		send1[0].SendType=1;
		send1[0].RemoteFlag=0;
		send1[0].ExternFlag=0;
		send1[0].DataLen=2;
		send1[0].Data[0] = 0x02;
		send1[0].Data[1] = 0x01;
		send1[0].Data[2] = 0x00;
		send1[0].Data[3] = 00;
		send1[0].Data[4] = 0x00;
		send1[0].Data[5] = 00;
		send1[0].Data[6] = 00;
		send1[0].Data[7] = 00;
		
		
		send1[1].ID=0x00;
		send1[1].SendType=1;
		send1[1].RemoteFlag=0;
		send1[1].ExternFlag=0;
		send1[1].DataLen=2;
		send1[1].Data[0] = 0x02;
		send1[1].Data[1] = 0x02;
		send1[1].Data[2] = 0x00;
		send1[1].Data[3] = 00;
		send1[1].Data[4] = 0x00;
		send1[1].Data[5] = 00;
		send1[1].Data[6] = 00;
		send1[1].Data[7] = 00;
		int ret;
 
		for(int i = 0; i < 2; i++) {
    		ret = VCI_Transmit(VCI_USBCAN2, 0, 0, &send1[i], 1);
    		usleep(20000);	
		}
		return 1;
}
Wheel::~Wheel(){
	VCI_CloseDevice(4,0);
}
