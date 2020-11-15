//xwgi5651_gpfpd_pub.cpp
//gpfpd节点，发布gpfpd数据

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "xwgi5651/Xwgi5651Gpfpd.h"//自定义消息的头文件
#include "pos320/Pos320Nav.h"//自定义消息的头文件
#include <serial/serial.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

using namespace std; 

#define CLEAR() printf("\033[2J")//清屏
#define MOVEUP(x) printf("\033[%dA",(x))//上移x行
#define MOVEDOWN(x) printf("\033[%dB",(x))//下移x行
#define CLEAR_LINE() printf("\033[K")//清除光标到行尾的内容

//#define XWGI5651_DEBUG

//$GPFPD,GPSWeek,GPSTime,Heading,Pitch,Roll,Lattitude,Longitude,Altitude,Ve,Vn,Vu,Baseline,NSV1,NSV2,Status*cs<CR><LF>
//$GPFPD,wwww,ssssss.sss,hhh.hh,+/-pp.pp,+/-rrr.rr,+/-ll.lllllll,+/-lll.lllllll,+/-aaaaa.aa,+/-eee.eee,+/-nnn.nnn,+/-uuu.uuu,bb.bbb,nn,nn,ss,*hh<CR><LF>
//$GPFPD,2050,193365.700,316.668,0.081,-0.740,31.2842571,121.1778514,15.81,-0.031,-0.028,-0.002,-1.000,14,16,0B*17<CR><LF>
#define LENMAX 114//gpfpd每帧数据长度最长114个字符，包括头字符‘$’和尾字符<LF>，单位：字节

//gpfpd帧格式
struct FormatGPFPD
{
	int   	GPSWeek;//自1980-1-6至当前的星期数（格林尼治时间）
	double 	GPSTime;//自本周日0:00:00至当前的秒数（格林尼治时间）
	double  Heading;//偏航角，0 - 359.99
	double  Pitch;//俯仰角，-90 - 90
	double 	Roll;//横滚角，-180 - 180
	double  Lat;//纬度，-90 - 90
	double 	Lon;//经度，-180 - 180
	double 	Alt;//高度，m
	double 	Ve;//东向速度，m/s
	double 	Vn;//北向速度，m/s
	double 	Vu;//天向速度，m/s
	double	Baseline;//基线长度，m
	int		NSV1;//天线1卫星数
	int		NSV2;//天线2卫星树
	//系统状态，
	//低半字节：0-初始化;1-粗对准;2-精对准;3-GPS定位;4-GPS定向;5-RTK;6-DMI组合;7-DMI标定;8-纯惯性;9-零速校正;A-VG模式
	//高半字节：0-GPS;1-BD（定制）;2-双模（定制）
	int		Status;
};

//全局变量
serial::Serial ser;//声明串口对象 
int StateParser = 0;//解析处理状态机状态
int CntByte = 0;//用于记录OneFrame中的实际数据长度
int PosDelimiter[15] = {0};//用于记录分隔符位置
int field_len[15];//字符串长度
int CntDelimiter = 0;//分隔符计数
unsigned char rbuf[500];//接收缓冲区，要足够大，需要通过测试得出
char OneFrame[200];//存放一帧数据，长度大于115即可，这里取200
double gpstime_pre;//上一个gps时间
double delta_t;
char str[3];
unsigned int tmpint = 0;
int cscomputed;//计算得到的校验，除去$*hh<CR><LF>共6个字符
int csreceived;//接收到的校验
char strtemp[3];
char temp_field[30] = {0};
pos320::Pos320Nav msg_pos320_nav;//自定义消息
xwgi5651::Xwgi5651Gpfpd msg_xwgi5651_gpfpd;//自定义消息

ros::Publisher talker_xwgi5651_gpfpd;//发布导航数据
ros::Publisher talker_pos320;//发布导航数据

//$GPFPD,2050,193365.700,316.668,0.081,-0.740,31.2842571,121.1778514,15.81,-0.031,-0.028,-0.002,-1.000,14,16,0B*17
//$GPFPD,2061,295164.020,166.648,0.588,2.248,33.01278382,111.79223354,-44915.523,-885.415,-73.734,-40.589,-1.000,0,0,
//$GPFPD,<1>,<2>,<3>,<4>,<5>,<6>,<7>,<8>,<9>,<10>,<11>,<12>,<13>,<14>,<15>*xx<CR><LF>
//      0   1   2   3   4   5   6   7   8   9    10   11   12   13   14   15 --分隔符位置及个数，共15个逗号，星号将被换成逗号
//        1   2   3   4   5   6   7   8   9   10   11   12   13   14   15    --字段位置及个数，共15个字段

/*****************************
  功能：计算校验，字符串中所有字符的异或
  返回：返回一个无符号整数
  输入参数：<1>字符串指针，<2>字符串长度（指有效长度，不包括字符串结束标志）
  输出参数：校验结果
******************************/
unsigned int GetXorChecksum(const char *pch, int len) 
{
    unsigned int cs = 0;
    int i;

    for(i=0; i<len; i++)
        cs ^= pch[i];

    return cs;
}

/********************************
	功能：度分格式转换为度GPFPD
********************************/
double DegMin2Deg(double dddmmpmmmm)
{
	double ddd = floor(dddmmpmmmm/100.0);
	double mmpmmmm = dddmmpmmmm - ddd*100.0;
	return ddd + mmpmmmm/60.0;
}

/********************************
	ros消息包装并发送
*********************************/
void PubMsg(void)
{
	//按pos320格式发送
	msg_pos320_nav.weeknum = msg_xwgi5651_gpfpd.gpsweek;
	msg_pos320_nav.weekms = msg_xwgi5651_gpfpd.gpstime * 1000;
	msg_pos320_nav.satnum = msg_xwgi5651_gpfpd.nsv1;//有效卫星数
	msg_pos320_nav.lat = msg_xwgi5651_gpfpd.lat;//纬度
	msg_pos320_nav.lon = msg_xwgi5651_gpfpd.lon;//经度
	msg_pos320_nav.alt = msg_xwgi5651_gpfpd.alt;//高度
	msg_pos320_nav.ve = msg_xwgi5651_gpfpd.ve;//东向速度
	msg_pos320_nav.vn = msg_xwgi5651_gpfpd.vn;//北向速度
	msg_pos320_nav.vd = -msg_xwgi5651_gpfpd.vu;//天向速度
	msg_pos320_nav.pitch = msg_xwgi5651_gpfpd.pitch;//俯仰角
	msg_pos320_nav.yaw = msg_xwgi5651_gpfpd.heading;//航向角
	msg_pos320_nav.roll = msg_xwgi5651_gpfpd.roll;//横滚角
	msg_pos320_nav.header.stamp = ros::Time::now();//ros时刻

	//导航状态
	//0x03,0x04: single, 0x03=3, 0x04=4
	//0x45,0x4B: rtk-fiexd, 0x45=69, 0x4B=75
	//0x55,0x5B: rtk-float, 0x55=85, 0x5B=91
	if(msg_xwgi5651_gpfpd.status == 0x03 || msg_xwgi5651_gpfpd.status == 0x04)//单点解	
		msg_pos320_nav.navstate = 0;
	else if(msg_xwgi5651_gpfpd.status == 0x45 || msg_xwgi5651_gpfpd.status == 0x4b)//定点解
		msg_pos320_nav.navstate = 2;
	else if(msg_xwgi5651_gpfpd.status == 0x55 || msg_xwgi5651_gpfpd.status == 0x5b)//浮点解
		msg_pos320_nav.navstate = 1;
	else//定位无效
		msg_pos320_nav.navstate = 3;

	msg_xwgi5651_gpfpd.header.stamp = ros::Time::now();//ros时刻
	talker_xwgi5651_gpfpd.publish(msg_xwgi5651_gpfpd);//发布xwgi5651消息
	talker_pos320.publish(msg_pos320_nav);//发布pos320消息
}

/******************************
            主函数
*******************************/
int main(int argc, char *argv[])
{
	ros::init(argc, argv, "node_xwgi5651_gpfpd_pub");//节点名
	ros::NodeHandle nh_gpfpd;//节点句柄
	
    //注册发布者，使用自定义消息结构Xwgi5651Gpfpd，topic为topic_xwgi5651_gpfpd，发布队列长度为1000
	talker_xwgi5651_gpfpd = nh_gpfpd.advertise<xwgi5651::Xwgi5651Gpfpd>("topic_xwgi5651_gpfpd", 1000);//发布导航数据
	talker_pos320 = nh_gpfpd.advertise<pos320::Pos320Nav>("topic_pos320_nav", 1000);//发布导航数据

  	try 
    { 
    	//设置串口属性，并打开串口 
        ser.setPort("/dev/ttyUSB0"); 
        ser.setBaudrate(115200); 
        serial::Timeout to = serial::Timeout::simpleTimeout(1000); //超时定义，单位：ms
        ser.setTimeout(to); 
        ser.open(); 
    } 
    catch (serial::IOException& e) 
    { 
        printf("XWGI5651 open failed，please check cable and try again！\n");
		getchar(); 
		return -1;
	} 

    //检测串口是否已经打开，并给出提示信息 
    if(ser.isOpen()) 
    { 
        printf("/dev/ttyUSB0 open successed！\n"); 
    } 
    else 
    { 
        printf("/dev/ttyUSB0 open failed！\n");
		getchar(); 
		return -1;
	} 
		
	ros::Rate loop_rate(120);//设置循环频率为120Hz

	ser.flushInput();//在开始正式接收数据前先清除串口的接收缓冲区 	
	memset(OneFrame, 0, sizeof(OneFrame));//清空gps字符串
	int framecnt = 0;
	CntByte = 0;//指向OneFrame的第一个位置
	while (ros::ok())
	{
		int i, j;
		int start;//当前位置
		int pos;//下一个分隔符的位置
		int numinbuf;
		int numgetted;
		
		try
		{
			numinbuf = ser.available();//available()返回从串口缓冲区读回的字符数
			//printf("bytes in buf = %d\n",numinbuf);
		}
		catch (serial::IOException& e)
		{
			printf("Port crashed！ Please check cable!\n");
			getchar(); 
			return -1;
		}
				
		if(numinbuf > 0)//串口缓冲区有数据
		{ 
            numgetted = ser.read(rbuf, numinbuf);//串口缓冲区数据读到rbuf中
			//CLEAR_LINE();
			//printf("byte get out = %d\n",numgetted);
			if(numgetted == numinbuf)//取回的数据个数与缓冲区中有的数据个数相同，说明读串口成功
			{
				for(int i=0; i<numgetted; i++)//对收到的字符逐个处理
				{
					//在一帧数据的接收过程中，只要收到'$'就重新开始
					//状态在1和9之间说明正处于一帧数据的接收过程
					//此处理具有最高优先级，会重置状态机
					if(rbuf[i]=='$' && StateParser>=1 && StateParser<=9)
					{
						printf("STATE %d is broken by $！\n", StateParser);//输出正处在哪个处理阶段
						printf("Ahead $ is: %s\n",OneFrame);//输出接收到$前收到的数据

						//PubMsg();
						memset(OneFrame, 0, sizeof(OneFrame));//清空字符串
						OneFrame[0] = '$';
						CntByte = 1;//开始对帧长度的计数
						StateParser = 1;

						continue;//中断本次循环，直接进入下次循环
					}

					//正常处理过程
					switch (StateParser)
					{
						//等待语句开始标志'$'
						case 0:
							if(rbuf[i] == '$')//收到语句开始标志
							{
								memset(OneFrame, 0, sizeof(OneFrame));
								OneFrame[0] = '$';
								CntByte = 1;//开始对帧长度的计数
								StateParser = 1;
							}
							break;
						
						//寻找帧头"$GPFPD,"
						case 1:
							OneFrame[CntByte] = rbuf[i];
							CntByte++;//指向下一个空位
						
							if(rbuf[i]==',')
							{
								//printf("%s\n",OneFrame);
								if(strncmp(OneFrame, "$GPFPD,", 7) == 0)
								{
									CntDelimiter = 0;//分隔符计数从0开始
									PosDelimiter[0] = CntByte - 1;//记录分隔符在OneFrame中的位置
									//printf("%s\n",OneFrame);
									StateParser = 2;
								}	
								else//帧头错误
								{
									printf("$GPFPD ERROR!\n");
									printf("%s\n", OneFrame);
									memset(OneFrame, 0, sizeof(OneFrame));
									StateParser = 0;
								}
							}
							break;

						//接收各数据域
						case 2:
							OneFrame[CntByte] = rbuf[i];
							CntByte++;//指向下一个空位

							if(rbuf[i]==',')
							{
								CntDelimiter++;//分隔符计数
								PosDelimiter[CntDelimiter] = CntByte - 1;//记下分隔符位置
								if(CntDelimiter == 14)//0-14，共15个分隔符，开始数据解析
								{
									//CLEAR();
									printf("One frame: %s\n",OneFrame);
									for(int j=1; j<=14; j++)//1-14，14个字段
									{
										//printf("PosDelimiter[%d] = %d\n", j, PosDelimiter[j]);
										field_len[j-1] = PosDelimiter[j] - PosDelimiter[j-1] - 1;//数据域长度
										//printf("field_len[%d]=%d\n", j-1, field_len[j-1]);
									}

									if(field_len[0] > 0)
									{
										memset(temp_field, 0, sizeof(temp_field));
										strncpy(temp_field, &OneFrame[PosDelimiter[0]+1], field_len[0]);
										//printf("temp_field=%s\n",temp_field);
										msg_xwgi5651_gpfpd.gpsweek = atoi(temp_field);
										//printf("gpsweek=%d\n", msg_xwgi5651_gpfpd.gpsweek);
									}

									if(field_len[1] > 0)
									{
										memset(temp_field, 0, sizeof(temp_field));
										strncpy(temp_field, &OneFrame[PosDelimiter[1]+1], field_len[1]);
										//printf("temp_field=%s\n",temp_field);
										msg_xwgi5651_gpfpd.gpstime = atof(temp_field);
										//printf("gpstime=%lf\n", msg_xwgi5651_gpfpd.gpstime);
									}

									if(field_len[2] > 0)
									{
										memset(temp_field, 0, sizeof(temp_field));
										strncpy(temp_field, &OneFrame[PosDelimiter[2]+1], field_len[2]);
										//printf("temp_field=%s\n",temp_field);
										msg_xwgi5651_gpfpd.heading = atof(temp_field);
										//printf("heading=%lf\n", msg_xwgi5651_gpfpd.heading);
									}

									if(field_len[3] > 0)
									{
										memset(temp_field, 0, sizeof(temp_field));
										strncpy(temp_field, &OneFrame[PosDelimiter[3]+1], field_len[3]);
										//printf("temp_field=%s\n",temp_field);
										msg_xwgi5651_gpfpd.pitch = atof(temp_field);
										//printf("pitch=%lf\n", msg_xwgi5651_gpfpd.pitch);
									}

									if(field_len[4] > 0)
									{
										memset(temp_field, 0, sizeof(temp_field));
										strncpy(temp_field, &OneFrame[PosDelimiter[4]+1], field_len[4]);
										//printf("temp_field=%s\n",temp_field);
										msg_xwgi5651_gpfpd.roll = atof(temp_field);
										//printf("roll=%lf\n", msg_xwgi5651_gpfpd.roll);
									}
									
									if(field_len[5] > 0)
									{
										memset(temp_field, 0, sizeof(temp_field));
										strncpy(temp_field, &OneFrame[PosDelimiter[5]+1], field_len[5]);
										//printf("temp_field=%s\n",temp_field);
										msg_xwgi5651_gpfpd.lat = atof(temp_field);
										//printf("lat=%15.10lf\n", msg_xwgi5651_gpfpd.lat);
									}

									if(field_len[6] > 0)
									{
										memset(temp_field, 0, sizeof(temp_field));
										strncpy(temp_field, &OneFrame[PosDelimiter[6]+1], field_len[6]);
										//printf("temp_field=%s\n",temp_field);
										msg_xwgi5651_gpfpd.lon = atof(temp_field);
										//printf("lon=%15.10lf\n", msg_xwgi5651_gpfpd.lon);
									}

									if(field_len[7] > 0)
									{
										memset(temp_field, 0, sizeof(temp_field));
										strncpy(temp_field, &OneFrame[PosDelimiter[7]+1], field_len[7]);
										//printf("temp_field=%s\n",temp_field);
										msg_xwgi5651_gpfpd.alt = atof(temp_field);
										//printf("alt=%lf\n", msg_xwgi5651_gpfpd.alt);
									}

									if(field_len[8] > 0)
									{
										memset(temp_field, 0, sizeof(temp_field));
										strncpy(temp_field, &OneFrame[PosDelimiter[8]+1], field_len[8]);
										//printf("temp_field=%s\n",temp_field);
										msg_xwgi5651_gpfpd.ve = atof(temp_field);
										//printf("ve=%lf\n", msg_xwgi5651_gpfpd.ve);
									}

									if(field_len[9] > 0)
									{
										memset(temp_field, 0, sizeof(temp_field));
										strncpy(temp_field, &OneFrame[PosDelimiter[9]+1], field_len[9]);
										//printf("temp_field=%s\n",temp_field);
										msg_xwgi5651_gpfpd.vn = atof(temp_field);
										//printf("vn=%lf\n", msg_xwgi5651_gpfpd.vn);
									}

									if(field_len[10] > 0)
									{
										memset(temp_field, 0, sizeof(temp_field));
										strncpy(temp_field, &OneFrame[PosDelimiter[10]+1], field_len[10]);
										//printf("temp_field=%s\n",temp_field);
										msg_xwgi5651_gpfpd.vu = atof(temp_field);
										//printf("vu=%lf\n", msg_xwgi5651_gpfpd.vu);
									}

									if(field_len[11] > 0)
									{
										memset(temp_field, 0, sizeof(temp_field));
										strncpy(temp_field, &OneFrame[PosDelimiter[11]+1], field_len[11]);
										//printf("temp_field=%s\n",temp_field);
										msg_xwgi5651_gpfpd.baseline = atof(temp_field);
										//printf("baseline=%lf\n", msg_xwgi5651_gpfpd.baseline);
									}

									//nsv1
									if(field_len[12] > 0)
									{
										memset(temp_field, 0, sizeof(temp_field)); 
										strncpy(temp_field, &OneFrame[PosDelimiter[12]+1], field_len[12]);
										//printf("temp_field=%s\n",temp_field);
										msg_xwgi5651_gpfpd.nsv1 = atoi(temp_field);
										//printf("nsv1=%d\n", msg_xwgi5651_gpfpd.nsv1);
									}

									//nsv2
									if(field_len[13] > 0)
									{
										memset(temp_field, 0, sizeof(temp_field));
										strncpy(temp_field, &OneFrame[PosDelimiter[13]+1], field_len[13]);
										//printf("temp_field=%s\n",temp_field);
										msg_xwgi5651_gpfpd.nsv2 = atoi(temp_field);
										//printf("nsv2=%d\n", msg_xwgi5651_gpfpd.nsv2);
									}

									StateParser = 3;
								}//if(CntDelimiter == 14)
							}//if(rbuf[i]==',')
							break;
						
						//等待接收状态字节，第一个字符
						case 3:
							OneFrame[CntByte] = rbuf[i];
							CntByte++;//指向下一个空位

							if((rbuf[i]>='0' && rbuf[i]<='9') || (rbuf[i]>='A' && rbuf[i]<='F'))//状态字节应是一个十六进制数
							{
								//msg_xwgi5651_gpfpd.flag_status = 0;
								StateParser = 4;
							}
							else
							{
								//msg_xwgi5651_gpfpd.flag_status = 1;
								//PubMsg();
								memset(OneFrame, 0, sizeof(OneFrame));
								StateParser = 0;
							}

							break;
						
						//等待接收状态字节，第二个字符
						case 4:
							OneFrame[CntByte] = rbuf[i];
							CntByte++;//指向下一个空位
							//printf("status=0x%c%c\n",OneFrame[CntByte-2],OneFrame[CntByte-1]);
							
							if((rbuf[i]>='0' && rbuf[i]<='9') || (rbuf[i]>='A' && rbuf[i]<='F'))//状态字节应是一个十六进制数
							{
								//msg_xwgi5651_gpfpd.flag_status = 0;
								StateParser = 5;
							}
							else
							{
								//msg_xwgi5651_gpfpd.flag_status = 1;
								//PubMsg();
								memset(OneFrame, 0, sizeof(OneFrame));
								StateParser = 0;
							}

							str[0] = OneFrame[CntByte-2];
							str[1] = OneFrame[CntByte-1];
							str[2] = '\0';//字符串结束标志
							sscanf(str, "%x", &tmpint);//字符串str按16进制数转换为整数
							msg_xwgi5651_gpfpd.status = tmpint;
							//printf("status=%d\n", msg_xwgi5651_gpfpd.status);

							// if(delta_t != 0.01)
							// 	printf("Period error!\n");
							
							//talker_xwgi5651_gpfpd.publish(msg_xwgi5651_gpfpd);//发布消息
							//talker_pos320.publish(msg_pos320_nav);//发布消息
							
							break;
						
						//等待校验和标志*
						case 5:
							OneFrame[CntByte] = rbuf[i];
							CntByte++;//指向下一个空位
							
							if(rbuf[i]=='*')
							{
								//msg_xwgi5651_gpfpd.flag_star = 0;
								StateParser = 6;
							}
							else
							{
								//msg_xwgi5651_gpfpd.flag_star = 1;
								//PubMsg();
								memset(OneFrame, 0, sizeof(OneFrame));
								StateParser = 0;
								//printf("Checksum flag * errror!\n");//校验标志接收错误
							}
							break;
						
						//校验和第一个字符
						case 6:
							OneFrame[CntByte] = rbuf[i];
							CntByte++;//指向下一个空位
							if((rbuf[i]>='0' && rbuf[i]<='9') || (rbuf[i]>='A' && rbuf[i]<='F'))//校验和字节应是一个十六进制数
							{
								StateParser = 7;
							}
							else
							{
								//printf("ERROR: Checksum not hex!\n");
								memset(OneFrame, 0, sizeof(OneFrame));
								StateParser = 0;
							}
							break;

						//校验和第二个字符	
						case 7:
							OneFrame[CntByte] = rbuf[i];
							CntByte++;//指向下一个空位

							if((rbuf[i]>='0' && rbuf[i]<='9') || (rbuf[i]>='A' && rbuf[i]<='F'))//校验和字节应是一个十六进制数
							{
								//检查校验
								cscomputed = GetXorChecksum((char*)(OneFrame+1), CntByte-4);//计算得到的校验，除去$*hh<CR><LF>共6个字符
								csreceived = 0;//接收到的校验
								strtemp[0] = OneFrame[CntByte-2];
								strtemp[1] = OneFrame[CntByte-1];
								strtemp[2] = '\0';//字符串结束标志
								sscanf(strtemp, "%x", &csreceived);//字符串strtemp转换为16进制数
											
								//printf("caculated checksum=0x0%x\n",cscomputed);
								//printf("received  checksum=0x0%x\n",csreceived);
								
								//检查校验是否正确
								if(cscomputed != csreceived)//校验和不匹配
								{
									//msg_xwgi5651_gpfpd.flag_cs = 1;
									//PubMsg();
									memset(OneFrame, 0, sizeof(OneFrame));
									StateParser = 0;
									//printf("Checksum error!\n");
								}
								else//校验和匹配
								{
									//msg_xwgi5651_gpfpd.flag_cs = 0;			
									StateParser = 8;
								}	
							}//校验和字节是hex
							else
							{
								//printf("ERROR: Checksum not hex!\n");
								memset(OneFrame, 0, sizeof(OneFrame));
								StateParser = 0;
							}
							
							break;
						
						//等待结束标志<CR>=0x0d
						case 8:
							OneFrame[CntByte] = rbuf[i];
							CntByte++;//指向下一个空位
							if(rbuf[i] == '\r')
							{
								//msg_xwgi5651_gpfpd.flag_cr = 0;
								//printf("End flag <CR>=0x0%x OK!\n",rbuf[i]);
								StateParser = 9;
							}
							else
							{
								//msg_xwgi5651_gpfpd.flag_cr = 1;
								//PubMsg();
								memset(OneFrame, 0, sizeof(OneFrame));
								StateParser = 0;
								//printf("End flag <CR> error!\n");
							}
							break;
						
						//等待结束标志<LF>=0x0a
						case 9:
							OneFrame[CntByte] = rbuf[i];
							if(rbuf[i]=='\n')
							{
								//msg_xwgi5651_gpfpd.flag_lf = 0;
								//printf("End flag <LF>=0x0%x OK!\n",rbuf[i]);
								delta_t = msg_xwgi5651_gpfpd.gpstime - gpstime_pre;//前后两帧之间的时间差
								gpstime_pre = msg_xwgi5651_gpfpd.gpstime;
								//CLEAR_LINE();
								printf("Frame period=%f(sec)\n", delta_t);
								//CLEAR_LINE();
								printf("A good frame!\n");
								//MOVEUP(3);//光标上移3行
								//CLEAR();//清屏
							}
							else
							{
								printf("End flag <LF> error!\n");
							}
							
							PubMsg();
							memset(OneFrame, 0, sizeof(OneFrame));
							StateParser = 0;
							break;

						default:
							memset(OneFrame, 0, sizeof(OneFrame));
							StateParser = 0;
						 	break;

					}//switch(StateParser)
				}//for(int i=0; i<numgetted; i++)
			}//if(numgetted == numinbuf)
		}//if(numinbuf > 0) 

		ros::spinOnce();//执行等待队列中所有回调函数
		loop_rate.sleep();
	}//while

	return 0;
}//main
