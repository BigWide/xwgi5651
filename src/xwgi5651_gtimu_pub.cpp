//xwgi5651_gtimu_pub.cpp
//gtimu节点，发布gtimu数据

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "xwgi5651/Xwgi5651Gtimu.h"//自定义消息的头文件
#include "sensor_msgs/Imu.h"
#include <serial/serial.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#define XWGI5651_DEBUG
//$GTIMU,GPSWeek,GPSTime,GyroX,GyroY,GyroZ,AccX,AccY,AccZ,Tpr*cs<CR><LF>
//$GTIMU,wwww,ssssss.sss,±ggg.gggg,±ggg.gggg,±ggg.gggg,±aaa.aaaa,±aaa.aaaa,±aaa.aaaa,±tt.t,*hh<CR><LF>
#define MAXLEN 100//每帧数据最大长度，100个字符，包括头字符‘$’和尾字符<LF>，单位：字节

//全局变量
serial::Serial ser;//串口对象 
unsigned char rbuf[5*MAXLEN];//接收缓冲区，要足够大，需要通过测试得出
int state = 0;//解析处理状态机状态
char dataframe[100] = {0};//长度大于100即可
int cnt = 0;//用于记录dataframe中的实际数据长度

struct FormatGTIMU
{
	int   	GPSWeek;//自1980-1-6至当前的星期数（格林尼治时间）
	double 	GPSTime;//自本周日0:00:00至当前的秒数（格林尼治时间）
	double 	Gx;//x轴陀螺仪角速率，deg/s
	double 	Gy;//y
	double 	Gz;//z
	double 	Ax;//x轴加速度计输出，g
	double 	Ay;//y
	double 	Az;//z
	double 	Temp;//温度，摄氏度，比例系数0.001 
};

//$GTIMU,GPSWeek,GPSTime,GyroX,GyroY,GyroZ,AccX,AccY,AccZ,Tpr*cs<CR><LF>
//              1       2     3     4     5    6    7    8   9	--分隔符位置及个数，共9个逗号（星号将被换成逗号）
//          1       2      3     4     5     6    7    8   9  	--字段位置及个数，共9个字段
struct stGTIMUData {
	int		len[9];//各字段长度，用于判断是否有数据，0意味着没有数据
	char 	data[9][20];//存放各字段字符串，最大字段长度设为20
} GtimuData;

/*****************************
  功能：计算异或校验，字符串中所有字符的异或
  返回：返回一个无符号整数
  输入参数：<1>字符串指针，<2>字符串长度（指有效长度，不包括字符串结束标志）
  输出参数：校验结果
******************************/
unsigned char GetXorChecksum(const char *pch, int len) 
{
    unsigned char cs = 0;
    int i;

    for(i=0; i<len; i++)
        cs ^= pch[i];

    return cs;
}

/********************************
	功能：度分格式转换为度
	输入浮点数格式为：dddmm.mmmm
	输出为double类型
********************************/
double DegMin2Deg(double dddmmpmmmm)
{
	double ddd = floor(dddmmpmmmm/100.0);
	double mmpmmmm = dddmmpmmmm - ddd*100.0;
	return ddd + mmpmmmm/60.0;
}

/******************************
            主函数
*******************************/
int main(int argc, char *argv[])
{
	ros::init(argc, argv, "node_xwgi5651_gtimu_pub");//节点名

	ros::NodeHandle nh_gtimu;
	
    //注册发布者，使用自定义消息结构Xwgi5651gtimu，topic为topic_xwgi5651_gtimu，发布队列长度为1000
	ros::Publisher talker_xwgi5651_gtimu = nh_gtimu.advertise<xwgi5651::Xwgi5651Gtimu>("topic_xwgi5651_gtimu", 1000);//发布导航数据
	ros::Publisher talker_xwgi5651_rosimu = nh_gtimu.advertise<sensor_msgs::Imu>("topic_xwgi5651_rosimu", 1000);//发布导航数据

	//打开串口
  	try 
    { 
    	//设置串口属性，并打开串口 
        ser.setPort("/dev/ttyUSB0"); 
        //ser.setBaudrate(230400);
	ser.setBaudrate(115200); 
        serial::Timeout to = serial::Timeout::simpleTimeout(1000); //超时定义，单位：ms
        ser.setTimeout(to); 
        ser.open(); 
    } 
    catch (serial::IOException& e) 
    { 
        ROS_ERROR_STREAM("Fail to open serial port\n"); 
        return -1; 
    } 

    //检测串口是否已经打开，并给出提示信息 
    if(ser.isOpen()) 
    { 
        ROS_INFO_STREAM("/dev/ttyUSB0 open succeed\n"); 
    } 
    else 
    { 
        return -1; 
    } 
		
	ros::Rate loop_rate(200);//设置循环频率为200Hz，gtimu帧的频率为100Hz

	xwgi5651::Xwgi5651Gtimu msg_xwgi5651_gtimu;//自定义消息
	sensor_msgs::Imu msg_xwgi5651_rosimu;//定义消息
	
	ser.flushInput();//在开始正式接收数据前先清除串口的接收缓冲区 	
	memset(dataframe, 0, sizeof(dataframe));//清空gtimu字符串
	int framecnt = 0;
	cnt = 0;//指向dataframe的第一个位置
	while (ros::ok())
	{
		int i, j;
		int start;//当前位置
		int pos;//下一个分隔符的位置
		
		int numinbuf = ser.available();//available()返回从串口缓冲区读回的字符数
		//printf("bytes in buf = %d\n",numinbuf);
		if(numinbuf > 0)//串口缓冲区有数据
		{ 
            int numgetted = ser.read(rbuf, numinbuf);//串口缓冲区数据读到rbuf中
			printf("byte get out = %d\n%s\n",numgetted,rbuf);
			if(numgetted == numinbuf)//取回的数据个数与缓冲区中有的数据个数相同，说明读串口成功
			{
				for(int i=0; i<numgetted; i++)//对收到的字符逐个处理
				{
					switch (state)
					{
						case 0:
							if(rbuf[i] == '$')//收到语句开始标志
							{
								dataframe[0] = '$';
								//printf("%c\n",rbuf[i]);
								cnt = 1;//开始对帧长度的计数
								state = 1;
							}
							else
							{
								cnt = 0;
								state = 0;//回到初始状态
							}
							break;
						case 1:
							if(rbuf[i]=='G')
							{
								dataframe[cnt] = 'G';
								//printf("%c\n",rbuf[i]);
								cnt = 2;
								state = 2;
							}
							else
							{
								cnt = 0;
								state = 0;
							}
							break;
						case 2:
							if(rbuf[i]=='T')
							{
								dataframe[cnt] = 'T';
								//printf("%c\n",rbuf[i]);
								cnt = 3;
								state = 3;
							}
							else
							{
								cnt = 0;
								state = 0;
							}
							break;

						case 3:
							if(rbuf[i]=='I')
							{
								dataframe[cnt] = 'I';
								//printf("%c\n",rbuf[i]);
								cnt = 4;
								state = 4;
							}
							else
							{
								cnt = 0;
								state = 0;
							}
							break;

						case 4:
							if(rbuf[i]=='M')
							{
								dataframe[cnt] = 'M';
								//printf("%c\n",rbuf[i]);
								cnt = 5;
								state = 5;
							}
							else
							{
								cnt = 0;
								state = 0;
							}
							break;

						case 5:
							if(rbuf[i]=='U')
							{
								dataframe[cnt] = 'U';
								//printf("%c\n",rbuf[i]);
								cnt = 6;
								state = 6;
							}
							else
							{
								cnt = 0;
								state = 0;
							}
							break;

						case 6:
							dataframe[cnt] = rbuf[i];
							cnt++;
							
							if(cnt > 100)//数据长度超出限度但仍未收到结束字符
							{
								cnt = 0;
								state = 0;
								break;
							}
							else if(dataframe[cnt-1]=='\n' && dataframe[cnt-2]=='\r')//在到达最大长度之前遇到字符LF
							{
								printf("Total byte num = %d\n",cnt);//cnt就是一条语句的总字节数
								dataframe[cnt]='\0';//手动添加字符串结束标志
								//printf("%s\n",dataframe);

								if(dataframe[cnt-5] != '*')//校验标志不正确，直接返回初始状态 
								{
									printf("Checksum flag '*' error\n");
									cnt = 0;
									state = 0;
									break;
								}
								
								//检查校验
								int cscomputed = GetXorChecksum((char*)(dataframe+1), cnt-6);//计算得到的校验，除去$*hh<CR><LF>共6个字符
								int csreceived = 0;//接收到的校验
								char strtemp[3];
								strtemp[0] = dataframe[cnt-4];
								strtemp[1] = dataframe[cnt-3];
								strtemp[2] = '\0';//字符串结束标志
								sscanf(strtemp, "%x", &csreceived);//字符串strtemp转换为16进制数
											
								//printf("计算的校验=%d\n",cscomputed);
								//printf("接收的校验=%d\n",csreceived);
								
								//检查校验是否正确
								if(cscomputed != csreceived)
								{
									printf("Checksum error!\n");
									cnt = 0;
									state = 0;//回到初始状态
									break;
								}

								dataframe[cnt-5] = ',';//校验标志'*'换成','，以便于统一处理
								//printf("%s\n",dataframe);

								//此时dataframe中已经存放了完整的一帧gtimu数据，包括从'$'开始到<LF>的所有字符，以及字符串结束标志，例如：
								//$gtimu,wwww,ssssss.sss,±ggg.gggg,±ggg.gggg,±ggg.gggg,±aaa.aaaa,±aaa.aaaa,±aaa.aaaa,±tt.t,*hh<CR><LF>\0
								//检查数据域个数，同时记下分隔符位置，并对数据域进行计数
								j = 0;//用于分隔符个数计数
								char *pos = NULL;//找到的分隔符位置
								char *start = dataframe + 7;//"$gtimu,-"，初始位置设为"$gtimu,"后的那个位置
								do
								{
									pos = strchr(start, ',');//从start位置开始寻找下一个分隔符的位置
									if(pos == NULL)//未找到分隔符，跳出循环
									{
										break;
									}
									else//成功找到分隔符，分隔符为9个逗号
									{
										GtimuData.len[j] = pos - start;//如果是0，说明当前位置就是逗号
										if(GtimuData.len[j] > 0)//当前位置不是逗号，说明数据长度大于等于1
										{
											strncpy(GtimuData.data[j], start, GtimuData.len[j]);
											GtimuData.data[j][GtimuData.len[j]] = '\0';//手动添加字符串结束标志
										}
										start = pos + 1;//从下一个字符开始
										j++;//分隔符个数计数
									}
								}while(j < 9);//应为9个逗号

								if(j == 9)//分隔符个数正确，从strtmp中提取数据
								{
									//gpsweek
									if(GtimuData.len[0] > 0)
										msg_xwgi5651_gtimu.gpsweek = atoi(GtimuData.data[0]);
									else 
										msg_xwgi5651_gtimu.gpsweek = 0;
									//printf("gpsweek = %lf\n", msg_xwgi5651_gtimu.gpsweek);

									//gpstime
									if(GtimuData.len[1] > 0)
										msg_xwgi5651_gtimu.gpstime = atof(GtimuData.data[1]);
									else 
										msg_xwgi5651_gtimu.gpstime = 0.0;
									//printf("gpstime = %lf\n", msg_xwgi5651_gtimu.gpstime);

									//gx
									if(GtimuData.len[2] > 0)
									{
										msg_xwgi5651_gtimu.gx = atof(GtimuData.data[2]);
										msg_xwgi5651_rosimu.angular_velocity.x = msg_xwgi5651_gtimu.gx;
									}	
									else 
										msg_xwgi5651_gtimu.gx = 0.0;
									//printf("gx = %lf\n", msg_xwgi5651_gtimu.gx);

									//gy
									if(GtimuData.len[3] > 0)
									{
										msg_xwgi5651_gtimu.gy = atof(GtimuData.data[3]);
										msg_xwgi5651_rosimu.angular_velocity.y = msg_xwgi5651_gtimu.gy;
									}
									else 
										msg_xwgi5651_gtimu.gy = 0.0;
									//printf("gy = %lf\n", msg_xwgi5651_gtimu.gy);

									//gz
									if(GtimuData.len[4] > 0)
									{
										msg_xwgi5651_gtimu.gz = atof(GtimuData.data[4]);
										msg_xwgi5651_rosimu.angular_velocity.z = msg_xwgi5651_gtimu.gz;
									}
									else 
										msg_xwgi5651_gtimu.gz = 0.0;
									//printf("gz = %lf\n", msg_xwgi5651_gtimu.gz);

									//ax
									if(GtimuData.len[5] > 0)
									{
										msg_xwgi5651_gtimu.ax = atof(GtimuData.data[5]);
										msg_xwgi5651_rosimu.linear_acceleration.x = msg_xwgi5651_gtimu.ax;
									}
									else 
										msg_xwgi5651_gtimu.ax = 0.0;
									//printf("ax = %lf\n", msg_xwgi5651_gtimu.ax);

									//ay
									if(GtimuData.len[6] > 0)
									{
										msg_xwgi5651_gtimu.ay = atof(GtimuData.data[6]);
										msg_xwgi5651_rosimu.linear_acceleration.y = msg_xwgi5651_gtimu.ay;
									}
									else 
										msg_xwgi5651_gtimu.ay = 0.0;
									//printf("ay = %lf\n", msg_xwgi5651_gtimu.ay);

									//az
									if(GtimuData.len[7] > 0)
									{
										msg_xwgi5651_gtimu.az = atof(GtimuData.data[7]);
										msg_xwgi5651_rosimu.linear_acceleration.z = msg_xwgi5651_gtimu.az;
									}
									else 
										msg_xwgi5651_gtimu.az = 0.0;
									//printf("az = %lf\n", msg_xwgi5651_gtimu.az);

									//tpr
									if(GtimuData.len[8] > 0)
										msg_xwgi5651_gtimu.tpr = atof(GtimuData.data[8]);
									else 
										msg_xwgi5651_gtimu.tpr = 0.0;
									//printf("tpr = %lf\n", msg_xwgi5651_gtimu.tpr);

									msg_xwgi5651_gtimu.header.stamp = ros::Time::now();
									talker_xwgi5651_gtimu.publish(msg_xwgi5651_gtimu);//发布消息
									
									msg_xwgi5651_rosimu.header.stamp = ros::Time::now();
									talker_xwgi5651_rosimu.publish(msg_xwgi5651_rosimu);
									
#ifdef XWGI5651_DEBUG							
									ROS_INFO("%d, %d, %d, %f, %f, %f, %f, %f, %f, %f, %f\n", 
									(int)msg_xwgi5651_gtimu.header.stamp.sec,
									(int)msg_xwgi5651_gtimu.header.stamp.nsec,
									msg_xwgi5651_gtimu.gpsweek,
									msg_xwgi5651_gtimu.gpstime,
									msg_xwgi5651_gtimu.gx,
									msg_xwgi5651_gtimu.gy,
									msg_xwgi5651_gtimu.gz,
									msg_xwgi5651_gtimu.ax,
									msg_xwgi5651_gtimu.ay,
									msg_xwgi5651_gtimu.az,
									msg_xwgi5651_gtimu.tpr);
#endif
									//接收完一帧数据，回到初始状态
									cnt = 0;
									state = 0;
									break;
								}
								else//字段个数不等于9
								{
									printf("Data num not equal 9 error!\n");
									cnt = 0;
									state = 0;//回到初始状态
									break;
								}
							}//if(cnt<100 && dataframe[cnt-1]=='\n' && dataframe[cnt-2]=='\r')
					}//switch
				}//for
			}//if
		}//if 

		ros::spinOnce();
		loop_rate.sleep();
	}//while

	return 0;
}//main
