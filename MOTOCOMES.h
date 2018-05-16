// MOTOCOMES.h : MOTOCOMES.DLL 主头文件
//

#pragma once
#undef AFX_DATA
#define AFX_DATA AFX_EXT_DATA

#ifndef __AFXWIN_H__
	#error "PCH 在包含此头文件之前请包括 'stdafx.h' "
#endif

#include "resource.h"		// 主要符号


// CMOTOCOMESApp
// 请参阅MOTOCOMES.cpp实现这个类
//

#pragma pack(4)				//对齐规范

class CMOTOCOMESApp : public CWinApp
{
public:
	CMOTOCOMESApp();

//  覆盖
public:
	virtual BOOL InitInstance();

	DECLARE_MESSAGE_MAP()
};

//超时初始值
static const long	TIMEOUT					= 500;							//接收超时时间（毫秒）：初始值
static const long	RETRY					= 3;							//接收超时重试计数：初始值

/****************请不要更改以下结构体的定义*******************/
//空间大小定义
static const short	LENGTH_OF_TIME = 16;							//数据时间长度(ex. 2007/05/10 15:49)
static const short	LENGTH_OF_NAME = 32;							//字符串名称的数据长度（最多32个字符）
static const short	LENGTH_OF_SUBCODE_ADDINFO = 16;							//详细数据附加信息字符串长度（最多16个字符）
static const short	LENGTH_OF_SUBCODE_STRINGDATA = 96;							//详细数据字符串长度（最多96个字符）
static const short	LENGTH_OF_ALARMLIST = 4;							//报警列表长度
static const short	NUMBER_OF_AXIS = 8;							//机器人的最大轴数
static const short	LENGTH_OF_CONFIGNAME = 4;							//轴配置名称的数据长度
static const short	LENGTH_OF_ELAPSETIME = 12;							//数据经过时间的长度
static const short	LENGTH_OF_SYSTEMVER = 24;							//系统版本的数据长度
static const short	LENGTH_OF_ROBOTNAME = 16;							//型号名称的数据长度
static const short	LENGTH_OF_PARAMNO = 8;							//参数号的数据长度
static const short	NUMBER_OF_BASE_AXIS = 3;							//行走轴的最大轴数
static const short	NUMBER_OF_STATION_AXIS = 6;							//站轴最大轴号

static const short	LENGTH_OF_MULTI_1 = 474;							//1个字节大小的多个数据的最大数目
static const short	LENGTH_OF_MULTI_2 = 237;							//2个字节大小的多个数据的最大数目
static const short	LENGTH_OF_MULTI_4 = 118;							//4个字节大小的多个数据的最大数目
static const short	LENGTH_OF_MULTI_STR = 29;							//多个数据最大数量的字符串
static const short	LENGTH_OF_MULTI_POS = 9;							//多个数据位置数据的最大数量
static const short	LENGTH_OF_MULTI_BPEX = 13;							//基准轴位置/外部轴位置数据的多个数据的最大数量
static const short	LENGTH_OF_STRING = 16;							//数据字符串变量的长度

//报警数据
typedef struct {
	long alarmCode;														//报警代码
	long alarmData;														//报警数据
	long alarmType;														//报警数据类型
	char alarmTime[LENGTH_OF_TIME + 1];									//报警发生时间
	char alarmName[LENGTH_OF_NAME + 1];									//报警字符串名称
} ESAlarmData;

//报警子代码数据
typedef struct {
		char  alarmAddInfo[LENGTH_OF_SUBCODE_ADDINFO+1];					//详细数据附加信息字符串
		char  alarmStrData[LENGTH_OF_SUBCODE_STRINGDATA+1];					//详细数据字符串
		char  alarmHighlightData[LENGTH_OF_SUBCODE_STRINGDATA+1];			//详细的数据反向显示信息
} ESSubcodeData;

//报警数据（对应子码字符串）
typedef struct {
		ESAlarmData	alarmData;												//报警数据
		ESSubcodeData subcodeData;											//子代码数据
} ESAlarmDataEx;

//警报列表
typedef struct {
	ESAlarmData data[LENGTH_OF_ALARMLIST];									//警报数据
} ESAlarmList;

//报警列表（对应于子码字符串）
typedef struct {
	ESAlarmDataEx data[LENGTH_OF_ALARMLIST];								//报警数据（对应于子代码）
} ESAlarmListEx;

//状态数据
typedef struct {
			long status1;													//状态数据1
			long status2;													//状态数据2
} ESStatusData;

//作业状态数据
typedef struct {
			char jobName[LENGTH_OF_NAME+1];									//作业名称
			long lineNo;													//行号
			long stepNo;													//步数
			long speedOverride;												//速度覆盖值
} ESJobStatusData;

//轴配置数据
typedef struct {
			char configurations[NUMBER_OF_AXIS][LENGTH_OF_CONFIGNAME+1];	//轴名称（SLURBT，XYZRxRyRz）
} ESConfigurationData;

//轴数据
typedef struct {
			double axis [NUMBER_OF_AXIS];									//轴数据
} ESAxisData;

//机器人位置数据
typedef struct {
	long dataType;			//数据类型（脉冲值/正交值）
	long fig;				//形式
	long toolNo;			//工具编号
	long userFrameNo;		//用户坐标编号
	long exFig;				//扩展形式
	ESAxisData axesData;	//轴数据
} ESPositionData;

//基准位置/外部轴位置数据
typedef struct {
			long dataType;													//数据类型（脉冲值/正交值）
			ESAxisData axesData;											//轴数据
} ESBpexPositionData;

//管理时间数据
typedef struct {
			char startTime[LENGTH_OF_TIME+1];								//运行开始时间
			char elapseTime[LENGTH_OF_ELAPSETIME+1];						//经过的时间
} ESMonitoringTimeData;

//系统信息数据
typedef struct {
			char systemVersion[LENGTH_OF_SYSTEMVER+1];						//系统软件版本
			char name[LENGTH_OF_ROBOTNAME+1];								//型号名称/用法名称
			char parameterNo[LENGTH_OF_PARAMNO+1];							//参数号
} ESSystemInfoData;

//移动信息数据
typedef struct
{
	long robotNo;		//机器人编号
	long stationNo;		//站号
	long speedType;		//速度等级
	double speed;		//速度规格
} ESMoveData;

//机器人的目标位置数据（直角坐标）
typedef ESPositionData ESCartPosData;										//机器人的目标位置数据（正交值）

//机器人的目标位置数据（脉冲）
typedef ESAxisData ESPulsePosData;											//机器人的目标位置数据（脉冲值）

//目标基准位置数据
typedef struct {
			double axis[NUMBER_OF_BASE_AXIS];								//基准目标位置数据
} ESBaseData;

//站的目标位置数据
typedef struct {
			double axis[NUMBER_OF_STATION_AXIS];							//目标站位置数据
} ESStationData;

//移动指令数据（笛卡尔坐标）
typedef struct {
			ESMoveData		moveData;										//移动信息数据
			ESCartPosData	robotPos;										//机器人的目标位置数据
			ESBaseData		basePos;										//基础目标位置数据
			ESStationData	stationPos;										//目标站位置数据
}ESCartMoveData;

//移动指令数据（脉冲）
typedef struct {
			ESMoveData		moveData;										//移动信息数据
			ESPulsePosData	robotPos;										//机器人的目标位置数据
			ESBaseData		basePos;										//基础目标位置数据
			ESStationData	stationPos;										//目标站位置数据
			long			toolNo;											//工具编号
}ESPulseMoveData;

//多个1字节的数据
typedef struct {
			char data[LENGTH_OF_MULTI_1];
}ESMultiByteData;

//2个字节的多个数据
typedef struct {
			short data[LENGTH_OF_MULTI_2];
}ESMultiShortData;

//多个字节的数据（无符号）
typedef struct {
			unsigned short data[LENGTH_OF_MULTI_2];
}ESMultiUShortData;

//LONG的多个数据
typedef struct {
			long data[LENGTH_OF_MULTI_4];
}ESMultiLongData;

//DOUBLE的多个数据
typedef struct {
			double data[LENGTH_OF_MULTI_4];
}ESMultiRealData;

//字符串的多个数据
typedef struct {
			char data[LENGTH_OF_MULTI_STR][LENGTH_OF_STRING+1];
}ESMultiStrData;

//ESPositionData的多个数据
typedef struct {
			ESPositionData data[LENGTH_OF_MULTI_POS];
}ESMultiPositionData;

//ESBpexPositionData的多个数据
typedef struct {
			ESBpexPositionData data[LENGTH_OF_MULTI_BPEX];
}ESMultiBpexPositionData;

#define STDCALL __stdcall

/*命令*/
//其他功能
LONG	STDCALL	ESOpen(long controllerType, char *ipAddress, HANDLE *handle);						//连接
LONG	STDCALL	ESClose(HANDLE handle);																//断线
LONG	STDCALL ESSetTimeOut(HANDLE handle, long timeOut, long retry);								//设置超时/重试
//监控系统
LONG	STDCALL	ESGetAlarm(HANDLE handle, ESAlarmList *alarmList);									//读取警报读数
LONG	STDCALL	ESGetAlarmHist(HANDLE handle, long alarmHistNo, ESAlarmData *alarmData);			//读取历史纪录
LONG	STDCALL	ESGetAlarmEx(HANDLE handle, ESAlarmListEx *alarmList);								//读取报警（对应子代码字符串）
LONG	STDCALL	ESGetAlarmHistEx(HANDLE handle, long alarmHistNo, ESAlarmDataEx *alarmData);		//读取历史警报（与子代码字符串对应）
LONG	STDCALL	ESGetStatus(HANDLE handle, ESStatusData *statusData);								//读取状态
LONG	STDCALL	ESGetJobStatus(HANDLE handle, long taskNo, ESJobStatusData *jobStatusData);			//读取执行作业信息
LONG	STDCALL	ESGetConfiguration(HANDLE handle, long ctrlGrp, ESConfigurationData *configData);	//读取轴配置
LONG	STDCALL	ESGetPosition(HANDLE handle, long ctrlGrp, ESPositionData *positionData);			//读取机器人位置
LONG	STDCALL	ESGetDeviation(HANDLE handle, long ctrlGrp, ESAxisData *deviationData);				//读取每个轴的位置偏差
LONG	STDCALL	ESGetTorque(HANDLE handle, long ctrlGrp, ESAxisData *torqueData);					//读取每个轴的扭矩
LONG	STDCALL ESGetMonitoringTime(HANDLE handle, long timeType, ESMonitoringTimeData *timeData);	//读取管理时间
LONG	STDCALL ESGetSystemInfo(HANDLE handle, long systemType, ESSystemInfoData *infoData);		//读取系统信息

//I/O读写系统
LONG	STDCALL	ESReadIO1(HANDLE handle, long ioNumber, short *ioData);								//IO读取
LONG	STDCALL	ESWriteIO1(HANDLE handle, long ioNumber, short ioData);								//IO写入
LONG	STDCALL	ESReadIO2(HANDLE handle, long ioNumber, short *ioData);								//IO读取(1字节IO禁止指定）
LONG	STDCALL	ESWriteIO2(HANDLE handle, long ioNumber, short ioData);								//IO写入(1字节IO禁止指定）
LONG	STDCALL	ESReadRegister(HANDLE handle, long regNumber, unsigned short *regData);				//读取寄存器
LONG	STDCALL	ESWriteRegister(HANDLE handle, long regNumber, unsigned short regData);				//写入寄存器
LONG	STDCALL	ESReadIOM(HANDLE handle, long ioNumber, long number, ESMultiByteData *ioData);		//IO读取(Multi)
LONG	STDCALL	ESWriteIOM(HANDLE handle, long ioNumber, long number, ESMultiByteData ioData);		//IO写入(Multi)
LONG	STDCALL	ESReadRegisterM(HANDLE handle, long regNumber, long number,							//读取寄存器(Multi)
								ESMultiUShortData *regData);
LONG	STDCALL	ESWriteRegisterM(HANDLE handle, long regNumber, long number,						//写入寄存器(Multi)
								 ESMultiUShortData regData);

//读取数据访问系统和编辑系统
LONG	STDCALL	ESGetVarData1(HANDLE handle, long type, long number, double *data);					//读取变量
LONG	STDCALL	ESSetVarData1(HANDLE handle, long type, long number, double data);					//写入变量
LONG	STDCALL	ESGetVarData2(HANDLE handle, long type, long number, double *data);					//读取变量（1字节IO禁止指定）
LONG	STDCALL	ESSetVarData2(HANDLE handle, long type, long number, double data);					//写入变量（1字节IO禁止指定）
LONG	STDCALL	ESGetStrData(HANDLE handle, long number, char *cp);									//读取字符变量（S）
LONG	STDCALL	ESSetStrData(HANDLE handle, long number, char *cp);									//写入字符变量（S）
LONG	STDCALL	ESGetPositionData(HANDLE handle, long number, ESPositionData *positionData);		//读取机器人位置类型变量（P）
LONG	STDCALL	ESSetPositionData(HANDLE handle, long number, ESPositionData positionData);			//写入机器人位置类型变量（P）
LONG	STDCALL	ESGetBpexPositionData(HANDLE handle, long type, long number,						//读取基准位置类型变量（BP）/外轴位置类型变量（EX）
									  ESBpexPositionData *positionData);
LONG	STDCALL	ESSetBpexPositionData(HANDLE handle, long type, long number,						//写入基准位置类型变量（BP）/外轴位置类型变量（EX）
									  ESBpexPositionData positionData);
LONG	STDCALL	ESGetVarDataMB(HANDLE handle, long varNo, long number, ESMultiByteData *varData);	//读取B变量(Multi)
LONG	STDCALL	ESSetVarDataMB(HANDLE handle, long varNo, long number, ESMultiByteData varData);	//写入B变量(Multi)
LONG	STDCALL	ESGetVarDataMI(HANDLE handle, long varNo, long number, ESMultiShortData *varData);	//读取I变量(Multi)
LONG	STDCALL	ESSetVarDataMI(HANDLE handle, long varNo, long number, ESMultiShortData varData);	//写入I变量(Multi)	
LONG	STDCALL	ESGetVarDataMD(HANDLE handle, long varNo, long number, ESMultiLongData *varData);	//读取D变量(Multi)
LONG	STDCALL	ESSetVarDataMD(HANDLE handle, long varNo, long number, ESMultiLongData varData);	//写入D变量(Multi)
LONG	STDCALL	ESGetVarDataMR(HANDLE handle, long varNo, long number, ESMultiRealData *varData);	//读取R变量(Multi)
LONG	STDCALL	ESSetVarDataMR(HANDLE handle, long varNo, long number, ESMultiRealData varData);	//写入R变量(Multi)
LONG	STDCALL	ESGetStrDataM(HANDLE handle, long varNo, long number, ESMultiStrData *varData);		//读取S变量(Multi)
LONG	STDCALL	ESSetStrDataM(HANDLE handle, long varNo, long number, ESMultiStrData varData);		//写入S变量(Multi)
LONG	STDCALL	ESGetPositionDataM(HANDLE handle, long varNo, long number,							//读取P变量(Multi)
								   ESMultiPositionData *positionData);
LONG	STDCALL	ESSetPositionDataM(HANDLE handle, long varNo, long number,							//写入P变量(Multi)
								   ESMultiPositionData positionData);
LONG	STDCALL	ESGetBpexPositionDataM(HANDLE handle, long type, long varNo, long number,			//读取BP、EX变量(Multi)
									   ESMultiBpexPositionData *positionData);
LONG	STDCALL	ESSetBpexPositionDataM(HANDLE handle, long type, long varNo, long number,			//写入BP、EX变量(Multi)
									   ESMultiBpexPositionData positionData);

//操作系统
LONG	STDCALL	ESReset(HANDLE handle);																//复位
LONG	STDCALL	ESCancel(HANDLE handle);															//取消
LONG	STDCALL	ESHold(HANDLE handle, long onOff);													//保持
LONG	STDCALL	ESServo(HANDLE handle, long onOff);													//伺服ON
LONG	STDCALL	ESHlock(HANDLE handle, long onOff);													//PP和IO操作系统信号的互锁
LONG	STDCALL	ESCycle(HANDLE handle, long cycle);													//步进/循环/连续自动
LONG	STDCALL	ESBDSP(HANDLE handle, char *message);												//字符串指令到控件

//激活系统
LONG	STDCALL	ESStartJob(HANDLE handle);															//作业开始
LONG	STDCALL ESCartMove(HANDLE handle, long moveType, ESCartMoveData moveData);					//移动（笛卡尔坐标）
LONG	STDCALL ESPulseMove(HANDLE handle, long moveType, ESPulseMoveData moveData);				//移动（脉冲，关节？）

//工作选择系统
LONG	STDCALL	ESSelectJob(HANDLE handle, long jobType, long lineNo, char *jobName);				//工作选择 坐标系？

//文件控制系统
LONG	STDCALL	ESSaveFile(HANDLE handle, char *savePath, char *fileName);							//保存文件
LONG	STDCALL	ESLoadFile(HANDLE handle, char *filePath);											//加载文件
LONG	STDCALL	ESDeleteJob(HANDLE handle, char *jobName);											//删除文件
LONG	STDCALL	ESFileListFirst(HANDLE handle, long fileType, char *fileName);						//更新文件列表并读取开始
LONG	STDCALL	ESFileListNext(HANDLE handle, char *fileName);										//读取文件列表
#undef AFX_DATA
#define AFX_DATA
