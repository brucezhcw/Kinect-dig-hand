// MOTOCOMES.h : MOTOCOMES.DLL ��ͷ�ļ�
//

#pragma once
#undef AFX_DATA
#define AFX_DATA AFX_EXT_DATA

#ifndef __AFXWIN_H__
	#error "PCH �ڰ�����ͷ�ļ�֮ǰ����� 'stdafx.h' "
#endif

#include "resource.h"		// ��Ҫ����


// CMOTOCOMESApp
// �����MOTOCOMES.cppʵ�������
//

#pragma pack(4)				//����淶

class CMOTOCOMESApp : public CWinApp
{
public:
	CMOTOCOMESApp();

//  ����
public:
	virtual BOOL InitInstance();

	DECLARE_MESSAGE_MAP()
};

//��ʱ��ʼֵ
static const long	TIMEOUT					= 500;							//���ճ�ʱʱ�䣨���룩����ʼֵ
static const long	RETRY					= 3;							//���ճ�ʱ���Լ�������ʼֵ

/****************�벻Ҫ�������½ṹ��Ķ���*******************/
//�ռ��С����
static const short	LENGTH_OF_TIME = 16;							//����ʱ�䳤��(ex. 2007/05/10 15:49)
static const short	LENGTH_OF_NAME = 32;							//�ַ������Ƶ����ݳ��ȣ����32���ַ���
static const short	LENGTH_OF_SUBCODE_ADDINFO = 16;							//��ϸ���ݸ�����Ϣ�ַ������ȣ����16���ַ���
static const short	LENGTH_OF_SUBCODE_STRINGDATA = 96;							//��ϸ�����ַ������ȣ����96���ַ���
static const short	LENGTH_OF_ALARMLIST = 4;							//�����б���
static const short	NUMBER_OF_AXIS = 8;							//�����˵��������
static const short	LENGTH_OF_CONFIGNAME = 4;							//���������Ƶ����ݳ���
static const short	LENGTH_OF_ELAPSETIME = 12;							//���ݾ���ʱ��ĳ���
static const short	LENGTH_OF_SYSTEMVER = 24;							//ϵͳ�汾�����ݳ���
static const short	LENGTH_OF_ROBOTNAME = 16;							//�ͺ����Ƶ����ݳ���
static const short	LENGTH_OF_PARAMNO = 8;							//�����ŵ����ݳ���
static const short	NUMBER_OF_BASE_AXIS = 3;							//��������������
static const short	NUMBER_OF_STATION_AXIS = 6;							//վ��������

static const short	LENGTH_OF_MULTI_1 = 474;							//1���ֽڴ�С�Ķ�����ݵ������Ŀ
static const short	LENGTH_OF_MULTI_2 = 237;							//2���ֽڴ�С�Ķ�����ݵ������Ŀ
static const short	LENGTH_OF_MULTI_4 = 118;							//4���ֽڴ�С�Ķ�����ݵ������Ŀ
static const short	LENGTH_OF_MULTI_STR = 29;							//�����������������ַ���
static const short	LENGTH_OF_MULTI_POS = 9;							//�������λ�����ݵ��������
static const short	LENGTH_OF_MULTI_BPEX = 13;							//��׼��λ��/�ⲿ��λ�����ݵĶ�����ݵ��������
static const short	LENGTH_OF_STRING = 16;							//�����ַ��������ĳ���

//��������
typedef struct {
	long alarmCode;														//��������
	long alarmData;														//��������
	long alarmType;														//������������
	char alarmTime[LENGTH_OF_TIME + 1];									//��������ʱ��
	char alarmName[LENGTH_OF_NAME + 1];									//�����ַ�������
} ESAlarmData;

//�����Ӵ�������
typedef struct {
		char  alarmAddInfo[LENGTH_OF_SUBCODE_ADDINFO+1];					//��ϸ���ݸ�����Ϣ�ַ���
		char  alarmStrData[LENGTH_OF_SUBCODE_STRINGDATA+1];					//��ϸ�����ַ���
		char  alarmHighlightData[LENGTH_OF_SUBCODE_STRINGDATA+1];			//��ϸ�����ݷ�����ʾ��Ϣ
} ESSubcodeData;

//�������ݣ���Ӧ�����ַ�����
typedef struct {
		ESAlarmData	alarmData;												//��������
		ESSubcodeData subcodeData;											//�Ӵ�������
} ESAlarmDataEx;

//�����б�
typedef struct {
	ESAlarmData data[LENGTH_OF_ALARMLIST];									//��������
} ESAlarmList;

//�����б���Ӧ�������ַ�����
typedef struct {
	ESAlarmDataEx data[LENGTH_OF_ALARMLIST];								//�������ݣ���Ӧ���Ӵ��룩
} ESAlarmListEx;

//״̬����
typedef struct {
			long status1;													//״̬����1
			long status2;													//״̬����2
} ESStatusData;

//��ҵ״̬����
typedef struct {
			char jobName[LENGTH_OF_NAME+1];									//��ҵ����
			long lineNo;													//�к�
			long stepNo;													//����
			long speedOverride;												//�ٶȸ���ֵ
} ESJobStatusData;

//����������
typedef struct {
			char configurations[NUMBER_OF_AXIS][LENGTH_OF_CONFIGNAME+1];	//�����ƣ�SLURBT��XYZRxRyRz��
} ESConfigurationData;

//������
typedef struct {
			double axis [NUMBER_OF_AXIS];									//������
} ESAxisData;

//������λ������
typedef struct {
	long dataType;			//�������ͣ�����ֵ/����ֵ��
	long fig;				//��ʽ
	long toolNo;			//���߱��
	long userFrameNo;		//�û�������
	long exFig;				//��չ��ʽ
	ESAxisData axesData;	//������
} ESPositionData;

//��׼λ��/�ⲿ��λ������
typedef struct {
			long dataType;													//�������ͣ�����ֵ/����ֵ��
			ESAxisData axesData;											//������
} ESBpexPositionData;

//����ʱ������
typedef struct {
			char startTime[LENGTH_OF_TIME+1];								//���п�ʼʱ��
			char elapseTime[LENGTH_OF_ELAPSETIME+1];						//������ʱ��
} ESMonitoringTimeData;

//ϵͳ��Ϣ����
typedef struct {
			char systemVersion[LENGTH_OF_SYSTEMVER+1];						//ϵͳ����汾
			char name[LENGTH_OF_ROBOTNAME+1];								//�ͺ�����/�÷�����
			char parameterNo[LENGTH_OF_PARAMNO+1];							//������
} ESSystemInfoData;

//�ƶ���Ϣ����
typedef struct
{
	long robotNo;		//�����˱��
	long stationNo;		//վ��
	long speedType;		//�ٶȵȼ�
	double speed;		//�ٶȹ��
} ESMoveData;

//�����˵�Ŀ��λ�����ݣ�ֱ�����꣩
typedef ESPositionData ESCartPosData;										//�����˵�Ŀ��λ�����ݣ�����ֵ��

//�����˵�Ŀ��λ�����ݣ����壩
typedef ESAxisData ESPulsePosData;											//�����˵�Ŀ��λ�����ݣ�����ֵ��

//Ŀ���׼λ������
typedef struct {
			double axis[NUMBER_OF_BASE_AXIS];								//��׼Ŀ��λ������
} ESBaseData;

//վ��Ŀ��λ������
typedef struct {
			double axis[NUMBER_OF_STATION_AXIS];							//Ŀ��վλ������
} ESStationData;

//�ƶ�ָ�����ݣ��ѿ������꣩
typedef struct {
			ESMoveData		moveData;										//�ƶ���Ϣ����
			ESCartPosData	robotPos;										//�����˵�Ŀ��λ������
			ESBaseData		basePos;										//����Ŀ��λ������
			ESStationData	stationPos;										//Ŀ��վλ������
}ESCartMoveData;

//�ƶ�ָ�����ݣ����壩
typedef struct {
			ESMoveData		moveData;										//�ƶ���Ϣ����
			ESPulsePosData	robotPos;										//�����˵�Ŀ��λ������
			ESBaseData		basePos;										//����Ŀ��λ������
			ESStationData	stationPos;										//Ŀ��վλ������
			long			toolNo;											//���߱��
}ESPulseMoveData;

//���1�ֽڵ�����
typedef struct {
			char data[LENGTH_OF_MULTI_1];
}ESMultiByteData;

//2���ֽڵĶ������
typedef struct {
			short data[LENGTH_OF_MULTI_2];
}ESMultiShortData;

//����ֽڵ����ݣ��޷��ţ�
typedef struct {
			unsigned short data[LENGTH_OF_MULTI_2];
}ESMultiUShortData;

//LONG�Ķ������
typedef struct {
			long data[LENGTH_OF_MULTI_4];
}ESMultiLongData;

//DOUBLE�Ķ������
typedef struct {
			double data[LENGTH_OF_MULTI_4];
}ESMultiRealData;

//�ַ����Ķ������
typedef struct {
			char data[LENGTH_OF_MULTI_STR][LENGTH_OF_STRING+1];
}ESMultiStrData;

//ESPositionData�Ķ������
typedef struct {
			ESPositionData data[LENGTH_OF_MULTI_POS];
}ESMultiPositionData;

//ESBpexPositionData�Ķ������
typedef struct {
			ESBpexPositionData data[LENGTH_OF_MULTI_BPEX];
}ESMultiBpexPositionData;

#define STDCALL __stdcall

/*����*/
//��������
LONG	STDCALL	ESOpen(long controllerType, char *ipAddress, HANDLE *handle);						//����
LONG	STDCALL	ESClose(HANDLE handle);																//����
LONG	STDCALL ESSetTimeOut(HANDLE handle, long timeOut, long retry);								//���ó�ʱ/����
//���ϵͳ
LONG	STDCALL	ESGetAlarm(HANDLE handle, ESAlarmList *alarmList);									//��ȡ��������
LONG	STDCALL	ESGetAlarmHist(HANDLE handle, long alarmHistNo, ESAlarmData *alarmData);			//��ȡ��ʷ��¼
LONG	STDCALL	ESGetAlarmEx(HANDLE handle, ESAlarmListEx *alarmList);								//��ȡ��������Ӧ�Ӵ����ַ�����
LONG	STDCALL	ESGetAlarmHistEx(HANDLE handle, long alarmHistNo, ESAlarmDataEx *alarmData);		//��ȡ��ʷ���������Ӵ����ַ�����Ӧ��
LONG	STDCALL	ESGetStatus(HANDLE handle, ESStatusData *statusData);								//��ȡ״̬
LONG	STDCALL	ESGetJobStatus(HANDLE handle, long taskNo, ESJobStatusData *jobStatusData);			//��ȡִ����ҵ��Ϣ
LONG	STDCALL	ESGetConfiguration(HANDLE handle, long ctrlGrp, ESConfigurationData *configData);	//��ȡ������
LONG	STDCALL	ESGetPosition(HANDLE handle, long ctrlGrp, ESPositionData *positionData);			//��ȡ������λ��
LONG	STDCALL	ESGetDeviation(HANDLE handle, long ctrlGrp, ESAxisData *deviationData);				//��ȡÿ�����λ��ƫ��
LONG	STDCALL	ESGetTorque(HANDLE handle, long ctrlGrp, ESAxisData *torqueData);					//��ȡÿ�����Ť��
LONG	STDCALL ESGetMonitoringTime(HANDLE handle, long timeType, ESMonitoringTimeData *timeData);	//��ȡ����ʱ��
LONG	STDCALL ESGetSystemInfo(HANDLE handle, long systemType, ESSystemInfoData *infoData);		//��ȡϵͳ��Ϣ

//I/O��дϵͳ
LONG	STDCALL	ESReadIO1(HANDLE handle, long ioNumber, short *ioData);								//IO��ȡ
LONG	STDCALL	ESWriteIO1(HANDLE handle, long ioNumber, short ioData);								//IOд��
LONG	STDCALL	ESReadIO2(HANDLE handle, long ioNumber, short *ioData);								//IO��ȡ(1�ֽ�IO��ָֹ����
LONG	STDCALL	ESWriteIO2(HANDLE handle, long ioNumber, short ioData);								//IOд��(1�ֽ�IO��ָֹ����
LONG	STDCALL	ESReadRegister(HANDLE handle, long regNumber, unsigned short *regData);				//��ȡ�Ĵ���
LONG	STDCALL	ESWriteRegister(HANDLE handle, long regNumber, unsigned short regData);				//д��Ĵ���
LONG	STDCALL	ESReadIOM(HANDLE handle, long ioNumber, long number, ESMultiByteData *ioData);		//IO��ȡ(Multi)
LONG	STDCALL	ESWriteIOM(HANDLE handle, long ioNumber, long number, ESMultiByteData ioData);		//IOд��(Multi)
LONG	STDCALL	ESReadRegisterM(HANDLE handle, long regNumber, long number,							//��ȡ�Ĵ���(Multi)
								ESMultiUShortData *regData);
LONG	STDCALL	ESWriteRegisterM(HANDLE handle, long regNumber, long number,						//д��Ĵ���(Multi)
								 ESMultiUShortData regData);

//��ȡ���ݷ���ϵͳ�ͱ༭ϵͳ
LONG	STDCALL	ESGetVarData1(HANDLE handle, long type, long number, double *data);					//��ȡ����
LONG	STDCALL	ESSetVarData1(HANDLE handle, long type, long number, double data);					//д�����
LONG	STDCALL	ESGetVarData2(HANDLE handle, long type, long number, double *data);					//��ȡ������1�ֽ�IO��ָֹ����
LONG	STDCALL	ESSetVarData2(HANDLE handle, long type, long number, double data);					//д�������1�ֽ�IO��ָֹ����
LONG	STDCALL	ESGetStrData(HANDLE handle, long number, char *cp);									//��ȡ�ַ�������S��
LONG	STDCALL	ESSetStrData(HANDLE handle, long number, char *cp);									//д���ַ�������S��
LONG	STDCALL	ESGetPositionData(HANDLE handle, long number, ESPositionData *positionData);		//��ȡ������λ�����ͱ�����P��
LONG	STDCALL	ESSetPositionData(HANDLE handle, long number, ESPositionData positionData);			//д�������λ�����ͱ�����P��
LONG	STDCALL	ESGetBpexPositionData(HANDLE handle, long type, long number,						//��ȡ��׼λ�����ͱ�����BP��/����λ�����ͱ�����EX��
									  ESBpexPositionData *positionData);
LONG	STDCALL	ESSetBpexPositionData(HANDLE handle, long type, long number,						//д���׼λ�����ͱ�����BP��/����λ�����ͱ�����EX��
									  ESBpexPositionData positionData);
LONG	STDCALL	ESGetVarDataMB(HANDLE handle, long varNo, long number, ESMultiByteData *varData);	//��ȡB����(Multi)
LONG	STDCALL	ESSetVarDataMB(HANDLE handle, long varNo, long number, ESMultiByteData varData);	//д��B����(Multi)
LONG	STDCALL	ESGetVarDataMI(HANDLE handle, long varNo, long number, ESMultiShortData *varData);	//��ȡI����(Multi)
LONG	STDCALL	ESSetVarDataMI(HANDLE handle, long varNo, long number, ESMultiShortData varData);	//д��I����(Multi)	
LONG	STDCALL	ESGetVarDataMD(HANDLE handle, long varNo, long number, ESMultiLongData *varData);	//��ȡD����(Multi)
LONG	STDCALL	ESSetVarDataMD(HANDLE handle, long varNo, long number, ESMultiLongData varData);	//д��D����(Multi)
LONG	STDCALL	ESGetVarDataMR(HANDLE handle, long varNo, long number, ESMultiRealData *varData);	//��ȡR����(Multi)
LONG	STDCALL	ESSetVarDataMR(HANDLE handle, long varNo, long number, ESMultiRealData varData);	//д��R����(Multi)
LONG	STDCALL	ESGetStrDataM(HANDLE handle, long varNo, long number, ESMultiStrData *varData);		//��ȡS����(Multi)
LONG	STDCALL	ESSetStrDataM(HANDLE handle, long varNo, long number, ESMultiStrData varData);		//д��S����(Multi)
LONG	STDCALL	ESGetPositionDataM(HANDLE handle, long varNo, long number,							//��ȡP����(Multi)
								   ESMultiPositionData *positionData);
LONG	STDCALL	ESSetPositionDataM(HANDLE handle, long varNo, long number,							//д��P����(Multi)
								   ESMultiPositionData positionData);
LONG	STDCALL	ESGetBpexPositionDataM(HANDLE handle, long type, long varNo, long number,			//��ȡBP��EX����(Multi)
									   ESMultiBpexPositionData *positionData);
LONG	STDCALL	ESSetBpexPositionDataM(HANDLE handle, long type, long varNo, long number,			//д��BP��EX����(Multi)
									   ESMultiBpexPositionData positionData);

//����ϵͳ
LONG	STDCALL	ESReset(HANDLE handle);																//��λ
LONG	STDCALL	ESCancel(HANDLE handle);															//ȡ��
LONG	STDCALL	ESHold(HANDLE handle, long onOff);													//����
LONG	STDCALL	ESServo(HANDLE handle, long onOff);													//�ŷ�ON
LONG	STDCALL	ESHlock(HANDLE handle, long onOff);													//PP��IO����ϵͳ�źŵĻ���
LONG	STDCALL	ESCycle(HANDLE handle, long cycle);													//����/ѭ��/�����Զ�
LONG	STDCALL	ESBDSP(HANDLE handle, char *message);												//�ַ���ָ��ؼ�

//����ϵͳ
LONG	STDCALL	ESStartJob(HANDLE handle);															//��ҵ��ʼ
LONG	STDCALL ESCartMove(HANDLE handle, long moveType, ESCartMoveData moveData);					//�ƶ����ѿ������꣩
LONG	STDCALL ESPulseMove(HANDLE handle, long moveType, ESPulseMoveData moveData);				//�ƶ������壬�ؽڣ���

//����ѡ��ϵͳ
LONG	STDCALL	ESSelectJob(HANDLE handle, long jobType, long lineNo, char *jobName);				//����ѡ�� ����ϵ��

//�ļ�����ϵͳ
LONG	STDCALL	ESSaveFile(HANDLE handle, char *savePath, char *fileName);							//�����ļ�
LONG	STDCALL	ESLoadFile(HANDLE handle, char *filePath);											//�����ļ�
LONG	STDCALL	ESDeleteJob(HANDLE handle, char *jobName);											//ɾ���ļ�
LONG	STDCALL	ESFileListFirst(HANDLE handle, long fileType, char *fileName);						//�����ļ��б���ȡ��ʼ
LONG	STDCALL	ESFileListNext(HANDLE handle, char *fileName);										//��ȡ�ļ��б�
#undef AFX_DATA
#define AFX_DATA
