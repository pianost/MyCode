#include "DataProcess.H"

UART_DataPack RemoteData;
Remote_Handle_t Remote_Control;

//void DataSend(uint32_t CMDID,uint8_t *CMD)
//{
//		
//		uint8_t data[8];
//		memset(data,0x55,sizeof data);
//	
//		for(int i = 0; i<3; ++i)
//	{
//		data[i] = CMD[i];
//	}
//	
//	CAN_Send_StdDataFrame(&hcan2,CMDID,data);
//}

//void CalculateSpeed(motor_t Motor1,motor_t Motor2, motor_t Motor3)
//{
//		
//		Motor1.v = Remote_Control.Ex * 0.866 + Remote_Control.Ey * 0.5+ Remote_Control.Eangle * 5;
//		Motor2.v = - Remote_Control.Ex* 0.866 + Remote_Control.Ey * 0.5+ Remote_Control.Eangle * 5;
//		Motor3.v = 0 - Remote_Control.Ey + Remote_Control.Eangle * 5;
//		
//		Motor1.vH = ((int16_t)Motor1.v>>8)&0xFF;
//		Motor1.vL = (int16_t)Motor1.v&0xFF;
//	
//		Motor2.vH = ((int16_t)Motor1.v>>8)&0xFF;
//		Motor2.vL = (int16_t)Motor1.v&0xFF;
//	
//		Motor3.vH = ((int16_t)Motor1.v>>8)&0xFF;
//		Motor3.vL = (int16_t)Motor1.v&0xFF;
//}
