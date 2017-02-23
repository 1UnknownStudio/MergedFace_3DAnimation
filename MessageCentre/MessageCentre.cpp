// MessageCentre.cpp : �������̨Ӧ�ó������ڵ㡣
//

#include "stdafx.h"
#include"MessageCentre.h"
#include <vector>
using namespace std;

C_UDPSandG SGUDP(8012, "any", 1024, UDPRecive);//��Ϊ�ͻ��� �����κ�IP��ַ ����+����
C_UDPSandG SGUDP2unity(8010, "127.0.0.1", 1024, UDPSend);//�������ݵ�unity ����+����
int TimerID = 0;

int main(int argc, _TCHAR* argv[])
{
	const int THREAD_NUM = 5;
	HANDLE handle[THREAD_NUM];
	handle[0] = (HANDLE)_beginthreadex(NULL, 0, ThreadA, NULL, 0, NULL);
	//WaitForSingleobject(hand, INFINITE);//�ȴ��߳�����Ժ����ߺ���ĳ���INFINITE�������޵ȴ���
	
	TimerID = SetTimer(NULL, 1, 33, TimerProc);//������ʱ����33ms(30FPS)����һ�����ݵ�unity

	MSG msg;
	while (GetMessage(&msg, NULL, 0, 0))
	{
		if (msg.message == WM_TIMER)
		{
			DispatchMessage(&msg);
		}
	}

	while (1);

	return 0;
}

bool Locker = false;//��Դ�� false����û��ʹ��
bool IsDataValid = false;//�����յ����ݺ����Ч
unsigned int __stdcall ThreadA(PVOID pM)//һֱ���չ������������ݣ����Ǵ�һ�������з�����
{
	
	while (1)
	{
		int reNum = SGUDP.ReciveData(SGUDP.buffer, SGUDP.BufLength);//UDP�յ�������
		if (reNum != 0)
		{
			if (Locker == false)//��δ���ڴ���
			{
				memcpy(SGUDP2unity.buffer, SGUDP.buffer, SGUDP.BufLength);//��������������Socket !!!!ǰ��bufferһ��Ҫ���ڵ��ں��ߣ������ڴ�й©
				IsDataValid = true;
			}
		}
	}
}


void CALLBACK TimerProc(HWND   hWnd, UINT   nMsg, UINT   nTimerid, DWORD   dwTime)
{
	if (IsDataValid == true)
	{
		if (nTimerid == TimerID)//����������unity
		{
			Locker = true;
			//string test = "27814962179063720164712214708217-32-1749072189734892013";
			//SGUDP2unity.SendData(&(test.at(0)), test.size() + 1);
			//Joint joints[JointType_Count];
			//memcpy(joints, SGUDP2unity.buffer, SGUDP2unity.BufLength);
			SGUDP2unity.SendData(SGUDP2unity.buffer, SGUDP2unity.BufLength);
			Locker = false;
		}
	}
}