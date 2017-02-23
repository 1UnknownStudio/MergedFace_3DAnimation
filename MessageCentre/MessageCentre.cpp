// MessageCentre.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include"MessageCentre.h"
#include <vector>
using namespace std;

C_UDPSandG SGUDP(8012, "any", 1024, UDPRecive);//作为客户端 接收任何IP地址 手势+骨骼
C_UDPSandG SGUDP2unity(8010, "127.0.0.1", 1024, UDPSend);//传输数据到unity 手势+骨骼
int TimerID = 0;

int main(int argc, _TCHAR* argv[])
{
	const int THREAD_NUM = 5;
	HANDLE handle[THREAD_NUM];
	handle[0] = (HANDLE)_beginthreadex(NULL, 0, ThreadA, NULL, 0, NULL);
	//WaitForSingleobject(hand, INFINITE);//等待线程完成以后再走后面的程序，INFINITE就是无限等待。
	
	TimerID = SetTimer(NULL, 1, 33, TimerProc);//开启定时器，33ms(30FPS)传输一次数据到unity

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

bool Locker = false;//资源锁 false代表没人使用
bool IsDataValid = false;//初次收到数据后才有效
unsigned int __stdcall ThreadA(PVOID pM)//一直接收骨骼，手势数据（他们从一个程序中发出）
{
	
	while (1)
	{
		int reNum = SGUDP.ReciveData(SGUDP.buffer, SGUDP.BufLength);//UDP收到数据了
		if (reNum != 0)
		{
			if (Locker == false)//并未正在传输
			{
				memcpy(SGUDP2unity.buffer, SGUDP.buffer, SGUDP.BufLength);//拷贝数据至传输Socket !!!!前者buffer一定要大于等于后者，否则内存泄漏
				IsDataValid = true;
			}
		}
	}
}


void CALLBACK TimerProc(HWND   hWnd, UINT   nMsg, UINT   nTimerid, DWORD   dwTime)
{
	if (IsDataValid == true)
	{
		if (nTimerid == TimerID)//传输数据至unity
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