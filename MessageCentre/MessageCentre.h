#include"MessageHander.h"
#include<process.h>
#include<iostream>

unsigned int __stdcall ThreadA(PVOID pM);//�߳� ����һֱ���չ������������ݣ����Ǵ�һ�������з�����
void CALLBACK TimerProc(HWND   hWnd, UINT   nMsg, UINT   nTimerid, DWORD   dwTime);//��ʱ���ص�����