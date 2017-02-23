#include"MessageHander.h"
#include<process.h>
#include<iostream>

unsigned int __stdcall ThreadA(PVOID pM);//线程 函数一直接收骨骼，手势数据（他们从一个程序中发出）
void CALLBACK TimerProc(HWND   hWnd, UINT   nMsg, UINT   nTimerid, DWORD   dwTime);//定时器回调函数