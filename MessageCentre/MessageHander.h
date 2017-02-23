#pragma once
#include<string.h>
#include<string>
#include<WinSock2.h>
#pragma comment(lib,"WS2_32.lib")
using namespace std;

enum UDPTYPE{UDPSend,UDPRecive};

class C_UDPBase
{
private:
	const int Port;
	const string IPAddress;
	WSADATA wsaData;//初始化
	SOCKET Socket_;
	sockaddr_in SocketAddr;//服务器地址

public:
	sockaddr_in SenderAddr;//发送端信息，PS 只有接收后才有效
	int SenderAddrSize = sizeof(SenderAddr);

	C_UDPBase(const int port, const string ip_address, UDPTYPE cmd);//"Any" 代表任何IP地址
	virtual ~C_UDPBase(){ closesocket(Socket_); WSACleanup(); };

	inline int ReciveData(char * buffer, int buffLen)
	{
		int RecBytesNum = recvfrom(Socket_, buffer, buffLen, 0, (SOCKADDR *)&SenderAddr, &SenderAddrSize);
		return RecBytesNum;//收到的字节数
	}

	inline void SendData(char * buffer, int buffLen)
	{
		sendto(Socket_, buffer, buffLen, 0, (SOCKADDR *)&SocketAddr, sizeof(SocketAddr));
	}
};

class C_UDPSandG :public C_UDPBase//skeleton & gesture
{
public:
	char* buffer;
	int BufLength;
	C_UDPSandG(const int port, const string ip_address, int buf_data_len, UDPTYPE type);
	~C_UDPSandG(){ delete[]buffer; buffer = NULL; };
	void ChangeBufLen(int len){ delete[]buffer; buffer = NULL; buffer = new char[len]; BufLength = len; }
};