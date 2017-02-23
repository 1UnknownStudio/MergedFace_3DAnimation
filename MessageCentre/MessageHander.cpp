#include"stdafx.h"
#include"MessageHander.h"


C_UDPBase::C_UDPBase(const int port, const string ip_address, UDPTYPE cmd) :Port(port), IPAddress(ip_address)
{
	int i_result = 0;
	//初始化Socket
	i_result = WSAStartup(MAKEWORD(2, 2), &wsaData);
	if (i_result != NO_ERROR)
	{
		throw("socket_init_error");
		exit(-1);
	}
	//创建接收数据报的socket
	Socket_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (Socket_ == INVALID_SOCKET)
	{
		throw("socket_init_error");
		exit(-1);
	}
	//将socket与制定端口和IP绑定
	SocketAddr.sin_family = AF_INET;
	SocketAddr.sin_port = htons(Port);

	if ((IPAddress == "Any") || (IPAddress == "any") || (IPAddress == "ANY"))
		SocketAddr.sin_addr.s_addr = htonl(INADDR_ANY);
	else
		SocketAddr.sin_addr.s_addr = inet_addr(IPAddress.c_str());//127.0.0.1

	if (cmd == UDPRecive)
	{
		i_result = bind(Socket_, (SOCKADDR *)&SocketAddr, sizeof(SocketAddr));
		if (i_result != NO_ERROR)
		{
			throw("socket_init_error");
			exit(-1);
		}
	}
};


C_UDPSandG::C_UDPSandG(const int port, const string ip_address, int buf_data_len, UDPTYPE type) :C_UDPBase(port, ip_address, type), BufLength(buf_data_len)
{
	buffer = new char[buf_data_len];
};