#include <string>
#include <vector>
#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <cstring>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <thread>
#include <mutex>
#include <iostream>

using std::string;

class Server
{
public:
	Server(int port);
	~Server();
	void closeSocket();
	bool init();
	void run();
private:
	bool initSocket();
	void serveThread(const struct sockaddr_in& addr);
private:
	
	string socket_ip_;
	int socket_port_;
	int udp_fd_;
	int tcp_fd_;
	struct sockaddr_in sockaddr_;
	std::vector<struct sockaddr_in> clients_;
	std::vector<std::shared_ptr<std::thread>> thread_handles_;

};

Server::Server(int port)
{
	udp_fd_ = -1;
	tcp_fd_ = -1;
	socket_ip_ = "0.0.0.0";
	socket_port_ = port;
}

Server::~Server()
{
	
}

void Server::closeSocket()
{
	if(udp_fd_ != -1)
		close(udp_fd_);
	if(tcp_fd_ != -1)
		close(tcp_fd_);
}

bool Server::initSocket()
{
	bzero(&sockaddr_,sizeof(sockaddr_));//init 0

	sockaddr_.sin_port = htons(socket_port_);
	sockaddr_.sin_family = AF_INET;
	int convert_ret = inet_pton(AF_INET, socket_ip_.c_str(), &sockaddr_.sin_addr);
	if(convert_ret !=1)
	{
		perror("convert socket ip failed, please check the format!");
		return false;
	}
	
	//UDP
	udp_fd_ = socket(PF_INET,SOCK_DGRAM , 0);
	if(udp_fd_ < 0)
	{
		perror("build socket error");
		return false;
	}
	int udp_opt = 1;
	setsockopt(udp_fd_, SOL_SOCKET, SO_REUSEADDR, &udp_opt, sizeof(udp_opt));
	
	int ret = bind(udp_fd_, (struct sockaddr*)&sockaddr_,sizeof(sockaddr_));
	if(ret < 0)
	{
		std::cout << "udp bind ip: "<< socket_ip_ 
				  << ",port: "<< socket_port_ << "failed!!" << std:: endl;
		return false;
	}
	
	//TCP
//	tcp_fd_ = socket(PF_INET,SOCK_STREAM , 0);
//	int recv_len = 4096;
//	int tcp_opt = 1;
//	setsockopt( tcp_fd_, SOL_SOCKET, SO_RCVBUF, &recv_len, sizeof(recv_len));
//	setsockopt(tcp_fd_, SOL_SOCKET, SO_REUSEADDR, &tcp_opt, sizeof(tcp_opt));
//	int ret1 = bind(tcp_fd_, (struct sockaddr*)&sockaddr_,sizeof(sockaddr_));
//	if(ret1 < 0)
//	{
//		std::cout << "tcp bind ip: "<< socket_ip_ 
//				  << ",port: "<< socket_port_ << "failed!!" << std:: endl;
//		return false;
//	}
	
	return true;
}


bool Server::init()
{
	if(!initSocket())
		return false;

	std::cout << "port: " << socket_port_ << "  init ok." << std:: endl;
	return true;
}


void Server::run()
{
	const int BufLen = 100000;
	uint8_t *recvbuf = new uint8_t [BufLen+1];
	char msg_type[6] = "image"; msg_type[5] = '\0';
	
	struct sockaddr_in client_addr;
	socklen_t clientLen = sizeof(client_addr);
	
	struct sockaddr_in move_addr, static_addr;
	bool move_addr_empty = true;
	bool static_addr_empty = true;
	
	while(1)
	{
		//udp
		int len = recvfrom(udp_fd_, recvbuf, BufLen,0,(struct sockaddr*)&client_addr, &clientLen);
		//std::cout << "len: " <<len << std::endl;
		if(len < 5)
			continue;
		
		memcpy(msg_type,recvbuf,5);
		const std::string type(msg_type);
		
		if(type == "fixed")
		{
			char answer[] = "fixedok";
			sendto(udp_fd_, answer, sizeof(answer),0, 
						 (struct sockaddr*)&client_addr, sizeof(client_addr));
			static_addr = client_addr;
			static_addr_empty = false;
			std::cout << "static client connect ok.\n"; 
		}
		else if(type == "move0")
		{
			char answer[] = "move0ok";
			sendto(udp_fd_, answer, sizeof(answer),0, 
						 (struct sockaddr*)&client_addr, sizeof(client_addr));
			move_addr = client_addr;
			move_addr_empty = false;
			std::cout << "move client connect ok.\n"; 
		}
		else if(type == "joy00")
		{
			if(!static_addr_empty && !move_addr_empty) //retransmit
			{
				sendto(udp_fd_, recvbuf, len,0, 
						 (struct sockaddr*)&move_addr, sizeof(move_addr));
			}
		}
		else if(len > 1000) //image
		{
			if(!static_addr_empty && !move_addr_empty) //retransmit
			{
				sendto(udp_fd_, recvbuf, len,0, 
						 (struct sockaddr*)&static_addr, sizeof(static_addr));
			}
			
		}
		
//		std::shared_ptr<std::thread> thread_ptr = std::shared_ptr<std::thread>
//			(new std::thread(std::bind(&Server::serveThread,this,client_addr)));
//		thread_handles_.push_back(thread_ptr);
	}
	
	delete [] recvbuf;
}

void Server::serveThread(const struct sockaddr_in& addr)
{
	bool flag = true;
	uint8_t * buf = new uint8_t[10000];
	socklen_t clientLen = sizeof(addr);
	while(flag)
	{
		//UDP
		recvfrom(udp_fd_, buf, 20,0,(struct sockaddr*)&addr, &clientLen);
		printf("%s\r\n",buf);
	}
	delete [] buf;
}


int main(int argc,char** argv)
{
	int port = 8617;
	if(argc > 1)
		port = atoi(argv[1]);
	Server server(port);
	if(!server.init())
		return 0;
	server.run();
	server.closeSocket();
	return 0;
}


