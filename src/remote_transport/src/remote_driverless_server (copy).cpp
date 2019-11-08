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
	Server();
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

Server::Server()
{
	udp_fd_ = -1;
	tcp_fd_ = -1;
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


bool Server::init()
{
	socket_ip_ = "0.0.0.0";
	socket_port_ = 8008;
	

	if(!initSocket())
		return false;

	std::cout << "port: " << socket_port_ << "  init ok." << std:: endl;
	return true;
}

void Server::run()
{
	char recvbuf[100];
	while(1)
	{
		std::cout << "client num: " <<clients_.size() << " listening..." << std::endl;
		int ret = listen(tcp_fd_, 5);
		if(ret < 0)
		{
			usleep(500000);
			continue;
		}
		
		//tcp
//		struct sockaddr_in client_addr;
//		socklen_t clientLen = sizeof(client_addr);
//		int connfd = accept(tcp_fd_, (struct sockaddr*)&client_addr, &clientLen);
//		
//		if(connfd < 0)
//		{
//			perror("accept error");
//			continue;
//		}
		
		//udp
		struct sockaddr_in client_addr;
		socklen_t clientLen = sizeof(client_addr);
		int len = recvfrom(fd, recvbuf, 99,0,(struct sockaddr*)&client_addr, &clientLen);
		
		clients_.push_back(client_addr);
		
		std::shared_ptr<std::thread> thread_ptr = std::shared_ptr<std::thread>
			(new std::thread(std::bind(&Server::serveThread,this,client_addr)));
		thread_handles_.push_back(thread_ptr);
	}
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

int main(int argc,char** argv)
{
	Server server;
	if(!server.init())
		return 0;
	server.run();
	server.closeSocket();
	return 0;
}


