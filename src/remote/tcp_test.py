# #_*_ coding:utf-8 _*_
import socket
import time
import cv2
import numpy
import threading
class Server:
	def __init__(self):
	#客户端连接列表
		self.connList = []
		self.connSock = {}
		#IP地址'0.0.0.0'为等待客户端连接
		self.ServerAddress = ('172.16.101.210', 8002)
		#建立socket对象
		#socket.AF_INET：服务器之间网络通信 
		#socket.SOCK_STREAM：流式socket , for TCP
		self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		#端口可复用
		self.s.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1) 
		#将套接字绑定到地址, 在AF_INET下,以元组（host,port）的形式表示地址.
		self.s.bind(self.ServerAddress)
		#开始监听TCP传入连接。参数指定在拒绝连接之前，操作系统可以挂起的最大连接数量。该值至少为1，大部分应用程序设为5就可以了。
		self.s.listen(5)
	def watchConnecting(self):
		while 1:
			#接受TCP连接并返回（conn,address）,其中conn是新的套接字对象，可以用来接收和发送数据。addr是连接客户端的地址。
			#没有连接则等待有连接
			clientSock, clientAddr = self.s.accept()
			if clientAddr not in self.connList:
				self.connList.append(clientAddr)
				self.connSock[clientAddr] = clientSock
			print('connect from:'+str(clientAddr))
			#创建并启动线程,保持通信
			t = threading.Thread(target = ReceiveVideo,args = (self.connList,self.connSock,self.ServerAddress,self.s,clientAddr,clientSock))
			t.start()
def ReceiveContent(sock, count):
	buf = b''#buf是一个byte类型
        while count:
        #接受TCP套接字的数据。数据以字符串形式返回，count指定要接收的最大数据量.
        	newbuf = sock.recv(count)
                #print newbuf
		if not newbuf: return None
               	buf += newbuf
                count -= len(newbuf)
        return buf

def ForwardVideo(ConnSock,Sock,stringData,length):
	for addr in ConnSock:
		if ConnSock[addr]!= Sock:
			try:
				ConnSock[addr].send(str.encode(str(length)).ljust(16))
				ConnSock[addr].send(stringData)
			except:
				print "Send Error!"

def SplitData(StringData,num):
	return StringData.split('@',int(num))

def ReceiveVideo(ConnList,ConnSock,ServerAddr,ServerSock,ClientAddr,ClientSock):
	Frame_Num = 0
	print str(ClientAddr),' enter function ReceiveVideo'

	

if __name__ == '__main__':
	#while 1:
	ServerKJW =Server()
	ServerKJW.watchConnecting()
