# #_*_ coding:utf-8 _*_
#此程序是3个相机，从云服务器上接受数据
import socket
import cv2
import numpy
import time
import threading

def SplitData(StringData,num):
	return StringData.split('@',int(num))

def decode2img(stringData):
	print len(stringData)
	data = numpy.frombuffer(stringData, numpy.uint8)#将获取到的字符流数据转>换成1维数组
	decimg=cv2.imdecode(data,cv2.IMREAD_COLOR)#将数组解码成图像
	return decimg


def recvall(sock, count):
	buf = b''#buf是一个byte类型
	while count:
		#接受TCP套接字的数据。数据以字符串形式返回，count指定要接收的最>大数据量.
		newbuf = sock.recv(count)
		if not newbuf: return None
		buf += newbuf
		count -= len(newbuf)
	return buf

def GetData(sock,count):
	string_f = recvall(sock,16)
	string_split = SplitData(string_f,2)
	cam_type = string_split[0]
	frame_num = string_split[1]
	frame_len = string_split[2]
	return cam_type,frame_num,frame_len

def ReceiveVideo():
	#address要连接的服务器IP地址和端口号
	address = ('47.98.51.23', 8002)
	#建立socket对象
	#socket.AF_INET：服务器之间网络通信 
	#socket.SOCK_STREAM：流式socket , for TCP
	sock = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
	#开启连接
	sock.connect(address)
	while 1:
		cam_0_data = ''
		cam_type,frame_num,frame_len = GetData(sock,16)
		if int(cam_type) ==0:
			cam_0_data = recvall(sock,int(frame_len))

			decimg_0 = decode2img(cam_0_data)
			cv2.imshow('0Receive from Server',decimg_0)#显示图像
	

		if cv2.waitKey(10) == 27:
			end_flag = -1
			print end_flag
			string_f = str('0')+'@'+str('0')+'@'+str(end_flag)
			sock.send(str.encode(str(float(string_f)).ljust(16)))
			break
	end_flag = -1
	sock.send(str.encode(str(float(end_flag)).ljust(16)))
	sock.close()
	cv2.destroyAllWindows()
		
if __name__ == '__main__':
	ReceiveVideo()




