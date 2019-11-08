# #_*_ coding:utf-8 _*_
#此程序是3个相机，向云服务器上发送数据
import socket
import cv2
import numpy
import time
import threading
import re

	
def CreatSock(address):
	try:
		#建立socket对象
		#socket.AF_INET：服务器之间网络通信 
		#流socket , for tcp
		sock = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
		sock.connect(address)
	except socket.error as msg:
		print(msg)
		exit(1)
	return sock

def Frame2Str(frame):
	#压缩参数，后面cv2.imencode将会用到，对于jpeg来说，15代表图像质量，越高代表图像质量越好为 0-100，默认95
	encode_param=[int(cv2.IMWRITE_JPEG_QUALITY),50]
	#cv2.imencode将图片格式转换(编码)成流数据，赋值到内存缓存中;主要用于图像数据格式的压缩，方便网络传输
	#'.jpg'表示将图片按照jpg格式编码。
	result, imgencode = cv2.imencode('.jpg', frame, encode_param)
	#建立矩阵
	data = numpy.array(imgencode)
	#将numpy矩阵转换成字符形式，以便在网络中传输
	stringData = data.tostring()
	return stringData

def cut_text(text,lenth):
	textArr = []
	text_Len = len(text)
	#按照长度length分割字符串text
	group = text_Len//lenth+1
	for i in range(group):
	#textArr = re.findall('.{'+str(lenth)+'}', text)
		if i == group - 1:
			textArr.append(text[i*lenth:])
		else:
			textArr.append(text[i*lenth:(i+1)*lenth])
	return textArr

def Send2Server(sock,stringData,flag,frame_num):
	stringLen = len(stringData)
	#先发送flag@frameNum@Len
	string_f = str(flag)+'@'+str(frame_num)+'@'+str(stringLen)
	sock.send(str.encode(str(string_f).ljust(16)))
	#再发送图像数据
	sock.send(stringData)
	
def SendVideo():
	capture_0 = cv2.VideoCapture(0)
	if(not capture_0.isOpened()):
		return False

	#建立sock连接
	#address要连接的服务器IP地址和端口号
	#address = ('localhost', 8002)
	address = ('47.98.51.23', 7123)
	sock = CreatSock(address)
	Frame_Num = 0
	while True:
		ret_0, frame_0 = capture_0.read()
		if( not ret_0):
			time.sleep(0.01)
			continue
		cv2.imshow("camera",frame_0)
		
		stringData_0 = Frame2Str(frame_0)
		#发送数据
		Send2Server(sock,stringData_0,0,Frame_Num)
		Frame_Num = Frame_Num + 1
		
		
	
		if cv2.waitKey(10) == 27:
			end_flag = -1
			Send2Server(sock,str(end_flag))
			break
		time.sleep(0.03)
		
	sock.close()

if __name__ == '__main__':
	SendVideo()

