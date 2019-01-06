clc;  
  
global t;  
global x;  
global m;  
global ii;  
  
t = [0];  
m = [0];  
ii = 0;  
x = 0;  
p = plot(t,m,'EraseMode','background','MarkerSize',5);  
axis([x-100 x+100 -1 3.6]);                                %��ʼ����ͼ���
grid on;                                                   %������ʾ
  
%%  
 
s = instrfind('Type', 'serial', 'Port', 'COM4', 'Tag', ''); %��ѯ��⴮��com3
% Create the serial port object if it does not exist
% otherwise use the object that was found.
if isempty(s)
    s = serial('COM4');
else
    fclose(s);
    s = s(1)
end
set(s,'BaudRate', 115200,'DataBits',8,'StopBits',1,'Parity','none','FlowControl','none');  
s.BytesAvailableFcnMode = 'terminator';   % ���ݻس���\n������ص�����
s.BytesAvailableFcn = {@callback,p};       %���ûص�����

fopen(s);  
  
pause ;  
fclose(s);  
% delete(s); 
delete(instrfindall);  
clear s  
close all;  
clear all; 
