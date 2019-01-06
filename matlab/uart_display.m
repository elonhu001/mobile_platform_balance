clc;
clear;
s1=serial('COM3','BaudRate',9600,'Parity','none','DataBits',8,'StopBits',1);
s1.InputBufferSize=4096;
s1.OutputBufferSize=4096;
fopen(s1);