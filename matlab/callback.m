function callback(s, BytesAvailable,p)  
      
    global t;  
    global x;  
    global m;  
    global ii;  
  
    out = fscanf(s);     %���ַ������Ͷ�����
    data = str2num(out)     %ת��Ϊ����������ʾ
      
    t = [t ii];  
    m = [m data];  
    set(p, 'XData',t,'YData',m(1,:));    %����XY������ֵ
      
    drawnow  
    x = x + 1;  
    axis([x-100 x+100 -1 3.6]);          %�ƶ������ͼ
    ii=ii+1;  
end  

