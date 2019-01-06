function callback(s, BytesAvailable,p)  
      
    global t;  
    global x;  
    global m;  
    global ii;  
  
    out = fscanf(s);     %以字符串类型读数据
    data = str2num(out)     %转化为数字类型显示
      
    t = [t ii];  
    m = [m data];  
    set(p, 'XData',t,'YData',m(1,:));    %定义XY的坐标值
      
    drawnow  
    x = x + 1;  
    axis([x-100 x+100 -1 3.6]);          %移动坐标绘图
    ii=ii+1;  
end  

