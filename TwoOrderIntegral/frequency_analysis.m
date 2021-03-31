function [] = frequency_analysis(data,dt)

aaa = fft(data); 
%将横坐标转化，显示为频率f= n*(fs/N)
f = (0:length(aaa)-1)/dt/length(aaa);
figure;
plot(f,abs(aaa));
title('Magnitude');
end

