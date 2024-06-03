clear all
close all
clc
%instrreset % reset todos los puertos serie de la PC


a = arduino('/dev/ttyACM0','UNO');
N = 500; %numero de lecturas
t_ini = 1; %tiempo inicial
x=0;
tic
while (t_ini<N) %ciclo para tomar N lecturas
  voltaje = readVoltage(a,'A1');
  %toma la lectura del canal A1 de arduino, el resultado es el voltaje
  %directamente calculado por:
  % voltaje = (lectura*5)/1023;
  
  %esto es para graficar en "tiempo-real"
  x=[x,voltaje];
  plot(x)
  ylabel("Voltaje")
  xlabel("Mediciones")
  title("Voltaje leido con un Arduino en tiempo real")
  grid on
  t_ini=t_ini+1; %cada ciclo se incrementa la variable de conteo
  drawnow
end
TiempoTotal=toc;

