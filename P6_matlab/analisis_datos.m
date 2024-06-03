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
  voltaje = readVoltage(a,'A0');

  x=[x,voltaje];
  t_ini=t_ini+1;

end
TiempoTotal=toc;
%% 

% load("datos_lectura.mat");

%plot(x);
%grid on;

Ps = TiempoTotal / N;
ti = 0;
tf = TiempoTotal;

fs=1/Ps; % Frecuencia de muestreo
fmax = fs/2;

t=ti:Ps:tf-Ps; % Vector de tiempo 
%% 

plot(t, x)
xlabel('Tiempo (s)')
ylabel ('Voltaje (V)')
title ('Señales sinusoidales originales')
grid on

%% 

frec=1/(Ps*N)*(0:N);

L=1:floor(N/2);

Y = fft(x,N);
PSD = Y.*conj(Y)/N; % PSD de la señal original


plot(frec(L),PSD(L),'red','Linewidth',1)
xlabel('Frecuencia (Hz)')
ylabel ('PSD (V^2/Hz)')
title ('Espectro de la señal original')
grid on

%% Diseño del fitro paso altas
% queremos que solo pasen las señales mayores a 1 Hz

fcorte4PA = 3;  %8 Hz
% la frecuencia normalizada es
fn4PA = fcorte4PA/fmax;

N_Orden_PA= 2; % escogemos el orden del filtro

[b_num_PA,a_den_PA] = butter(N_Orden_PA,fn4PA,'high');
Se_PasoAltas = filter(b_num_PA,a_den_PA,x);
Y_Se_PasoAltas = fft(Se_PasoAltas, N);
PSD_Se_PasoAltas= Y_Se_PasoAltas.*conj(Y_Se_PasoAltas)/N;
[G_PA,Frec_PA] = freqz(b_num_PA,a_den_PA,N,fs);

plot(t, Se_PasoAltas,'blue','Linewidth',1)
%xlim([1000 2000]);
xlabel('Tiempo (s)')
ylabel('Amplitud (V)')
legend('Señal con filtro paso altas')
grid on;

%% 

plot(frec(L), PSD_Se_PasoAltas(L),'blue','Linewidth',1)
xlabel('Frecuencia (Hz)')
ylabel ('PSD (V^2/Hz)')
legend ('Señales con filtro paso altas')
grid on;