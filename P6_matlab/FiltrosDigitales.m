clear all
close all
clc

%%

% Creación de las señales sinusoidales

% Una señal en general requiere de una amplitud (A), una frecuencia (f) y
% un intervalo de tiempo (t) en el que está activa. Vamos a generar tres
% señales, S1, S2 y S3.

% Comenzamos definiendo el intervalo de tiempo (t) a partir de un tiempo
% inicial (ti), un tiempo final (tf) y un periodo de separación entre
% muestras llamado periodo de muestreo (Ps)

ti=0; % Tiempo inicial de adquisición en segundos
tf=5; % Tiempo final de adquisición en segundos 
Ps=1e-3;% Periodo de muestreo en segundos
t=ti:Ps:tf-Ps; % Vector de tiempo 
N=length(t);


% Ahora definimos a las señales S1, S2 y S3

% Para la señal S1
A1= 5;   % Amplitud 10V Pico-Pico
f1 = 1; % Frecuencia de la señal S1 1 Hz
S1 = A1*sin(2*pi*f1*t); % Señal sinusoidal S1

% Para la señal S2
A2= 4;   % Amplitud 8V Pico-Pico
f2 = 6; % Frecuencia de la señal S2 60 Hz
S2 = A2*sin(2*pi*f2*t); % Señal sinusoidal S2

% Para la señal S3
A3= 3;   % Amplitud 6V Pico-Pico
f3 = 12; % Frecuencia de la señal S3 1500 Hz
S3 = A3*sin(2*pi*f3*t); % Señal sinusoidal S3

% Gráficas temporales de las señales S1, S2 y S3
figure
hold on
plot(t,S1,'red','Linewidth',1)
plot(t,S2,'blue','Linewidth',1)
plot(t,S3,'black','Linewidth',1)
ylim([-6 6])
xlabel('Tiempo (s)')
ylabel ('Voltaje (V)')
title ('Señales sinusoidales originales')
legend ('Señal S1','Señal S2','Señal S3')

%%
% Ahora comprobemos que nuestras señales S1, S2 y S3 posean las frecuencias
% que previamente definimos. Para esto, tendremos que calcular la
% transformada rápida de Fourier (fft), la densidad de potencia espectral
% (PSD) y un vector de frecuencias.

% Comencemos con el vector de frecuencias, este se obtiene del vector de
% tiempo t y se calcula de la siguiente manera:

frec=1/(Ps*N)*(0:N); % Definimos un vector de frecuencias sobre las cuales estaríamos trabajando

% Un detalle de la transformada de Fourier es que es simétrica,´por lo que
% sólo requerimos graficar la mitad de información. Por esto crearemos un
% limitante para graficar sólo la mitad de los datos (L)

L=1:floor(N/2);


% Para la señal S1
Y_S1 = fft(S1, N); % Transformada de fourier de la señal S1
PSD_S1 = Y_S1.*conj(Y_S1)/N; % PSD de la señal S1

% Para la señal S2
Y_S2 = fft(S2, N); % Transformada de fourier de la señal S2
PSD_S2 = Y_S2.*conj(Y_S2)/N; % PSD de la señal S2

% Para la señal S3
Y_S3 = fft(S3, N); % Transformada de fourier de la señal S3
PSD_S3 = Y_S3.*conj(Y_S3)/N; % PSD de la señal S3



% Gráficas en frecuencia de las señales S1, S2 y S3
figure
hold on
plot(frec(L),PSD_S1(L),'red','Linewidth',1)
plot(frec(L),PSD_S2(L),'blue','Linewidth',1)
plot(frec(L),PSD_S3(L),'black','Linewidth',1)
xlim([-1 20])
xlabel('Frecuencia (Hz)')
ylabel ('PSD (V^2/Hz)')
title ('Espectro de las señales originales')
legend ('Señal S1','Señal S2','Señal S3')


%% Ahora pasemos a un caso real en el que se tengan múltiples señales y ruido de fondo
% para esto sumaremos a S1, S2 y S3.

Se_123= S1 + S2 + S3; % Suma de las señales generadas
Ruido = 20*randn(size(t)); % Vector de ruido con el mismo tamaño que el intervalo de tiempo
SeConRuido = Se_123 + Ruido; % Señal contaminada 

% Calculemos la PSD de nuestra señal para demostrar que aún en el ruido se
% puede localizar nuestra señal 

Y_Se_123 = fft(Se_123, N); % Transformada de fourier de la señal S3
PSD_Se_123 = Y_Se_123.*conj(Y_Se_123)/N; % PSD de la señal S3

Y_SeConRuido = fft(SeConRuido, N); % Transformada de fourier de la señal S3
PSD_SeConRuido = Y_SeConRuido.*conj(Y_SeConRuido)/N; % PSD de la señal S3



figure
hold on
plot(frec(L),PSD_Se_123(L),'blue','Linewidth',1)
plot(frec(L),PSD_SeConRuido(L),'red','Linewidth',1)
xlim([-1 20])
xlabel('Frecuencia (Hz)')
ylabel ('PSD (V^2/Hz)')
title ('Comparación de espectros')
legend ('Señales sumadas','Señales sumadas con ruido')


figure
hold on
plot(t,Se_123,'red','Linewidth',1)
plot(t,SeConRuido,'blue','Linewidth',0.3)
%ylim([-6 6])
xlabel('Tiempo (s)')
ylabel ('Voltaje (V)')
title ('Comparación de las señales')
legend ('Señales sumadas','Señales sumadas con ruido')



%% Ahora implementemos el primer tipo de filtro denominado "Filtro espectral de amplitudes"
% La idea es simple, dejar pasar sólo a los picos más grandes del espectro


Amplitudes = PSD_SeConRuido > 4000; % Se define un vector de amplitudes que guarde como unos (1) a
% los índices de la PSD con ruido que posean una amplitud mayor a 2000
PSDLimpia = PSD_SeConRuido.*Amplitudes; % Se multiplica el vector de Amplitudes por la PSD con ruido y se obtiene el filtro

Y_SeSinRuido = Amplitudes.*Y_SeConRuido; % Transformada de Fourier limpia
SeSinRuido = ifft(Y_SeSinRuido); %REGRESO AL TIEMPO


figure 
subplot(3,1,1);
plot(t, Se_123)
xlabel('Tiempo (s)')
ylabel('Amplitud (V)')
legend('Suma de señales S1+S2+S3')
subplot(3,1,2); 
plot(t, SeConRuido)
%xlim([1000 2000]);
xlabel('Tiempo (s)')
ylabel('Amplitud (V)')
legend('Señal contaminada')
subplot(3,1,3); 
plot(t, SeSinRuido)
%xlim([0 20]);
xlabel('Tiempo (s)')
ylabel('Amplitud (V)')
legend('Señal filtrada')


figure
hold on
plot(t,Se_123,'red','Linewidth',1)
plot(t,SeSinRuido,'blue','Linewidth',0.3)
%ylim([-6 6])
xlabel('Tiempo (s)')
ylabel ('Voltaje (V)')
title ('Comparación de las señales')
legend ('Suma de señales S1+S2+S3','Señal filtrada')

%% Diseño del fitro suprime banda y su diagrama de Bode
% IMPORTANTE]= Las frecuencias de corte deben de estar 
% normalizadas: fn1=fcorte1/fmax y fn2=fcorte2/fmax

% Definimos la frecuencia de corte baja y alta, 
% queremos que se rechace la frecuencia de S2

fcorte1SB = 4;  %4 Hz Primera frecuencia de corte suprime bandas
fcorte2SB = 8;  %8 Hz Segunda frecuencia de corte suprime bandas

% definimos la frecuencia de muestrreo fs=1/ts
fs=1/Ps;
% definimos la frecuencia maxima dada por
fmax = fs/2;

% Las frecuencias normalizadas son:
fn1SB = fcorte1SB/fmax;
fn2SB = fcorte2SB/fmax;

% El orden del filtro suprime bandas
N_Orden_SB= 2;

% Consideremos un filtro con aproximación Butterworth
% la funcion butter devuelve los coeficientes de los polinomios de la
% función del transferencia del filtro

[b_numSB,a_denSB] = butter(N_Orden_SB ,[fn1SB fn2SB],'stop'); % 'stop': rechaza banda
%[b_numSB,a_denSB] = butter(N_Orden_SB ,[fn1SB fn2SB],'bandpass'); % 'stop': rechaza banda

%otras opciones 
% 'high': pasa altas
%'low' indica pasa bajas
% 'bandpass': pasa banda


% Aplicamos el filtro a la señal 
Se_SuprimeBanda = filter(b_numSB,a_denSB,SeSinRuido);
% Se_SuprimeBanda = filtfilt(b_numSB,a_denSB,SeSinRuido); %Te da la gráfica sin desface
% la finción filter aplica el filtro con coeficientes b_num y a_den a la 
% señal de entrada SeSinRuido

Y_Se_SuprimeBanda = fft(Se_SuprimeBanda, N); % Transformada de fourier de la señal S3
PSD_Se_SuprimeBanda = Y_Se_SuprimeBanda.*conj(Y_Se_SuprimeBanda)/N; % PSD de la señal S3


% Respuesta en magnitud del filtro rechaza banda
%Obtenemos la respuesta en frecuencia del filtro con la función freqz 
% G guarda el valor de la función de transferencia 
% frec guarda el vector de frecuencias
[G_SB,Frec_SB] = freqz(b_numSB,a_denSB,N,fs);




figure 
subplot(4,1,1);
plot(t, SeSinRuido,'red','Linewidth',1)
xlabel('Tiempo (s)')
ylabel('Amplitud (V)')
legend('Señal sin ruido')
subplot(4,1,2);
plot(frec(L), PSDLimpia(L),'red','Linewidth',1)
xlim([0 20]);
xlabel('Frecuencia (Hz)')
ylabel ('PSD (V^2/Hz)')
legend ('Señal sin ruido')
subplot(4,1,3); 
plot(t, Se_SuprimeBanda,'blue','Linewidth',1)
%xlim([1000 2000]);
xlabel('Tiempo (s)')
ylabel('Amplitud (V)')
legend('Señal con banda suprimida')
subplot(4,1,4);
plot(frec(L), PSD_Se_SuprimeBanda(L),'blue','Linewidth',1)
xlim([0 20]);
xlabel('Frecuencia (Hz)')
ylabel ('PSD (V^2/Hz)')
legend ('Señales con banda suprimida')


figure %Diagrama de Bode
semilogx(Frec_SB,abs(G_SB),'k','LineWidth',1.5)
%semilogx(frec,db(G),'k','LineWidth',1.5) %en decibeles
ylim([-0.1 1.1]);
xlabel('Frecuencia (Hz)')
ylabel('Magnitud |G(j\omega)|')
title ('Diagrama de Bode del filtro rechaza banda')


%% Diseño del fitro pasa bajas
% definimos la frecuencia de corte normalizada fn3PB=fcorte3PB/fmax
% queremos que solo pasen las señales menores a 8 Hz

fcorte3PB = 8;  %8 Hz
% la frecuencia normalizada es
fn3PB = fcorte3PB/fmax;

N_Orden_PB= 8; % escogemos el orden del filtro


% Definimos un filtro con aproximación Butterworth
% la funcion butter devuelve los coeficientes de los polinomios de la
% función del transferencia del filtro

[b_num_PB,a_den_PB] = butter(N_Orden_PB,fn3PB,'low'); %'low' indica pasa bajas
%otras opciones 
% 'high': pasa altas
%'low' indica pasa bajas
% 'bandpass': pasa banda

% Aplicamos el filtro a la señal Vi y tendremos como resultado Vo
Se_PasoBajas = filter(b_num_PB,a_den_PB,SeSinRuido);
% Se_PasoBajas = filtfilt(b_num_PB,a_den_PB,SeSinRuido); %Te da la gráfica sin desface
% la finción filter aplica el filtro con coeficientes b_num y a_den a la 
% señal de entrada Vi


Y_Se_PasoBajas = fft(Se_PasoBajas, N); % Transformada de fourier de la señal S3
PSD_Se_PasoBajas = Y_Se_PasoBajas.*conj(Y_Se_PasoBajas)/N; % PSD de la señal S3



% Respuesta en magnitud del filtro paso bajas
%Obtenemos la respuesta en frecuencia del filtro con la 
% función freqz 
% G_PB guarda el valor de la función de transferencia 
% Frec_PB guarda el vector de frecuencias
[G_PB,Frec_PB] = freqz(b_num_PB,a_den_PB,N,fs);




figure 
subplot(4,1,1);
plot(t, SeSinRuido,'red','Linewidth',1)
xlabel('Tiempo (s)')
ylabel('Amplitud (V)')
legend('Señal sin ruido')
subplot(4,1,2);
plot(frec(L), PSDLimpia(L),'red','Linewidth',1)
xlim([0 20]);
xlabel('Frecuencia (Hz)')
ylabel ('PSD (V^2/Hz)')
legend ('Señal sin ruido')
subplot(4,1,3); 
plot(t, Se_PasoBajas,'blue','Linewidth',1)
%xlim([1000 2000]);
xlabel('Tiempo (s)')
ylabel('Amplitud (V)')
legend('Señal con filtro paso bajas')
subplot(4,1,4);
plot(frec(L), PSD_Se_PasoBajas(L),'blue','Linewidth',1)
xlim([0 20]);
xlabel('Frecuencia (Hz)')
ylabel ('PSD (V^2/Hz)')
legend ('Señales con filtro paso bajas')


figure %Diagrama de Bode
semilogx(Frec_PB,abs(G_PB),'k','LineWidth',1.5)
%semilogx(Frec_PB,db(G_PB),'k','LineWidth',1.5)
ylim([-0.1 1.1]);
xlabel('Frecuencia (Hz)')
ylabel('Magnitud |G(j\omega)|')
title ('Diagrama de Bode del filtro paso bajas')


%% Diseño del fitro paso altas
% definimos la frecuencia de corte normalizada fn3PB=fcorte3PB/fmax
% queremos que solo pasen las señales mayores a 8 Hz

fcorte4PA = 8;  %8 Hz
% la frecuencia normalizada es
fn4PA = fcorte4PA/fmax;

N_Orden_PA= 2; % escogemos el orden del filtro


% Definimos un filtro con aproximación Butterworth
% la funcion butter devuelve los coeficientes de los polinomios de la
% función del transferencia del filtro

[b_num_PA,a_den_PA] = butter(N_Orden_PA,fn4PA,'high'); %'low' indica pasa bajas
%otras opciones 
% 'high': pasa altas
%'low' indica pasa bajas
% 'bandpass': pasa banda


Se_PasoAltas = filter(b_num_PA,a_den_PA,SeSinRuido);
% Se_PasoAltas = filtfilt(b_num_PA,a_den_PA,SeSinRuido); %Te da la gráfica sin desface
% la finción filter aplica el filtro con coeficientes b_num_PA y a_den_PA a la 
% señal de entrada SeSinRuido


Y_Se_PasoAltas = fft(Se_PasoAltas, N); % Transformada de fourier de la señal S3
PSD_Se_PasoAltas= Y_Se_PasoAltas.*conj(Y_Se_PasoAltas)/N; % PSD de la señal S3



% Respuesta en magnitud del filtro paso bajas
%Obtenemos la respuesta en frecuencia del filtro con la 
% función freqz 
% G_PB guarda el valor de la función de transferencia 
% Frec_PB guarda el vector de frecuencias
[G_PA,Frec_PA] = freqz(b_num_PA,a_den_PA,N,fs);




figure 
subplot(4,1,1);
plot(t, SeSinRuido,'red','Linewidth',1)
xlabel('Tiempo (s)')
ylabel('Amplitud (V)')
legend('Señal sin ruido')
subplot(4,1,2);
plot(frec(L), PSDLimpia(L),'red','Linewidth',1)
xlim([0 20]);
xlabel('Frecuencia (Hz)')
ylabel ('PSD (V^2/Hz)')
legend ('Señal sin ruido')
subplot(4,1,3); 
plot(t, Se_PasoAltas,'blue','Linewidth',1)
%xlim([1000 2000]);
xlabel('Tiempo (s)')
ylabel('Amplitud (V)')
legend('Señal con filtro paso altas')
subplot(4,1,4);
plot(frec(L), PSD_Se_PasoAltas(L),'blue','Linewidth',1)
xlim([0 20]);
xlabel('Frecuencia (Hz)')
ylabel ('PSD (V^2/Hz)')
legend ('Señales con filtro paso altas')


figure %Diagrama de Bode
semilogx(Frec_PA,abs(G_PA),'k','LineWidth',1.5)
%semilogx(Frec_PB,db(G_PB),'k','LineWidth',1.5)
ylim([-0.1 1.1]);
xlabel('Frecuencia (Hz)')
ylabel('Magnitud |G(j\omega)|')
title ('Diagrama de Bode del filtro paso altas')



