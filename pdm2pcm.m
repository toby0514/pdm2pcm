clear all;
Fs = 16e3;

%% Read PDM  
fp = fopen('test.txt','r');
formatSpec = '%f';
A = fscanf(fp,formatSpec);

%% CIC filter
cicDecim = dsp.CICDecimator(128,1,3);
cicDecim.FixedPointDataType = 'Minimum section word lengths'; 
info(cicDecim,'long')
nt = numerictype(1,16,15)
[WLs,FLs] = getFixedPointInfo(cicDecim,nt)
B = cicDecim(A);
C(16000,1) = 0;
D(16000,1) = 0;
Mean = mean(B);
for i = 1:length(B)
   C(i) = B(i) - Mean;
end
Max = max(C);
for i = 1:length(C)
    D(i) = C(i)/Max;
end  

%% Bandpass filter
Y = fft(D);
W = bandpass(Y,[20 8000],Fs);
X = ifft(W,'symmetric');

%% Write PCM
%audioFile='testtest.wav';
%dlmwrite ('test.txt',D,'\n');
%audiowrite(audioFile,X,Fs);
