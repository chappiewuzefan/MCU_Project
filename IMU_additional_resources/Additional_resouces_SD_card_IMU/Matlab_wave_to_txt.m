close all;clear;clc;

source_file = 'piano_1ch_long.wav';
[source_sig, Fs] = audioread(source_file, 'native');
%[source_sig, Fs] = audioread(source_file);
%%% down sample a audio signal
Fs_new = 8000;
[Numer, Denom] = rat(Fs_new/Fs);
sig_new = resample(double(source_sig), Numer, Denom);

writematrix(int16(sig_new(5000:15000)), "data_8k.txt");
%sound(sig_new (2000:48000), Fs_new);   %%% This can be only uncommented if you donot use 'native'


%% read data from txt


%samples = readmatrix('data_string.txt');
samples = readmatrix('data_8k.txt');
hold on 
subplot(2,1,1);
plot(samples);
audio_samples= (samples/32767);
subplot(2,1,2);
plot(audio_samples);
sound(audio_samples, Fs_new);
finish=1;