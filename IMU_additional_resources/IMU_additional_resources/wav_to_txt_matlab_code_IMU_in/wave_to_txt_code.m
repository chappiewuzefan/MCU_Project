clc;
clear;
close all;
%% Read Music file
source_file = 'music.wav';
[source_sig, Fs] = audioread(source_file, 'native');

%% Down-sample an audio signal
Fs_new = 8000;
[Numer, Denom] = rat(Fs_new/Fs);
sig_new = resample(double(source_sig), Numer, Denom);

%% Write to .txt file
writematrix(int16(sig_new(5000:15000)), "music_data_8k.txt");
%sound(sig_new (2000:48000), Fs_new);   %%% This can be only uncommented if you donot use 'native'


%% Read data from .txt file
samples = readmatrix('music_data_8k.txt');
hold on 
subplot(2,1,1);
plot(samples);
audio_samples= (samples/32767);
subplot(2,1,2);
plot(audio_samples);
sound(audio_samples, Fs_new);
finish=1;