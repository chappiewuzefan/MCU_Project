close all;
clear;
clc;

%% Code to convert .wav data into .txt %%
source_file = 'music_2ch.wav'; % Load the desired Audio Sample
[source_sig, Fs] = audioread(source_file, 'native');

%% Downsampling the Audio to 8kHz
Fs_new = 8000; % Desired Sampling Frequency
[Numer, Denom] = rat(Fs_new/Fs);
sig_new = resample(double(source_sig), Numer, Denom);

%% Seperating the two Channel Data
source_sig_1 = sig_new(:,1); % Channel 1 Data
source_sig_2 = sig_new(:,2); % Channel 2 Data
plot(source_sig_1(15000:30000)); % Plot for Source Signal channel 1
hold on;
plot(source_sig_2(15000:30000)); % Plot for Source Signal channel 2
writematrix(source_sig_1(15000:30000), "data_1.txt"); % Writing Source Signal ch1 Data into .txt
writematrix(source_sig_2(15000:30000), "data_2.txt"); % Writing Source Signal ch2 Data into .txt
finish=1;

%% Read Data from .txt %%
fs = 8000;    % Desired Sampling frequency - this value needs to be known
samples_1 = readmatrix('binaural_signal_witoutifft_1.txt');
samples_2 = readmatrix('binaural_signal_witoutifft_2.txt');

%% Generating plots %%
hold on 
subplot(4,1,1);
plot(samples_1); % Plot for Signed Data obtained from .txt
audio_samples_1= (samples_1/32767);
subplot(4,1,2);
plot(samples_2); % Plot for Signed Data obtained from .txt
audio_samples_2= (samples_2/32767);
title("Plot for Audio Sample");
subplot(4,1,3);
plot(audio_samples_1);
sound(audio_samples_1, fs); % Plot for Normalised Data (in the range of -1 to 1)
subplot(4,1,4);
plot(audio_samples_2);
title("Plot for Normalised Audio Sample"); % Plot for Normalised Data (in the range of -1 to 1)
output_audio = [audio_samples_1 audio_samples_2]; % Combining channel 1 and channel 2 data
sound(output_audio, fs); % Playing the Output Audio
finish=1;