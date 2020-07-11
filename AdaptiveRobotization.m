%Zhiguang Eric Zhang - DSTII - Final Project - Adaptive Robotization
%
%input = input signal (mono or stereo)
%winlength = feature extraction analysis window length in samples (suggested 512 or 1024)
%hopmethod = robot pitch mapping method: 'centroid' or 'yin'
%
%The function adaptively maps features extracted 1) RMS and 2) spectral
%centroid or YIN autocorrelation from input signal to 1) analysis/synthesis
%block size, related to robot roughness and 2) synthesis hop size, related
%to robot pitch.  The samples are set to zero between synthesis grains.
%
%
%AdaptiveRobotization('hello.wav',1024,'yin');
%
%function output = AdaptiveRobotization(input,winlength,hopmethod)
function output = AdaptiveRobotization(input,winlength,hopmethod)

%read the input file
[wave,fs] = audioread(input);

%hann window
analywin = hann(winlength);

%initial analysis block from which features are to be extracted
analysis = wave(1:winlength,1);

%RMS plot
RMSOUT = [];

%take RMS
RMS = (sum(analysis .^ 2) / length(analysis)) ^ .5;

%block size is RMS * winlength
blocksize = round(RMS*winlength);

%set minimum block size
if blocksize < 64
    
    blocksize = 64;

end

%append analysis block value
RMSOUT = [RMSOUT;blocksize];

%output vector for hop size mapping
HOPOUT = [];

%switch case for hop size method
switch hopmethod
    
    case 'centroid'
    
    %get magnitude from fft
    ANALYSIS = abs(fft(analysis .* analywin));

    %vector of FFT bins
    analysiswin = (1:round(winlength/2)+1)';

    %drop symmetric part
    ANALYSIS = ANALYSIS(1:length(analysiswin));

    %spectral centroid in bins
    ANALYSISCGS = round(sum((ANALYSIS.^2) .* (analysiswin-1)) / sum(ANALYSIS.^2));

    %hop size is calculated from centroid
    %hop = round(winlength/ANALYSISCGS);
    %hop = round(ANALYSISCGS);
    %hop = round(ANALYSISCGS/length(analysiswin)*fs/2/10);
    hop = round(fs/ANALYSISCGS/100);
    
    %append to hop size mapping output
    HOPOUT = [HOPOUT;hop];
    
    case 'yin'
        
    %pad chunk
    chunk = [analysis;zeros(2,1)];
    
    %autocorrelation function
    for k = 0:length(chunk)/2-1
    
        ACF(k+1) = chunk(1)*chunk(1+k) + chunk(2)*chunk(2+k);

    end
    
    %yin
    ACFYIN = (ACF .^ 2) ./ (length(ACF));
    
    %find peaks
    [peaks_values,peaks_index] = findpeaks(ACFYIN,'MINPEAKDISTANCE',44);
    
    %analysis window and acf dependent hop size mapping
    if isempty(peaks_index)
        
        hop = 0;
    
    elseif length(peaks_index) < 2
        
        hop = peaks_index(1);
        
    elseif length(peaks_index) < 3
        
        hop = peaks_index(2) - peaks_index(1);
        
    elseif length(peaks_index) < 4
        
        hop = peaks_index(3) - peaks_index(2);
        
    elseif length(peaks_index) >= 4
            
        hop = peaks_index(4) - peaks_index(3);    
    
    end
    
    otherwise
        
    error('Hop size / pitch mapping method must be centroid or yin.');

end

%restrict hop size to prevent excessive timestretching
if hop > 128
            
    hop = 128;
            
end

%append to hop size mapping output
HOPOUT = [HOPOUT;hop];

%set starting sample
startingSamp = 1;

%output vector
output = [];

%while loop to move through signal
while startingSamp + blocksize - 1 <= length(wave);
    
    %make hann window
    win = hann(blocksize);

    %grab block of signal
    block = wave(startingSamp:startingSamp + blocksize - 1, 1);
    
    %window signal
    block = block .* win;

    %get magnitude spectrum from fft of block
    MAGBLOCK = abs(fft(block));

    %take inverse fft of zero-phase magnitude spectrum and set time
    %domain component to center, and window
    outblock = fftshift(real(ifft(MAGBLOCK))) .* win;
    
    %normalize outblock by scaling by RMS ratio of input and output
    outblock = outblock .* (((sum(block .^ 2) / length(block)) ^ .5) / ((sum(outblock .^ 2) / length(outblock)) ^ .5));

    %add to output vector and set samples between synthesis windows to zero
    output = [output;outblock;zeros(hop,1)];
    
    %set starting sample
    startingSamp = startingSamp + blocksize;
    
    %look ahead to extract features
    if startingSamp + winlength -1 > length(wave)
        
        break;
        
    else
        
        %analyis chunk
        analysis = wave(startingSamp:startingSamp + winlength - 1,1);
    
        %take RMS
        RMS = (sum(analysis .^ 2) / length(analysis)) ^ .5;
    
        %block size is RMS * winlength
        blocksize = round(RMS*winlength);

        %set minimum block size
        if blocksize < 64
        
            blocksize = 64;
        
        end
        
        %append analysis block value
        RMSOUT = [RMSOUT;blocksize];
        
        %switch case for hop size method
        switch hopmethod
            
            case 'centroid'
    
            %get magnitude from fft
            ANALYSIS = abs(fft(analysis .* analywin));

            %vector of FFT bins
            analysiswin = (1:round(winlength/2)+1)';

            %drop symmetric part
            ANALYSIS = ANALYSIS(1:length(analysiswin));

            %spectral centroid in bins
            ANALYSISCGS = round(sum((ANALYSIS.^2) .* (analysiswin-1)) / sum(ANALYSIS.^2));

            %hop size is calculated from centroid
            %hop = round(winlength/ANALYSISCGS);
            %hop = round(ANALYSISCGS);
            %hop = round(ANALYSISCGS/length(analysiswin)*fs/2/10);
            hop = round(fs/ANALYSISCGS/100);
        
            case 'yin'
                
            %pad chunk
            chunk = [analysis;zeros(2,1)];
    
            %autocorrelation function
            for k = 0:length(chunk)/2-1
    
                ACF(k+1) = chunk(1)*chunk(1+k) + chunk(2)*chunk(2+k);

            end
            
            %yin
            ACFYIN = (ACF .^ 2) ./ (length(ACF));
            
            %find peaks
            [peaks_values,peaks_index] = findpeaks(ACFYIN,'MINPEAKDISTANCE',44);
               
            %analysis window and acf dependent hop size mapping
            if isempty(peaks_index)
        
                hop = 0;
                
            elseif length(peaks_index) < 2
        
                hop = peaks_index(1);
    
            elseif length(peaks_index) < 3
        
                hop = peaks_index(2) - peaks_index(1);
        
            elseif length(peaks_index) < 4
        
                hop = peaks_index(3) - peaks_index(2);
                
            elseif length(peaks_index) >= 4
            
                hop = peaks_index(4) - peaks_index(3);
    
            end
            
            otherwise
        
            error('Hop size / pitch mapping method must be centroid or yin.');
            
        end
        
        %restrict hop size to prevent excessive timestretching
        if hop > 128
            
            hop = 128;
            
        end
        
        %append to hop size mapping output
        HOPOUT = [HOPOUT;hop];
    
    end
    
end

%decreasing feature extraction window size until the last analysis block
while startingSamp + blocksize - 1 <= length(wave)

    %end analysis chunk
    analysis = wave(startingSamp:end, 1);

    %hann window
    analywin = hann(length(analysis));

    %take RMS
    RMS = (sum(analysis .^ 2) / length(analysis)) ^ .5;

    %block size is RMS * winlength
    blocksize = round(RMS*length(analysis));

    %append analysis block value
    RMSOUT = [RMSOUT;blocksize];

    %set minimum block size
    if blocksize < 64
    
        blocksize = 64;
    
    end

    %switch case for hop size method
    switch hopmethod
    
        case 'centroid'

        %get magnitude from fft
        ANALYSIS = abs(fft(analysis .* analywin));

        %vector of FFT bins
        analysiswin = (1:round(length(analysis)/2)+1)';

        %drop symmetric part
        ANALYSIS = ANALYSIS(1:length(analysiswin));

        %spectral centroid in bins
        ANALYSISCGS = round(sum((ANALYSIS.^2) .* (analysiswin-1)) / sum(ANALYSIS.^2));

        %hop size is calculated from centroid
        %hop = round(winlength/ANALYSISCGS);
        %hop = round(ANALYSISCGS);
        %hop = round(ANALYSISCGS/length(analysiswin)*fs/2/10);
        hop = round(fs/ANALYSISCGS/100);

        case 'yin'
        
        %pad chunk
        chunk = [analysis;zeros(2,1)];
    
        %autocorrelation function
        for k = 0:length(chunk)/2-1
    
            ACF(k+1) = chunk(1)*chunk(1+k) + chunk(2)*chunk(2+k);

        end
    
        %yin
        ACFYIN = (ACF .^ 2) ./ (length(ACF));
    
        %find peaks
        [peaks_values,peaks_index] = findpeaks(ACFYIN,'MINPEAKDISTANCE',44);
    
        %analysis window and acf dependent hop size mapping
        if isempty(peaks_index)
        
            hop = 0;
    
        elseif length(peaks_index) < 2
        
            hop = peaks_index(1);
    
        elseif length(peaks_index) < 3
        
            hop = peaks_index(2) - peaks_index(1);
        
        elseif length(peaks_index) < 4
        
            hop = peaks_index(3) - peaks_index(2);
            
        elseif length(peaks_index) >= 4
            
            hop = peaks_index(4) - peaks_index(3);
    
        end
    
        otherwise
        
        error('Hop size / pitch mapping method must be centroid or yin.');
    
    end

    %restrict hop size to prevent excessive timestretching
    if hop > 128
            
        hop = 128;
            
    end

    %append to hop size mapping output
    HOPOUT = [HOPOUT;hop];
    
    %make hann window
    win = hann(blocksize);

    %grab block of signal
    block = wave(startingSamp:startingSamp + blocksize - 1, 1);

    %window signal
    block = block .* win;

    %get magnitude spectrum from fft of block
    MAGBLOCK = abs(fft(block));

    %take inverse fft of zero-phase magnitude spectrum and set time
    %domain component to center, and window
    outblock = fftshift(real(ifft(MAGBLOCK))) .* win;

    %normalize outblock by scaling by RMS ratio of input to output
    outblock = outblock .* (((sum(block .^ 2) / length(block)) ^ .5) / ((sum(outblock .^ 2) / length(outblock)) ^ .5));
    
    %add to output vector and set samples between synthesis windows to zero
    output = [output;outblock;zeros(hop,1)];
    
    %set starting sample
    startingSamp = startingSamp + blocksize;
    
end

%last block
block = wave(startingSamp:end, 1);

%make hann window
win = hann(length(block));

%window signal
block = block .* win;

%get magnitude spectrum from fft of block
MAGBLOCK = abs(fft(block));

%take inverse fft of zero-phase magnitude spectrum and set time
%domain component to center, and window
outblock = fftshift(real(ifft(MAGBLOCK))) .* win;

%normalize outblock by scaling by RMS ratio of input to output
outblock = outblock .* (((sum(block .^ 2) / length(block)) ^ .5) / ((sum(outblock .^ 2) / length(outblock)) ^ .5));

%add to output vector
output = [output;outblock];

%normalize signal    
output = .9999 * output / max(abs(output));

%plot signal and analysis and hop size mapping
subplot(3,1,1);
plot(output);
xlabel('Samples');
ylabel('Amplitude');
title('Adaptive Robotization Output');
subplot(3,1,2);
plot(RMSOUT);
xlabel('Window Number');
ylabel('Analysis Window Size');
title('Window Number vs Window Size');
subplot(3,1,3);
plot(HOPOUT);
xlabel('Window Number');
ylabel('Synthesis Hop Size');
title('Window Number vs Hop Size');

%output the sound
soundsc(output,fs);

%write the sound
audiowrite('robot.wav',output,fs);

end