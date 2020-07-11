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
%robotization2('hello.wav',1024,'yin');
%
%function output = robotization2(input,winlength,hopmethod)
function output = robotization2(input,winlength,hopmethod)

%read the input file
[wave,fs] = audioread(input);

%hann window
analywin = hann(winlength);

%initial analysis block from which features are to be extracted
analysis = wave(1:winlength,1);

%take RMS
RMS = (sum(analysis .^ 2) / length(analysis)) ^ .5;

%block size is RMS * winlength
blocksize = round(RMS*winlength);

%set minimum block size
if blocksize < 64
    
    blocksize = 64;

end

YINOUT = [];

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
    hop = round(winlength/ANALYSISCGS);
    %hop = round(ANALYSISCGS);
    %hop = round(ANALYSISCGS/length(analysiswin)*fs/2/10);
    
    case 'yin'
        
        %take fft
    WAVE = abs(fft(analysis));

    %throw away symmetric part
    WAVE = WAVE(1:round(length(WAVE)/2)+1);

    %get length
    lwave = length(WAVE);

    %create output vector
    mult = zeros(1,length(WAVE));

    %copy spectrum
    WAVE2 = WAVE;

        for k = 1:4
    
        %downsample by 1/2
        WAVE = WAVE(1:2:end);
    
        %zero pad to original length
        WAVE = [WAVE;zeros(lwave-length(WAVE),1)];
    
        %HPS
        if k == 1
        
            mult = WAVE2' .* WAVE';
        
        else
        
            mult = mult .* WAVE';
        
        end
        
        end
        
        %get peak
        [Y,I] = max(mult);
        
        %calculate freq and insert into output vector
        hop=I(1);
    
    otherwise
        
    error('Hop size / pitch mapping method must be centroid or yin.');

end

%restrict hop size
if hop >= 128
            
    hop = 64;
            
end

%avoid NaN and Inf
if isnan(hop) || hop == Inf
    
    hop = 0;
    
end

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
            hop = round(winlength/ANALYSISCGS);
            %hop = round(ANALYSISCGS);
            %hop = round(ANALYSISCGS/length(analysiswin)*fs/2/10);
        
            case 'yin'
                
                %take fft
    WAVE = abs(fft(analysis));

    %throw away symmetric part
    WAVE = WAVE(1:round(length(WAVE)/2)+1);

    %get length
    lwave = length(WAVE);

    %create output vector
    mult = zeros(1,length(WAVE));

    %copy spectrum
    WAVE2 = WAVE;

        for k = 1:4
    
        %downsample by 1/2
        WAVE = WAVE(1:2:end);
    
        %zero pad to original length
        WAVE = [WAVE;zeros(lwave-length(WAVE),1)];
    
        %HPS
        if k == 1
        
            mult = WAVE2' .* WAVE';
        
        else
        
            mult = mult .* WAVE';
        
        end
        
        end
        
        %get peak
        [Y,I] = max(mult);
        
        %calculate freq and insert into output vector
        hop=I(1);
    
            otherwise
        
            error('Hop size / pitch mapping method must be centroid or yin.');
            
        end
        
        %restrict hop size
        if hop >= 128
            
            hop = 64;
            
        end
        
        %avoid NaN and Inf
        if isnan(hop) || hop == Inf
    
            hop = 0;
    
        end
    
    end
        
end

%end analysis chunk
analysis = wave(startingSamp:end, 1);

%hann window
analywin = hann(length(analysis));

%take RMS
RMS = (sum(analysis .^ 2) / length(analysis)) ^ .5;

%block size is RMS * winlength
blocksize = round(RMS*winlength);

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
    analysiswin = (1:round(winlength/2)+1)';

    %drop symmetric part
    ANALYSIS = ANALYSIS(1:length(analysiswin));

    %spectral centroid in bins
    ANALYSISCGS = round(sum((ANALYSIS.^2) .* (analysiswin-1)) / sum(ANALYSIS.^2));

    %hop size is calculated from centroid
    hop = round(winlength/ANALYSISCGS);
    %hop = round(ANALYSISCGS);
    %hop = round(ANALYSISCGS/length(analysiswin)*fs/2/10);
    
    case 'yin'
        
        %take fft
    WAVE = abs(fft(analysis));

    %throw away symmetric part
    WAVE = WAVE(1:round(length(WAVE)/2)+1);

    %get length
    lwave = length(WAVE);

    %create output vector
    mult = zeros(1,length(WAVE));

    %copy spectrum
    WAVE2 = WAVE;

        for k = 1:4
    
        %downsample by 1/2
        WAVE = WAVE(1:2:end);
    
        %zero pad to original length
        WAVE = [WAVE;zeros(lwave-length(WAVE),1)];
    
        %HPS
        if k == 1
        
            mult = WAVE2' .* WAVE';
        
        else
        
            mult = mult .* WAVE';
        
        end
        
        end
        
        %get peak
        [Y,I] = max(mult);
        
        %calculate freq and insert into output vector
        hop=I(1);
    
    otherwise
        
    error('Hop size / pitch mapping method must be centroid or yin.');
    
end

%restrict hop size
if hop >= 128
            
    hop = 64;
            
end

%avoid NaN and Inf
if isnan(hop) || hop == Inf
    
    hop = 0;
    
end

%while loop to move through end of signal
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

%plot signal
%plot(output);
%plot(YINOUT);

%output the sound
soundsc(output,fs);

end