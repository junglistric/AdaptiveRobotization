# AdaptiveRobotization
Adaptive Robotization

%input = input signal (mono or stereo)
%winlength = feature extraction analysis window length in samples (suggested 512 or 1024)
%hopmethod = robot pitch mapping method: 'centroid' or 'yin'
%
%The function adaptively maps features extracted 1) RMS and 2) spectral
%centroid or YIN autocorrelation from input signal to 1) analysis/synthesis
%block size, related to robot roughness and 2) synthesis hop size, related
%to robot pitch.  The samples are set to zero between synthesis grains.
