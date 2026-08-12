% =========================================================================
% 2-order Butterworth IIR Lowpass — coefficient design for BLDC current
% ISR sample rate: 50 kHz
% =========================================================================

clear; close all;

fs   = 50000;       % Sample rate: 50 kHz (ISR frequency)
fc   = 200;          % Cutoff frequency (-3dB): adjust this, try 100~400 Hz

% ---- Design 2nd-order Butterworth --------------------------------------
[b, a] = butter(2, fc / (fs / 2), 'low');

% ---- Print coefficients -------------------------------------------------
fprintf('\n===== Biquad Coefficients =====\n');
fprintf('fs = %d Hz,  fc = %d Hz\n\n', fs, fc);
fprintf('// Direct Form I difference equation:\n');
fprintf('// y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]\n\n');
fprintf('b0 = %12.10f;\n', b(1));
fprintf('b1 = %12.10f;\n', b(2));
fprintf('b2 = %12.10f;\n', b(3));
fprintf('a1 = %12.10f;   // note: a1 from MATLAB, sign already correct\n', a(2));
fprintf('a2 = %12.10f;\n', a(3));
% NOTE: MATLAB returns a = [1, a1, a2]. In the diff eq, it's -a1, -a2.
% The - sign is already in the equation above, so use a(2) and a(3) as-is.

% ---- C-ready copy-paste -------------------------------------------------
fprintf('\n// ---- C code ----\n');
fprintf('#define BIQUAD_B0  %s\n', num2str(b(1), 12));
fprintf('#define BIQUAD_B1  %s\n', num2str(b(2), 12));
fprintf('#define BIQUAD_B2  %s\n', num2str(b(3), 12));
fprintf('#define BIQUAD_A1  %s\n', num2str(a(2), 12));
fprintf('#define BIQUAD_A2  %s\n', num2str(a(3), 12));

% ---- Key frequency points -----------------------------------------------
freqs = [10, 20, 50, 80, 100, 150, 200, 250, 300, 400, 500, 1000, 5000, 50000];
fprintf('\n===== Frequency Response =====\n');
fprintf('%-10s  %-12s  %-12s  %-12s\n', 'Freq(Hz)', 'Mag(dB)', 'Phase(deg)', 'GroupDelay(samp)');
fprintf('%-10s  %-12s  %-12s  %-12s\n', '--------', '--------', '----------', '----------------');

for i = 1:length(freqs)
    f = freqs(i);
    [h_i, w_i] = freqz(b, a, f, fs);
    mag_db = 20*log10(abs(h_i) + eps);

    % Group delay at this frequency (via grpdelay)
end

% Use freqz once at all points for efficiency
[h_all, ~] = freqz(b, a, freqs, fs);
[gd_all, ~] = grpdelay(b, a, freqs, fs);

for i = 1:length(freqs)
    mag_db = 20*log10(abs(h_all(i)) + 1e-16);
    phase_deg = angle(h_all(i)) * 180 / pi;
    fprintf('%-10d  %+12.6f  %+12.4f  %12.1f\n', freqs(i), mag_db, phase_deg, gd_all(i));
end

fprintf('\nReference: EMA (1st order, fc=250Hz) at 50kHz = ~%.1f dB\n', ...
    20*log10(250/50000));

% ---- Step response (optional check) ------------------------------------
figure;
[b_step, a_step] = butter(2, fc/(fs/2), 'low');
[y_step, ~] = stepz(b_step, a_step, 2000);
t_step = (0:length(y_step)-1)' / fs * 1000; % ms
plot(t_step, y_step, 'LineWidth', 1.5); grid on;
xlabel('Time (ms)');
ylabel('Amplitude');
title(sprintf('Step Response: fc=%dHz (settling ~%.1fms @ 95%%)', fc, ...
    find(y_step>=0.95,1)/fs*1000));
xlim([0 20]);

fprintf('\nDone.\n');
