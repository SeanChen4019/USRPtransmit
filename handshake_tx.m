% =========== Minimal Handshake Transmitter ===========
% Sends BEACON on anchor_freq, waits for ACK on feedback_freq.
% Uses proven BPSK+spreading from old working version.
% ChannelMapping = 1 for BOTH TX and RX (single antenna on TX/RX port).
clear; clc; close all force; warning('off','all');
fprintf('========== Handshake Transmitter ==========\n');

%% Physical layer constants (from old working version)
sps = 4;
sf = 15;
M = 2;  % BPSK

% ---- LDPC ----
P = [16 17 22 24  9  3 14 -1  4  2  7 -1 26 -1  2 -1 21 -1  1  0 -1 -1 -1 -1
    25 12 12  3  3 26  6 21 -1 15 22 -1 15 -1  4 -1 -1 16 -1  0  0 -1 -1 -1
    25 18 26 16 22 23  9 -1  0 -1  4 -1  4 -1  8 23 11 -1 -1 -1  0  0 -1 -1
    9  7  0  1 17 -1 -1  7  3 -1  3 23 -1 16 -1 -1 21 -1  0 -1 -1  0  0 -1
    24  5 26  7  1 -1 -1 15 24 15 -1  8 -1 13 -1 13 -1 11 -1 -1 -1 -1  0  0
    2  2 19 14 24  1 15 19 -1 21 -1  2 -1 24 -1  3 -1  2  1 -1 -1 -1 -1  0];
blockSize = 27;
pcmatrix = ldpcQuasiCyclicMatrix(blockSize, P);
cfgLDPCEnc = ldpcEncoderConfig(pcmatrix);
cfgLDPCDec = ldpcDecoderConfig(pcmatrix);
poly = 'z^32 + z^26 + z^23 + z^22 + z^16 + z^12 + z^11 + z^10 + z^8 + z^7 + z^5 + z^4 + z^2 + z + 1';
crcgenerator = comm.CRCGenerator(poly);
crcdetector = comm.CRCDetector(poly);

% ---- Modulation ----
qpskmod = comm.PSKModulator(M, 'BitInput', true);
qpskmod.PhaseOffset = pi/4;
qpskdemod = comm.PSKDemodulator(M, 'BitOutput', true, ...
    'DecisionMethod', 'Approximate log-likelihood ratio');
qpskdemod.PhaseOffset = pi/4;
txfilter = comm.RaisedCosineTransmitFilter('OutputSamplesPerSymbol', sps, 'RolloffFactor', 0.25);
rxfilter = comm.RaisedCosineReceiveFilter('InputSamplesPerSymbol', sps, 'DecimationFactor', 1, 'RolloffFactor', 0.25);

% ---- Preamble & spreading (feedback channel, from old working version) ----
head_fb = [-1,-1,-1,-1,-1,-1,-1,-1,-1,1,-1,-1,-1,-1,-1,-1,1,-1,-1,1,-1,-1,-1,1,-1,-1,-1,-1,-1,1,1,-1,-1,1,-1,-1,1,1,-1,1,-1,-1,-1,-1,1,-1,-1,1,-1,1,-1,1,-1,-1,-1,-1,1,1,1,1,-1,1,-1,1,1,1,-1,1,-1,1,1,-1,1,1,-1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,-1,-1,-1,-1,-1,1,1,-1,1,1,-1,-1,1,1,-1,-1,-1,-1,1,-1,1,-1,1,1,-1,1,-1,1,1,1,-1,-1,-1,1,1,-1,1,1,1,1,1,1,-1,-1,-1,1,-1,-1,-1,1,1,1,1,-1,-1,1,1,1,1,-1,1,1,-1,1,1,-1,1,-1,-1,-1,-1,-1,-1,-1,1,-1,1,-1,-1,-1,-1,1,-1,1,1,-1,1,-1,1,-1,1,-1,-1,-1,1,1,1,1,1,-1,1,1,1,1,-1,-1,1,-1,-1,1,-1,1,1,-1,-1,-1,-1,-1,1,-1,-1,1,1,-1,-1,1,-1,-1,-1,1,-1,1,-1,-1,-1,1,1,-1,1,1,-1,1,1,1,-1,-1,-1,-1,-1,-1,1,1,1,1,-1,-1,-1,1,1,1,-1,1,1,1,1,1,1,1,-1,-1,1,-1,-1,-1,-1,1,1,-1,-1,-1,1,-1,1,1,-1,1,1,1,-1,1,-1,-1,-1,-1,1,1,-1,1,-1,1,-1,1,1,-1,-1,1,1,1,1,-1,-1,1,-1,1,1,-1,1,1,-1,-1,1,-1,-1,-1,-1,-1,1,-1,-1,-1,1,-1,-1,1,-1,-1,1,1,-1,-1,-1,-1,-1,-1,1,-1,1,1,-1,-1,-1,1,-1,1,-1,-1,1,1,1,-1,1,1,-1,-1,1,1,1,-1,-1,-1,1,-1,1,1,1,1,1,1,-1,1,-1,1,-1,-1,-1,1,-1,1,1,1,-1,1,1,-1,1,-1,1,1,-1,-1,-1,-1,1,1,-1,-1,1,1,-1,1,1,-1,1,-1,1,-1,-1,-1,-1,-1,1,1,1,-1,1,-1,-1,1,1,1,1,-1,1,-1,-1,1,1,-1,1,-1,1,-1,-1,1,-1,-1,1,1,1,-1,-1,-1,-1,-1,1,1,1,1,1,-1,-1,1,1,1,-1,-1,1,1,-1,1,1,1,1,-1,1,-1,-1,-1,1,-1,1,-1,1,-1,1,1,-1,1,1,1,1,1,-1,-1,-1,-1,1,-1,-1,1,1,1,-1,1,-1,-1,-1,1,1,1,-1,1,-1,1,1,1,1,1,-1,1,1,-1,1,-1,-1,1,-1,-1,-1,-1,1,-1,-1,-1,-1,1,-1,1,-1,-1,1,-1,1,-1,1,1,-1,-1,-1,1,1,1,-1,-1,1,1,1,1,1,1,1,-1,1,1,-1,-1,-1,-1,1,-1,-1,-1,1,1,-1,1,-1,-1,1,1,1,-1,-1,1,-1,-1,1,1,1,1,-1,-1,-1,-1,1,1,-1,1,1,1,-1,1,1,-1,-1,-1,1,1,-1,-1,-1,1,1,1,1,-1,1,1,1,1,1,-1,1,-1,-1,1,-1,-1,1,-1,1,-1,-1,-1,-1,-1,-1,1,1,-1,1,-1,-1,-1,1,1,-1,-1,1,-1,1,1,1,-1,1,-1,-1,1,-1,1,1,-1,1,-1,-1,-1,1,-1,-1,-1,1,-1,1,1,-1,-1,1,1,-1,1,-1,-1,1,-1,1,-1,-1,1,-1,-1,-1,1,1,-1,-1,-1,-1,1,1,1,-1,1,1,-1,1,1,1,1,-1,-1,-1,-1,-1,1,-1,1,1,1,-1,-1,1,-1,1,-1,1,1,1,-1,-1,1,1,1,-1,1,1,1,-1,1,1,1,-1,-1,1,1,-1,-1,1,1,1,-1,1,-1,1,-1,1,1,1,-1,1,1,1,1,-1,1,1,-1,-1,1,-1,1,-1,-1,-1,1,-1,-1,1,1,-1,1,1,-1,-1,-1,1,-1,-1,-1,-1,1,1,1,-1,-1,1,-1,1,1,1,1,1,-1,-1,1,-1,1,-1,-1,1,1,-1,-1,1,1,-1,-1,1,-1,1,-1,1,-1,1,-1,-1,1,1,1,1,1,1,-1,-1,1,1,-1,-1,-1,1,1,-1,1,-1,1,1,1,1,-1,-1,1,1,-1,1,-1,1,1,-1,1,-1,-1,1,1,-1,-1,-1,1,-1,-1,1,-1,1,1,1,-1,-1,-1,-1,1,-1,1,1,1,1,-1,1,-1,1,-1,1,-1,1,-1,1,1,1,1,1,1,1,1,-1,1,-1,-1,-1,-1,-1,1,-1,1,-1,1,-1,-1,1,-1,1,1,1,1,-1,-1,-1,1,-1,1,-1,1,1,1,1,-1,1,1,1,-1,1,-1,1,-1,-1,1,1,-1,1,1,1,-1,-1,1,-1,-1,-1,1,1,1,-1,-1,-1,1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,1,1,1,-1,-1,-1,-1,1,1,1,1,1,1,-1,1,1,1,-1,-1,-1,1,-1,-1,1,1,1,1,1,-1,-1,-1,1,1,-1,-1,1,1,1,1,1,-1,1,-1,1,1,-1,-1,1,-1,1,1,-1,-1,1,-1,-1,1,-1,-1,1]';
pn_fb = [1,-1,-1,-1,1,1,1,1,-1,1,-1,1,1,-1,-1]';
scr_seq = [1 1 0 1 1 0 1 0 0 1 0 0 0 0 1 0 1 0 1 1 1 0 1 1 0 0 0]';

% ---- Frame header bits ----
Frame_head = [1,1,1,0,1,0,1,0]';
Usr_ID = [0,0,0,0,0,1,0,1]';
Frame_type_beacon = double(dec2bin(100, 8) == '1')';  % 100 = BEACON
Frame_type_ack    = double(dec2bin(101, 8) == '1')';  % 101 = ACK
Session_ID = double(dec2bin(1, 16) == '1')';
Frame_end = zeros(358, 1);  % padding to 486 bits before LDPC

%% Frequencies & gains
anchor_freq   = 2.5e9;   % forward: TX -> RX
feedback_freq = 1.45e9;  % feedback: RX -> TX
tx_gain = 30;
rx_gain = 30;

%% Build beacon waveform
fprintf('[TX] Building BEACON waveform...\n');
payload_beacon = [Frame_head; Usr_ID; Frame_type_beacon; Session_ID];
enc_beacon = crcgenerator(payload_beacon);
% LDPC requires exactly 486 input bits; pad to match
pad_len_beacon = 486 - length(enc_beacon);
payload_frame_beacon = [enc_beacon; zeros(pad_len_beacon, 1)];

% Scramble
scr_data = zeros(length(payload_frame_beacon), 1);
for i = 1:floor(length(payload_frame_beacon)/length(scr_seq))
    st_ = (i-1)*length(scr_seq) + 1;
    ed_ = i*length(scr_seq);
    scr_data(st_:ed_) = xor(payload_frame_beacon(st_:ed_), scr_seq);
end

% LDPC encode
enc_bits = ldpcEncode(scr_data, cfgLDPCEnc);

% Interleave (18x36)
inter_matrix = reshape(enc_bits, 36, 18).';
inter_bits = inter_matrix(:);

% 15x spreading
inter_polar = 2*inter_bits - 1;
spread_seq = zeros(length(inter_polar)*sf, 1);
for ii = 1:length(inter_polar)
    spread_seq((ii-1)*sf+1 : ii*sf) = inter_polar(ii) * pn_fb;
end

% BPSK modulate
mod_sig = qpskmod(0.5*(spread_seq + 1));
tx_in = [head_fb; mod_sig; zeros(sps*10, 1)];
beacon_wave = txfilter(tx_in);
beacon_wave = [zeros(2000, 1); beacon_wave];
fprintf('[TX] Beacon waveform: %d samples (%.2f ms)\n', ...
    length(beacon_wave), length(beacon_wave)/200e6*512*1000);

%% Init SDR - BOTH on ChannelMapping=1 (matching old working version)
fprintf('[TX-HW] Initializing USRP...\n');
radio_tx = comm.SDRuTransmitter('Platform', 'X310', 'IPAddress', '192.168.10.2');
radio_tx.ChannelMapping = 1;
radio_tx.CenterFrequency = anchor_freq;
radio_tx.Gain = tx_gain;
radio_tx.MasterClockRate = 200e6;
radio_tx.InterpolationFactor = 512;
radio_tx.ClockSource = 'External';

radio_rx = comm.SDRuReceiver( ...
    'Platform', 'X310', ...
    'IPAddress', '192.168.10.2', ...
    'OutputDataType', 'double', ...
    'MasterClockRate', 200e6, ...
    'DecimationFactor', 512, ...
    'SamplesPerFrame', 80000);
radio_rx.ClockSource = 'External';
radio_rx.ChannelMapping = 1;  % Same as old version!
radio_rx.CenterFrequency = feedback_freq;
radio_rx.Gain = rx_gain;

cleanupObj = onCleanup(@() safe_release(radio_tx, radio_rx));
fprintf('[TX-HW] USRP ready. TX on %.2f GHz, RX on %.2f GHz, ChMapping=1\n', ...
    anchor_freq/1e9, feedback_freq/1e9);

%% Handshake loop
% Strategy: send beacon periodically, listen continuously.
% RX needs time to receive beacon, decode, and send ACK - so
% we spend most iterations just listening.
state = 'SENDING_BEACON';
max_iter = 2000;
BEACON_PERIOD = 8;  % send beacon every N iterations

for idx = 1:max_iter
    % ---- Transmit beacon periodically ----
    if mod(idx, BEACON_PERIOD) == 0
        radio_tx(beacon_wave);
    end

    % ---- Listen for ACK on feedback channel ----
    try
        [rx_sig, ~, overrun] = radio_rx();
        if overrun, warning('[TX] Overrun'); end
    catch ME
        warning('[TX] HW error: %s', ME.message);
        continue;
    end

    % ---- Try to decode ACK ----
    [ack_valid, ack_data] = decode_ack(rx_sig, rxfilter, head_fb, pn_fb, ...
        scr_seq, cfgLDPCDec, crcdetector, qpskdemod, sps, sf);

    if ack_valid && ack_data.frame_type == 101
        fprintf('\n[TX] *** HANDSHAKE SUCCESS! ACK received at iteration %d. ***\n', idx);
        fprintf('[TX] Frame type: %d, Session: %d\n', ack_data.frame_type, ack_data.session_id);
        state = 'DONE';
        break;
    end

    % ---- Periodic status ----
    if mod(idx, 10) == 0
        fprintf('[TX] Loop %d | state=%s | rms=%.4f | pk=%.4f\n', ...
            idx, state, rms(rx_sig), max(abs(rx_sig)));
    end
end

if ~strcmp(state, 'DONE')
    fprintf('\n[TX] Handshake timeout - no ACK received.\n');
end

release(radio_rx);
release(radio_tx);
fprintf('[TX] Shutdown complete.\n');

%% =========== Decode ACK function ===========
function [valid, data] = decode_ack(rx_sig, rxfilter, head_fb, pn_fb, ...
    scr_seq, cfgLDPCDec, crcdetector, qpskdemod, sps, sf)
valid = false;
data = struct();
Threshold = 250;
maxnumiter = 10;

Rec_sig = rxfilter(complex(rx_sig(:)));
data_frame_len = 648 / log2(2) * sf;  % = 9720

PN_head = flip(head_fb);
data_sys = [];
buffer_h = [];
index_val = zeros(1, sps);
index_loc = cell(1, sps);
loc_num = zeros(1, sps);
syn_flag = false;

for i = 1:sps
    data_sys_col = Rec_sig(i:sps:end);
    data_sys(:, i) = data_sys_col;
    buffer_h(:, i) = abs(conv(PN_head, sign(data_sys_col)));
    if max(buffer_h(:, i)) >= Threshold
        syn_flag = true;
        above = find(buffer_h(:, i) >= Threshold);
        index_loc{i} = above;
        loc_num(i) = length(above);
        index_val(i) = mean(buffer_h(above, i));
    end
end

if ~syn_flag, return; end

[~, op_idx] = max(index_val);
Rec_sig_afr_temp = data_sys(1:length(data_sys_col), op_idx);
idx_start_temp = index_loc{op_idx};

% Remove candidates too close to end
idx_start_temp = idx_start_temp(idx_start_temp + data_frame_len <= length(Rec_sig_afr_temp));
if isempty(idx_start_temp), return; end

for j = 1:length(idx_start_temp)
    idx_start = idx_start_temp(j);

    train_len = min(511, idx_start);
    receive_train = Rec_sig_afr_temp(idx_start-train_len+1 : idx_start);
    desire_seq = head_fb(end-train_len+1 : end);
    phase_est = -angle(mean(conj(desire_seq) .* receive_train));

    Rec_sig_afr = Rec_sig_afr_temp(idx_start+1 : idx_start+data_frame_len) .* exp(1j*phase_est);
    demod_sig = qpskdemod(Rec_sig_afr);

    % Despread
    data_desp = zeros(length(demod_sig)/sf, 1);
    for ii = 1:length(demod_sig)/sf
        data_desp(ii) = sum(demod_sig((ii-1)*sf+1 : ii*sf) .* pn_fb);
    end

    % Deinterleave (18x36 �?36x18)
    deinter_matrix = reshape(data_desp, 18, 36).';
    deinter_bits = deinter_matrix(:);

    % LDPC decode
    rx_bits = ldpcDecode(deinter_bits, cfgLDPCDec, maxnumiter);

    % Descramble
    descr_data = zeros(length(rx_bits), 1);
    for ii = 1:floor(length(rx_bits)/length(scr_seq))
        st_ = (ii-1)*length(scr_seq) + 1;
        ed_ = ii*length(scr_seq);
        descr_data(st_:ed_) = xor(rx_bits(st_:ed_), scr_seq);
    end

    % CRC check
    % Info=40bit + CRC32=72bit, rest is LDPC zero-padding
    [data_rec, err] = crcdetector(descr_data(1:72));
    if err ~= 0, continue; end

    % Parse
    offset = 0;
    data.frame_head = bits2int(data_rec(offset+1:offset+8)); offset = offset+8;
    data.user_id    = bits2int(data_rec(offset+1:offset+8)); offset = offset+8;
    data.frame_type = bits2int(data_rec(offset+1:offset+8)); offset = offset+8;
    data.session_id = bits2int(data_rec(offset+1:offset+16));

    valid = true;
    return;
end
end

function v = bits2int(bits)
v = (2.^(length(bits)-1:-1:0)) * bits(:);
end

function safe_release(tx, rx)
try; if ~isempty(tx) && isvalid(tx), release(tx); end; catch; end
try; if ~isempty(rx) && isvalid(rx), release(rx); end; catch; end
disp('SDR resources released.');
end
