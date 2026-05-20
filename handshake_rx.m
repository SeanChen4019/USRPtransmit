% =========== Minimal Handshake Receiver ===========
% Listens for BEACON on anchor_freq, responds with ACK on feedback_freq.
% Uses proven BPSK+spreading from old working version.
% ChannelMapping = 1 for BOTH TX and RX (single antenna on TX/RX port).
clear; clc; close all force; warning('off','all');
fprintf('========== Handshake Receiver ==========\n');

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
tx_gain_rx_side = 20;
rx_gain_rx_side = 25;

%% Pre-build ACK waveform (to send when beacon detected)
fprintf('[RX] Building ACK waveform...\n');
payload_ack = [Frame_head; Usr_ID; Frame_type_ack; Session_ID];
enc_ack = crcgenerator(payload_ack);
% LDPC requires exactly 486 input bits; pad to match
pad_len_ack = 486 - length(enc_ack);
payload_frame_ack = [enc_ack; zeros(pad_len_ack, 1)];

scr_ack = zeros(length(payload_frame_ack), 1);
for i = 1:floor(length(payload_frame_ack)/length(scr_seq))
    st_ = (i-1)*length(scr_seq) + 1;
    ed_ = i*length(scr_seq);
    scr_ack(st_:ed_) = xor(payload_frame_ack(st_:ed_), scr_seq);
end

enc_ack_bits = ldpcEncode(scr_ack, cfgLDPCEnc);
inter_ack = reshape(enc_ack_bits, 36, 18).';
inter_ack_bits = inter_ack(:);

inter_ack_polar = 2*inter_ack_bits - 1;
spread_ack = zeros(length(inter_ack_polar)*sf, 1);
for ii = 1:length(inter_ack_polar)
    spread_ack((ii-1)*sf+1 : ii*sf) = inter_ack_polar(ii) * pn_fb;
end

mod_ack = qpskmod(0.5*(spread_ack + 1));
tx_in_ack = [head_fb; mod_ack; zeros(sps*10, 1)];
ack_wave = txfilter(tx_in_ack);
ack_wave = [zeros(2000, 1); ack_wave];
fprintf('[RX] ACK waveform: %d samples (%.2f ms)\n', ...
    length(ack_wave), length(ack_wave)/200e6*512*1000);

%% Init SDR - BOTH on ChannelMapping=1 (matching old working version)
fprintf('[RX-HW] Initializing USRP...\n');
radio_tx = comm.SDRuTransmitter('Platform', 'X310', 'IPAddress', '192.168.10.2');
radio_tx.ChannelMapping = 1;
radio_tx.CenterFrequency = feedback_freq;
radio_tx.Gain = tx_gain_rx_side;
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
radio_rx.CenterFrequency = anchor_freq;
radio_rx.Gain = rx_gain_rx_side;

cleanupObj = onCleanup(@() safe_release(radio_tx, radio_rx));
fprintf('[RX-HW] USRP ready. RX on %.2f GHz, TX on %.2f GHz, ChMapping=1\n', ...
    anchor_freq/1e9, feedback_freq/1e9);

%% Handshake loop
state = 'WAITING_FOR_BEACON';
data_frame_len = 648 / log2(2) * sf;  % = 9720
Threshold = 250;
maxnumiter = 10;
PN_head = flip(head_fb);

for idx = 1:1000
    % ---- Listen for beacon on anchor frequency ----
    try
        [rx_sig, ~, overrun] = radio_rx();
        if overrun, warning('[RX] Overrun'); end
    catch ME
        warning('[RX] HW error: %s', ME.message);
        continue;
    end

    % ---- Try to decode beacon ----
    beacon_found = false;
    Rec_sig = rxfilter(rx_sig(:));

    syn_flag = false;
    index_val = zeros(1, sps);
    loc_num = zeros(1, sps);
    index_loc_cell = cell(1, sps);
    data_sys_cell = cell(1, sps);

    for i_ch = 1:sps
        data_sys_col = Rec_sig(i_ch:sps:end);
        data_sys_cell{i_ch} = data_sys_col;
        buf = abs(conv(PN_head, sign(data_sys_col)));
        if max(buf) >= Threshold
            syn_flag = true;
            above = find(buf >= Threshold);
            index_loc_cell{i_ch} = above;
            loc_num(i_ch) = length(above);
            index_val(i_ch) = mean(buf(above));
        end
    end

    if syn_flag
        [~, op_ch] = max(index_val);
        Rec_sig_temp = data_sys_cell{op_ch};
        idx_starts = index_loc_cell{op_ch};
        idx_starts = idx_starts(idx_starts + data_frame_len <= length(Rec_sig_temp));

        for j = 1:min(1, length(idx_starts))  % Just decode first candidate
            idx_s = idx_starts(j);
            train_len = min(511, idx_s);
            rx_train = Rec_sig_temp(idx_s-train_len+1 : idx_s);
            desire_seq = head_fb(end-train_len+1 : end);
            phase_est = -angle(mean(conj(desire_seq) .* rx_train));

            Rec_sig_afr = Rec_sig_temp(idx_s+1 : idx_s+data_frame_len) .* exp(1j*phase_est);
            demod_sig = qpskdemod(Rec_sig_afr);

            data_desp = zeros(length(demod_sig)/sf, 1);
            for ii = 1:length(demod_sig)/sf
                data_desp(ii) = sum(demod_sig((ii-1)*sf+1 : ii*sf) .* pn_fb);
            end

            deinter_matrix = reshape(data_desp, 18, 36).';
            deinter_bits = deinter_matrix(:);

            rx_bits = ldpcDecode(deinter_bits, cfgLDPCDec, maxnumiter);

            descr = zeros(length(rx_bits), 1);
            for ii = 1:floor(length(rx_bits)/length(scr_seq))
                st_ = (ii-1)*length(scr_seq) + 1;
                ed_ = ii*length(scr_seq);
                descr(st_:ed_) = xor(rx_bits(st_:ed_), scr_seq);
            end

            % Info=40bit + CRC32=72bit, rest is LDPC zero-padding
            [data_rec, err] = crcdetector(descr(1:72));
            if err ~= 0, continue; end

            offset = 0;
            fh = bits2int(data_rec(offset+1:offset+8)); offset = offset+8;
            uid = bits2int(data_rec(offset+1:offset+8)); offset = offset+8;
            ftype = bits2int(data_rec(offset+1:offset+8)); offset = offset+8;
            sid = bits2int(data_rec(offset+1:offset+16));

            if ftype == 100  % BEACON
                beacon_found = true;
                fprintf('\n[RX] *** BEACON DETECTED! ***\n');
                fprintf('[RX] Frame type: %d, Session: %d\n', ftype, sid);
            end
        end
    end

    % ---- Respond with ACK if beacon found ----
    if beacon_found && strcmp(state, 'WAITING_FOR_BEACON')
        fprintf('[RX] Sending ACK on feedback channel (%.2f GHz)...\n', feedback_freq/1e9);
        for rep = 1:3  % Send ACK 3 times for reliability
            radio_tx(ack_wave);
        end
        state = 'ACK_SENT';
        fprintf('[RX] *** HANDSHAKE SUCCESS! ACK sent. ***\n');
        break;
    end

    % ---- Periodic status ----
    if mod(idx, 10) == 0
        fprintf('[RX] Loop %d | state=%s | rms=%.4f | pk=%.4f\n', ...
            idx, state, rms(rx_sig), max(abs(rx_sig)));
    end
end

if ~strcmp(state, 'ACK_SENT')
    fprintf('\n[RX] Handshake timeout - no beacon received.\n');
end

release(radio_rx);
release(radio_tx);
fprintf('[RX] Shutdown complete.\n');

%% Helper
function v = bits2int(bits)
v = (2.^(length(bits)-1:-1:0)) * bits(:);
end

function safe_release(tx, rx)
try; if ~isempty(tx) && isvalid(tx), release(tx); end; catch; end
try; if ~isempty(rx) && isvalid(rx), release(rx); end; catch; end
disp('SDR resources released.');
end
