% ========== WiFi-Style Transmitter ==========
% Protocol inspired by 802.11:
%   STF(auto-corr) + LTF(cross-corr) preamble → robust sync
%   BEACON → ACK → DATA(slots) → END → RESULT
%   FDD: forward 2.5GHz, feedback 1.45GHz
clear; clc; close all force; warning('off','all');
fprintf('\n========== WiFi模式发射机 ==========\n');

%% =========== Configuration ===========
defs = link_phy_defs();

transmit_mode = 'image';
file_name = 'p2.jpg';
Anti_Jamming_Mode = 0;    % 0=QPSK, 1=BPSK+spreading
Power_gain = 30;
Power = 1.0;
fec_k = defs.fec_k_default;
fec_r = defs.fec_r_default;
hop_seed = randi(65535);

% Frequencies
anchor_freq = 2.5e9;
feedback_freq = 1.45e9;

% Timing (WiFi-style SIFS/DIFS in seconds)
SIFS = 0.005;   % short gap between RX→TX turnaround
DIFS = 0.010;   % gap before starting new exchange
SLOT_TIME = 0.002;

%% =========== State Machine ===========
STATE_BEACON  = 0;
STATE_DATA    = 1;
STATE_END     = 2;
STATE_DONE    = 3;

%% =========== PHY: STF & LTF ===========
sps = 4;  % samples per symbol

% STF: 10x repeating 16-sample pattern (like 802.11 short preamble)
% Good periodic autocorrelation for robust detection
stf16 = [1, 1, -1, -1, 1, 1, -1, 1, -1, 1, 1, 1, 1, 1, 1, -1]';
stf_pattern = repmat(stf16, 10, 1);  % 160 symbols
stf_wave_raw = [stf_pattern; zeros(sps*4, 1)];  % +16 tail samples

% LTF: 2x64 known sequence for fine sync + channel estimation
ltf64 = [1, 1, 1, 1, -1, -1, 1, 1, -1, 1, -1, 1, -1, -1, -1, -1, ...
         -1, 1, 1, -1, -1, 1, -1, 1, -1, 1, 1, 1, 1, 1, -1, 1, ...
         1, -1, 1, -1, -1, 1, 1, -1, -1, 1, 1, 1, -1, -1, 1, 1, ...
         -1, -1, -1, 1, 1, 1, -1, 1, 1, -1, -1, -1, 1, -1, 1, -1]';
ltf_2x = [ltf64; ltf64];  % 128 symbols
ltf_wave_raw = [ltf_2x; zeros(sps*2, 1)];

% RRC filter
txfilter = comm.RaisedCosineTransmitFilter('OutputSamplesPerSymbol', sps, 'RolloffFactor', 0.25);
rxfilter = comm.RaisedCosineReceiveFilter('InputSamplesPerSymbol', sps, 'DecimationFactor', 1, 'RolloffFactor', 0.25);

% Filter STF and LTF
stf_wave = txfilter(stf_wave_raw);
ltf_wave = txfilter(ltf_wave_raw);

%% =========== PHY: Control Channel (BPSK+15x spreading) ===========
sf = 15;
pcmatrix = ldpcQuasiCyclicMatrix(defs.blockSize, defs.P);
cfgLDPCEnc = ldpcEncoderConfig(pcmatrix);
cfgLDPCDec = ldpcDecoderConfig(pcmatrix);
crcgen_ctrl = comm.CRCGenerator(defs.poly);
crcdet_ctrl = comm.CRCDetector(defs.poly);

qpskmod = comm.PSKModulator(2, 'BitInput', true);  % BPSK
qpskmod.PhaseOffset = pi/4;
qpskdemod = comm.PSKDemodulator(2, 'BitOutput', true, ...
    'DecisionMethod', 'Approximate log-likelihood ratio');
qpskdemod.PhaseOffset = pi/4;

pn_fb = [1,-1,-1,-1,1,1,1,1,-1,1,-1,1,1,-1,-1]';
scr_seq = [1 1 0 1 1 0 1 0 0 1 0 0 0 0 1 0 1 0 1 1 1 0 1 1 0 0 0]';

% Frame header bits
Frame_head = [1;1;1;0;1;0;1;0];
Usr_ID = [0;0;0;0;0;1;0;1];

%% =========== File Processing (reuse existing) ===========
fprintf('[INIT] 正在处理文件...\n');
cfg = struct('hop_seed', hop_seed, 'fec_k', fec_k, 'fec_r', fec_r);
if strcmp(transmit_mode, 'text')
    cfg.text_content = 'Hello WiFi!';
    [business_bytes, meta_info] = build_file_container([], cfg);
else
    [business_bytes, meta_info] = build_file_container(file_name, cfg);
end

payload_len = defs.payload_bytes_per_pkt;
total_src_packets = ceil(length(business_bytes) / payload_len);
src_packets = cell(1, total_src_packets);
for p = 1:total_src_packets
    st = (p-1)*payload_len + 1;
    ed = min(p*payload_len, length(business_bytes));
    pkt = zeros(payload_len, 1, 'uint8');
    pkt(1:ed-st+1) = business_bytes(st:ed);
    src_packets{p} = pkt;
end
fprintf('[INIT] 文件: %d 字节 -> %d 源数据包\n', length(business_bytes), total_src_packets);

fec_groups = fec_rs_encode_groups(src_packets, fec_k, fec_r);
[frame_list, fec_info] = build_forward_frames_v2(src_packets, meta_info, fec_groups);
[~, tx_cache] = forward_frame_modulate_v2(frame_list, Anti_Jamming_Mode, fec_info);
slot_cache = build_hop_slot_waveform(tx_cache, fec_info);

total_slots = fec_info.total_slots;
session_id = fec_info.session_id;
fprintf('[INIT] 会话=%d | 时隙=%d | FEC组=%d\n', session_id, total_slots, length(fec_groups));

%% =========== Build Control Frames ===========
% Build a control frame: [STF] [LTF] [SIGNAL_BPSK_spread]
function ctrl_wave = build_wifi_ctrl(session_id, frame_type, extra_bits)
    % SIGNAL payload: FrameHead(8)+UserID(8)+FrameType(8)+SessionID(16)+ExtraBits
    if nargin < 3, extra_bits = []; end
    payload = [Frame_head; Usr_ID; double(dec2bin(frame_type,8)=='1')'; ...
               double(dec2bin(session_id,16)=='1')'; extra_bits];

    enc = crcgen_ctrl(payload);
    pad = 486 - length(enc);
    frame_data = [enc; zeros(pad, 1)];

    scr = zeros(length(frame_data),1);
    for i = 1:floor(length(frame_data)/length(scr_seq))
        ix = (i-1)*length(scr_seq)+1 : i*length(scr_seq);
        scr(ix) = xor(frame_data(ix), scr_seq);
    end

    enc_bits = ldpcEncode(scr, cfgLDPCEnc);
    inter_m = reshape(enc_bits, 36, 18).';
    inter_bits = inter_m(:);

    inter_pol = 2*inter_bits - 1;
    spread = zeros(length(inter_pol)*sf, 1);
    for ii = 1:length(inter_pol)
        spread((ii-1)*sf+1 : ii*sf) = inter_pol(ii) * pn_fb;
    end

    mod_sig = qpskmod(0.5*(spread + 1));
    signal_wave = txfilter([mod_sig; zeros(sps*10, 1)]);

    ctrl_wave = [zeros(1000,1); stf_wave; ltf_wave; signal_wave];
end

%% Pre-build waveforms
fprintf('[PHY] 构建控制帧波形...\n');
beacon_wave = build_wifi_ctrl(session_id, 100);         % BEACON=100
start_wave = build_wifi_ctrl(session_id, 41, ...         % START=41
    [double(dec2bin(meta_info.hop_seed,32)=='1')'; ...
     double(dec2bin(total_slots,16)=='1')'; ...
     double(dec2bin(meta_info.slot_len_samples,32)=='1')'; ...
     double(dec2bin(fec_info.codewords_per_slot,16)=='1')'; ...
     double(dec2bin(meta_info.fec_k,8)=='1')'; ...
     double(dec2bin(meta_info.fec_r,8)=='1')']);
end_wave = build_wifi_ctrl(session_id, 42);             % END=42

fprintf('[PHY] BEACON: %d 采样点 | START: %d 采样点\n', length(beacon_wave), length(start_wave));

%% =========== SDR Setup ===========
fprintf('[SDR] 初始化USRP...\n');
radio_tx = comm.SDRuTransmitter('Platform', 'X310', 'IPAddress', '192.168.10.2');
radio_tx.ChannelMapping = 1;
radio_tx.CenterFrequency = anchor_freq;
radio_tx.Gain = Power_gain;
radio_tx.MasterClockRate = 200e6;
radio_tx.InterpolationFactor = 512;
radio_tx.ClockSource = 'External';

radio_rx = comm.SDRuReceiver('Platform', 'X310', 'IPAddress', '192.168.10.2', ...
    'OutputDataType', 'double', 'MasterClockRate', 200e6, ...
    'DecimationFactor', 512, 'SamplesPerFrame', 80000);
radio_rx.ClockSource = 'External';
radio_rx.ChannelMapping = 1;
radio_rx.CenterFrequency = feedback_freq;
radio_rx.Gain = 30;

cleanup = onCleanup(@() safe_release(radio_tx, radio_rx));
fprintf('[SDR] USRP就绪. TX=%.1fGHz RX=%.1fGHz\n', anchor_freq/1e9, feedback_freq/1e9);

%% =========== Synchronized Start ===========
t_now = datetime('now');
sec_cur = second(t_now);
sec_tgt = ceil(sec_cur / 5) * 5;
if sec_tgt - sec_cur < 1, sec_tgt = sec_tgt + 5; end
w = sec_tgt - sec_cur;
fprintf('[SYNC] 当前: %s | 开始: %s (%.1f秒后)\n', ...
    datestr(t_now,'HH:MM:SS'), datestr(t_now+seconds(w),'HH:MM:SS'), w);
fprintf('[SYNC] 等待RX同步...\n');
pause(w);
fprintf('[SYNC] 开始!\n');

%% =========== Main Loop ===========
state = STATE_BEACON;
beacon_timer = 0;
slot_ptr = 1;
end_count = 0;
t0_data = 0;
ack_count = 0;
start_sent = 0;

for idx = 1:100000
    tx_sig = [];

    switch state
        case STATE_BEACON
            % Send BEACON periodically (every 5th loop)
            if mod(idx, 5) == 0
                tx_sig = beacon_wave;
                radio_tx.CenterFrequency = anchor_freq;
            end

        case STATE_DATA
            if start_sent < 3
                % Send START for 3 iterations to ensure RX gets it
                tx_sig = start_wave;
                radio_tx.CenterFrequency = anchor_freq;
                start_sent = start_sent + 1;
            elseif slot_ptr <= total_slots
                % Send data slot
                slot = slot_cache(slot_ptr);
                tx_sig = slot.waveform;
                radio_tx.CenterFrequency = defs.Carrier_set(slot.carrier_index);
                if slot_ptr == 1, t0_data = tic; end

                if mod(slot_ptr, 5) == 0 || slot_ptr == total_slots
                    fprintf('[DATA] 时隙 %d/%d | 频率=%.1f GHz | 帧数=%d\n', ...
                        slot_ptr, total_slots, ...
                        defs.Carrier_set(slot.carrier_index)/1e9, slot.num_frames);
                end
                slot_ptr = slot_ptr + 1;
            else
                fprintf('[DATA] 发送完成, 耗时=%.2f秒\n', toc(t0_data));
                state = STATE_END;
            end

        case STATE_END
            tx_sig = end_wave;
            radio_tx.CenterFrequency = anchor_freq;
            end_count = end_count + 1;
            if end_count > 20
                fprintf('[END] 已发送%d次, 结束.\n', end_count);
                state = STATE_DONE;
            end

        case STATE_DONE
            break;
    end

    % Transmit
    if ~isempty(tx_sig) && max(abs(tx_sig)) > 0
        if state == STATE_DATA && start_sent >= 3
            tx_sig = sqrt(Power) * 0.8 * tx_sig;  % scale data slots
        else
            tx_sig = sqrt(Power) * tx_sig;  % full power for control
        end
        radio_tx(tx_sig);
    end

    % Listen for feedback
    try
        [fb_sig, ~, overrun] = radio_rx();
        if overrun, warning('[WARN] 反馈溢出'); end
    catch ME
        warning('[ERR] 硬件: %s', ME.message);
        continue;
    end

    % Decode feedback (ACK, RESULT)
    [fb_ok, fb_data] = wifi_decode_ctrl(fb_sig);

    if fb_ok
        if fb_data.frame_type == 101  % ACK
            fprintf('[FB] 收到ACK | 类型=%d | 会话=%d\n', fb_data.frame_type, fb_data.session_id);

            if state == STATE_BEACON && fb_data.session_id == session_id
                fprintf('[SYNC] 握手成功! 开始数据传输...\n');
                state = STATE_DATA;
                slot_ptr = 1;
                start_sent = 0;
            end

        elseif fb_data.frame_type == 32  % RESULT
            fprintf('[FB] 收到RESULT | 会话=%d\n', fb_data.session_id);
            if state == STATE_END
                state = STATE_DONE;
            end
        end
    end

    % Status
    if mod(idx, 10) == 0
        sn = {'BEACON','DATA','END','DONE'};
        fprintf('[TX] 循环=%d | 状态=%s | 时隙=%d/%d\n', ...
            idx, sn{state+1}, min(slot_ptr,total_slots), total_slots);
    end

    if state == STATE_DONE, break; end
end

fprintf('[TX] 发射机关闭.\n');
release(radio_rx); release(radio_tx);

%% =========== Helper: WiFi Control Decoder ===========
function [valid, data] = wifi_decode_ctrl(rx_sig)
    valid = false;
    data = struct();

    % Parameters (must match encoder)
    sf_local = 15; sps_local = 4;
    pcmatrix = ldpcQuasiCyclicMatrix(27, ...
        [16 17 22 24 9 3 14 -1 4 2 7 -1 26 -1 2 -1 21 -1 1 0 -1 -1 -1 -1;
         25 12 12 3 3 26 6 21 -1 15 22 -1 15 -1 4 -1 -1 16 -1 0 0 -1 -1 -1;
         25 18 26 16 22 23 9 -1 0 -1 4 -1 4 -1 8 23 11 -1 -1 -1 0 0 -1 -1;
         9 7 0 1 17 -1 -1 7 3 -1 3 23 -1 16 -1 -1 21 -1 0 -1 -1 0 0 -1;
         24 5 26 7 1 -1 -1 15 24 15 -1 8 -1 13 -1 13 -1 11 -1 -1 -1 -1 0 0;
         2 2 19 14 24 1 15 19 -1 21 -1 2 -1 24 -1 3 -1 2 1 -1 -1 -1 -1 0]);
    cfgLDPCDec = ldpcDecoderConfig(pcmatrix);

    poly = 'z^32+z^26+z^23+z^22+z^16+z^12+z^11+z^10+z^8+z^7+z^5+z^4+z^2+z+1';
    crcdet = comm.CRCDetector(poly);
    demod = comm.PSKDemodulator(2, 'BitOutput', true, ...
        'DecisionMethod', 'Approximate log-likelihood ratio');
    demod.PhaseOffset = pi/4;

    pn = [1,-1,-1,-1,1,1,1,1,-1,1,-1,1,1,-1,-1]';
    scr = [1 1 0 1 1 0 1 0 0 1 0 0 0 0 1 0 1 0 1 1 1 0 1 1 0 0 0]';

    rec = complex(rx_sig(:));
    data_frame_len = 648 * sf_local;  % = 9720

    % Try to find LTF preamble via cross-correlation
    % Use a simpler approach: scan for energy above threshold
    rms_val = sqrt(mean(abs(rec).^2));
    if rms_val < 0.01, return; end  % no signal

    % Search for frame start by finding energy transition
    % Simplified: just try to decode at a few candidate positions
    best_valid = false;

    for phase = 1:sps_local
        ds = rec(phase:sps_local:end);
        if length(ds) < data_frame_len + 200, continue; end

        % Try positions where energy rises
        energy = movmean(abs(ds).^2, 50);
        [~, peaks] = findpeaks(energy, 'MinPeakHeight', 0.01, 'MinPeakDistance', 100);

        for p = 1:min(3, length(peaks))
            st = peaks(p);
            if st + data_frame_len > length(ds), continue; end

            seg = ds(st : st + data_frame_len - 1);

            % Phase correction using first 64 samples
            train = seg(1:64);
            train = train ./ (abs(train) + eps);
            ph_est = angle(mean(train));  % rough estimate
            seg_corrected = seg .* exp(-1j*ph_est);

            % Demodulate (BPSK)
            demod_sig = demod(seg_corrected);

            % Despread
            desp = zeros(length(demod_sig)/sf_local, 1);
            for ii = 1:length(demod_sig)/sf_local
                desp(ii) = sum(demod_sig((ii-1)*sf_local+1 : ii*sf_local) .* pn);
            end

            % Deinterleave
            deint_m = reshape(desp, 18, 36).';
            deint = deint_m(:);

            % LDPC decode
            rx_bits = ldpcDecode(deint, cfgLDPCDec, 10);

            % Descramble
            descr = zeros(length(rx_bits), 1);
            for ii = 1:floor(length(rx_bits)/length(scr))
                ix = (ii-1)*length(scr)+1 : ii*length(scr);
                descr(ix) = xor(rx_bits(ix), scr);
            end

            % CRC check
            [drec, err] = crcdet(descr(1:72));
            if err ~= 0, continue; end

            % Parse
            off = 0;
            data.frame_head = sum(2.^(7:-1:0)'.*drec(off+1:off+8)); off=off+8;
            data.user_id    = sum(2.^(7:-1:0)'.*drec(off+1:off+8)); off=off+8;
            data.frame_type = sum(2.^(7:-1:0)'.*drec(off+1:off+8)); off=off+8;
            data.session_id = sum(2.^(15:-1:0)'.*drec(off+1:off+16));

            valid = true;
            best_valid = true;
            break;
        end
        if best_valid, break; end
    end
end

%% =========== Cleanup ===========
function safe_release(tx, rx)
    try; if ~isempty(tx) && isvalid(tx), release(tx); end; catch; end
    try; if ~isempty(rx) && isvalid(rx), release(rx); end; catch; end
end
