% =========== One-Shot Frequency Hopping Transmitter ===========
% V2 Protocol: TX_BEACON -> RX_READY -> TX_START -> DATA_ONCE -> TX_END -> DONE
% Business data sent exactly once, no retransmission.
clear
clc
close all force
warning('off', 'all');
fprintf('\n========== 单次跳频发射机 ==========\n');

%% =========== Configuration ===========
defs = link_phy_defs();

% ---- Transmission Parameters ----
transmit_mode = 'image';  % 'text' | 'image' | 'video'
file_name = 'p2.jpg';
text_content = 'Hello, One-Shot FH!';

Anti_Jamming_Mode = 1;    % 0=QPSK, 1=BPSK+spreading (抗干扰+处理增益)
Power_gain = 30;   % OTA: increased for over-the-air
Power = 1.0;

fec_k = defs.fec_k_default;  % 24
fec_r = defs.fec_r_default;  % 8

hop_seed = randi(65535);

%% =========== State Machine Constants ===========
STATE_INIT           = 0;
STATE_WAIT_READY     = 1;
STATE_START_COUNTDOWN = 2;
STATE_DATA_ONCE      = 3;
STATE_END_LISTEN     = 4;
STATE_DONE           = 5;

FB_RX_SAMPLES = 80000;
BUS_SLOT_SAMPLES = defs.slot_len_samples;
CONTROL_SAMPLES = 40000;

BEACON_PERIOD = 5;        % loops between beacons
MIN_START_SENDS = 5;       % min START transmissions before accepting ACK
MAX_START_ATTEMPTS = 50;   % timeout (~30s) before falling back to WAIT_READY
END_REPEAT = 5;

%% =========== Phase 1: INIT - File Processing ===========
state = STATE_INIT;
fprintf('[TX-INIT] 正在处理文件...\n');

cfg = struct();
cfg.hop_seed = hop_seed;
cfg.fec_k = fec_k;
cfg.fec_r = fec_r;
if strcmp(transmit_mode, 'text')
    cfg.text_content = text_content;
    [business_bytes, meta_info] = build_file_container([], cfg);
else
    [business_bytes, meta_info] = build_file_container(file_name, cfg);
end

% Split into 40B source packets
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

fprintf('[TX-INIT] 文件: %d 字节 -> %d 个源数据包(每包40字节)\n', ...
    length(business_bytes), total_src_packets);

% FEC encoding
fec_groups = fec_rs_encode_groups(src_packets, fec_k, fec_r);
fprintf('[TX-INIT] 前向纠错: %d 组, K=%d, R=%d\n', length(fec_groups), fec_k, fec_r);

% Build V2 frames
[frame_list, fec_info] = build_forward_frames_v2(src_packets, meta_info, fec_groups);

% Pre-modulate all frames
[~, tx_cache] = forward_frame_modulate_v2(frame_list, Anti_Jamming_Mode, fec_info);

% Build hop slots
slot_cache = build_hop_slot_waveform(tx_cache, fec_info);

total_slots = fec_info.total_slots;
session_id = fec_info.session_id;

% Debug: print first 5 hop frequencies
hop_freqs_str = '';
for hi = 1:min(5, total_slots)
    hop_freqs_str = [hop_freqs_str sprintf('%.1f ', defs.Carrier_set(fec_info.hop_seq(hi))/1e9)];
end
fprintf('[TX-INIT] 就绪: 会话=%d | 时隙=%d | 跳频(前5)=%s| hop_seed=%d\n', ...
    session_id, total_slots, hop_freqs_str, meta_info.hop_seed);

%% =========== BPSK Handshake PHY Setup (from proven handshake_tx.m) ===========
fprintf('[TX-HS] 正在设置BPSK握手物理层...\n');
hs_sps = 4;
hs_sf = 15;
hs_M = 2;

pcmatrix = ldpcQuasiCyclicMatrix(defs.blockSize, defs.P);
hs_cfgLDPCEnc = ldpcEncoderConfig(pcmatrix);
hs_cfgLDPCDec = ldpcDecoderConfig(pcmatrix);
hs_crcgenerator = comm.CRCGenerator(defs.poly);
hs_crcdetector = comm.CRCDetector(defs.poly);

hs_qpskmod = comm.PSKModulator(hs_M, 'BitInput', true);
hs_qpskmod.PhaseOffset = pi/4;
hs_qpskdemod = comm.PSKDemodulator(hs_M, 'BitOutput', true, ...
    'DecisionMethod', 'Approximate log-likelihood ratio');
hs_qpskdemod.PhaseOffset = pi/4;
hs_txfilter = comm.RaisedCosineTransmitFilter('OutputSamplesPerSymbol', hs_sps, 'RolloffFactor', 0.25);
hs_rxfilter = comm.RaisedCosineReceiveFilter('InputSamplesPerSymbol', hs_sps, 'DecimationFactor', 1, 'RolloffFactor', 0.25);

hs_head_fb = [-1,-1,-1,-1,-1,-1,-1,-1,-1,1,-1,-1,-1,-1,-1,-1,1,-1,-1,1,-1,-1,-1,1,-1,-1,-1,-1,-1,1,1,-1,-1,1,-1,-1,1,1,-1,1,-1,-1,-1,-1,1,-1,-1,1,-1,1,-1,1,-1,-1,-1,-1,1,1,1,1,-1,1,-1,1,1,1,-1,1,-1,1,1,-1,1,1,-1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,-1,-1,-1,-1,-1,1,1,-1,1,1,-1,-1,1,1,-1,-1,-1,-1,1,-1,1,-1,1,1,-1,1,-1,1,1,1,-1,-1,-1,1,1,-1,1,1,1,1,1,1,-1,-1,-1,1,-1,-1,-1,1,1,1,1,-1,-1,1,1,1,1,-1,1,1,-1,1,1,-1,1,-1,-1,-1,-1,-1,-1,-1,1,-1,1,-1,-1,-1,-1,1,-1,1,1,-1,1,-1,1,-1,1,-1,-1,-1,1,1,1,1,1,-1,1,1,1,1,-1,-1,1,-1,-1,1,-1,1,1,-1,-1,-1,-1,-1,1,-1,-1,1,1,-1,-1,1,-1,-1,-1,1,-1,1,-1,-1,-1,1,1,-1,1,1,-1,1,1,1,-1,-1,-1,-1,-1,-1,1,1,1,1,-1,-1,-1,1,1,1,-1,1,1,1,1,1,1,1,-1,-1,1,-1,-1,-1,-1,1,1,-1,-1,-1,1,-1,1,1,-1,1,1,1,-1,1,-1,-1,-1,-1,1,1,-1,1,-1,1,-1,1,1,-1,-1,1,1,1,1,-1,-1,1,-1,1,1,-1,1,1,-1,-1,1,-1,-1,-1,-1,-1,1,-1,-1,-1,1,-1,-1,1,-1,-1,1,1,-1,-1,-1,-1,-1,-1,1,-1,1,1,-1,-1,-1,1,-1,1,-1,-1,1,1,1,-1,1,1,-1,-1,1,1,1,-1,-1,-1,1,-1,1,1,1,1,1,1,-1,1,-1,1,-1,-1,-1,1,-1,1,1,1,-1,1,1,-1,1,-1,1,1,-1,-1,-1,-1,1,1,-1,-1,1,1,-1,1,1,-1,1,-1,1,-1,-1,-1,-1,-1,1,1,1,-1,1,-1,-1,1,1,1,1,-1,1,-1,-1,1,1,-1,1,-1,1,-1,-1,1,-1,-1,1,1,1,-1,-1,-1,-1,-1,1,1,1,1,1,-1,-1,1,1,1,-1,-1,1,1,-1,1,1,1,1,-1,1,-1,-1,-1,1,-1,1,-1,1,-1,1,1,-1,1,1,1,1,1,-1,-1,-1,-1,1,-1,-1,1,1,1,-1,1,-1,-1,-1,1,1,1,-1,1,-1,1,1,1,1,1,-1,1,1,-1,1,-1,-1,1,-1,-1,-1,-1,1,-1,-1,-1,-1,1,-1,1,-1,-1,1,-1,1,-1,1,1,-1,-1,-1,1,1,1,-1,-1,1,1,1,1,1,1,1,-1,1,1,-1,-1,-1,-1,1,-1,-1,-1,1,1,-1,1,-1,-1,1,1,1,-1,-1,1,-1,-1,1,1,1,1,-1,-1,-1,-1,1,1,-1,1,1,1,-1,1,1,-1,-1,-1,1,1,-1,-1,-1,1,1,1,1,-1,1,1,1,1,1,-1,1,-1,-1,1,-1,-1,1,-1,1,-1,-1,-1,-1,-1,-1,1,1,-1,1,-1,-1,-1,1,1,-1,-1,1,-1,1,1,1,-1,1,-1,-1,1,-1,1,1,-1,1,-1,-1,-1,1,-1,-1,-1,1,-1,1,1,-1,-1,1,1,-1,1,-1,-1,1,-1,1,-1,-1,1,-1,-1,-1,1,1,-1,-1,-1,-1,1,1,1,-1,1,1,-1,1,1,1,1,-1,-1,-1,-1,-1,1,-1,1,1,1,-1,-1,1,-1,1,-1,1,1,1,-1,-1,1,1,1,-1,1,1,1,-1,1,1,1,-1,-1,1,1,-1,-1,1,1,1,-1,1,-1,1,-1,1,1,1,-1,1,1,1,1,-1,1,1,-1,-1,1,-1,1,-1,-1,-1,1,-1,-1,1,1,-1,1,1,-1,-1,-1,1,-1,-1,-1,-1,1,1,1,-1,-1,1,-1,1,1,1,1,1,-1,-1,1,-1,1,-1,-1,1,1,-1,-1,1,1,-1,-1,1,-1,1,-1,1,-1,1,-1,-1,1,1,1,1,1,1,-1,-1,1,1,-1,-1,-1,1,1,-1,1,-1,1,1,1,1,-1,-1,1,1,-1,1,-1,1,1,-1,1,-1,-1,1,1,-1,-1,-1,1,-1,-1,1,-1,1,1,1,-1,-1,-1,-1,1,-1,1,1,1,1,-1,1,-1,1,-1,1,-1,1,-1,1,1,1,1,1,1,1,1,-1,1,-1,-1,-1,-1,-1,1,-1,1,-1,1,-1,-1,1,-1,1,1,1,1,-1,-1,-1,1,-1,1,-1,1,1,1,1,-1,1,1,1,-1,1,-1,1,-1,-1,1,1,-1,1,1,1,-1,-1,1,-1,-1,-1,1,1,1,-1,-1,-1,1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,1,1,1,-1,-1,-1,-1,1,1,1,1,1,1,-1,1,1,1,-1,-1,-1,1,-1,-1,1,1,1,1,1,-1,-1,-1,1,1,-1,-1,1,1,1,1,1,-1,1,-1,1,1,-1,-1,1,-1,1,1,-1,-1,1,-1,-1,1,-1,-1,1]';
hs_pn_fb = [1,-1,-1,-1,1,1,1,1,-1,1,-1,1,1,-1,-1]';
hs_scr_seq = [1 1 0 1 1 0 1 0 0 1 0 0 0 0 1 0 1 0 1 1 1 0 1 1 0 0 0]';

hs_Frame_head = [1;1;1;0;1;0;1;0];
hs_Usr_ID = [0;0;0;0;0;1;0;1];
hs_Frame_type_beacon = double(dec2bin(100, 8) == '1')';
hs_Frame_type_start  = double(dec2bin(41, 8) == '1')';
hs_Frame_type_end    = double(dec2bin(42, 8) == '1')';
hs_Session_ID = double(dec2bin(session_id, 16) == '1')';

%% Pre-build BEACON waveform (BPSK+spreading, same as proven handshake)
fprintf('[TX-HS] 正在构建BEACON波形...\n');
hs_payload_beacon = [hs_Frame_head; hs_Usr_ID; hs_Frame_type_beacon; hs_Session_ID];
hs_enc_beacon = hs_crcgenerator(hs_payload_beacon);
hs_pad_len = 486 - length(hs_enc_beacon);
hs_payload_frame_beacon = [hs_enc_beacon; zeros(hs_pad_len, 1)];

hs_scr_beacon = scramble_bits_hs(hs_payload_frame_beacon, hs_scr_seq);
hs_enc_bits_beacon = ldpcEncode(hs_scr_beacon, hs_cfgLDPCEnc);
hs_inter_matrix_beacon = reshape(hs_enc_bits_beacon, 36, 18).';
hs_inter_bits_beacon = hs_inter_matrix_beacon(:);

hs_inter_polar_beacon = 2*hs_inter_bits_beacon - 1;
hs_spread_beacon = zeros(length(hs_inter_polar_beacon)*hs_sf, 1);
for ii = 1:length(hs_inter_polar_beacon)
    hs_spread_beacon((ii-1)*hs_sf+1 : ii*hs_sf) = hs_inter_polar_beacon(ii) * hs_pn_fb;
end
hs_mod_beacon = hs_qpskmod(0.5*(hs_spread_beacon + 1));
hs_tx_in_beacon = [hs_head_fb; hs_mod_beacon; zeros(hs_sps*10, 1)];
hs_beacon_wave_full = hs_txfilter(hs_tx_in_beacon);
hs_beacon_wave_full = [zeros(2000, 1); hs_beacon_wave_full];
fprintf('[TX-HS] BEACON波形: %d 采样点 (%.2f 毫秒)\n', ...
    length(hs_beacon_wave_full), length(hs_beacon_wave_full)/200e6*512*1000);

%% Pre-build START control waveform (carries hop_seed, total_slots, etc.)
fprintf('[TX-HS] 正在构建START波形...\n');
hs_start_info_bits = [ ...
    int_to_bits_hs(meta_info.hop_seed, 32); ...
    int_to_bits_hs(total_slots, 16); ...
    int_to_bits_hs(meta_info.slot_len_samples, 32); ...
    int_to_bits_hs(fec_info.codewords_per_slot, 16); ...
    int_to_bits_hs(meta_info.fec_k, 8); ...
    int_to_bits_hs(meta_info.fec_r, 8)];

hs_payload_start = [hs_Frame_head; hs_Usr_ID; hs_Frame_type_start; hs_Session_ID; hs_start_info_bits];
hs_enc_start = hs_crcgenerator(hs_payload_start);
hs_pad_len_start = 486 - length(hs_enc_start);
hs_payload_frame_start = [hs_enc_start; zeros(hs_pad_len_start, 1)];

hs_scr_start = scramble_bits_hs(hs_payload_frame_start, hs_scr_seq);
hs_enc_bits_start = ldpcEncode(hs_scr_start, hs_cfgLDPCEnc);
hs_inter_matrix_start = reshape(hs_enc_bits_start, 36, 18).';
hs_inter_bits_start = hs_inter_matrix_start(:);

hs_inter_polar_start = 2*hs_inter_bits_start - 1;
hs_spread_start = zeros(length(hs_inter_polar_start)*hs_sf, 1);
for ii = 1:length(hs_inter_polar_start)
    hs_spread_start((ii-1)*hs_sf+1 : ii*hs_sf) = hs_inter_polar_start(ii) * hs_pn_fb;
end
hs_mod_start = hs_qpskmod(0.5*(hs_spread_start + 1));
hs_tx_in_start = [hs_head_fb; hs_mod_start; zeros(hs_sps*10, 1)];
hs_start_wave_full = hs_txfilter(hs_tx_in_start);
hs_start_wave_full = [zeros(2000, 1); hs_start_wave_full];
fprintf('[TX-HS] START波形: %d 采样点 (%.2f 毫秒)\n', ...
    length(hs_start_wave_full), length(hs_start_wave_full)/200e6*512*1000);

%% Pre-build END waveform
hs_payload_end = [hs_Frame_head; hs_Usr_ID; hs_Frame_type_end; hs_Session_ID; ...
    zeros(112, 1)];  % same structure as START, padded with zeros
hs_enc_end = hs_crcgenerator(hs_payload_end);
hs_pad_len_end = 486 - length(hs_enc_end);
hs_payload_frame_end = [hs_enc_end; zeros(hs_pad_len_end, 1)];

hs_scr_end = scramble_bits_hs(hs_payload_frame_end, hs_scr_seq);
hs_enc_bits_end = ldpcEncode(hs_scr_end, hs_cfgLDPCEnc);
hs_inter_matrix_end = reshape(hs_enc_bits_end, 36, 18).';
hs_inter_bits_end = hs_inter_matrix_end(:);

hs_inter_polar_end = 2*hs_inter_bits_end - 1;
hs_spread_end = zeros(length(hs_inter_polar_end)*hs_sf, 1);
for ii = 1:length(hs_inter_polar_end)
    hs_spread_end((ii-1)*hs_sf+1 : ii*hs_sf) = hs_inter_polar_end(ii) * hs_pn_fb;
end
hs_mod_end = hs_qpskmod(0.5*(hs_spread_end + 1));
hs_tx_in_end = [hs_head_fb; hs_mod_end; zeros(hs_sps*10, 1)];
hs_end_wave_full = hs_txfilter(hs_tx_in_end);
hs_end_wave_full = [zeros(2000, 1); hs_end_wave_full];

%% Anchor / feedback frequencies (matching proven handshake)
hs_anchor_freq = 2.5e9;
hs_feedback_freq = 1.45e9;

%% =========== SDR Initialization ===========
fprintf('[TX-HW] 正在初始化USRP...\n');

radio_tx = comm.SDRuTransmitter('Platform', 'X310', 'IPAddress', '192.168.10.2');
radio_tx.ChannelMapping = 1;
radio_tx.CenterFrequency = hs_anchor_freq;
radio_tx.Gain = Power_gain;
radio_tx.MasterClockRate = 200e6;
radio_tx.InterpolationFactor = 512;
radio_tx.ClockSource = 'External';

radio_rx = comm.SDRuReceiver( ...
    'Platform', 'X310', ...
    'IPAddress', '192.168.10.2', ...
    'OutputDataType', 'double', ...
    'MasterClockRate', 200e6, ...
    'DecimationFactor', 512, ...
    'SamplesPerFrame', FB_RX_SAMPLES);
radio_rx.ClockSource = 'External';
radio_rx.ChannelMapping = 1;
radio_rx.CenterFrequency = hs_feedback_freq;
radio_rx.Gain = 30;

cleanupObj = onCleanup(@() safe_release(radio_tx, radio_rx));
fprintf('[TX-HW] USRP就绪.\n');

%% =========== UI Configuration ===========
tx_ui.enable = true;
tx_ui.url = 'http://127.0.0.1:5001';
tx_ui.health_endpoint = '/api/control';
tx_ui.post_period = 10;
tx_ui.ctrl_period = 20;
tx_ui.timeout = 0.03;

state = STATE_WAIT_READY;
beacon_count = 0;
start_count = 0;
end_count = 0;
slot_ptr = 1;
tx_duration = 0;
freq_received = false;
pending_carrier = 0;
last_freq_time = tic;

%% =========== Synchronized Start ===========
t_now = datetime('now');
sec_current = second(t_now);
sec_target = ceil(sec_current / 5) * 5;  % next 5-second boundary
if sec_target - sec_current < 1
    sec_target = sec_target + 5;
end
wait_secs = sec_target - sec_current;
fprintf('[SYNC] 当前时间: %s\n', datestr(t_now, 'HH:MM:SS.FFF'));
fprintf('[SYNC] 预定开始: %s (%.1f秒后)\n', ...
    datestr(t_now + seconds(wait_secs), 'HH:MM:SS'), wait_secs);
fprintf('[SYNC] 等待中... 请确保RX也在同步等待\n');
pause(wait_secs);
fprintf('[SYNC] 开始! 进入等待就绪状态, 在%.2f GHz发送BEACON\n', hs_anchor_freq/1e9);

%% =========== Main Loop ===========
for idx = 1:100000
    tx_sig = zeros(BUS_SLOT_SAMPLES, 1);
    fb_sig = zeros(FB_RX_SAMPLES, 1);
    use_bus_slot = false;

    % ---- State Machine ----
    switch state
        case STATE_WAIT_READY
            % Periodic BEACON using proven BPSK+spreading handshake
            if mod(beacon_count, BEACON_PERIOD) == 0
                tx_sig = hs_beacon_wave_full;
                radio_tx.CenterFrequency = hs_anchor_freq;
            else
                tx_sig = zeros(CONTROL_SAMPLES, 1);
            end
            beacon_count = beacon_count + 1;

        case STATE_START_COUNTDOWN
            % Send START repeatedly until RX confirms via ACK
            tx_sig = hs_start_wave_full;
            radio_tx.CenterFrequency = hs_anchor_freq;
            start_count = start_count + 1;

            % Timeout: fall back to WAIT_READY if no ACK after many attempts
            if start_count > MAX_START_ATTEMPTS
                fprintf('[TX] START超时(%d次), 退回等待就绪...\n', start_count);
                state = STATE_WAIT_READY;
                start_count = 0;
            end

        case STATE_DATA_ONCE
            % ACK-driven: wait for RX frequency instruction before each slot
            if slot_ptr <= total_slots
                if freq_received
                    slot = slot_cache(slot_ptr);
                    tx_sig = slot.waveform;
                    radio_tx.CenterFrequency = defs.Carrier_set(pending_carrier);
                    use_bus_slot = true;
                    if slot_ptr == 1 || mod(slot_ptr, 5) == 0 || slot_ptr == total_slots
                        fprintf('[TX-DATA] 时隙 %d/%d | 频率=%.1f GHz (RX指定) | 帧数=%d\n', ...
                            slot_ptr, total_slots, ...
                            defs.Carrier_set(pending_carrier)/1e9, ...
                            slot.num_frames);
                    end
                    slot_ptr = slot_ptr + 1;
                    freq_received = false;
                    last_freq_time = tic;
                else
                    tx_sig = zeros(CONTROL_SAMPLES, 1);
                    use_bus_slot = false;
                    if toc(last_freq_time) > 10.0 && slot_ptr == 1
                        fprintf('[TX-DATA] 等待RX频率指令超时(10s), 回退...\n');
                        state = STATE_WAIT_READY;
                    end
                end
            else
                slot_dur = BUS_SLOT_SAMPLES / (200e6/512);
                pause(slot_dur);
                tx_duration = toc(t0_data);
                fprintf('[TX-DATA] 所有时隙已发送, 耗时=%.2f 秒\n', tx_duration);
                state = STATE_END_LISTEN;
            end

        case STATE_END_LISTEN
            % Send TX_END using proven BPSK+spreading, listen for RESULT
            tx_sig = hs_end_wave_full;
            radio_tx.CenterFrequency = hs_anchor_freq;
            end_count = end_count + 1;
            if end_count > 30
                fprintf('[TX] END已发送%d次, 未收到RESULT, 结束.\n', end_count);
                state = STATE_DONE;
            end

        case STATE_DONE
            tx_sig = zeros(BUS_SLOT_SAMPLES, 1);
    end

    % ---- Transmit and Listen ----
    % Scale: 0.8 for OFDM data slots (high PAPR), 1.0 for handshake (BPSK)
    if use_bus_slot
        tx_sig = sqrt(Power) * tx_sig;  % full power for data too
    else
        tx_sig = sqrt(Power) * tx_sig;
    end

    try
        if use_bus_slot
            % DATA_ONCE: radio_tx是非阻塞的, USRP实际发送一帧需~0.41s
            % 必须等USRP发完再发下一时隙，否则缓冲区溢出
            radio_tx(tx_sig);
            pause(0.35);
        elseif state == STATE_WAIT_READY || state == STATE_START_COUNTDOWN || state == STATE_END_LISTEN || state == STATE_DATA_ONCE
            % Handshake + DATA_ONCE-wait states: transmit only if real signal present
            if max(abs(tx_sig)) > 0
                radio_tx(tx_sig);
            end
            if state == STATE_DATA_ONCE
                pause(0.01);  % short pause when waiting for ACK
            end
        else
            % Other states: pad to bus slot size
            pad_sig = zeros(BUS_SLOT_SAMPLES, 1);
            pad_sig(1:min(length(tx_sig), BUS_SLOT_SAMPLES)) = tx_sig(1:min(length(tx_sig), BUS_SLOT_SAMPLES));
            radio_tx(pad_sig);
        end
        [fb_sig, ~, rx_overrun] = radio_rx();
        if rx_overrun
            warning('[TX-WARN] 反馈通道溢出');
        end
    catch ME
        warning('[TX-ERR] 硬件错误: %s', ME.message);
        continue;
    end

    % ---- Decode Feedback (proven decode_ack from handshake_tx.m) ----
    [fb_valid, fb_data] = decode_ack_hs(fb_sig, hs_rxfilter, hs_head_fb, hs_pn_fb, ...
        hs_scr_seq, hs_cfgLDPCDec, hs_crcdetector, hs_qpskdemod, hs_sps, hs_sf);

    if fb_valid
        fprintf('[TX-FB] 收到ACK: 类型=%d | 会话=%d\n', ...
            fb_data.frame_type, fb_data.session_id);

        % Handle ACK (frame_type == 101) from handshake
        % Require session_id match to avoid false trigger from RX discovery blips
        if fb_data.frame_type == 101 && fb_data.session_id == session_id
            if state == STATE_WAIT_READY
                fprintf('[TX] 收到ACK, 开始发送START...\n');
                state = STATE_START_COUNTDOWN;
                start_count = 0;
            elseif state == STATE_START_COUNTDOWN && start_count >= MIN_START_SENDS
                fprintf('[TX] RX已确认START(第%d次ACK), 开始数据传输...\n', start_count);
                state = STATE_DATA_ONCE;
                slot_ptr = 1;
                t0_data = tic;
                freq_received = false;
                last_freq_time = tic;
            elseif state == STATE_DATA_ONCE && fb_data.next_carrier > 0
                pending_carrier = fb_data.next_carrier;
                freq_received = true;
                last_freq_time = tic;
                fprintf('[TX-FB] RX指定频率: %.1f GHz (时隙=%d)\n', ...
                    defs.Carrier_set(pending_carrier)/1e9, fb_data.slot_ack);
            end
        end

        % Handle RX_RESULT (frame_type == 32)
        if fb_data.frame_type == 32 && state == STATE_END_LISTEN
            fprintf('[TX] 收到接收机结果, 传输完成.\n');
            state = STATE_DONE;
        end
    end

    % ---- Periodic Status ----
    if mod(idx, 10) == 0
        state_names = {'INIT', 'WAIT_READY', 'START_COUNTDOWN', 'DATA_ONCE', 'END_LISTEN', 'DONE'};
        fprintf('[TX] 循环=%d | 状态=%s | 时隙=%d/%d\n', ...
            idx, state_names{state+1}, min(slot_ptr, total_slots), total_slots);
    end

    % ---- Exit Conditions ----
    if state == STATE_DONE
        fprintf('[TX] 会话完成, 退出.\n');
        break;
    end

end

release(radio_rx);
release(radio_tx);
fprintf('[TX] 发射机关闭完成.\n');

%% =========== Helper Functions ===========
function sig = build_control_frame(session_id, frame_type, meta_info, fec_info, slot_len)
% Build a control frame (BEACON/START/END) using BPSK+spreading for robustness
defs = link_phy_defs();
sps = 4;

pcmatrix = ldpcQuasiCyclicMatrix(defs.blockSize, defs.P);
cfgLDPCEnc = ldpcEncoderConfig(pcmatrix);
crcgenerator = comm.CRCGenerator(defs.poly);

bpskmod = comm.PSKModulator(2, 'BitInput', true);
bpskmod.PhaseOffset = pi/4;

txfilter = comm.RaisedCosineTransmitFilter( ...
    'OutputSamplesPerSymbol', sps, 'RolloffFactor', 0.25);

% Control frame payload: FrameHead(8) + UserID(8) + FrameType(8) + ProtoVer(4)
% + SessionID(16) + HopSeed(32) + TotalSlots(16) + SlotLen(32) + CodewordsPerSlot(16)
% + Countdown(8) = ~148 bits + CRC32 + padding

hop_seed_bits = int_to_bits(meta_info.hop_seed, 32);
slot_len_bits = int_to_bits(meta_info.slot_len_samples, 32);
cw_per_slot_bits = int_to_bits(fec_info.codewords_per_slot, 16);

% Use actual values from fec_info
total_slots_actual = fec_info.total_slots;

ctrl_bits = [ ...
    defs.frame_head; ...
    defs.user_id; ...
    int_to_bits(frame_type, 8); ...
    int_to_bits(defs.proto_ver, 4); ...
    int_to_bits(4, 4); ...                    % control header len
    int_to_bits(session_id, 16); ...
    hop_seed_bits; ...
    int_to_bits(total_slots_actual, 16); ...
    slot_len_bits; ...
    cw_per_slot_bits; ...
    int_to_bits(meta_info.fec_k, 8); ...
    int_to_bits(meta_info.fec_r, 8)];

coded = [crcgenerator(ctrl_bits); defs.ctrl_frame_end];
scr_bits = scramble_bits_ctrl(coded, defs.scr_seq);
enc_bits = ldpcEncode(scr_bits, cfgLDPCEnc);
inter_matrix = reshape(enc_bits, 36, 18).';
inter_bits = inter_matrix(:);

inter_polar = 2 * inter_bits - 1;
spread = zeros(length(inter_polar)*15, 1);
for ii = 1:length(inter_polar)
    spread((ii-1)*15+1:ii*15) = inter_polar(ii) * defs.pn_data;
end

mod_sig = bpskmod(0.5*(spread+1));
tx_in = [defs.head_data; mod_sig; zeros(sps*10,1)];
one_wave = txfilter(tx_in);

% Pad/trim to slot length
sig = zeros(slot_len, 1);
L = min(length(one_wave), slot_len);
sig(1:L) = one_wave(1:L);
end

function bits = int_to_bits(v, width)
bits = double(dec2bin(max(0, v), width) == '1').';
end

function out = scramble_bits_ctrl(in, scr_seq)
out = zeros(size(in));
grp = length(scr_seq);
for ii = 1:floor(length(in)/grp)
    st = (ii-1)*grp + 1;
    ed = ii*grp;
    out(st:ed) = xor(in(st:ed), scr_seq);
end
end

function safe_release(tx, rx)
try
    if ~isempty(tx) && isvalid(tx), release(tx); end
catch
end
try
    if ~isempty(rx) && isvalid(rx), release(rx); end
catch
end
disp('SDR资源已释放.');
end

%% =========== Handshake Helper Functions (from proven handshake_tx.m) ===========

function [valid, data] = decode_ack_hs(rx_sig, rxfilter, head_fb, pn_fb, ...
    scr_seq, cfgLDPCDec, crcdetector, qpskdemod, sps, sf)
% Decode ACK feedback frame using proven BPSK+spreading approach
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

    data_desp = zeros(length(demod_sig)/sf, 1);
    for ii = 1:length(demod_sig)/sf
        data_desp(ii) = sum(demod_sig((ii-1)*sf+1 : ii*sf) .* pn_fb);
    end

    deinter_matrix = reshape(data_desp, 18, 36).';
    deinter_bits = deinter_matrix(:);

    rx_bits = ldpcDecode(deinter_bits, cfgLDPCDec, maxnumiter);

    descr_data = zeros(length(rx_bits), 1);
    for ii = 1:floor(length(rx_bits)/length(scr_seq))
        st_ = (ii-1)*length(scr_seq) + 1;
        ed_ = ii*length(scr_seq);
        descr_data(st_:ed_) = xor(rx_bits(st_:ed_), scr_seq);
    end

    % Try short format first (BEACON/ACK: 40 info + 32 CRC = 72 bits)
    [data_rec, err] = crcdetector(descr_data(1:72));
    if err == 0
        offset = 0;
        data.frame_head = bits2int_hs(data_rec(offset+1:offset+8)); offset = offset+8;
        data.user_id    = bits2int_hs(data_rec(offset+1:offset+8)); offset = offset+8;
        data.frame_type = bits2int_hs(data_rec(offset+1:offset+8)); offset = offset+8;
        data.session_id = bits2int_hs(data_rec(offset+1:offset+16));
        data.next_carrier = 0;
        data.slot_ack = 0;
        valid = true;
        return;
    end

    % Try long format (ACK_FREQ: 152 info + 32 CRC = 184 bits)
    [data_rec2, err2] = crcdetector(descr_data(1:184));
    if err2 == 0
        offset = 0;
        data.frame_head = bits2int_hs(data_rec2(offset+1:offset+8)); offset = offset+8;
        data.user_id    = bits2int_hs(data_rec2(offset+1:offset+8)); offset = offset+8;
        data.frame_type = bits2int_hs(data_rec2(offset+1:offset+8)); offset = offset+8;
        data.session_id = bits2int_hs(data_rec2(offset+1:offset+16)); offset = offset+16;
        data.next_carrier = bits2int_hs(data_rec2(offset+1:offset+8)); offset = offset+8;
        data.slot_ack = bits2int_hs(data_rec2(offset+1:offset+16));
        valid = true;
        return;
    end
end
end

function out = scramble_bits_hs(in, scr_seq)
out = zeros(size(in));
grp = length(scr_seq);
for ii = 1:floor(length(in)/grp)
    st = (ii-1)*grp + 1;
    ed = ii*grp;
    out(st:ed) = xor(in(st:ed), scr_seq);
end
end

function bits = int_to_bits_hs(v, width)
bits = double(dec2bin(max(0, v), width) == '1').';
end

function v = bits2int_hs(bits)
v = (2.^(length(bits)-1:-1:0)) * bits(:);
end
