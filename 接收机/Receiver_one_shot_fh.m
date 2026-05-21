% =========== One-Shot Frequency Hopping Receiver ===========
% V2 Protocol: WAIT_BEACON -> READY_SENT -> FOLLOW_HOP -> FEC_REBUILD -> RESULT_REPORT
% Receives business data exactly once, uses FEC for recovery, no retransmission requests.
clear
clc
close all force
warning('off', 'all');
fprintf('\n========== 单次跳频接收机 ==========\n');

%% =========== Configuration ===========
defs = link_phy_defs();

Anti_Jamming_Mode = 0;  % 0=QPSK, 1=BPSK+spreading
Threshold = 180;  % data slot preamble detection threshold
Threshold_FB = 220;
BUS_RX_SAMPLES = defs.slot_len_samples;
FB_TX_SAMPLES = 80000;
CONTROL_RX_SAMPLES = 80000;  % must be > beacon length (~43084 samples)

TELEMETRY_PERIOD_LOOPS = 10;  % Send telemetry every N data slots

% ---- Transmission Mode ----
SKIP_HANDSHAKE = false;       % true=跳过握手直接监听数据
SINGLE_FREQ_MODE = true;      % true=单频率传输(不跳频), false=跳频
SINGLE_FREQ = 2.5e9;          % 单频率模式下的载波频率(与TX一致)
FIXED_HOP_SEED = 12345;       % 无握手模式下的固定跳频种子(与TX一致)

if SINGLE_FREQ_MODE
    defs.Carrier_set = SINGLE_FREQ;
    defs.num_carriers = 1;
    fprintf('[RX] 单频率模式: %.1f GHz (不跳频)\n', SINGLE_FREQ/1e9);
end
RX_IDLE_TIMEOUT = 5;        % 无握手模式下连续无数据超时秒数
MAX_IDLE_SLOTS = 10;        % 无握手模式下连续空闲时隙数(超过则判定TX已结束)

%% =========== State Machine Constants ===========
STATE_WAIT_BEACON  = 0;
STATE_READY_SENT   = 1;
STATE_FOLLOW_HOP   = 2;
STATE_FEC_REBUILD  = 3;
STATE_RESULT_REPORT = 4;
STATE_DONE         = 5;

state = STATE_WAIT_BEACON;

%% =========== BPSK Handshake PHY Setup (matching proven handshake_tx.m) ===========
fprintf('[RX-HS] 正在设置BPSK握手物理层...\n');
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
hs_Frame_type_ack    = double(dec2bin(101, 8) == '1')';
hs_Frame_type_ready  = double(dec2bin(30, 8) == '1')';
hs_Frame_type_telem  = double(dec2bin(31, 8) == '1')';
hs_Frame_type_result = double(dec2bin(32, 8) == '1')';

%% Pre-build READY waveform for discovery (BPSK+spreading, same as proven handshake)
fprintf('[RX-HS] 正在构建READY波形...\n');
hs_payload_ready = [hs_Frame_head; hs_Usr_ID; hs_Frame_type_ready; zeros(16, 1)];  % READY=30, session_id=0
hs_enc_ready = hs_crcgenerator(hs_payload_ready);
hs_pad_len_ready = 486 - length(hs_enc_ready);
hs_payload_frame_ready = [hs_enc_ready; zeros(hs_pad_len_ready, 1)];

hs_scr_ready = scramble_bits_hs(hs_payload_frame_ready, hs_scr_seq);
hs_enc_bits_ready = ldpcEncode(hs_scr_ready, hs_cfgLDPCEnc);
hs_inter_matrix_ready = reshape(hs_enc_bits_ready, 36, 18).';
hs_inter_bits_ready = hs_inter_matrix_ready(:);

hs_inter_polar_ready = 2*hs_inter_bits_ready - 1;
hs_spread_ready = zeros(length(hs_inter_polar_ready)*hs_sf, 1);
for ii = 1:length(hs_inter_polar_ready)
    hs_spread_ready((ii-1)*hs_sf+1 : ii*hs_sf) = hs_inter_polar_ready(ii) * hs_pn_fb;
end
hs_mod_ready = hs_qpskmod(0.5*(hs_spread_ready + 1));
hs_tx_in_ready = [hs_head_fb; hs_mod_ready; zeros(hs_sps*10, 1)];
hs_ready_wave_full = hs_txfilter(hs_tx_in_ready);
hs_ready_wave_full = [zeros(2000, 1); hs_ready_wave_full];
fprintf('[RX-HS] READY波形: %d 采样点 (%.2f 毫秒)\n', ...
    length(hs_ready_wave_full), length(hs_ready_wave_full)/200e6*512*1000);

% Placeholder ACK - will be rebuilt with correct session_id after BEACON detection
hs_ack_wave_full = hs_ready_wave_full;  % temporary, replaced on BEACON detect

%% Anchor / feedback frequencies (matching proven handshake)
hs_anchor_freq = 2.5e9;
hs_feedback_freq = 1.45e9;

%% =========== Session Variables ===========
session_id = 0;
hop_seed = 0;
total_slots = 0;
slot_len_samples = defs.slot_len_samples;
codewords_per_slot = defs.codewords_per_slot_default;
hop_seq = [];

frame_cache = struct();
frame_cache = rx_frame_cache_update(frame_cache, [], 0);

data_start_time = 0;
late_join = 0;
feedback_seq = 0;
ready_discovery_count = 0;
slot_ptr = 1;
last_sync_time = 0;
ack_burst_remaining = 0;
idle_slot_count = 0;
phy_metrics = [];  % store latest physical metrics

%% =========== SDR Initialization ===========
disp('[RX-HW] 强制释放残留USRP句柄...');
% Force-clear any stuck USRP handles from previous runs
try
    old_radios = instrfindall('Type', 'usrp');
    if ~isempty(old_radios)
        for r = 1:length(old_radios)
            try release(old_radios(r)); catch; end
        end
    end
catch
end
% Additional safety: clear persistent SDR connections
try
    comm.internal.SDRuBase.closeAllSessions();
catch
end
pause(1);  % let hardware fully release

disp('[RX-HW] 正在初始化USRP...');

radio_tx = comm.SDRuTransmitter('Platform', 'X310', 'IPAddress', '192.168.10.2');
radio_tx.ChannelMapping = 1;
radio_tx.CenterFrequency = hs_feedback_freq;
radio_tx.Gain = 30;
radio_tx.MasterClockRate = 200e6;
radio_tx.InterpolationFactor = 512;
radio_tx.ClockSource = 'External';

radio_rx = comm.SDRuReceiver( ...
    'Platform', 'X310', ...
    'IPAddress', '192.168.10.2', ...
    'OutputDataType', 'double', ...
    'MasterClockRate', 200e6, ...
    'DecimationFactor', 512, ...
    'SamplesPerFrame', CONTROL_RX_SAMPLES);  % start in WAIT_BEACON mode (40k)
radio_rx.ClockSource = 'External';
radio_rx.ChannelMapping = 1;
radio_rx.CenterFrequency = hs_anchor_freq;
radio_rx.Gain = 30;

cleanupObj = onCleanup(@() safe_release(radio_tx, radio_rx));
disp('[RX-HW] USRP就绪.');

% Data slot RRC receive filter (created once, shared across all slots)
data_rxfilter = comm.RaisedCosineReceiveFilter( ...
    'InputSamplesPerSymbol', 4, ...
    'DecimationFactor', 1, ...
    'RolloffFactor', 0.25);

%% =========== UI Configuration ===========
rx_ui.enable = true;
rx_ui.url = 'http://127.0.0.1:5000';
rx_ui.health_endpoint = '/api/health';
rx_ui.post_period = 10;
rx_ui.ctrl_period = 20;
rx_ui.timeout = 0.03;


%% =========== No-Handshake Initialization ===========
if SKIP_HANDSHAKE
    hop_seed = FIXED_HOP_SEED;
    total_slots = 999;  % large number, will stop on idle timeout
    codewords_per_slot = defs.codewords_per_slot_default;
    slot_len_samples = defs.slot_len_samples;
    hop_seq = build_hop_sequence(hop_seed, total_slots, defs.num_carriers);

    frame_cache = struct();
    frame_cache = rx_frame_cache_update(frame_cache, [], 0);

    % Switch RX to data slot sample size immediately
    release(radio_rx);
    radio_rx.SamplesPerFrame = BUS_RX_SAMPLES;

    state = STATE_FOLLOW_HOP;
    slot_ptr = 1;
    idle_slot_count = 0;
    last_sync_time = tic;
    data_start_time = tic;

    fprintf('[RX] 跳过握手模式, hop_seed=%d, 直接监听跳频数据...\n', hop_seed);
end

%% =========== Main Loop ===========
rx_diag_count = 0;
if ~SKIP_HANDSHAKE
    state = STATE_WAIT_BEACON;
end
for idx = 1:100000
    tx_sig = zeros(FB_TX_SAMPLES, 1);

    % ---- State Machine ----
    switch state
        case STATE_WAIT_BEACON
            % Listen on anchor frequency for BEACON (RX already configured for CONTROL_RX_SAMPLES)
            radio_rx.CenterFrequency = hs_anchor_freq;

            try
                [rx_sig, ~, rx_overrun] = radio_rx();
                if rx_overrun, warning('[RX-WARN] 接收溢出'); end
            catch ME
                warning('[RX-ERR] 硬件错误: %s', ME.message);
                pause(0.05);  % brief backoff on error
                continue;
            end

            % Try to decode BEACON using proven BPSK handshake decoder
            rx_diag_count = rx_diag_count + 1;
            if mod(rx_diag_count, 20) == 0
                fprintf('[RX-DIAG] #%d | 有效值=%.4f | 峰值=%.4f\n', ...
                    rx_diag_count, rms(rx_sig), max(abs(rx_sig)));
            end
            [ctrl_valid, ctrl_data] = decode_ctrl_hs(rx_sig, hs_rxfilter, hs_head_fb, ...
                hs_pn_fb, hs_scr_seq, hs_cfgLDPCDec, hs_crcdetector, hs_qpskdemod, hs_sps, hs_sf);

            if ctrl_valid && ctrl_data.frame_type == 100  % BEACON
                session_id = ctrl_data.session_id;
                fprintf('[RX] 检测到BEACON 循环%d: 会话=%d\n', idx, session_id);

                % Build ACK with correct session_id (not the pre-built placeholder)
                hs_ack_wave_full = build_ctrl_wave_hs(session_id, 101, hs_head_fb, ...
                    hs_pn_fb, hs_scr_seq, hs_cfgLDPCEnc, hs_crcgenerator, ...
                    hs_qpskmod, hs_txfilter, hs_sps, hs_sf);
                tx_sig = hs_ack_wave_full;
                radio_tx.CenterFrequency = hs_feedback_freq;

                state = STATE_READY_SENT;
                slot_ptr = 1;
                ready_discovery_count = 0;
                ack_burst_remaining = 5;  % send ACK bursts to ensure TX receives it
            elseif ctrl_valid && ctrl_data.frame_type == 41  % START (direct, beacon skipped)
                session_id = ctrl_data.session_id;
                hop_seed = ctrl_data.hop_seed;
                total_slots = ctrl_data.total_slots;
                slot_len_samples = ctrl_data.slot_len;
                codewords_per_slot = ctrl_data.codewords_per_slot;

                fprintf('[RX] 直接收到START: 会话=%d | 跳频种子=%d | 时隙=%d\n', ...
                    session_id, hop_seed, total_slots);

                % Generate hop sequence, init cache
                hop_seq = build_hop_sequence(hop_seed, total_slots, defs.num_carriers);
                frame_cache = struct();
                frame_cache = rx_frame_cache_update(frame_cache, [], session_id);

                % Send ACK to confirm START receipt
                tx_sig = hs_ack_wave_full;
                radio_tx.CenterFrequency = hs_feedback_freq;

                state = STATE_FOLLOW_HOP;
                slot_ptr = 1;
                last_sync_time = tic;
                data_start_time = tic;
                % One-time switch to bus RX sample size for data phase
                release(radio_rx);
                radio_rx.SamplesPerFrame = BUS_RX_SAMPLES;
            else
                % Send low-duty-cycle discovery via READY (frame_type=30)
                ready_discovery_count = ready_discovery_count + 1;
                if mod(ready_discovery_count, 20) == 0
                    tx_sig = hs_ready_wave_full;
                    radio_tx.CenterFrequency = hs_feedback_freq;
                end
            end

        case STATE_READY_SENT
            % Wait for START on anchor frequency (RX still at CONTROL_RX_SAMPLES from WAIT_BEACON)
            radio_rx.CenterFrequency = hs_anchor_freq;

            try
                [rx_sig, ~, rx_overrun] = radio_rx();
                if rx_overrun, warning('[RX-WARN] 接收溢出'); end
            catch ME
                warning('[RX-ERR] 硬件错误: %s', ME.message);
                pause(0.05);
                continue;
            end

            [ctrl_valid, ctrl_data] = decode_ctrl_hs(rx_sig, hs_rxfilter, hs_head_fb, ...
                hs_pn_fb, hs_scr_seq, hs_cfgLDPCDec, hs_crcdetector, hs_qpskdemod, hs_sps, hs_sf);

            if ctrl_valid && ctrl_data.frame_type == 41  % START
                session_id = ctrl_data.session_id;
                hop_seed = ctrl_data.hop_seed;
                total_slots = ctrl_data.total_slots;
                slot_len_samples = ctrl_data.slot_len;
                codewords_per_slot = ctrl_data.codewords_per_slot;

                % Generate hop sequence
                hop_seq = build_hop_sequence(hop_seed, total_slots, defs.num_carriers);

                % Initialize frame cache
                frame_cache = struct();
                frame_cache = rx_frame_cache_update(frame_cache, [], session_id);

                fprintf('[RX] 收到START, 开始跳频跟随: 时隙=%d | 码字/时隙=%d\n', ...
                    total_slots, codewords_per_slot);

                % Send ACK immediately to confirm START receipt
                tx_sig = hs_ack_wave_full;
                radio_tx.CenterFrequency = hs_feedback_freq;

                state = STATE_FOLLOW_HOP;
                slot_ptr = 1;
                last_sync_time = tic;
                data_start_time = tic;
                % One-time switch to bus RX sample size for data phase
                release(radio_rx);
                radio_rx.SamplesPerFrame = BUS_RX_SAMPLES;
            end

            % Re-send ACK: burst mode first, then periodic keep-alive
            if ack_burst_remaining > 0
                % Burst phase: send ACK every iteration to ensure TX receives it
                tx_sig = hs_ack_wave_full;
                radio_tx.CenterFrequency = hs_feedback_freq;
                ack_burst_remaining = ack_burst_remaining - 1;
            elseif mod(idx, 5) == 0
                % Periodic keep-alive after burst ends
                tx_sig = hs_ack_wave_full;
                radio_tx.CenterFrequency = hs_feedback_freq;
            end

        case STATE_FOLLOW_HOP
            % Follow hop sequence, receive slots
            if slot_ptr <= total_slots
                % Tune to hop frequency
                carrier_idx = hop_seq(slot_ptr);
                radio_rx.CenterFrequency = defs.Carrier_set(carrier_idx);

                % Settle time (simplified - use a small delay)
                pause(0.01);  % 10ms retune guard

                try
                    [rx_sig, ~, rx_overrun] = radio_rx();
                    if rx_overrun, warning('[RX-WARN] 时隙%d接收溢出', slot_ptr); end
                catch ME
                    warning('[RX-ERR] 时隙%d硬件错误: %s', slot_ptr, ME.message);
                    pause(0.05);
                    continue;
                end

                % Detect and decode codewords in this slot
                [detections, phy_metrics] = detect_hop_slot(rx_sig, Anti_Jamming_Mode, Threshold, data_rxfilter);

                if phy_metrics.sync_success
                    [frame_packets, ~] = decode_forward_codewords_v2(rx_sig, detections, Anti_Jamming_Mode, rx_sig);

                    if ~isempty(frame_packets)
                        frame_cache = rx_frame_cache_update(frame_cache, frame_packets, session_id);
                        fprintf('[RX-DATA] 时隙 %d/%d | 频率=%.1f GHz | 信噪比=%.1f dB | 收到%d帧 | 缓存: %d/%d\n', ...
                            slot_ptr, total_slots, defs.Carrier_set(carrier_idx)/1e9, ...
                            phy_metrics.snr_est, length(frame_packets), ...
                            sum(frame_cache.received_map), frame_cache.total_frame_num);
                        last_sync_time = tic;  % reset timeout on successful receive
                    end
                else
                    fprintf('[RX-DATA] 时隙 %d/%d | 频率=%.1f GHz | 无同步\n', ...
                        slot_ptr, total_slots, defs.Carrier_set(carrier_idx)/1e9);
                end

                % Only advance slot on successful sync (stay on same slot otherwise)
                if phy_metrics.sync_success && ~isempty(frame_packets)
                    slot_ptr = slot_ptr + 1;
                    idle_slot_count = 0;  % reset idle counter on success
                else
                    idle_slot_count = idle_slot_count + 1;
                end

                if SKIP_HANDSHAKE
                    % No-handshake mode: stop when idle for too many consecutive slots
                    if idle_slot_count > MAX_IDLE_SLOTS
                        fprintf('[RX] 连续%d个时隙无数据, 判定TX已完成, 进入重建...\n', idle_slot_count);
                        state = STATE_FEC_REBUILD;
                    end
                else
                    % Timeout: if no data for 3 seconds, re-send ACK to re-sync with TX
                    if toc(last_sync_time) > 3.0
                        fprintf('[RX] 3秒无数据, 重发ACK...\n');
                        tx_sig = hs_ack_wave_full;
                        radio_tx.CenterFrequency = hs_feedback_freq;
                        last_sync_time = tic;
                    end

                    % Periodic keep-alive (ACK blip on feedback channel)
                    if mod(slot_ptr, TELEMETRY_PERIOD_LOOPS) == 0
                        tx_sig = hs_ack_wave_full;
                        radio_tx.CenterFrequency = hs_feedback_freq;
                    end
                end

            else
                % All slots received
                fprintf('[RX] 所有%d个时隙已接收, 进入前向纠错重建...\n', total_slots);
                state = STATE_FEC_REBUILD;
            end

        case STATE_FEC_REBUILD
            fprintf('[RX-FEC] 正在从%d个已接收帧重建文件 (%d/%d)...\n', ...
                sum(frame_cache.received_map), sum(frame_cache.received_map), frame_cache.total_frame_num);

            [file_bytes, rebuild_info] = rebuild_file_from_fec(frame_cache, pwd);

            if rebuild_info.success
                fprintf('[RX-FEC] 恢复率: %.1f%% | 组: %d 成功, %d 失败 | CRC匹配: %d\n', ...
                    rebuild_info.recovery_rate*100, rebuild_info.fec_groups_recovered, ...
                    rebuild_info.fec_groups_failed, rebuild_info.file_crc_match);
            else
                fprintf('[RX-FEC] 恢复失败: %s\n', rebuild_info.status);
            end

            state = STATE_RESULT_REPORT;

        case STATE_RESULT_REPORT
            metrics = compute_rx_metrics(phy_metrics, frame_cache, rebuild_info, toc(data_start_time));
            fprintf('[RX-RESULT] %s\n', metrics.summary);

            if SKIP_HANDSHAKE
                state = STATE_DONE;  % no feedback channel, go directly to done
            else
                % Send RESULT (frame_type=32) with correct session_id
                hs_result_wave = build_ctrl_wave_hs(session_id, 32, hs_head_fb, ...
                    hs_pn_fb, hs_scr_seq, hs_cfgLDPCEnc, hs_crcgenerator, ...
                    hs_qpskmod, hs_txfilter, hs_sps, hs_sf);
                tx_sig = hs_result_wave;
                radio_tx.CenterFrequency = hs_feedback_freq;

                if idx > 100  % Send for a while, then done
                    state = STATE_DONE;
                end
            end

        case STATE_DONE
            tx_sig = zeros(FB_TX_SAMPLES, 1);
    end

    % ---- Transmit feedback (skip if nothing to send) ----
    try
        if max(abs(tx_sig)) > 0
            radio_tx(tx_sig);
        end
    catch ME
        warning('[RX-ERR] 反馈发送错误: %s', ME.message);
    end

    % ---- Status ----
    if mod(idx, 10) == 0
        state_names = {'WAIT_BEACON', 'READY_SENT', 'FOLLOW_HOP', 'FEC_REBUILD', 'RESULT_REPORT', 'DONE'};
        if state == STATE_FOLLOW_HOP
            fprintf('[RX] 循环=%d | 状态=%s | 时隙=%d/%d | 接收=%d/%d\n', ...
                idx, state_names{state+1}, min(slot_ptr, total_slots), total_slots, ...
                sum(frame_cache.received_map), frame_cache.total_frame_num);
        else
            fprintf('[RX] 循环=%d | 状态=%s\n', idx, state_names{state+1});
        end
    end

    if state == STATE_DONE
        fprintf('[RX] 会话完成, 退出.\n');
        break;
    end
end

release(radio_rx);
release(radio_tx);
fprintf('[RX] 接收机关闭完成.\n');

%% =========== Helper Functions ===========
function [valid, ctrl_data] = decode_ctrl_hs(rx_sig, rxfilter, head_fb, pn_fb, ...
    scr_seq, cfgLDPCDec, crcdetector, qpskdemod, sps, sf)
% Decode handshake control frame (BEACON/START/END) using proven BPSK+spreading
% Handles both short format (BEACON/ACK: 40 info bits) and long format (START: 152 info bits)
valid = false;
ctrl_data = struct();
Threshold = 250;  % match proven handshake_rx.m
maxnumiter = 10;

Rec_sig = rxfilter(complex(rx_sig(:)));
data_frame_len = 648 * sf;  % BPSK+15x spreading → 9720

PN_head = flip(head_fb);
data_sys = [];
buffer_h = [];
index_val = zeros(1, sps);
index_loc_h = cell(1, sps);

for i = 1:sps
    data_sys(:, i) = Rec_sig(i:sps:end);
    buffer_h(:, i) = abs(conv(PN_head, sign(data_sys(:, i))));
    cand = pick_sync_peaks_ctrl(buffer_h(:, i), Threshold);
    if ~isempty(cand)
        index_loc_h{i} = cand(:);
        index_val(i) = mean(buffer_h(cand, i));
    else
        index_loc_h{i} = [];
    end
end

if all(index_val == 0)
    % Diagnostic: print max correlation when preamble not found
    persistent diag_ctrl_count;
    if isempty(diag_ctrl_count), diag_ctrl_count = 0; end
    diag_ctrl_count = diag_ctrl_count + 1;
    if mod(diag_ctrl_count, 10) == 0
        max_corr = zeros(1, sps);
        for k = 1:sps
            if ~isempty(buffer_h)
                max_corr(k) = max(buffer_h(:, k));
            end
        end
        fprintf('[RX-CTRL] 无前导 | 最大相关=[%.1f, %.1f, %.1f, %.1f] | 阈值=%d\n', ...
            max_corr(1), max_corr(2), max_corr(3), max_corr(4), Threshold);
    end
    return;
end

[~, op_index] = max(index_val);
Rec_sig_afr_temp = data_sys(:, op_index);
idx_start = index_loc_h{op_index};
idx_start = idx_start(idx_start + data_frame_len <= length(Rec_sig_afr_temp));

if isempty(idx_start), return; end

for j = 1:min(1, length(idx_start))
    idx = idx_start(j);
    train_len = min(511, idx);
    receive_train = Rec_sig_afr_temp(idx-train_len+1:idx);
    desire_seq = head_fb(end-train_len+1:end);
    phase_est = -angle(mean(conj(desire_seq) .* receive_train));

    Rec_sig_afr = Rec_sig_afr_temp(idx+1:idx+data_frame_len) .* exp(1j*phase_est);
    demod_signal = qpskdemod(Rec_sig_afr);

    data_desp = zeros(length(demod_signal)/sf, 1);
    for ii = 1:length(demod_signal)/sf
        data_desp(ii) = sum(demod_signal((ii-1)*sf+1:ii*sf) .* pn_fb);
    end

    deinter_matrix = reshape(data_desp, 18, 36).';
    de_interleaved = deinter_matrix(:);
    received_bits = ldpcDecode(de_interleaved, cfgLDPCDec, maxnumiter);
    de_scr = descramble_bits_hs(received_bits, scr_seq);

    % Try short format first (BEACON/ACK: 40 info bits + 32 CRC = 72 bits)
    [data_rec, err] = crcdetector(de_scr(1:72));
    if err == 0
        offset = 0;
        ctrl_data.frame_head = bits2int_hs(data_rec(offset+1:offset+8)); offset = offset+8;
        ctrl_data.user_id    = bits2int_hs(data_rec(offset+1:offset+8)); offset = offset+8;
        ctrl_data.frame_type = bits2int_hs(data_rec(offset+1:offset+8)); offset = offset+8;
        ctrl_data.session_id = bits2int_hs(data_rec(offset+1:offset+16));
        fprintf('[RX-CTRL] 解码成功(短格式): 类型=%d | 会话=%d | 相关=%.1f\n', ...
            ctrl_data.frame_type, ctrl_data.session_id, index_val(op_index));
        valid = true;
        return;
    end

    % Try long format (START: 152 info bits + 32 CRC = 184 bits)
    [data_rec2, err2] = crcdetector(de_scr(1:184));
    if err2 == 0
        offset = 0;
        ctrl_data.frame_head = bits2int_hs(data_rec2(offset+1:offset+8)); offset = offset+8;
        ctrl_data.user_id    = bits2int_hs(data_rec2(offset+1:offset+8)); offset = offset+8;
        ctrl_data.frame_type = bits2int_hs(data_rec2(offset+1:offset+8)); offset = offset+8;
        ctrl_data.session_id = bits2int_hs(data_rec2(offset+1:offset+16)); offset = offset+16;
        ctrl_data.hop_seed   = bits2int_hs(data_rec2(offset+1:offset+32)); offset = offset+32;
        ctrl_data.total_slots = bits2int_hs(data_rec2(offset+1:offset+16)); offset = offset+16;
        ctrl_data.slot_len   = bits2int_hs(data_rec2(offset+1:offset+32)); offset = offset+32;
        ctrl_data.codewords_per_slot = bits2int_hs(data_rec2(offset+1:offset+16)); offset = offset+16;
        ctrl_data.fec_k      = bits2int_hs(data_rec2(offset+1:offset+8)); offset = offset+8;
        ctrl_data.fec_r      = bits2int_hs(data_rec2(offset+1:offset+8));
        valid = true;
        return;
    end
end
% Preamble found but CRC failed on all candidates
fprintf('[RX-CTRL] 前导找到(相关峰值=%.1f)但CRC校验失败 | 信号rms=%.4f pk=%.4f\n', ...
    index_val(op_index), rms(rx_sig), max(abs(rx_sig)));
end

%% =========== Handshake Helper Functions ===========

function cand = pick_sync_peaks_ctrl(metric, thr)
raw_idx = find(metric >= thr);
cand = [];
if isempty(raw_idx), return; end
group_gap = 20;
st = 1;
while st <= length(raw_idx)
    ed = st;
    while ed < length(raw_idx) && (raw_idx(ed+1) - raw_idx(ed)) <= group_gap
        ed = ed + 1;
    end
    group = raw_idx(st:ed);
    [~, loc] = max(metric(group));
    cand(end+1,1) = group(loc); %#ok<AGROW>
    st = ed + 1;
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

function out = descramble_bits_hs(in, scr_seq)
out = zeros(size(in));
grp = length(scr_seq);
for ii = 1:floor(length(in)/grp)
    st = (ii-1)*grp + 1;
    ed = ii*grp;
    out(st:ed) = xor(in(st:ed), scr_seq);
end
end

function v = bits2int_hs(bits)
v = (2.^(length(bits)-1:-1:0)) * bits(:);
end

function wave = build_ctrl_wave_hs(session_id, frame_type, hs_head_fb, hs_pn_fb, ...
    hs_scr_seq, hs_cfgLDPCEnc, hs_crcgenerator, hs_qpskmod, hs_txfilter, hs_sps, hs_sf)
% Build a handshake control waveform (ACK/RESULT) with given session_id and frame_type
% Payload: FrameHead(8) + UserID(8) + FrameType(8) + SessionID(16) = 40 bits
hs_Frame_head = [1;1;1;0;1;0;1;0];
hs_Usr_ID = [0;0;0;0;0;1;0;1];
session_bits = double(dec2bin(session_id, 16) == '1')';
frame_type_bits = double(dec2bin(frame_type, 8) == '1')';

payload = [hs_Frame_head; hs_Usr_ID; frame_type_bits; session_bits];
enc = hs_crcgenerator(payload);
pad_len = 486 - length(enc);
payload_frame = [enc; zeros(pad_len, 1)];

scr = scramble_bits_hs(payload_frame, hs_scr_seq);
enc_bits = ldpcEncode(scr, hs_cfgLDPCEnc);
inter_matrix = reshape(enc_bits, 36, 18).';
inter_bits = inter_matrix(:);

inter_polar = 2*inter_bits - 1;
spread = zeros(length(inter_polar)*hs_sf, 1);
for ii = 1:length(inter_polar)
    spread((ii-1)*hs_sf+1 : ii*hs_sf) = inter_polar(ii) * hs_pn_fb;
end
mod_sig = hs_qpskmod(0.5*(spread + 1));
tx_in = [hs_head_fb; mod_sig; zeros(hs_sps*10, 1)];
wave = hs_txfilter(tx_in);
wave = [zeros(2000, 1); wave];
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
