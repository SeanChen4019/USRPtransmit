% =========== Stop-and-Wait ARQ Receiver ===========
% Protocol: DATA_ARQ -> FEC_REBUILD -> RESULT_REPORT -> DONE
% Listens for data slots, sends ACK with slot number after each.
clear
clc
close all force
warning('off', 'all');
fprintf('\n========== 停等ARQ接收机 ==========\n');

%% =========== Configuration ===========
defs = link_phy_defs();

Anti_Jamming_Mode = 1;  % 0=QPSK, 1=BPSK+spreading (与TX一致)
Threshold = 60;  % data slot preamble detection threshold
Threshold_FB = 220;
BUS_RX_SAMPLES = defs.slot_len_samples;
FB_TX_SAMPLES = 80000;
CONTROL_RX_SAMPLES = 80000;  % must be > beacon length (~43084 samples)

TELEMETRY_PERIOD_LOOPS = 10;  % Send telemetry every N data slots

%% =========== State Machine Constants ===========
STATE_DATA_ARQ     = 0;  % Listen for data slots, send ACK
STATE_FEC_REBUILD  = 1;
STATE_RESULT_REPORT = 2;
STATE_DONE         = 3;

state = STATE_DATA_ARQ;
ARQ_IDLE_TIMEOUT = 10.0;  % seconds of no data before assuming TX is done

%% =========== BPSK PHY Setup (for ACK/RESULT) ===========
fprintf('[RX-PHY] 正在设置BPSK物理层...\n');
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
hs_Frame_type_result = double(dec2bin(32, 8) == '1')';

%% Anchor / feedback frequencies
hs_feedback_freq = 1.45e9;
data_freq = 2.5e9;

%% =========== Session Variables ===========
session_id = 0;
frame_cache = struct();
frame_cache = rx_frame_cache_update(frame_cache, [], 0);

data_start_time = 0;
slot_ptr = 1;
last_data_time = tic;
phy_metrics = [];
session_known = false;  % session_id discovered from first decoded frame

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
    'SamplesPerFrame', BUS_RX_SAMPLES);  % data slot size
radio_rx.ClockSource = 'External';
radio_rx.ChannelMapping = 1;
radio_rx.CenterFrequency = data_freq;
radio_rx.Gain = 30;

cleanupObj = onCleanup(@() safe_release(radio_tx, radio_rx));
disp('[RX-HW] USRP就绪.');

% Data slot RRC receive filter (created once, shared across all slots)
data_rxfilter = comm.RaisedCosineReceiveFilter( ...
    'InputSamplesPerSymbol', 4, ...
    'DecimationFactor', 1, ...
    'RolloffFactor', 0.25);
% Pre-lock filter to complex input type (first radio_rx may return real zeros)
data_rxfilter(complex(zeros(100, 1)));

%% =========== UI Configuration ===========
rx_ui.enable = true;
rx_ui.url = 'http://127.0.0.1:5000';
rx_ui.health_endpoint = '/api/health';
rx_ui.post_period = 10;
rx_ui.ctrl_period = 20;
rx_ui.timeout = 0.03;


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
fprintf('[SYNC] 等待中... 请确保TX也在同步等待\n');
pause(wait_secs);
fprintf('[SYNC] 开始! 停等ARQ模式, 数据频率=%.1f GHz\n', data_freq/1e9);

%% =========== Main Loop ===========
for idx = 1:100000
    tx_sig = zeros(FB_TX_SAMPLES, 1);

    % ---- State Machine (Stop-and-Wait ARQ) ----
    switch state
        case STATE_DATA_ARQ
            % Listen on data frequency for slots
            radio_rx.CenterFrequency = data_freq;
            pause(0.01);

            try
                [rx_sig, ~, rx_overrun] = radio_rx();
                if rx_overrun, warning('[RX-WARN] 接收溢出'); end
            catch ME
                warning('[RX-ERR] 硬件错误: %s', ME.message);
                pause(0.05);
                continue;
            end

            % Detect and decode data slot (+0i forces complex to prevent filter lock)
            [detections, phy_metrics] = detect_hop_slot(rx_sig + 0i, Anti_Jamming_Mode, Threshold, data_rxfilter);

            if phy_metrics.sync_success
                [frame_packets, ~] = decode_forward_codewords_v2(rx_sig, detections, Anti_Jamming_Mode, rx_sig);

                if ~isempty(frame_packets)
                    % Discover session_id from first decoded frame
                    if ~session_known
                        session_id = frame_packets{1}.session_id;
                        session_known = true;
                        frame_cache = rx_frame_cache_update(frame_cache, [], session_id);
                        data_start_time = tic;
                        fprintf('[RX-ARQ] 发现会话=%d | 缓存初始化\n', session_id);
                    end

                    frame_cache = rx_frame_cache_update(frame_cache, frame_packets, session_id);
                    last_data_time = tic;
                    slot_ptr = slot_ptr + 1;

                    % Send ACK with slot number
                    ack_slot = max(0, slot_ptr - 1);
                    tx_sig = build_ctrl_wave_hs(session_id, 101, hs_head_fb, ...
                        hs_pn_fb, hs_scr_seq, hs_cfgLDPCEnc, hs_crcgenerator, ...
                        hs_qpskmod, hs_txfilter, hs_sps, hs_sf, 0, ack_slot);
                    radio_tx.CenterFrequency = hs_feedback_freq;

                    fprintf('[RX-ARQ] 收到时隙=%d | 信噪比=%.1f dB | 帧=%d | ACK已发送 | 缓存: %d/%d\n', ...
                        ack_slot, phy_metrics.snr_est, length(frame_packets), ...
                        sum(frame_cache.received_map), frame_cache.total_frame_num);
                end
            else
                % No sync: check idle timeout (TX may be done)
                if slot_ptr > 1 && toc(last_data_time) > ARQ_IDLE_TIMEOUT
                    fprintf('[RX-ARQ] %.0f秒无数据, 传输结束 (收到%d个时隙)\n', ...
                        ARQ_IDLE_TIMEOUT, slot_ptr - 1);
                    state = STATE_FEC_REBUILD;
                end
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
            % Send RESULT (frame_type=32) with correct session_id
            hs_result_wave = build_ctrl_wave_hs(session_id, 32, hs_head_fb, ...
                hs_pn_fb, hs_scr_seq, hs_cfgLDPCEnc, hs_crcgenerator, ...
                hs_qpskmod, hs_txfilter, hs_sps, hs_sf);
            tx_sig = hs_result_wave;
            radio_tx.CenterFrequency = hs_feedback_freq;

            metrics = compute_rx_metrics(phy_metrics, frame_cache, rebuild_info, toc(data_start_time));
            fprintf('[RX-RESULT] %s\n', metrics.summary);

            if idx > 100  % Send for a while, then done
                state = STATE_DONE;
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
        state_names = {'DATA_ARQ', 'FEC_REBUILD', 'RESULT_REPORT', 'DONE'};
        if state == STATE_DATA_ARQ && session_known
            fprintf('[RX] 循环=%d | 状态=%s | 收到时隙=%d | 缓存=%d/%d\n', ...
                idx, state_names{state+1}, slot_ptr - 1, ...
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
    hs_scr_seq, hs_cfgLDPCEnc, hs_crcgenerator, hs_qpskmod, hs_txfilter, hs_sps, hs_sf, ...
    next_carrier, slot_ack)
% Build a handshake control waveform (ACK/RESULT) with given session_id and frame_type
% Short format (no extra args): FrameHead(8) + UserID(8) + FrameType(8) + SessionID(16) = 40 bits
% Long format (next_carrier provided): adds next_carrier(8) + slot_ack(16) + reserved(88) = 152 bits
hs_Frame_head = [1;1;1;0;1;0;1;0];
hs_Usr_ID = [0;0;0;0;0;1;0;1];
session_bits = double(dec2bin(session_id, 16) == '1')';
frame_type_bits = double(dec2bin(frame_type, 8) == '1')';

use_long = (nargin >= 14);  % long format when slot_ack is provided
if use_long
    carrier_bits = double(dec2bin(next_carrier, 8) == '1')';
    slot_ack_bits = double(dec2bin(slot_ack, 16) == '1')';
    payload = [hs_Frame_head; hs_Usr_ID; frame_type_bits; session_bits; ...
               carrier_bits; slot_ack_bits; zeros(88, 1)];
else
    payload = [hs_Frame_head; hs_Usr_ID; frame_type_bits; session_bits];
end
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
