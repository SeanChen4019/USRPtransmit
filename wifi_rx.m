% ========== WiFi-Style Receiver ==========
% Protocol inspired by 802.11:
%   STF auto-correlation for robust frame detection
%   LTF cross-correlation for fine sync
%   FDD: forward 2.5GHz (RX), feedback 1.45GHz (TX)
clear; clc; close all force; warning('off','all');
fprintf('\n========== WiFi模式接收机 ==========\n');

%% =========== Configuration ===========
defs = link_phy_defs();

Anti_Jamming_Mode = 0;
Threshold_STF = 0.5;     % STF auto-correlation threshold (0~1)
Threshold_sync = 0.15;   % LTF cross-correlation threshold

anchor_freq = 2.5e9;
feedback_freq = 1.45e9;
BUS_RX_SAMPLES = defs.slot_len_samples;  % 160000
CTRL_RX_SAMPLES = 80000;

%% =========== State Machine ===========
STATE_LISTEN  = 0;  % listen for BEACON/START
STATE_DATA    = 1;  % receive data slots
STATE_FINISH  = 2;  % rebuild and report
STATE_DONE    = 3;

%% =========== PHY Setup ===========
sps = 4;
sf = 15;

% STF: 10x repeating 16-sample pattern (SAME AS TX)
stf16 = [1, 1, -1, -1, 1, 1, -1, 1, -1, 1, 1, 1, 1, 1, 1, -1]';
stf_pattern = repmat(stf16, 10, 1);

% LTF: 64-sample known sequence (SAME AS TX)
ltf64 = [1, 1, 1, 1, -1, -1, 1, 1, -1, 1, -1, 1, -1, -1, -1, -1, ...
         -1, 1, 1, -1, -1, 1, -1, 1, -1, 1, 1, 1, 1, 1, -1, 1, ...
         1, -1, 1, -1, -1, 1, 1, -1, -1, 1, 1, 1, -1, -1, 1, 1, ...
         -1, -1, -1, 1, 1, 1, -1, 1, 1, -1, -1, -1, 1, -1, 1, -1]';

% LDPC
pcmatrix = ldpcQuasiCyclicMatrix(defs.blockSize, defs.P);
cfgLDPCEnc = ldpcEncoderConfig(pcmatrix);
cfgLDPCDec = ldpcDecoderConfig(pcmatrix);
crcgen_ctrl = comm.CRCGenerator(defs.poly);
crcdet_ctrl = comm.CRCDetector(defs.poly);

% BPSK for control frames
qpskmod = comm.PSKModulator(2, 'BitInput', true);
qpskmod.PhaseOffset = pi/4;
qpskdemod = comm.PSKDemodulator(2, 'BitOutput', true, ...
    'DecisionMethod', 'Approximate log-likelihood ratio');
qpskdemod.PhaseOffset = pi/4;

% Filters
txfilter = comm.RaisedCosineTransmitFilter('OutputSamplesPerSymbol', sps, 'RolloffFactor', 0.25);
rxfilter = comm.RaisedCosineReceiveFilter('InputSamplesPerSymbol', sps, 'DecimationFactor', 1, 'RolloffFactor', 0.25);

data_rxfilter = comm.RaisedCosineReceiveFilter('InputSamplesPerSymbol', 4, 'DecimationFactor', 1, 'RolloffFactor', 0.25);

pn_fb = [1,-1,-1,-1,1,1,1,1,-1,1,-1,1,1,-1,-1]';
scr_seq = [1 1 0 1 1 0 1 0 0 1 0 0 0 0 1 0 1 0 1 1 1 0 1 1 0 0 0]';

% Pre-build ACK waveform
Frame_head = [1;1;1;0;1;0;1;0];
Usr_ID = [0;0;0;0;0;1;0;1];

fprintf('[PHY] 构建ACK波形...\n');
function ack_w = build_ack(sid)
    payload = [Frame_head; Usr_ID; double(dec2bin(101,8)=='1')'; double(dec2bin(sid,16)=='1')'];
    enc = crcgen_ctrl(payload);
    pad = 486 - length(enc);
    frame_data = [enc; zeros(pad, 1)];
    scr = zeros(length(frame_data),1);
    for i = 1:floor(length(frame_data)/length(scr_seq))
        ix = (i-1)*length(scr_seq)+1 : i*length(scr_seq);
        scr(ix) = xor(frame_data(ix), scr_seq);
    end
    enc_bits = ldpcEncode(scr, cfgLDPCEnc);
    inter_m = reshape(enc_bits, 36, 18).'; inter_bits = inter_m(:);
    inter_pol = 2*inter_bits - 1;
    spread = zeros(length(inter_pol)*sf, 1);
    for ii = 1:length(inter_pol)
        spread((ii-1)*sf+1 : ii*sf) = inter_pol(ii) * pn_fb;
    end
    mod_sig = qpskmod(0.5*(spread + 1));
    sig_w = txfilter([mod_sig; zeros(sps*10, 1)]);
    ack_w = [zeros(2000,1); sig_w];
end

ack_wave_placeholder = build_ack(0);

%% =========== Session State ===========
session_id = 0;
hop_seed = 0;
total_slots = 0;
codewords_per_slot = defs.codewords_per_slot_default;
hop_seq = [];
frame_cache = struct();
frame_cache = rx_frame_cache_update(frame_cache, [], 0);
data_start_time = 0;
last_sync_time = 0;
slot_ptr = 1;

%% =========== SDR Setup ===========
disp('[SDR] 强制释放残留USRP句柄...');
try
    old_radios = instrfindall('Type', 'usrp');
    if ~isempty(old_radios)
        for r = 1:length(old_radios)
            try release(old_radios(r)); catch; end
        end
    end
catch
end
try; comm.internal.SDRuBase.closeAllSessions(); catch; end
pause(1);

fprintf('[SDR] 初始化USRP...\n');

radio_tx = comm.SDRuTransmitter('Platform', 'X310', 'IPAddress', '192.168.10.2');
radio_tx.ChannelMapping = 1;
radio_tx.CenterFrequency = feedback_freq;
radio_tx.Gain = 30;
radio_tx.MasterClockRate = 200e6;
radio_tx.InterpolationFactor = 512;
radio_tx.ClockSource = 'External';

radio_rx = comm.SDRuReceiver('Platform', 'X310', 'IPAddress', '192.168.10.2', ...
    'OutputDataType', 'double', 'MasterClockRate', 200e6, ...
    'DecimationFactor', 512, 'SamplesPerFrame', CTRL_RX_SAMPLES);
radio_rx.ClockSource = 'External';
radio_rx.ChannelMapping = 1;
radio_rx.CenterFrequency = anchor_freq;
radio_rx.Gain = 30;

cleanup = onCleanup(@() safe_release(radio_tx, radio_rx));
fprintf('[SDR] USRP就绪. RX=%.1fGHz TX=%.1fGHz\n', anchor_freq/1e9, feedback_freq/1e9);

%% =========== Synchronized Start ===========
t_now = datetime('now');
sec_cur = second(t_now);
sec_tgt = ceil(sec_cur / 5) * 5;
if sec_tgt - sec_cur < 1, sec_tgt = sec_tgt + 5; end
w = sec_tgt - sec_cur;
fprintf('[SYNC] 当前: %s | 开始: %s (%.1f秒后)\n', ...
    datestr(t_now,'HH:MM:SS'), datestr(t_now+seconds(w),'HH:MM:SS'), w);
fprintf('[SYNC] 等待TX同步...\n');
pause(w);
fprintf('[SYNC] 开始!\n');

%% =========== Main Loop ===========
state = STATE_LISTEN;
rx_diag = 0;
ack_burst = 0;
data_slot_idle = 0;

for idx = 1:100000
    tx_sig = [];

    switch state
        case STATE_LISTEN
            radio_rx.CenterFrequency = anchor_freq;

            try
                [rx_sig, ~, overrun] = radio_rx();
                if overrun, warning('[WARN] 接收溢出'); end
            catch ME
                warning('[ERR] 硬件: %s', ME.message);
                pause(0.05); continue;
            end

            % Diagnostic
            rx_diag = rx_diag + 1;
            if mod(rx_diag, 10) == 0
                fprintf('[DIAG] #%d | rms=%.4f | pk=%.4f\n', ...
                    rx_diag, rms(rx_sig), max(abs(rx_sig)));
            end

            % === WiFi-style STF auto-correlation detection ===
            [stf_detected, stf_pos] = detect_stf_wifi(rx_sig);

            if stf_detected
                fprintf('[STF] 检测到帧! 位置=%d | rms=%.4f\n', stf_pos, rms(rx_sig));

                % Try to decode the control frame
                [ctrl_ok, ctrl_data] = wifi_decode_ctrl_frame(rx_sig, stf_pos);

                if ctrl_ok
                    fprintf('[CTRL] 解码成功: 类型=%d | 会话=%d\n', ...
                        ctrl_data.frame_type, ctrl_data.session_id);

                    if ctrl_data.frame_type == 100  % BEACON
                        session_id = ctrl_data.session_id;
                        % Build ACK with correct session_id
                        ack_wave_full = build_ack(session_id);
                        tx_sig = ack_wave_full;
                        radio_tx.CenterFrequency = feedback_freq;
                        ack_burst = 5;  % send ACK burst
                        fprintf('[SYNC] BEACON收到! 会话=%d, 发送ACK burst\n', session_id);

                    elseif ctrl_data.frame_type == 41  % START
                        session_id = ctrl_data.session_id;
                        hop_seed = ctrl_data.hop_seed;
                        total_slots = ctrl_data.total_slots;
                        codewords_per_slot = ctrl_data.codewords_per_slot;

                        hop_seq = build_hop_sequence(hop_seed, total_slots, defs.num_carriers);
                        frame_cache = struct();
                        frame_cache = rx_frame_cache_update(frame_cache, [], session_id);

                        fprintf('[SYNC] START收到! 时隙=%d | 跳频种子=%d\n', total_slots, hop_seed);
                        fprintf('[SYNC] 进入数据接收模式...\n');

                        % Send ACK to confirm
                        ack_wave_full = build_ack(session_id);
                        tx_sig = ack_wave_full;
                        radio_tx.CenterFrequency = feedback_freq;

                        % Switch to data mode
                        release(radio_rx);
                        radio_rx.SamplesPerFrame = BUS_RX_SAMPLES;
                        state = STATE_DATA;
                        slot_ptr = 1;
                        last_sync_time = tic;
                        data_start_time = tic;

                    elseif ctrl_data.frame_type == 42  % END
                        fprintf('[SYNC] END收到, 进入重建...\n');
                        state = STATE_FINISH;
                    end
                else
                    fprintf('[CTRL] STF检测到但解码失败\n');
                end
            end

            % Send ACK burst (if active)
            if ack_burst > 0 && state == STATE_LISTEN
                if isempty(tx_sig)
                    tx_sig = ack_wave_full;
                    radio_tx.CenterFrequency = feedback_freq;
                end
                ack_burst = ack_burst - 1;
            end

        case STATE_DATA
            if slot_ptr <= total_slots
                carrier_idx = hop_seq(slot_ptr);
                radio_rx.CenterFrequency = defs.Carrier_set(carrier_idx);
                pause(0.01);

                try
                    [rx_sig, ~, overrun] = radio_rx();
                    if overrun, warning('[WARN] 时隙%d溢出', slot_ptr); end
                catch ME
                    warning('[ERR] 时隙%d硬件: %s', slot_ptr, ME.message);
                    pause(0.05); continue;
                end

                [detections, phy_m] = detect_hop_slot(rx_sig, Anti_Jamming_Mode, 180, data_rxfilter);

                if phy_m.sync_success
                    [frame_packets, ~] = decode_forward_codewords_v2(rx_sig, detections, Anti_Jamming_Mode, rx_sig);

                    if ~isempty(frame_packets)
                        frame_cache = rx_frame_cache_update(frame_cache, frame_packets, session_id);
                        fprintf('[DATA] 时隙 %d/%d | 频率=%.1f GHz | SNR=%.1f dB | 帧=%d | 缓存=%d/%d\n', ...
                            slot_ptr, total_slots, defs.Carrier_set(carrier_idx)/1e9, ...
                            phy_m.snr_est, length(frame_packets), ...
                            sum(frame_cache.received_map), frame_cache.total_frame_num);
                        slot_ptr = slot_ptr + 1;
                        last_sync_time = tic;
                        data_slot_idle = 0;
                    else
                        fprintf('[DATA] 时隙 %d/%d | 同步但无帧\n', slot_ptr, total_slots);
                        data_slot_idle = data_slot_idle + 1;
                    end
                else
                    fprintf('[DATA] 时隙 %d/%d | 频率=%.1f GHz | 无同步\n', ...
                        slot_ptr, total_slots, defs.Carrier_set(carrier_idx)/1e9);
                    data_slot_idle = data_slot_idle + 1;
                end

                % Timeout check
                if data_slot_idle > 15
                    fprintf('[DATA] 连续%d时隙无数据, 判定TX结束.\n', data_slot_idle);
                    state = STATE_FINISH;
                end
            else
                fprintf('[DATA] 所有%d时隙接收完成.\n', total_slots);
                state = STATE_FINISH;
            end

        case STATE_FINISH
            fprintf('[FEC] 从%d个接收帧重建文件...\n', sum(frame_cache.received_map));
            [file_bytes, rebuild_info] = rebuild_file_from_fec(frame_cache, pwd);

            if rebuild_info.success
                fprintf('[FEC] 恢复率: %.1f%% | 组: %d OK, %d 失败\n', ...
                    rebuild_info.recovery_rate*100, ...
                    rebuild_info.fec_groups_recovered, rebuild_info.fec_groups_failed);
            else
                fprintf('[FEC] 恢复失败: %s\n', rebuild_info.status);
            end

            metrics = compute_rx_metrics([], frame_cache, rebuild_info, toc(data_start_time));
            fprintf('[RESULT] %s\n', metrics.summary);
            state = STATE_DONE;

        case STATE_DONE
            break;
    end

    % Transmit feedback
    if ~isempty(tx_sig) && max(abs(tx_sig)) > 0
        try
            radio_tx(tx_sig);
        catch ME
            warning('[ERR] 发送反馈: %s', ME.message);
        end
    end

    % Status
    if mod(idx, 10) == 0
        sn = {'LISTEN','DATA','FINISH','DONE'};
        if state == STATE_DATA
            fprintf('[RX] 循环=%d | 状态=%s | 时隙=%d/%d | 缓存=%d/%d\n', ...
                idx, sn{state+1}, min(slot_ptr,total_slots), total_slots, ...
                sum(frame_cache.received_map), frame_cache.total_frame_num);
        else
            fprintf('[RX] 循环=%d | 状态=%s\n', idx, sn{state+1});
        end
    end

    if state == STATE_DONE, break; end
end

fprintf('[RX] 接收机关闭.\n');
release(radio_rx); release(radio_tx);

%% =========== STF Auto-Correlation Detector ===========
function [detected, pos] = detect_stf_wifi(rx_sig)
% WiFi-style STF detection using auto-correlation (vectorized)
%   r(d) = |sum(x[n]*conj(x[n-16]))| / sum(|x|^2) over sliding window
%   Insensitive to frequency offset and amplitude!

    detected = false;
    pos = 0;

    stf_len = 16;
    n_repeat = 9;
    corr_win = stf_len * n_repeat;  % 144 samples

    rec = rx_sig(:);
    if length(rec) < (corr_win + stf_len) * 4 + 200
        return;
    end

    best_metric = 0;
    best_phase = 1;

    for ph = 1:4
        ds = rec(ph:4:end);  % downsample to 1 sps
        n_sym = length(ds);
        if n_sym < corr_win + stf_len + 10, continue; end

        % Element-wise auto-correlation product: C[n] = x[n] * conj(x[n-16])
        C = ds(stf_len+1:end) .* conj(ds(1:end-stf_len));
        P = abs(ds(stf_len+1:end)).^2;  % power for normalization

        % Moving sum over correlation window (vectorized via filter)
        ones_win = ones(corr_win, 1);
        auto_corr = abs(filter(ones_win, 1, C));
        sig_power = filter(ones_win, 1, P) + eps;

        % Skip filter transient
        valid_start = corr_win;
        auto_corr = auto_corr(valid_start:end);
        sig_power = sig_power(valid_start:end);

        if isempty(auto_corr), continue; end

        metric = auto_corr ./ sig_power;

        [mx, mx_pos] = max(metric);
        if mx > best_metric
            best_metric = mx;
            best_phase = ph;
            pos = mx_pos + valid_start + corr_win;  % absolute symbol position
        end
    end

    detected = (best_metric > 0.5);
    if detected
        pos = (pos - 1) * 4 + best_phase;  % convert to sample index
    end
end

%% =========== Control Frame Decoder ===========
function [valid, data] = wifi_decode_ctrl_frame(rx_sig, stf_pos)
% Decode control frame starting from STF position
    valid = false;
    data = struct();

    rec = complex(rx_sig(:));
    n_total = length(rec);
    data_frame_len = 648 * 15;  % = 9720

    % Frame: [1000 zeros][STF ~720 samp][LTF ~560 samp][SIGNAL ~12000 samp]
    % Search for SIGNAL field in a window after STF detection
    search_start = max(1, stf_pos + 500);
    search_end = min(stf_pos + 5000, n_total);
    seg = rec(search_start : search_end);
    if length(seg) < data_frame_len, return; end

    % Try to decode at this position
    for phase = 1:4
        ds = seg(phase:4:end);
        if length(ds) < data_frame_len, continue; end
        ds = ds(1:data_frame_len);

        % Phase correction
        train = ds(1:64);
        train = train ./ (abs(train) + eps);
        ph_est = angle(mean(train));
        ds_corrected = ds .* exp(-1j*ph_est);

        % Demodulate (BPSK)
        qpskdemod_local = comm.PSKDemodulator(2, 'BitOutput', true, ...
            'DecisionMethod', 'Approximate log-likelihood ratio');
        qpskdemod_local.PhaseOffset = pi/4;
        demod_sig = qpskdemod_local(ds_corrected);

        % Despread
        pn = [1,-1,-1,-1,1,1,1,1,-1,1,-1,1,1,-1,-1]';
        desp = zeros(length(demod_sig)/15, 1);
        for ii = 1:length(demod_sig)/15
            desp(ii) = sum(demod_sig((ii-1)*15+1 : ii*15) .* pn);
        end

        % Deinterleave
        deint_m = reshape(desp, 18, 36).';
        deint = deint_m(:);

        % LDPC decode
        pcmatrix = ldpcQuasiCyclicMatrix(27, ...
            [16 17 22 24 9 3 14 -1 4 2 7 -1 26 -1 2 -1 21 -1 1 0 -1 -1 -1 -1;
             25 12 12 3 3 26 6 21 -1 15 22 -1 15 -1 4 -1 -1 16 -1 0 0 -1 -1 -1;
             25 18 26 16 22 23 9 -1 0 -1 4 -1 4 -1 8 23 11 -1 -1 -1 0 0 -1 -1;
             9 7 0 1 17 -1 -1 7 3 -1 3 23 -1 16 -1 -1 21 -1 0 -1 -1 0 0 -1;
             24 5 26 7 1 -1 -1 15 24 15 -1 8 -1 13 -1 13 -1 11 -1 -1 -1 -1 0 0;
             2 2 19 14 24 1 15 19 -1 21 -1 2 -1 24 -1 3 -1 2 1 -1 -1 -1 -1 0]);
        cfgLDPCDec_local = ldpcDecoderConfig(pcmatrix);
        rx_bits = ldpcDecode(deint, cfgLDPCDec_local, 10);

        % Descramble
        scr = [1 1 0 1 1 0 1 0 0 1 0 0 0 0 1 0 1 0 1 1 1 0 1 1 0 0 0]';
        descr = zeros(length(rx_bits), 1);
        for ii = 1:floor(length(rx_bits)/length(scr))
            ix = (ii-1)*length(scr)+1 : ii*length(scr);
            descr(ix) = xor(rx_bits(ix), scr);
        end

        % CRC check (try short format: 72 bits)
        poly = 'z^32+z^26+z^23+z^22+z^16+z^12+z^11+z^10+z^8+z^7+z^5+z^4+z^2+z+1';
        crcdet_local = comm.CRCDetector(poly);
        [drec, err] = crcdet_local(descr(1:72));
        if err == 0
            off = 0;
            data.frame_head = sum(2.^(7:-1:0)'.*drec(off+1:off+8)); off=off+8;
            data.user_id    = sum(2.^(7:-1:0)'.*drec(off+1:off+8)); off=off+8;
            data.frame_type = sum(2.^(7:-1:0)'.*drec(off+1:off+8)); off=off+8;
            data.session_id = sum(2.^(15:-1:0)'.*drec(off+1:off+16));
            valid = true;
            return;
        end

        % Try long format (START: 184 bits)
        [drec2, err2] = crcdet_local(descr(1:184));
        if err2 == 0
            off = 0;
            data.frame_head = sum(2.^(7:-1:0)'.*drec2(off+1:off+8)); off=off+8;
            data.user_id    = sum(2.^(7:-1:0)'.*drec2(off+1:off+8)); off=off+8;
            data.frame_type = sum(2.^(7:-1:0)'.*drec2(off+1:off+8)); off=off+8;
            data.session_id = sum(2.^(15:-1:0)'.*drec2(off+1:off+16)); off=off+16;
            data.hop_seed   = sum(2.^(31:-1:0)'.*drec2(off+1:off+32)); off=off+32;
            data.total_slots = sum(2.^(15:-1:0)'.*drec2(off+1:off+16)); off=off+16;
            data.slot_len   = sum(2.^(31:-1:0)'.*drec2(off+1:off+32)); off=off+32;
            data.codewords_per_slot = sum(2.^(15:-1:0)'.*drec2(off+1:off+16)); off=off+16;
            data.fec_k      = sum(2.^(7:-1:0)'.*drec2(off+1:off+8)); off=off+8;
            data.fec_r      = sum(2.^(7:-1:0)'.*drec2(off+1:off+8));
            valid = true;
            return;
        end
        break;  % only try first phase that has enough samples
    end
end

%% =========== Cleanup ===========
function safe_release(tx, rx)
    try; if ~isempty(tx) && isvalid(tx), release(tx); end; catch; end
    try; if ~isempty(rx) && isvalid(rx), release(rx); end; catch; end
end
