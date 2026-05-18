% =========== One-Shot Frequency Hopping Receiver ===========
% V2 Protocol: WAIT_BEACON -> READY_SENT -> FOLLOW_HOP -> FEC_REBUILD -> RESULT_REPORT
% Receives business data exactly once, uses FEC for recovery, no retransmission requests.
clear
clc
close all force
warning('off', 'all');
fprintf('\n========== One-Shot FH Receiver ==========\n');

%% =========== Configuration ===========
defs = link_phy_defs();

Anti_Jamming_Mode = 0;  % 0=QPSK, 1=BPSK+spreading
Threshold = 240;
Threshold_FB = 220;
BUS_RX_SAMPLES = defs.slot_len_samples;
FB_TX_SAMPLES = 80000;
CONTROL_RX_SAMPLES = 40000;

TELEMETRY_PERIOD_LOOPS = 10;  % Send telemetry every N data slots

%% =========== State Machine Constants ===========
STATE_WAIT_BEACON  = 0;
STATE_READY_SENT   = 1;
STATE_FOLLOW_HOP   = 2;
STATE_FEC_REBUILD  = 3;
STATE_RESULT_REPORT = 4;
STATE_DONE         = 5;

state = STATE_WAIT_BEACON;

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
phy_metrics = [];  % store latest physical metrics

%% =========== SDR Initialization ===========
disp('[RX-HW] Initializing USRP...');

radio_tx = comm.SDRuTransmitter('Platform', 'X310', 'IPAddress', '192.168.10.2');
radio_tx.ChannelMapping = 1;
radio_tx.CenterFrequency = defs.feedback_freq;
radio_tx.Gain = 24;
radio_tx.MasterClockRate = 200e6;
radio_tx.InterpolationFactor = 512;
radio_tx.ClockSource = 'External';

radio_rx = comm.SDRuReceiver( ...
    'Platform', 'X310', ...
    'IPAddress', '192.168.10.2', ...
    'OutputDataType', 'double', ...
    'MasterClockRate', 200e6, ...
    'DecimationFactor', 512, ...
    'SamplesPerFrame', BUS_RX_SAMPLES);
radio_rx.ClockSource = 'External';
radio_rx.ChannelMapping = 2;
radio_rx.CenterFrequency = defs.anchor_freq;
radio_rx.Gain = 30;  % OTA: increased for over-the-air

cleanupObj = onCleanup(@() safe_release(radio_tx, radio_rx));
disp('[RX-HW] USRP ready.');

%% =========== UI Configuration ===========
rx_ui.enable = true;
rx_ui.url = 'http://127.0.0.1:5000';
rx_ui.health_endpoint = '/api/health';
rx_ui.post_period = 10;
rx_ui.ctrl_period = 20;
rx_ui.timeout = 0.03;


%% =========== Main Loop ===========
rx_diag_count = 0;
for idx = 1:100000
    tx_sig = zeros(FB_TX_SAMPLES, 1);

    % ---- State Machine ----
    switch state
        case STATE_WAIT_BEACON
            % Listen on anchor frequency for BEACON
            radio_rx.CenterFrequency = defs.anchor_freq;
            radio_rx.SamplesPerFrame = CONTROL_RX_SAMPLES;

            try
                [rx_sig, ~, rx_overrun] = radio_rx();
                if rx_overrun, warning('[RX-WARN] Overrun'); end
            catch ME
                warning('[RX-ERR] HW error: %s', ME.message);
                continue;
            end

            % Try to decode BEACON/START control frame
            rx_diag_count = rx_diag_count + 1;
            if mod(rx_diag_count, 30) == 0
                fprintf('[RX-DIAG] #%d | rms=%.4f | pk=%.4f\n', ...
                    rx_diag_count, rms(rx_sig), max(abs(rx_sig)));
            end
            [ctrl_valid, ctrl_data] = decode_control_frame(rx_sig);

            if ctrl_valid && (ctrl_data.frame_type == 40 || ctrl_data.frame_type == 41)
                % Got BEACON or direct START
                session_id = ctrl_data.session_id;
                hop_seed = ctrl_data.hop_seed;
                total_slots = ctrl_data.total_slots;
                slot_len_samples = ctrl_data.slot_len;
                codewords_per_slot = ctrl_data.codewords_per_slot;

                fprintf('[RX] Got control frame: type=%d | session=%d | hop_seed=%d | slots=%d\n', ...
                    ctrl_data.frame_type, session_id, hop_seed, total_slots);

                % Send RX_READY
                fb_data = build_fb_data(defs.FRAME_TYPE_RX_READY, session_id, ...
                    defs.RX_STATE_READY);
                tx_sig = feedback_frame_modulate_v2(fb_data);

                state = STATE_READY_SENT;
                slot_ptr = 1;
                ready_discovery_count = 0;
            else
                % Send low-duty-cycle RX_READY_DISCOVERY
                ready_discovery_count = ready_discovery_count + 1;
                if mod(ready_discovery_count, 20) == 0
                    fb_data = build_fb_data(defs.FRAME_TYPE_RX_READY, 0, defs.RX_STATE_WAIT);
                    tx_sig = feedback_frame_modulate_v2(fb_data);
                end
            end

            % Reset to bus RX samples for data mode
            release(radio_rx);
            radio_rx.SamplesPerFrame = BUS_RX_SAMPLES;

        case STATE_READY_SENT
            % Wait for START on anchor frequency
            radio_rx.CenterFrequency = defs.anchor_freq;
            release(radio_rx);
            radio_rx.SamplesPerFrame = CONTROL_RX_SAMPLES;

            try
                [rx_sig, ~, rx_overrun] = radio_rx();
                if rx_overrun, warning('[RX-WARN] Overrun'); end
            catch ME
                warning('[RX-ERR] HW error: %s', ME.message);
                continue;
            end

            [ctrl_valid, ctrl_data] = decode_control_frame(rx_sig);

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

                fprintf('[RX] START received, beginning FOLLOW_HOP: slots=%d | cw/slot=%d\n', ...
                    total_slots, codewords_per_slot);

                state = STATE_FOLLOW_HOP;
                slot_ptr = 1;
                data_start_time = tic;
                release(radio_rx);
                radio_rx.SamplesPerFrame = BUS_RX_SAMPLES;
            end

            % Re-send RX_READY periodically
            if mod(idx, 5) == 0
                fb_data = build_fb_data(defs.FRAME_TYPE_RX_READY, session_id, defs.RX_STATE_READY);
                tx_sig = feedback_frame_modulate_v2(fb_data);
            end

            release(radio_rx);
            radio_rx.SamplesPerFrame = BUS_RX_SAMPLES;

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
                    if rx_overrun, warning('[RX-WARN] Overrun at slot %d', slot_ptr); end
                catch ME
                    warning('[RX-ERR] HW error at slot %d: %s', slot_ptr, ME.message);
                    slot_ptr = slot_ptr + 1;
                    continue;
                end

                % Detect and decode codewords in this slot
                [detections, phy_metrics] = detect_hop_slot(rx_sig, Anti_Jamming_Mode, Threshold);

                if phy_metrics.sync_success
                    [frame_packets, ~] = decode_forward_codewords_v2(rx_sig, detections, Anti_Jamming_Mode, rx_sig);

                    if ~isempty(frame_packets)
                        frame_cache = rx_frame_cache_update(frame_cache, frame_packets, session_id);
                        fprintf('[RX-DATA] Slot %d/%d | Freq=%.1f GHz | SNR=%.1f dB | Got %d frames | Cache: %d/%d\n', ...
                            slot_ptr, total_slots, defs.Carrier_set(carrier_idx)/1e9, ...
                            phy_metrics.snr_est, length(frame_packets), ...
                            sum(frame_cache.received_map), frame_cache.total_frame_num);
                    end
                else
                    fprintf('[RX-DATA] Slot %d/%d | Freq=%.1f GHz | No sync\n', ...
                        slot_ptr, total_slots, defs.Carrier_set(carrier_idx)/1e9);
                end

                % Periodic telemetry
                if mod(slot_ptr, TELEMETRY_PERIOD_LOOPS) == 0
                    fb_data = build_fb_data(defs.FRAME_TYPE_RX_TELEMETRY, session_id, ...
                        defs.RX_STATE_FOLLOW);
                    fb_data = fill_telemetry(fb_data, phy_metrics, frame_cache);
                    tx_sig = feedback_frame_modulate_v2(fb_data);
                end

                slot_ptr = slot_ptr + 1;

            else
                % All slots received
                fprintf('[RX] All %d slots received, entering FEC_REBUILD...\n', total_slots);
                state = STATE_FEC_REBUILD;
            end

        case STATE_FEC_REBUILD
            fprintf('[RX-FEC] Rebuilding file from %d received frames (%d/%d)...\n', ...
                sum(frame_cache.received_map), sum(frame_cache.received_map), frame_cache.total_frame_num);

            [file_bytes, rebuild_info] = rebuild_file_from_fec(frame_cache, pwd);

            if rebuild_info.success
                fprintf('[RX-FEC] Recovery: %.1f%% | Groups: %d OK, %d failed | CRC match: %d\n', ...
                    rebuild_info.recovery_rate*100, rebuild_info.fec_groups_recovered, ...
                    rebuild_info.fec_groups_failed, rebuild_info.file_crc_match);
            else
                fprintf('[RX-FEC] Recovery failed: %s\n', rebuild_info.status);
            end

            state = STATE_RESULT_REPORT;

        case STATE_RESULT_REPORT
            % Send RESULT repeatedly
            metrics = compute_rx_metrics(phy_metrics, frame_cache, rebuild_info, toc(data_start_time));

            fb_data = build_fb_data(defs.FRAME_TYPE_RX_RESULT, session_id, defs.RX_STATE_RESULT);
            fb_data = fill_telemetry(fb_data, phy_metrics, frame_cache);
            fb_data.result_code = metrics.result_code;
            fb_data.rx_crc_ok_num = metrics.rx_crc_ok_num;
            fb_data.rx_lost_num = metrics.rx_lost_num;
            fb_data.fec_recovered_num = metrics.fec_recovered_num;
            fb_data.pre_fec_per_q16 = round(metrics.pre_fec_per * 65535);
            fb_data.post_fec_per_q16 = round(metrics.post_fec_per * 65535);
            fb_data.goodput_kbps_q16 = round(metrics.goodput_kbps * 65535);

            tx_sig = feedback_frame_modulate_v2(fb_data);

            fprintf('[RX-RESULT] %s\n', metrics.summary);

            if idx > 100  % Send for a while, then done
                state = STATE_DONE;
            end

        case STATE_DONE
            tx_sig = zeros(FB_TX_SAMPLES, 1);
    end

    % ---- Transmit feedback ----
    try
        radio_tx(tx_sig);
    catch ME
        warning('[RX-ERR] FB TX error: %s', ME.message);
    end

    % ---- Status ----
    if mod(idx, 10) == 0
        state_names = {'WAIT_BEACON', 'READY_SENT', 'FOLLOW_HOP', 'FEC_REBUILD', 'RESULT_REPORT', 'DONE'};
        if state == STATE_FOLLOW_HOP
            fprintf('[RX] idx=%d | state=%s | slot=%d/%d | rx=%d/%d\n', ...
                idx, state_names{state+1}, min(slot_ptr, total_slots), total_slots, ...
                sum(frame_cache.received_map), frame_cache.total_frame_num);
        else
            fprintf('[RX] idx=%d | state=%s\n', idx, state_names{state+1});
        end
    end

    if state == STATE_DONE
        fprintf('[RX] Session complete, exiting.\n');
        break;
    end
end

release(radio_rx);
release(radio_tx);
fprintf('[RX] Receiver shutdown complete.\n');

%% =========== Helper Functions ===========
function [valid, ctrl_data] = decode_control_frame(rx_sig)
% Decode a control frame (BEACON/START/END) from received signal
valid = false;
ctrl_data = struct();

defs = link_phy_defs();
sps = 4;
Threshold = 200;

pcmatrix = ldpcQuasiCyclicMatrix(defs.blockSize, defs.P);
cfgLDPCDec = ldpcDecoderConfig(pcmatrix);
crcdetector = comm.CRCDetector(defs.poly);

demodulator = comm.PSKDemodulator(2, 'BitOutput', true, ...
    'DecisionMethod', 'Approximate log-likelihood ratio');
demodulator.PhaseOffset = pi/4;

rxfilter = comm.RaisedCosineReceiveFilter( ...
    'InputSamplesPerSymbol', sps, ...
    'DecimationFactor', 1, ...
    'RolloffFactor', 0.25);

Rec_sig = rxfilter(rx_sig(:));
data_sys = [];
buffer_h = [];
index_val = zeros(1, sps);
index_loc_h = cell(1, sps);

data_frame_len = 648 * 15;  % BPSK+15x spreading

for i = 1:sps
    data_sys(:, i) = Rec_sig(i:sps:end);
    buffer_h(:, i) = abs(conv(flip(defs.head_data), sign(data_sys(:, i))));
    cand = pick_sync_peaks_ctrl(buffer_h(:, i), Threshold);
    if ~isempty(cand)
        index_loc_h{i} = cand(:);
        index_val(i) = mean(buffer_h(cand, i));
    else
        index_loc_h{i} = [];
    end
end

	if all(index_val == 0), return; end

[~, op_index] = max(index_val);
Rec_sig_afr_temp = data_sys(:, op_index);
idx_start = index_loc_h{op_index};
idx_start = idx_start(idx_start + data_frame_len <= length(Rec_sig_afr_temp));

if isempty(idx_start), return; end

for j = 1:min(1, length(idx_start))  % Just decode first found
    idx = idx_start(j);
    train_len = min(511, idx);
    receive_train = Rec_sig_afr_temp(idx-train_len+1:idx);
    desire_seq = defs.head_data(end-train_len+1:end);
    temp = conj(desire_seq) .* receive_train;
    phase_est = -angle(mean(temp));

    Rec_sig_afr = Rec_sig_afr_temp(idx+1:idx+data_frame_len) .* exp(1j*phase_est);
    demod_signal = demodulator(Rec_sig_afr);

    data_desp = zeros(length(demod_signal)/15, 1);
    for ii = 1:length(demod_signal)/15
        data_desp(ii) = sum(demod_signal((ii-1)*15+1:ii*15) .* defs.pn_data);
    end

    deinter_matrix = reshape(data_desp, 18, 36).';
    de_interleaved = deinter_matrix(:);
    received_bits = ldpcDecode(de_interleaved, cfgLDPCDec, 10);
    de_scr = descramble_bits_ctrl(received_bits, defs.scr_seq);

    [data_rec, err] = crcdetector(de_scr(1:end-length(defs.ctrl_frame_end)));
    if err ~= 0, continue; end

    % Parse control frame
    offset = 0;
    ctrl_data.frame_head = bits_to_int_ctrl(data_rec(offset+1:offset+8)); offset = offset+8;
    ctrl_data.user_id    = bits_to_int_ctrl(data_rec(offset+1:offset+8)); offset = offset+8;
    ctrl_data.frame_type = bits_to_int_ctrl(data_rec(offset+1:offset+8)); offset = offset+8;
    ctrl_data.proto_ver  = bits_to_int_ctrl(data_rec(offset+1:offset+4)); offset = offset+4;
    ctrl_data.header_len = bits_to_int_ctrl(data_rec(offset+1:offset+4)); offset = offset+4;
    ctrl_data.session_id = bits_to_int_ctrl(data_rec(offset+1:offset+16)); offset = offset+16;
    ctrl_data.hop_seed   = bits_to_int_ctrl(data_rec(offset+1:offset+32)); offset = offset+32;
    ctrl_data.total_slots = bits_to_int_ctrl(data_rec(offset+1:offset+16)); offset = offset+16;
    ctrl_data.slot_len   = bits_to_int_ctrl(data_rec(offset+1:offset+32)); offset = offset+32;
    ctrl_data.codewords_per_slot = bits_to_int_ctrl(data_rec(offset+1:offset+16)); offset = offset+16;

    valid = true;
    return;
end
end

function fb_data = build_fb_data(frame_type, session_id, rx_state)
fb_data = struct();
fb_data.frame_type = frame_type;
fb_data.session_id = session_id;
fb_data.rx_state = rx_state;
end

function fb_data = fill_telemetry(fb_data, phy_metrics, frame_cache)
if ~isempty(phy_metrics)
    fb_data.snr_q8 = round((phy_metrics.snr_est + 20) * 4);
    fb_data.rssi_q8 = round(phy_metrics.rssi_dB * 4);
    fb_data.sync_metric_q8 = round(min(phy_metrics.sync_peak / 100, 255));
end
if ~isempty(frame_cache) && isfield(frame_cache, 'total_frame_num')
    fb_data.total_frame_num = frame_cache.total_frame_num;
    fb_data.rx_crc_ok_num = sum(frame_cache.received_map);
    fb_data.rx_lost_num = frame_cache.total_frame_num - fb_data.rx_crc_ok_num;
end
end

function v = bits_to_int_ctrl(bits)
v = (2.^(length(bits)-1:-1:0)) * bits(:);
end

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

function out = descramble_bits_ctrl(in, scr_seq)
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
disp('SDR resources released.');
end
