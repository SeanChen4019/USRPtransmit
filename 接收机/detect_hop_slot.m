function [detections, phy_metrics] = detect_hop_slot(rx_slot, mode, Threshold)
% DETECT_HOP_SLOT Preamble correlation and slot synchronization for one hop slot
%   [detections, phy_metrics] = detect_hop_slot(rx_slot, mode, Threshold)
%
%   rx_slot: complex baseband samples of one hop slot
%   mode: 0=QPSK, 1=BPSK+spreading
%   Threshold: sync peak threshold
%
%   Returns:
%     detections: struct array with fields .start_idx, .phase_est
%     phy_metrics: struct with SNR, RSSI, sync_peak, peak_to_avg

defs = link_phy_defs();
sps = 4;

rxfilter = comm.RaisedCosineReceiveFilter( ...
    'InputSamplesPerSymbol', sps, ...
    'DecimationFactor', 1, ...
    'RolloffFactor', 0.25);

if mode == 1
    sf = 15;
    data_frame_len = 648 / 1 * sf;  % BPSK symbols with spreading
else
    sf = 1;
    data_frame_len = 648 / 2;  % QPSK symbols
end

% Apply receive filter
Rec_sig = rxfilter(rx_slot(:));

% Multi-phase sync
data_sys = [];
buffer_h = [];
index_val = zeros(1, sps);
index_loc_h = cell(1, sps);

for i = 1:sps
    data_sys(:, i) = Rec_sig(i:sps:end);
    buffer_h(:, i) = abs(conv(flip(defs.head_data), sign(data_sys(:, i))));
    cand = pick_sync_peaks_local(buffer_h(:, i), Threshold);

    if ~isempty(cand)
        index_loc_h{i} = cand(:);
        index_val(i) = mean(buffer_h(cand, i));
    else
        index_loc_h{i} = [];
    end
end

detections = struct('start_idx', {}, 'phase_est', {});
phy_metrics = struct();
phy_metrics.sync_peak = 0;
phy_metrics.peak_to_avg = 0;
phy_metrics.rssi_dB = 0;
phy_metrics.snr_est = 0;
phy_metrics.sync_success = false;

if all(index_val == 0)
    return;
end

[~, op_index] = max(index_val);
Rec_sig_afr_temp = data_sys(:, op_index);
detect_idx = index_loc_h{op_index};

if isempty(detect_idx)
    return;
end

% Filter valid detections
detect_idx = detect_idx(detect_idx + data_frame_len <= length(Rec_sig_afr_temp));
if isempty(detect_idx)
    return;
end

% Phase estimation and metrics
train_len = min(defs.slot_preamble_len, length(defs.head_data));

for j = 1:length(detect_idx)
    idx = detect_idx(j);
    if idx < train_len, continue; end

    receive_train = Rec_sig_afr_temp(idx-train_len+1:idx);
    desire_seq = defs.head_data(end-train_len+1:end);
    temp = conj(desire_seq) .* receive_train;
    phase_est = -angle(mean(temp));

    detections(end+1).start_idx = idx; %#ok<AGROW>
    detections(end).phase_est = phase_est;
end

% Physical layer metrics
if ~isempty(detections)
    phy_metrics.sync_success = true;
    phy_metrics.sync_peak = index_val(op_index);

    % Peak-to-average ratio
    avg_metric = mean(buffer_h(:, op_index));
    if avg_metric > 0
        phy_metrics.peak_to_avg = index_val(op_index) / avg_metric;
    end

    % RSSI
    phy_metrics.rssi_dB = 10 * log10(mean(abs(rx_slot(:)).^2) + eps);

    % SNR estimate (simple)
    sig_power = mean(abs(rx_slot(:)).^2);
    noise_power = sig_power / (1 + phy_metrics.peak_to_avg);
    if noise_power > 0
        phy_metrics.snr_est = 10 * log10((sig_power - noise_power) / noise_power + eps);
    end
end
end

function cand = pick_sync_peaks_local(metric, thr)
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
