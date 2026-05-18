function metrics = compute_rx_metrics(phy_metrics, frame_cache, rebuild_info, tx_duration)
% COMPUTE_RX_METRICS Compute comprehensive reception quality metrics
%   metrics = compute_rx_metrics(phy_metrics, frame_cache, rebuild_info, tx_duration)
%
%   Returns:
%     metrics: struct with physical, link, and file-level quality indicators

metrics = struct();

%% Physical Layer Metrics
if ~isempty(phy_metrics)
    metrics.snr_db = phy_metrics.snr_est;
    metrics.rssi_dB = phy_metrics.rssi_dB;
    metrics.sync_peak = phy_metrics.sync_peak;
    metrics.peak_to_avg = phy_metrics.peak_to_avg;
    metrics.sync_success = phy_metrics.sync_success;
else
    metrics.snr_db = -Inf;
    metrics.rssi_dB = -Inf;
    metrics.sync_peak = 0;
    metrics.peak_to_avg = 0;
    metrics.sync_success = false;
end

%% Link Layer Metrics
if ~isempty(frame_cache) && isfield(frame_cache, 'total_frame_num')
    total_frames = frame_cache.total_frame_num;
    rx_crc_ok = sum(frame_cache.received_map);

    metrics.total_frame_num = total_frames;
    metrics.rx_crc_ok_num = rx_crc_ok;
    metrics.rx_lost_num = total_frames - rx_crc_ok;
    metrics.pre_fec_per = 1 - rx_crc_ok / max(1, total_frames);
else
    metrics.total_frame_num = 0;
    metrics.rx_crc_ok_num = 0;
    metrics.rx_lost_num = 0;
    metrics.pre_fec_per = 1.0;
end

%% FEC Recovery Metrics
if ~isempty(rebuild_info)
    metrics.fec_recovered_num = rebuild_info.recovered_source_packets;
    metrics.fec_groups_ok = rebuild_info.fec_groups_recovered;
    metrics.fec_groups_fail = rebuild_info.fec_groups_failed;
    metrics.post_fec_per = 1 - rebuild_info.recovery_rate;
    metrics.file_crc_match = rebuild_info.file_crc_match;
    metrics.recovery_rate = rebuild_info.recovery_rate;
else
    metrics.fec_recovered_num = 0;
    metrics.fec_groups_ok = 0;
    metrics.fec_groups_fail = 0;
    metrics.post_fec_per = 1.0;
    metrics.file_crc_match = false;
    metrics.recovery_rate = 0;
end

%% Throughput
if tx_duration > 0 && ~isempty(rebuild_info) && rebuild_info.success
    recovered_bytes = length(rebuild_info.meta) + rebuild_info.meta.file_size;
    metrics.goodput_kbps = (recovered_bytes * 8) / tx_duration / 1000;
else
    metrics.goodput_kbps = 0;
end

%% Result Code
if metrics.file_crc_match
    metrics.result_code = 2;  % RESULT_COMPLETE equivalent
elseif metrics.recovery_rate > 0
    metrics.result_code = 1;  % partial
else
    metrics.result_code = 0;  % failed
end

%% Summary
metrics.summary = sprintf(...
    'SNR=%.1fdB | CRC-OK=%d/%d | PreFEC-PER=%.1f%% | PostFEC-PER=%.1f%% | Goodput=%.1fkbps | CRC-Match=%d', ...
    metrics.snr_db, metrics.rx_crc_ok_num, metrics.total_frame_num, ...
    metrics.pre_fec_per*100, metrics.post_fec_per*100, ...
    metrics.goodput_kbps, metrics.file_crc_match);
end
