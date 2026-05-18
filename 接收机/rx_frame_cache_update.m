function cache = rx_frame_cache_update(cache, frame_packets, session_id)
% RX_FRAME_CACHE_UPDATE Update frame cache with newly decoded frames
%   cache = rx_frame_cache_update(cache, frame_packets, session_id)
%
%   cache: struct with fields:
%     .session_id, .total_frame_num, .fec_k (per-group)
%     .frames: cell array indexed by frame_id, each = struct with payload, fec info
%     .received_map: logical array, true for received frames
%     .meta_received: logical, whether META frames fully received
%     .meta_payload: accumulated META bytes

if isempty(frame_packets)
    return;
end

% Initialize cache on first use
if ~isfield(cache, 'session_id') || isempty(cache.session_id)
    cache.session_id = session_id;
    cache.total_frame_num = 0;
    cache.frames = {};
    cache.received_map = [];
    cache.meta_received = false;
    cache.meta_payload = [];
    cache.fec_info = struct();
end

defs = link_phy_defs();

% Determine max total_frame_num from received frames
for i = 1:length(frame_packets)
    pkt = frame_packets(i);
    if pkt.total_frame_num > cache.total_frame_num
        cache.total_frame_num = pkt.total_frame_num;
    end
end

% Expand cache if needed
if length(cache.received_map) < cache.total_frame_num
    cache.frames{end+1:cache.total_frame_num} = deal([]);
    cache.received_map(end+1:cache.total_frame_num) = false;
end

% Store frames
for i = 1:length(frame_packets)
    pkt = frame_packets(i);
    fid = pkt.frame_id;

    if fid < 1 || fid > cache.total_frame_num
        continue;
    end

    % Store frame data
    one_frame = struct();
    one_frame.frame_type = pkt.frame_type;
    one_frame.payload = pkt.payload_bytes;
    one_frame.valid_bytes = pkt.valid_bytes;
    one_frame.is_parity = pkt.is_parity;
    one_frame.fec_group_id = pkt.fec_group_id;
    one_frame.fec_index = pkt.fec_index;
    one_frame.fec_k = pkt.fec_k;

    cache.frames{fid} = one_frame;
    cache.received_map(fid) = true;

    % Accumulate META frames
    if pkt.frame_type == defs.FRAME_TYPE_META
        cache.meta_payload = [cache.meta_payload; pkt.payload_bytes(1:pkt.valid_bytes)];
    end

    % Track per-group FEC K
    if ~isfield(cache.fec_info, 'k_per_group')
        cache.fec_info.k_per_group = containers.Map('KeyType', 'double', 'ValueType', 'double');
    end
    gid = pkt.fec_group_id;
    if isKey(cache.fec_info.k_per_group, gid)
        cache.fec_info.k_per_group(gid) = max(cache.fec_info.k_per_group(gid), pkt.fec_k);
    else
        cache.fec_info.k_per_group(gid) = pkt.fec_k;
    end
end
end
