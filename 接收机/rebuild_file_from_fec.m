function [file_bytes, rebuild_info] = rebuild_file_from_fec(cache, save_dir)
% REBUILD_FILE_FROM_FEC Rebuild file from received frames using FEC recovery
%   [file_bytes, rebuild_info] = rebuild_file_from_fec(cache, save_dir)
%
%   cache: struct from rx_frame_cache_update with received frames
%   save_dir: directory to save recovered file
%
%   Returns:
%     file_bytes: recovered file bytes (uint8), or empty on failure
%     rebuild_info: struct with recovery statistics

file_bytes = [];
rebuild_info = struct();
rebuild_info.success = false;
rebuild_info.total_source_packets = 0;
rebuild_info.recovered_source_packets = 0;
rebuild_info.fec_groups_recovered = 0;
rebuild_info.fec_groups_failed = 0;
rebuild_info.recovery_rate = 0;
rebuild_info.file_crc_match = false;
rebuild_info.recovered_file_path = '';
rebuild_info.meta = [];

defs = link_phy_defs();
payload_len = defs.payload_bytes_per_pkt;

if cache.total_frame_num == 0
    rebuild_info.status = 'No frames received';
    return;
end

% Identify FEC groups
frame_data = cache.frames;
received_map = cache.received_map;
total_frames = cache.total_frame_num;

% Build list of FEC groups
group_ids = [];
for fid = 1:total_frames
    if received_map(fid) && ~isempty(frame_data{fid})
        gid = frame_data{fid}.fec_group_id;
        if ~ismember(gid, group_ids)
            group_ids(end+1) = gid; %#ok<AGROW>
        end
    end
end

if isempty(group_ids)
    rebuild_info.status = 'No valid FEC groups found';
    return;
end

% Count source packets
total_src = 0;
recovered_src = 0;
groups_ok = 0;
groups_fail = 0;

% Recover each FEC group
all_source_bytes = cell(1, max(group_ids) * defs.fec_k_default);
src_idx = 0;

for g = 1:length(group_ids)
    gid = group_ids(g);

    % Find frames belonging to this group
    group_frames = [];
    for fid = 1:total_frames
        if received_map(fid) && ~isempty(frame_data{fid}) && frame_data{fid}.fec_group_id == gid
            group_frames(end+1) = fid; %#ok<AGROW>
        end
    end

    if isempty(group_frames)
        continue;
    end

    % Determine K and N for this group
    K = frame_data{group_frames(1)}.fec_k;
    num_src_found = 0;
    num_par_found = 0;
    for fi = 1:length(group_frames)
        if frame_data{group_frames(fi)}.is_parity == 0
            num_src_found = num_src_found + 1;
        else
            num_par_found = num_par_found + 1;
        end
    end
    N = K + max(1, num_par_found);

    total_src = total_src + K;

    % Build received matrix for this group
    % Each packet = 1 row of N x payload_len
    group_data = zeros(N, payload_len, 'uint8');
    group_received = false(1, N);

    for fi = 1:length(group_frames)
        fid = group_frames(fi);
        pkt = frame_data{fid};
        row_idx = pkt.fec_index;
        if pkt.is_parity
            row_idx = K + row_idx;
        end
        if row_idx < 1 || row_idx > N
            continue;
        end
        payload = pkt.payload;
        L = min(length(payload), payload_len);
        group_data(row_idx, 1:L) = payload(1:L);
        group_received(row_idx) = true;
    end

    num_rcv = sum(group_received);

    if num_rcv >= K
        % Can recover
        encoded = struct();
        encoded.packets = group_data;
        encoded.K = K;
        encoded.N = N;
        encoded.R = N - K;

        recovered = fec_rs_decode_groups(encoded, group_received, payload_len);
        if ~isempty(recovered)
            for k = 1:K
                src_idx = src_idx + 1;
                all_source_bytes{src_idx} = uint8(recovered(k, :)).';
            end
            recovered_src = recovered_src + K;
            groups_ok = groups_ok + 1;
        else
            groups_fail = groups_fail + 1;
        end
    else
        % Not enough packets - take what we have
        for k = 1:K
            if group_received(k)
                src_idx = src_idx + 1;
                all_source_bytes{src_idx} = uint8(group_data(k, :)).';
                recovered_src = recovered_src + 1;
            end
        end
        if num_rcv < K
            groups_fail = groups_fail + 1;
        end
    end
end

% Concatenate all source bytes
all_bytes = [];
for i = 1:src_idx
    if ~isempty(all_source_bytes{i})
        all_bytes = [all_bytes; all_source_bytes{i}(:)];
    end
end

% Parse META from beginning of all_bytes
if length(all_bytes) < 4
    rebuild_info.status = 'Not enough data for META header';
    return;
end

meta = parse_meta_tlv(all_bytes, defs);
if isempty(meta)
    rebuild_info.status = 'Failed to parse META TLV';
    return;
end

% Extract file payload (after META header)
meta_total_size = 4+1+1+1+1+4+4+4+2+4+4+4+2+1+1;
name_len = double(all_bytes(meta_total_size-1));
ext_len = double(all_bytes(meta_total_size));
meta_size = meta_total_size + name_len + ext_len;

if length(all_bytes) < meta_size
    rebuild_info.status = 'Incomplete META TLV';
    return;
end

file_payload = all_bytes(meta_size+1:end);

% Trim to actual file size
if meta.file_size <= length(file_payload)
    file_payload = file_payload(1:meta.file_size);
end

% Verify file CRC
file_crc = crc32_bytes(file_payload);
rebuild_info.file_crc_match = (file_crc == meta.file_crc32);

% Write recovered file
if ~isempty(save_dir)
    recovered_name = fullfile(save_dir, ['recovered_', meta.name, meta.ext]);
    fid = fopen(recovered_name, 'wb');
    if fid ~= -1
        fwrite(fid, file_payload, 'uint8');
        fclose(fid);
        rebuild_info.recovered_file_path = recovered_name;
    end
end

file_bytes = file_payload;
rebuild_info.success = true;
rebuild_info.total_source_packets = total_src;
rebuild_info.recovered_source_packets = recovered_src;
rebuild_info.fec_groups_recovered = groups_ok;
rebuild_info.fec_groups_failed = groups_fail;
rebuild_info.recovery_rate = recovered_src / max(1, total_src);
rebuild_info.meta = meta;

if rebuild_info.file_crc_match
    rebuild_info.status = 'File fully recovered, CRC32 match';
else
    rebuild_info.status = sprintf('File partially recovered (%.1f%%), CRC32 mismatch', ...
        rebuild_info.recovery_rate * 100);
end
end

function meta = parse_meta_tlv(all_bytes, defs)
% Parse META TLV from byte stream
meta = [];
if length(all_bytes) < 4, return; end

magic = all_bytes(1:4);
if ~isequal(magic(:), defs.meta_magic(:))
    return;
end

meta = struct();
offset = 4;
meta.proto_ver = double(all_bytes(offset+1)); offset = offset+1;
meta.file_type = double(all_bytes(offset+1)); offset = offset+1;
meta.fec_k = double(all_bytes(offset+1)); offset = offset+1;
meta.fec_r = double(all_bytes(offset+1)); offset = offset+1;
meta.file_size = double(typecast(all_bytes(offset+1:offset+4), 'uint32')); offset = offset+4;
meta.source_packet_num = double(typecast(all_bytes(offset+1:offset+4), 'uint32')); offset = offset+4;
meta.parity_packet_num = double(typecast(all_bytes(offset+1:offset+4), 'uint32')); offset = offset+4;
meta.total_group_num = double(typecast(all_bytes(offset+1:offset+2), 'uint16')); offset = offset+2;
meta.file_crc32 = typecast(all_bytes(offset+1:offset+4), 'uint32'); offset = offset+4;
meta.hop_seed = double(typecast(all_bytes(offset+1:offset+4), 'uint32')); offset = offset+4;
meta.slot_len_samples = double(typecast(all_bytes(offset+1:offset+4), 'uint32')); offset = offset+4;
meta.codewords_per_slot = double(typecast(all_bytes(offset+1:offset+2), 'uint16')); offset = offset+2;
name_len = double(all_bytes(offset+1)); offset = offset+1;
ext_len = double(all_bytes(offset+1)); offset = offset+1;

if offset + name_len + ext_len > length(all_bytes)
    return;
end

meta.name = native2unicode(all_bytes(offset+1:offset+name_len).', 'UTF-8');
offset = offset + name_len;
meta.ext = native2unicode(all_bytes(offset+1:offset+ext_len).', 'UTF-8');
end
