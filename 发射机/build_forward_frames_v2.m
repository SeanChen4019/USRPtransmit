function [frame_list, fec_info] = build_forward_frames_v2(src_packets, meta_info, fec_groups)
% BUILD_FORWARD_FRAMES_V2 Build V2 physical frames from source packets + FEC parity
%   [frame_list, fec_info] = build_forward_frames_v2(src_packets, meta_info, fec_groups)
%
%   src_packets: cell array of 40B uint8 source packets
%   meta_info: struct from build_file_container
%   fec_groups: struct array from fec_rs_encode_groups (each has .packets, .K, .R, .N, .group_id)
%
%   Returns:
%     frame_list: struct array with fields:
%       .frame_type (20=DATA, 21=PARITY, 22=META)
%       .info_bits (454 x 1 logical)
%       .session_id, .frame_id, .hop_slot_id, .hop_index
%       .fec_group_id, .fec_index, .is_parity
%     fec_info: struct with totals, slot assignments

defs = link_phy_defs();

% Count total frames
num_groups = length(fec_groups);
total_data_frames = 0;
total_parity_frames = 0;

for g = 1:num_groups
    total_data_frames = total_data_frames + fec_groups(g).K;
    total_parity_frames = total_parity_frames + fec_groups(g).R;
end

total_business_frames = total_data_frames + total_parity_frames;
session_id = randi(65535);

% Store frame info
frame_list = struct('frame_type', {}, 'info_bits', {}, 'session_id', {}, ...
    'frame_id', {}, 'hop_slot_id', {}, 'hop_index', {}, ...
    'fec_group_id', {}, 'fec_index', {}, 'is_parity', {});

frame_id_counter = 0;

% Build source DATA frames
src_idx = 1;
for g = 1:num_groups
    group = fec_groups(g);
    for k = 1:group.K
        frame_id_counter = frame_id_counter + 1;
        payload = uint8(group.packets(k, :)).';

        info = build_frame_info(defs, defs.FRAME_TYPE_DATA, session_id, ...
            total_business_frames, frame_id_counter, payload, 0, ...
            meta_info.file_type, g, k, group.K, src_idx, meta_info);

        frame_list(end+1).frame_type = defs.FRAME_TYPE_DATA; %#ok<AGROW>
        frame_list(end).info_bits = info;
        frame_list(end).session_id = session_id;
        frame_list(end).frame_id = frame_id_counter;
        frame_list(end).fec_group_id = g;
        frame_list(end).fec_index = k;
        frame_list(end).is_parity = 0;
        src_idx = src_idx + 1;
    end

    % Build PARITY frames
    for r = 1:group.R
        frame_id_counter = frame_id_counter + 1;
        payload = uint8(group.packets(group.K + r, :)).';

        info = build_frame_info(defs, defs.FRAME_TYPE_PARITY, session_id, ...
            total_business_frames, frame_id_counter, payload, 1, ...
            meta_info.file_type, g, r, group.K, 0, meta_info);

        frame_list(end+1).frame_type = defs.FRAME_TYPE_PARITY; %#ok<AGROW>
        frame_list(end).info_bits = info;
        frame_list(end).session_id = session_id;
        frame_list(end).frame_id = frame_id_counter;
        frame_list(end).fec_group_id = g;
        frame_list(end).fec_index = r;
        frame_list(end).is_parity = 1;
    end
end

% Assign hop slot IDs (each slot holds codewords_per_slot frames)
codewords_per_slot = defs.codewords_per_slot_default;
total_slots = ceil(total_business_frames / codewords_per_slot);
hop_seq = build_hop_sequence(meta_info.hop_seed, total_slots, defs.num_carriers);

for i = 1:length(frame_list)
    slot_id = ceil(frame_list(i).frame_id / codewords_per_slot);
    frame_list(i).hop_slot_id = slot_id;
    frame_list(i).hop_index = hop_seq(slot_id);
end

% Pack FEC info
fec_info = struct();
fec_info.session_id = session_id;
fec_info.total_frames = total_business_frames;
fec_info.total_slots = total_slots;
fec_info.codewords_per_slot = codewords_per_slot;
fec_info.num_groups = num_groups;
fec_info.hop_seq = hop_seq;
fec_info.total_data_frames = total_data_frames;
fec_info.total_parity_frames = total_parity_frames;
fec_info.hop_seed = meta_info.hop_seed;
fec_info.fec_k = fec_groups(1).K;
fec_info.fec_r = fec_groups(1).R;

fprintf('[FRAMES-V2] total=%d (DATA=%d, PARITY=%d) | groups=%d | slots=%d\n', ...
    total_business_frames, total_data_frames, total_parity_frames, num_groups, total_slots);
end

function info_bits = build_frame_info(defs, frame_type, session_id, total_frames, frame_id, ...
    payload, is_parity, file_type, fec_group_id, fec_index, fec_k, src_idx, meta_info)

payload_bytes = payload(:);
valid_bytes = min(length(payload_bytes), 40);
payload_padded = zeros(40, 1, 'uint8');
payload_padded(1:valid_bytes) = payload_bytes(1:valid_bytes);

payload_bits = zeros(320, 1);
for b = 1:40
    byte_val = payload_padded(b);
    for bit = 1:8
        payload_bits((b-1)*8 + bit) = bitget(byte_val, 9 - bit);
    end
end

flags = 0;
if frame_id == total_frames
    flags = bitset(flags, 1);  % last_file_frame
end

% Determine hop_slot_id and hop_index (default values, updated later)
hop_slot_id = 0;
hop_index = 0;

info_bits = [ ...
    defs.frame_head; ...                                    % 8
    defs.user_id; ...                                       % 8
    int_to_bits(frame_type, 8); ...                         % 8
    int_to_bits(defs.proto_ver, 4); ...                     % 4
    int_to_bits(flags, 4); ...                              % 4
    int_to_bits(session_id, 16); ...                        % 16
    int_to_bits(total_frames, 16); ...                      % 16
    int_to_bits(frame_id, 16); ...                          % 16
    int_to_bits(valid_bytes, 6); ...                        % 6
    int_to_bits(file_type, 3); ...                          % 3
    int_to_bits(0, 3); ...                                  % 3 (StreamID)
    int_to_bits(is_parity, 1); ...                          % 1
    int_to_bits(frame_id == total_frames, 1); ...           % 1 (LastInSession)
    int_to_bits(hop_slot_id, 12); ...                       % 12
    int_to_bits(hop_index, 4); ...                          % 4
    int_to_bits(fec_group_id, 12); ...                      % 12
    int_to_bits(fec_index, 6); ...                          % 6
    int_to_bits(fec_k, 6); ...                              % 6
    payload_bits];                                          % 320
end

function bits = int_to_bits(v, width)
bits = double(dec2bin(max(0, v), width) == '1').';
end
