function slot_cache = build_hop_slot_waveform(tx_cache, fec_info)
% BUILD_HOP_SLOT_WAVEFORM Assemble hop slot waveforms from pre-modulated frames
%   slot_cache = build_hop_slot_waveform(tx_cache, fec_info)
%
%   Groups frames into hop slots, each slot containing:
%     GuardPre + Preamble + Codeword1 + Codeword2 + ... + CodewordN + GuardPost
%
%   Returns:
%     slot_cache: struct array with fields:
%       .slot_id, .carrier_index, .waveform, .frame_ids, .num_frames

defs = link_phy_defs();

total_frames = tx_cache.total_frames;
codewords_per_slot = fec_info.codewords_per_slot;
total_slots = fec_info.total_slots;
hop_seq = fec_info.hop_seq;
waveforms = tx_cache.waveforms;
wave_lens = tx_cache.wave_lens;

slot_cache = struct('slot_id', {}, 'carrier_index', {}, 'waveform', {}, ...
    'frame_ids', {}, 'num_frames', {});

% Use per-frame preamble for low-risk version (V2 phase 1)
% Each frame already has its own preamble from forward_frame_modulate_v2
% Slot just packs them together with guard intervals

frame_idx = 1;

for slot_id = 1:total_slots
    slot_wave = zeros(defs.slot_len_samples, 1);
    wr = defs.guard_pre_samples + 1;

    frame_ids_in_slot = [];
    num_in_slot = 0;

    for cw = 1:codewords_per_slot
        if frame_idx > total_frames
            break;
        end

        one_wave = waveforms{frame_idx};
        L = wave_lens(frame_idx);

        if wr + L - 1 > defs.slot_len_samples - defs.guard_post_samples
            break;
        end

        slot_wave(wr : wr + L - 1) = one_wave;
        wr = wr + L + 100;  % small gap between codewords
        frame_ids_in_slot(end+1) = frame_idx; %#ok<AGROW>
        num_in_slot = num_in_slot + 1;
        frame_idx = frame_idx + 1;
    end

    slot_cache(end+1).slot_id = slot_id; %#ok<AGROW>
    slot_cache(end).carrier_index = hop_seq(slot_id);
    slot_cache(end).waveform = slot_wave;
    slot_cache(end).frame_ids = frame_ids_in_slot;
    slot_cache(end).num_frames = num_in_slot;
end

fprintf('[SLOT-CACHE] 总时隙=%d | 每时隙码字=%d\n', total_slots, codewords_per_slot);
end
