function [frame_packets, decode_stats] = decode_forward_codewords_v2(rx_slot, detections, mode, slotted_rx_signal)
% DECODE_FORWARD_CODEWORDS_V2 Decode V2 forward codewords from hop slot
%   [frame_packets, decode_stats] = decode_forward_codewords_v2(rx_slot, detections, mode, slotted_rx_signal)
%
%   rx_slot: raw complex baseband samples (before RRC filter)
%   detections: from detect_hop_slot (start indices and phase estimates)
%   mode: 0=QPSK, 1=BPSK+spreading
%   slotted_rx_signal: the actual received signal vector
%
%   For each detection, tries all 4 sample phases and picks CRC-passing one.

defs = link_phy_defs();
sps = 4;

pcmatrix = ldpcQuasiCyclicMatrix(defs.blockSize, defs.P);
cfgLDPCDec = ldpcDecoderConfig(pcmatrix);
crcdetector = comm.CRCDetector(defs.poly);

rxfilter = comm.RaisedCosineReceiveFilter( ...
    'InputSamplesPerSymbol', sps, ...
    'DecimationFactor', 1, ...
    'RolloffFactor', 0.25);

if mode == 1
    M = 2; sf = 15;
    demodulator = comm.PSKDemodulator(M, 'BitOutput', true, ...
        'DecisionMethod', 'Approximate log-likelihood ratio');
    demodulator.PhaseOffset = pi;
    data_frame_len = 648 * sf;
else
    M = 4; sf = 1;
    demodulator = comm.PSKDemodulator(M, 'BitOutput', true, ...
        'DecisionMethod', 'Approximate log-likelihood ratio');
    demodulator.PhaseOffset = pi/4;
    data_frame_len = 648;
end

% Apply receive filter and build multi-phase arrays
Rec_sig = rxfilter(slotted_rx_signal(:));
data_sys_all = cell(1, sps);
for i = 1:sps
    data_sys_all{i} = Rec_sig(i:sps:end);
end

frame_packets = struct('frame_type', {}, 'session_id', {}, 'frame_id', {}, ...
    'total_frame_num', {}, 'payload_bytes', {}, 'valid_bytes', {}, ...
    'file_type', {}, 'is_parity', {}, 'last_in_session', {}, ...
    'hop_slot_id', {}, 'hop_index', {}, 'fec_group_id', {}, ...
    'fec_index', {}, 'fec_k', {}, 'crc_ok', {});

decode_stats = struct();
decode_stats.crc_ok_count = 0;
decode_stats.crc_fail_count = 0;
decode_stats.total_attempted = 0;

seen_frames = [];

for j = 1:length(detections)
    det = detections(j);

    % Try all four sample phases, pick the one with CRC pass
    data_rec_best = [];
    for p = 1:sps
        phase_data = data_sys_all{p};
        idx_in_phase = det.start_idx;

        if idx_in_phase + data_frame_len > length(phase_data)
            continue;
        end

        Rec_sig_afr = phase_data(idx_in_phase+1 : idx_in_phase+data_frame_len) .* exp(1j * det.phase_est);
        demod_signal = demodulator(Rec_sig_afr);

        % Despread if BPSK mode
        if mode == 1
            if length(demod_signal) < 648 * sf, continue; end
            demod_signal_trim = demod_signal(1:648*sf);
            data_desp = zeros(648, 1);
            for ii = 1:648
                data_desp(ii) = sum(demod_signal_trim((ii-1)*sf+1 : ii*sf) .* defs.pn_data);
            end
        else
            if length(demod_signal) < 648, continue; end
            data_desp = demod_signal(1:648);
        end

        % Deinterleave: 18 rows x 36 cols, read column-wise
        deinter_matrix = reshape(data_desp, 18, 36).';
        de_interleaved_data = deinter_matrix(:);

        % LDPC decode (max 10 iterations)
        received_bits = ldpcDecode(de_interleaved_data, cfgLDPCDec, 10);

        % Descramble
        de_scr_data = descramble_bits_local(received_bits, defs.scr_seq);

        % CRC check
        [data_rec, err] = crcdetector(de_scr_data(1:end-length(defs.data_frame_end)));
        decode_stats.total_attempted = decode_stats.total_attempted + 1;

        if err == 0
            data_rec_best = data_rec;
            decode_stats.crc_ok_count = decode_stats.crc_ok_count + 1;
            break;
        else
            decode_stats.crc_fail_count = decode_stats.crc_fail_count + 1;
        end
    end

    if isempty(data_rec_best)
        continue;
    end

    % Parse V2 frame header
    frame = parse_v2_frame(data_rec_best, defs);

    % Dedup within this slot
    if any(seen_frames == frame.frame_id)
        continue;
    end
    seen_frames(end+1) = frame.frame_id; %#ok<AGROW>

    frame_packets(end+1) = frame; %#ok<AGROW>
    frame_packets(end).crc_ok = true;
end
end

function frame = parse_v2_frame(data_rec, defs)
offset = 0;
frame.frame_head    = bits_to_int_local(data_rec(offset+1:offset+8)); offset = offset+8;
frame.user_id       = bits_to_int_local(data_rec(offset+1:offset+8)); offset = offset+8;
frame.frame_type    = bits_to_int_local(data_rec(offset+1:offset+8)); offset = offset+8;
frame.proto_ver     = bits_to_int_local(data_rec(offset+1:offset+4)); offset = offset+4;
frame.flags         = bits_to_int_local(data_rec(offset+1:offset+4)); offset = offset+4;
frame.session_id    = bits_to_int_local(data_rec(offset+1:offset+16)); offset = offset+16;
frame.total_frame_num = bits_to_int_local(data_rec(offset+1:offset+16)); offset = offset+16;
frame.frame_id      = bits_to_int_local(data_rec(offset+1:offset+16)); offset = offset+16;
frame.valid_bits    = bits_to_int_local(data_rec(offset+1:offset+6)); offset = offset+6;
frame.file_type     = bits_to_int_local(data_rec(offset+1:offset+3)); offset = offset+3;
frame.stream_id     = bits_to_int_local(data_rec(offset+1:offset+3)); offset = offset+3;
frame.is_parity     = bits_to_int_local(data_rec(offset+1:offset+1)); offset = offset+1;
frame.last_in_session = bits_to_int_local(data_rec(offset+1:offset+1)); offset = offset+1;
frame.hop_slot_id   = bits_to_int_local(data_rec(offset+1:offset+12)); offset = offset+12;
frame.hop_index     = bits_to_int_local(data_rec(offset+1:offset+4)); offset = offset+4;
frame.fec_group_id  = bits_to_int_local(data_rec(offset+1:offset+12)); offset = offset+12;
frame.fec_index     = bits_to_int_local(data_rec(offset+1:offset+6)); offset = offset+6;
frame.fec_k         = bits_to_int_local(data_rec(offset+1:offset+6)); offset = offset+6;

% Extract payload
valid_bytes = min(floor(frame.valid_bits / 8), 40);
payload_bits = data_rec(offset+1 : offset+320);
payload_uint8 = zeros(valid_bytes, 1, 'uint8');
for b = 1:valid_bytes
    byte_bits = payload_bits((b-1)*8+1 : b*8);
    payload_uint8(b) = uint8(bits_to_int_local(byte_bits));
end

frame.valid_bytes = valid_bytes;
frame.payload_bytes = payload_uint8;
end

function v = bits_to_int_local(bits)
v = (2.^(length(bits)-1:-1:0)) * bits(:);
end

function out = descramble_bits_local(in, scr_seq)
out = zeros(size(in));
grp = length(scr_seq);
for ii = 1:floor(length(in)/grp)
    st = (ii-1)*grp + 1;
    ed = ii*grp;
    out(st:ed) = xor(in(st:ed), scr_seq);
end
end
