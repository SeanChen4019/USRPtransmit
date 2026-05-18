function [datavalid, fb_data] = feedback_frame_decode_v2(Rec_sig0)
% FEEDBACK_FRAME_DECODE_V2 Decode V2 feedback frame (256-bit info) at transmitter
%   [datavalid, fb_data] = feedback_frame_decode_v2(Rec_sig0)
%
%   Returns fb_data struct with all V2 feedback fields

datavalid = 0;
fb_data = struct();

defs = link_phy_defs();
sps = 4;
Threshold = 220;

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

Rec_sig = rxfilter(Rec_sig0);
data_sys = [];
buffer_h = [];
index_val = zeros(1, sps);
index_loc_h = cell(1, sps);
syn_flag = 0;

data_frame_len = 648 * 15;  % BPSK + 15x spreading

for i = 1:sps
    data_sys(:, i) = Rec_sig(i:sps:end);
    buffer_h(:, i) = abs(conv(flip(defs.head_fb), sign(data_sys(:, i))));
    cand = pick_sync_peaks_local(buffer_h(:, i), Threshold);

    if ~isempty(cand)
        syn_flag = 1;
        index_loc_h{i} = cand(:);
        index_val(i) = mean(buffer_h(cand, i));
    else
        index_loc_h{i} = [];
    end
end

if syn_flag == 0, return; end

[~, op_index] = max(index_val);
Rec_sig_afr_temp = data_sys(:, op_index);
index_start_temp = index_loc_h{op_index};
index_start_temp = index_start_temp(index_start_temp + data_frame_len <= length(Rec_sig_afr_temp));

if isempty(index_start_temp), return; end

for j = 1:length(index_start_temp)
    index_start = index_start_temp(j);

    train_len = min(1023, index_start);
    receive_train_seq_tem = Rec_sig_afr_temp(index_start-train_len+1:index_start);
    desire_seq = defs.head_fb(end-train_len+1:end);
    temp = conj(desire_seq) .* receive_train_seq_tem;
    phase_est = -angle(mean(temp));

    Rec_sig_afr = Rec_sig_afr_temp(index_start+1:index_start+data_frame_len) .* exp(1j * phase_est);
    demod_signal = demodulator(Rec_sig_afr);

    data_desp = zeros(length(demod_signal)/15, 1);
    for ii = 1:length(demod_signal)/15
        data_desp(ii) = sum(demod_signal((ii-1)*15+1 : ii*15) .* defs.pn_fb);
    end

    deinter_matrix = reshape(data_desp, 18, 36).';
    de_interleaved_data = deinter_matrix(:);
    received_bits = ldpcDecode(de_interleaved_data, cfgLDPCDec, 10);
    de_scr_data = descramble_bits_local(received_bits, defs.scr_seq);

    [data_rec, err] = crcdetector(de_scr_data(1:end-length(defs.fb_frame_end)));
    if err ~= 0, continue; end

    % Parse 256-bit V2 feedback info
    fb_data = parse_feedback_v2(data_rec, defs);
    datavalid = 1;
    return;
end
end

function fb_data = parse_feedback_v2(data_rec, defs)
offset = 0;

fb_data.frame_head    = bits_to_int_local(data_rec(offset+1:offset+8)); offset = offset+8;
fb_data.user_id       = bits_to_int_local(data_rec(offset+1:offset+8)); offset = offset+8;
fb_data.frame_type    = bits_to_int_local(data_rec(offset+1:offset+8)); offset = offset+8;
fb_data.proto_ver     = bits_to_int_local(data_rec(offset+1:offset+4)); offset = offset+4;
fb_data.header_len    = bits_to_int_local(data_rec(offset+1:offset+4)); offset = offset+4;
fb_data.session_id    = bits_to_int_local(data_rec(offset+1:offset+16)); offset = offset+16;
fb_data.feedback_seq  = bits_to_int_local(data_rec(offset+1:offset+16)); offset = offset+16;
fb_data.rx_state      = bits_to_int_local(data_rec(offset+1:offset+4)); offset = offset+4;
fb_data.file_type     = bits_to_int_local(data_rec(offset+1:offset+4)); offset = offset+4;
fb_data.snr_q8        = bits_to_int_local(data_rec(offset+1:offset+8)); offset = offset+8;
fb_data.rssi_q8       = bits_to_int_local(data_rec(offset+1:offset+8)); offset = offset+8;
fb_data.noise_q8      = bits_to_int_local(data_rec(offset+1:offset+8)); offset = offset+8;
fb_data.evm_q8        = bits_to_int_local(data_rec(offset+1:offset+8)); offset = offset+8;
fb_data.cfo_i16       = bits_to_int_local(data_rec(offset+1:offset+16)); offset = offset+16;
fb_data.sync_metric_q8= bits_to_int_local(data_rec(offset+1:offset+8)); offset = offset+8;
fb_data.hop_index     = bits_to_int_local(data_rec(offset+1:offset+4)); offset = offset+4;
fb_data.mode          = bits_to_int_local(data_rec(offset+1:offset+4)); offset = offset+4;
fb_data.total_frame_num   = bits_to_int_local(data_rec(offset+1:offset+16)); offset = offset+16;
fb_data.rx_crc_ok_num     = bits_to_int_local(data_rec(offset+1:offset+16)); offset = offset+16;
fb_data.rx_lost_num       = bits_to_int_local(data_rec(offset+1:offset+16)); offset = offset+16;
fb_data.fec_recovered_num = bits_to_int_local(data_rec(offset+1:offset+16)); offset = offset+16;
fb_data.pre_fec_per_q16   = bits_to_int_local(data_rec(offset+1:offset+16)); offset = offset+16;
fb_data.post_fec_per_q16  = bits_to_int_local(data_rec(offset+1:offset+16)); offset = offset+16;
fb_data.goodput_kbps_q16  = bits_to_int_local(data_rec(offset+1:offset+16)); offset = offset+16;
fb_data.result_code   = bits_to_int_local(data_rec(offset+1:offset+8)); offset = offset+8;
fb_data.advice        = bits_to_int_local(data_rec(offset+1:offset+8)); offset = offset+8;

% Interpret quantized values
fb_data.snr_db = (double(fb_data.snr_q8) / 4) - 20;
fb_data.pre_fec_per = double(fb_data.pre_fec_per_q16) / 65535;
fb_data.post_fec_per = double(fb_data.post_fec_per_q16) / 65535;
fb_data.goodput_kbps = double(fb_data.goodput_kbps_q16) / 65535;
end

function v = bits_to_int_local(bits)
v = (2.^(length(bits)-1:-1:0)) * bits(:);
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

function out = descramble_bits_local(in, scr_seq)
out = zeros(size(in));
grp = length(scr_seq);
for ii = 1:floor(length(in)/grp)
    st = (ii-1)*grp + 1;
    ed = ii*grp;
    out(st:ed) = xor(in(st:ed), scr_seq);
end
end
