function [waveform, tx_cache] = forward_frame_modulate_v2(frame_list, mode, fec_info)
% FORWARD_FRAME_MODULATE_V2 Modulate V2 frame info bits to complex baseband waveforms
%   [waveform, tx_cache] = forward_frame_modulate_v2(frame_list, mode, fec_info)
%
%   frame_list: struct array from build_forward_frames_v2
%   mode: 0=QPSK, 1=BPSK+spreading(sf=15)
%   fec_info: struct from build_forward_frames_v2
%
%   Returns:
%     waveform: concatenated complex baseband for all frames
%     tx_cache: struct with pre-built waveforms per frame

defs = link_phy_defs();
sps = 4;

txfilter = comm.RaisedCosineTransmitFilter( ...
    'OutputSamplesPerSymbol', sps, ...
    'RolloffFactor', 0.25);

pcmatrix = ldpcQuasiCyclicMatrix(defs.blockSize, defs.P);
cfgLDPCEnc = ldpcEncoderConfig(pcmatrix);
crcgenerator = comm.CRCGenerator(defs.poly);

if mode == 1
    M = 2; sf = 15;
    modulator = comm.PSKModulator(M, 'BitInput', true);
    modulator.PhaseOffset = pi;
else
    M = 4; sf = 1;
    modulator = comm.PSKModulator(M, 'BitInput', true);
    modulator.PhaseOffset = pi/4;
end

total_frames = length(frame_list);
waveforms = cell(1, total_frames);
wave_lens = zeros(1, total_frames);

for i = 1:total_frames
    frame_bits = frame_list(i).info_bits;

    % CRC32 + zero-pad to 486 bits
    coded_in = [crcgenerator(frame_bits); defs.data_frame_end];

    % Scramble
    scr_bits = scramble_bits_local(coded_in, defs.scr_seq);

    % LDPC encode to 648 bits
    enc_bits = ldpcEncode(scr_bits, cfgLDPCEnc);

    % Interleave (36x18)
    inter_matrix = reshape(enc_bits, 36, 18).';
    inter_bits = inter_matrix(:);

    % Modulate
    if mode == 1
        % BPSK + spreading
        inter_polar = 2 * inter_bits - 1;
        spread_seq = zeros(length(inter_polar) * sf, 1);
        for ii = 1:length(inter_polar)
            spread_seq((ii-1)*sf+1 : ii*sf) = inter_polar(ii) * defs.pn_data;
        end
        mod_sig = modulator(0.5 * (spread_seq + 1));
    else
        % QPSK
        mod_sig = modulator(inter_bits);
    end

    % Add preamble and tail
    tx_in = [defs.head_data; mod_sig; zeros(sps * 10, 1)];
    one_wave = txfilter(tx_in);

    waveforms{i} = one_wave;
    wave_lens(i) = length(one_wave);
end

% Build cache
tx_cache = struct();
tx_cache.waveforms = waveforms;
tx_cache.wave_lens = wave_lens;
tx_cache.total_frames = total_frames;
tx_cache.mode = mode;
tx_cache.fec_info = fec_info;
tx_cache.frame_list = frame_list;

% Concatenate all waveforms
waveform = [];
for i = 1:total_frames
    waveform = [waveform; waveforms{i}];
end
end

function out = scramble_bits_local(in, scr_seq)
out = zeros(size(in));
grp = length(scr_seq);
for ii = 1:floor(length(in)/grp)
    st = (ii-1)*grp + 1;
    ed = ii*grp;
    out(st:ed) = xor(in(st:ed), scr_seq);
end
end
