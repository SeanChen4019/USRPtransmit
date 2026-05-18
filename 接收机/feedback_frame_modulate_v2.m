function Trans_sig = feedback_frame_modulate_v2(fb_data)
% FEEDBACK_FRAME_MODULATE_V2 Modulate V2 feedback frame
%   Trans_sig = feedback_frame_modulate_v2(fb_data)

defs = link_phy_defs();
sps = 4;

pcmatrix = ldpcQuasiCyclicMatrix(defs.blockSize, defs.P);
cfgLDPCEnc = ldpcEncoderConfig(pcmatrix);
crcgenerator = comm.CRCGenerator(defs.poly);

bpskmod = comm.PSKModulator(2, 'BitInput', true);
bpskmod.PhaseOffset = pi/4;

txfilter = comm.RaisedCosineTransmitFilter( ...
    'OutputSamplesPerSymbol', sps, ...
    'RolloffFactor', 0.25);

info_bits = feedback_frame_pack_v2(fb_data);
coded_in = [crcgenerator(info_bits); defs.fb_frame_end];

scr_bits = scramble_bits_local(coded_in, defs.scr_seq);
enc_bits = ldpcEncode(scr_bits, cfgLDPCEnc);

inter_matrix = reshape(enc_bits, 36, 18).';
inter_bits = inter_matrix(:);

% BPSK + 15x spreading
inter_polar = 2 * inter_bits - 1;
spread_seq = zeros(length(inter_polar) * 15, 1);
for ii = 1:length(inter_polar)
    spread_seq((ii-1)*15+1 : ii*15) = inter_polar(ii) * defs.pn_fb;
end

mod_signal = bpskmod(0.5 * (spread_seq + 1));
tx_in = [defs.head_fb; mod_signal; zeros(sps*10, 1)];
Trans_sig = txfilter(tx_in);
Trans_sig = [zeros(2000, 1); Trans_sig];
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
