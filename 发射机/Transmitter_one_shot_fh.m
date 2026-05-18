% =========== One-Shot Frequency Hopping Transmitter ===========
% V2 Protocol: TX_BEACON -> RX_READY -> TX_START -> DATA_ONCE -> TX_END -> DONE
% Business data sent exactly once, no retransmission.
clear
clc
close all force
warning('off', 'all');
fprintf('\n========== One-Shot FH Transmitter ==========\n');

%% =========== Configuration ===========
defs = link_phy_defs();

% ---- Transmission Parameters ----
transmit_mode = 'image';  % 'text' | 'image' | 'video'
file_name = 'p2.jpg';
text_content = 'Hello, One-Shot FH!';

Anti_Jamming_Mode = 0;    % 0=QPSK, 1=BPSK+spreading
Power_gain = 15;
Power = 0.7;

fec_k = defs.fec_k_default;  % 24
fec_r = defs.fec_r_default;  % 8

hop_seed = randi(65535);

%% =========== State Machine Constants ===========
STATE_INIT           = 0;
STATE_WAIT_READY     = 1;
STATE_START_COUNTDOWN = 2;
STATE_DATA_ONCE      = 3;
STATE_END_LISTEN     = 4;
STATE_DONE           = 5;

FB_RX_SAMPLES = 80000;
BUS_SLOT_SAMPLES = defs.slot_len_samples;
CONTROL_SAMPLES = 40000;

BEACON_PERIOD = 5;      % loops between beacons
START_COUNTDOWN_SLOTS = 3;
END_REPEAT = 5;

%% =========== Phase 1: INIT - File Processing ===========
state = STATE_INIT;
fprintf('[TX-INIT] Processing file...\n');

cfg = struct();
cfg.hop_seed = hop_seed;
cfg.fec_k = fec_k;
cfg.fec_r = fec_r;
if strcmp(transmit_mode, 'text')
    cfg.text_content = text_content;
    [business_bytes, meta_info] = build_file_container([], cfg);
else
    [business_bytes, meta_info] = build_file_container(file_name, cfg);
end

% Split into 40B source packets
payload_len = defs.payload_bytes_per_pkt;
total_src_packets = ceil(length(business_bytes) / payload_len);
src_packets = cell(1, total_src_packets);
for p = 1:total_src_packets
    st = (p-1)*payload_len + 1;
    ed = min(p*payload_len, length(business_bytes));
    pkt = zeros(payload_len, 1, 'uint8');
    pkt(1:ed-st+1) = business_bytes(st:ed);
    src_packets{p} = pkt;
end

fprintf('[TX-INIT] File: %d bytes -> %d source packets (40B each)\n', ...
    length(business_bytes), total_src_packets);

% FEC encoding
fec_groups = fec_rs_encode_groups(src_packets, fec_k, fec_r);
fprintf('[TX-INIT] FEC: %d groups, K=%d, R=%d\n', length(fec_groups), fec_k, fec_r);

% Build V2 frames
[frame_list, fec_info] = build_forward_frames_v2(src_packets, meta_info, fec_groups);

% Pre-modulate all frames
[~, tx_cache] = forward_frame_modulate_v2(frame_list, Anti_Jamming_Mode, fec_info);

% Build hop slots
slot_cache = build_hop_slot_waveform(tx_cache, fec_info);

total_slots = fec_info.total_slots;
session_id = fec_info.session_id;

fprintf('[TX-INIT] Ready: session=%d | slots=%d | carrier_set=%d freqs\n', ...
    session_id, total_slots, defs.num_carriers);

%% =========== SDR Initialization ===========
fprintf('[TX-HW] Initializing USRP...\n');

radio_tx = comm.SDRuTransmitter('Platform', 'X310', 'IPAddress', '192.168.10.2');
radio_tx.ChannelMapping = 1;
radio_tx.CenterFrequency = defs.anchor_freq;
radio_tx.Gain = Power_gain;
radio_tx.MasterClockRate = 200e6;
radio_tx.InterpolationFactor = 512;
radio_tx.ClockSource = 'External';

radio_rx = comm.SDRuReceiver( ...
    'Platform', 'X310', ...
    'IPAddress', '192.168.10.2', ...
    'OutputDataType', 'double', ...
    'MasterClockRate', 200e6, ...
    'DecimationFactor', 512, ...
    'SamplesPerFrame', FB_RX_SAMPLES);
radio_rx.ClockSource = 'External';
radio_rx.ChannelMapping = 1;
radio_rx.CenterFrequency = defs.feedback_freq;
radio_rx.Gain = 28;

cleanupObj = onCleanup(@() safe_release(radio_tx, radio_rx));
fprintf('[TX-HW] USRP ready.\n');

%% =========== UI Configuration ===========
tx_ui.enable = true;
tx_ui.url = 'http://127.0.0.1:5001';
tx_ui.health_endpoint = '/api/control';
tx_ui.post_period = 10;
tx_ui.ctrl_period = 20;
tx_ui.timeout = 0.03;

state = STATE_WAIT_READY;
beacon_count = 0;
countdown_remaining = 0;
slot_ptr = 1;
tx_duration = 0;

fprintf('[TX] Entering WAIT_READY state, sending BEACON on %.2f GHz\n', defs.anchor_freq/1e9);
fb_dbg_count = 0;

%% =========== Main Loop ===========
for idx = 1:100000
    tx_sig = zeros(BUS_SLOT_SAMPLES, 1);
    fb_sig = zeros(FB_RX_SAMPLES, 1);
    use_bus_slot = false;

    % ---- State Machine ----
    switch state
        case STATE_WAIT_READY
            % Periodic BEACON on anchor frequency
            if mod(beacon_count, BEACON_PERIOD) == 0
                tx_sig = build_control_frame(session_id, 40, meta_info, fec_info, CONTROL_SAMPLES);  % 40=BEACON
                radio_tx.CenterFrequency = defs.anchor_freq;
                if mod(beacon_count, 25) == 0
                    fprintf('[TX-BEACON] #%d | rms=%.6f | max=%.6f | freq=%.3f GHz\n', ...
                        beacon_count, rms(tx_sig), max(abs(tx_sig)), radio_tx.CenterFrequency/1e9);
                end
            else
                tx_sig = zeros(CONTROL_SAMPLES, 1);
            end
            beacon_count = beacon_count + 1;

        case STATE_START_COUNTDOWN
            % Send TX_START control frame with countdown info
            tx_sig = build_control_frame(session_id, 41, meta_info, fec_info, CONTROL_SAMPLES);  % 41=START
            radio_tx.CenterFrequency = defs.anchor_freq;
            countdown_remaining = countdown_remaining - 1;

            if countdown_remaining <= 0
                state = STATE_DATA_ONCE;
                slot_ptr = 1;
                fprintf('[TX] Countdown done, starting DATA_ONCE...\n');
            end

        case STATE_DATA_ONCE
            % Send one hop slot
            if slot_ptr <= total_slots
                slot = slot_cache(slot_ptr);
                tx_sig = slot.waveform;
                radio_tx.CenterFrequency = defs.Carrier_set(slot.carrier_index);
                use_bus_slot = true;
                slot_ptr = slot_ptr + 1;

                if mod(slot_ptr, 5) == 0 || slot_ptr > total_slots
                    fprintf('[TX-DATA] Slot %d/%d | Freq=%.1f GHz | frames=%d\n', ...
                        slot_ptr-1, total_slots, ...
                        defs.Carrier_set(slot.carrier_index)/1e9, slot.num_frames);
                end
            else
                tx_duration = toc(t0_data);
                fprintf('[TX-DATA] All slots sent, duration=%.2f s\n', tx_duration);
                state = STATE_END_LISTEN;
            end

        case STATE_END_LISTEN
            % Send TX_END and listen for RESULT
            tx_sig = build_control_frame(session_id, 42, meta_info, fec_info, CONTROL_SAMPLES);  % 42=END
            radio_tx.CenterFrequency = defs.anchor_freq;

        case STATE_DONE
            tx_sig = zeros(BUS_SLOT_SAMPLES, 1);
    end

    % ---- Transmit and Listen ----
    tx_sig = sqrt(Power) * 0.1 * tx_sig;

    try
        if use_bus_slot
            radio_tx(tx_sig);
        else
            % For control slots, pad to bus slot size
            pad_sig = zeros(BUS_SLOT_SAMPLES, 1);
            pad_sig(1:min(length(tx_sig), BUS_SLOT_SAMPLES)) = tx_sig(1:min(length(tx_sig), BUS_SLOT_SAMPLES));
            radio_tx(pad_sig);
        end
        [fb_sig, ~, rx_overrun] = radio_rx();
        if rx_overrun
            warning('[TX-WARN] Feedback overrun');
        end
    catch ME
        warning('[TX-ERR] HW error: %s', ME.message);
        continue;
    end

    % ---- Decode Feedback ----
    fb_dbg_count = fb_dbg_count + 1;
    if mod(fb_dbg_count, 50) == 0
        fprintf('[TX-FB-DEBUG] #%d | fb_rms=%.6f | fb_max=%.6f\n', ...
            fb_dbg_count, rms(fb_sig), max(abs(fb_sig)));
    end
    [fb_valid, fb_data] = feedback_frame_decode_v2(fb_sig);

    if fb_valid
        fprintf('[TX-FB] Got feedback: type=%d | session=%d | SNR=%.1f dB | state=%d\n', ...
            fb_data.frame_type, fb_data.session_id, fb_data.snr_db, fb_data.rx_state);

        % Handle RX_READY
        if fb_data.frame_type == defs.FRAME_TYPE_RX_READY && state == STATE_WAIT_READY
            fprintf('[TX] Received RX_READY, starting countdown...\n');
            state = STATE_START_COUNTDOWN;
            countdown_remaining = START_COUNTDOWN_SLOTS;
        end

        % Handle RX_RESULT
        if fb_data.frame_type == defs.FRAME_TYPE_RX_RESULT && state == STATE_END_LISTEN
            fprintf('[TX] Received RX_RESULT, transmission complete.\n');
            state = STATE_DONE;
        end
    end

    % ---- Periodic Status ----
    if mod(idx, 10) == 0
        state_names = {'INIT', 'WAIT_READY', 'START_COUNTDOWN', 'DATA_ONCE', 'END_LISTEN', 'DONE'};
        fprintf('[TX] idx=%d | state=%s | slot=%d/%d\n', ...
            idx, state_names{state+1}, min(slot_ptr, total_slots), total_slots);
    end

    % ---- Exit Conditions ----
    if state == STATE_DONE
        fprintf('[TX] Session complete, exiting.\n');
        break;
    end

    % State transitions
    if state == STATE_DATA_ONCE && slot_ptr == 1
        t0_data = tic;
    end
end

release(radio_rx);
release(radio_tx);
fprintf('[TX] Transmitter shutdown complete.\n');

%% =========== Helper Functions ===========
function sig = build_control_frame(session_id, frame_type, meta_info, fec_info, slot_len)
% Build a control frame (BEACON/START/END) using BPSK+spreading for robustness
defs = link_phy_defs();
sps = 4;

pcmatrix = ldpcQuasiCyclicMatrix(defs.blockSize, defs.P);
cfgLDPCEnc = ldpcEncoderConfig(pcmatrix);
crcgenerator = comm.CRCGenerator(defs.poly);

bpskmod = comm.PSKModulator(2, 'BitInput', true);
bpskmod.PhaseOffset = pi/4;

txfilter = comm.RaisedCosineTransmitFilter( ...
    'OutputSamplesPerSymbol', sps, 'RolloffFactor', 0.25);

% Control frame payload: FrameHead(8) + UserID(8) + FrameType(8) + ProtoVer(4)
% + SessionID(16) + HopSeed(32) + TotalSlots(16) + SlotLen(32) + CodewordsPerSlot(16)
% + Countdown(8) = ~148 bits + CRC32 + padding

hop_seed_bits = int_to_bits(meta_info.hop_seed, 32);
slot_len_bits = int_to_bits(meta_info.slot_len_samples, 32);
cw_per_slot_bits = int_to_bits(fec_info.codewords_per_slot, 16);

% Use actual values from fec_info
total_slots_actual = fec_info.total_slots;

ctrl_bits = [ ...
    defs.frame_head; ...
    defs.user_id; ...
    int_to_bits(frame_type, 8); ...
    int_to_bits(defs.proto_ver, 4); ...
    int_to_bits(4, 4); ...                    % control header len
    int_to_bits(session_id, 16); ...
    hop_seed_bits; ...
    int_to_bits(total_slots_actual, 16); ...
    slot_len_bits; ...
    cw_per_slot_bits; ...
    int_to_bits(meta_info.fec_k, 8); ...
    int_to_bits(meta_info.fec_r, 8)];

coded = [crcgenerator(ctrl_bits); defs.ctrl_frame_end];
scr_bits = scramble_bits_ctrl(coded, defs.scr_seq);
enc_bits = ldpcEncode(scr_bits, cfgLDPCEnc);
    fprintf('[CTRL-DBG] enc_bits: %dx%d | sum=%.0f\n', size(enc_bits,1), size(enc_bits,2), sum(enc_bits));
inter_matrix = reshape(enc_bits, 36, 18).';
inter_bits = inter_matrix(:);

inter_polar = 2 * inter_bits - 1;
spread = zeros(length(inter_polar)*15, 1);
for ii = 1:length(inter_polar)
    spread((ii-1)*15+1:ii*15) = inter_polar(ii) * defs.pn_data;
end

mod_sig = bpskmod(0.5*(spread+1));
    fprintf('[CTRL-DBG] mod_sig: %dx%d | rms=%.6f\n', size(mod_sig,1), size(mod_sig,2), rms(mod_sig));
tx_in = [defs.head_data; mod_sig; zeros(sps*10,1)];
one_wave = txfilter(tx_in);
    fprintf('[CTRL-DBG] one_wave: %dx%d | rms=%.6f | max=%.6f\n', size(one_wave,1), size(one_wave,2), rms(one_wave), max(abs(one_wave)));

% Pad/trim to slot length
sig = zeros(slot_len, 1);
L = min(length(one_wave), slot_len);
sig(1:L) = one_wave(1:L);
end

function bits = int_to_bits(v, width)
bits = double(dec2bin(max(0, v), width) == '1').';
end

function out = scramble_bits_ctrl(in, scr_seq)
out = zeros(size(in));
grp = length(scr_seq);
for ii = 1:floor(length(in)/grp)
    st = (ii-1)*grp + 1;
    ed = ii*grp;
    out(st:ed) = xor(in(st:ed), scr_seq);
end
end

function safe_release(tx, rx)
try
    if ~isempty(tx) && isvalid(tx), release(tx); end
catch
end
try
    if ~isempty(rx) && isvalid(rx), release(rx); end
catch
end
disp('SDR resources released.');
end
