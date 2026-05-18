function [business_bytes, meta_info] = build_file_container(file_path, cfg)
% BUILD_FILE_CONTAINER Unified file encapsulation for text/image/video
%   [business_bytes, meta_info] = build_file_container(file_path, cfg)
%
%   file_path: path to input file, or [] for text mode (uses cfg.text)
%   cfg: configuration struct with optional fields:
%       .demo_video_mode (0=raw file, 1=JPEG frame container)
%       .jpeg_quality (45-65 default)
%       .video_frame_rate (fps, default 8)
%       .video_duration (seconds, default 10)
%       .text_content (string for text mode)
%       .hop_seed (random seed for hopping)
%       .fec_k, .fec_r
%
%   Returns:
%       business_bytes: uint8 column vector, the complete file payload
%       meta_info: struct with all META fields

defs = link_phy_defs();

if nargin < 2
    cfg = struct();
end

% Default config
if ~isfield(cfg, 'demo_video_mode'), cfg.demo_video_mode = 0; end
if ~isfield(cfg, 'jpeg_quality'), cfg.jpeg_quality = 60; end
if ~isfield(cfg, 'video_frame_rate'), cfg.video_frame_rate = 8; end
if ~isfield(cfg, 'video_duration'), cfg.video_duration = 10; end
if ~isfield(cfg, 'hop_seed'), cfg.hop_seed = randi(65535); end
if ~isfield(cfg, 'fec_k'), cfg.fec_k = defs.fec_k_default; end
if ~isfield(cfg, 'fec_r'), cfg.fec_r = defs.fec_r_default; end

meta_info = struct();

% Determine file type and read data
if isempty(file_path) && isfield(cfg, 'text_content')
    % Text mode
    meta_info.file_type = defs.FILE_TYPE_TEXT;
    raw_bytes = uint8(unicode2native(cfg.text_content, 'UTF-8'));
    [~, name, ext] = fileparts('message.txt');
    ext = '.txt';
else
    [~, name, ext] = fileparts(file_path);
    ext = lower(ext);

    if ismember(ext, {'.jpg', '.jpeg', '.png', '.bmp'})
        meta_info.file_type = defs.FILE_TYPE_IMAGE;
    elseif ismember(ext, {'.mp4', '.avi', '.mov', '.mkv'})
        meta_info.file_type = defs.FILE_TYPE_VIDEO;
    else
        meta_info.file_type = defs.FILE_TYPE_BINARY;
    end

    fid = fopen(file_path, 'rb');
    if fid == -1
        error('build_file_container: cannot open file: %s', file_path);
    end
    raw_bytes = fread(fid, inf, 'uint8=>uint8');
    fclose(fid);
end

% Build name and extension as UTF-8 bytes
name_bytes = uint8(unicode2native(name, 'UTF-8'));
ext_bytes = uint8(unicode2native(ext, 'UTF-8'));

% Truncate name if too long (max 255 bytes)
max_name_len = 255;
if length(name_bytes) > max_name_len
    name_bytes = name_bytes(1:max_name_len);
end
if length(ext_bytes) > 255
    ext_bytes = ext_bytes(1:255);
end

% Compute file-level CRC32
meta_info.file_crc32 = crc32_bytes(raw_bytes);
meta_info.file_size = length(raw_bytes);
meta_info.name = name;
meta_info.ext = ext;
meta_info.fec_k = cfg.fec_k;
meta_info.fec_r = cfg.fec_r;
meta_info.hop_seed = cfg.hop_seed;
meta_info.proto_ver = defs.proto_ver;
meta_info.slot_len_samples = defs.slot_len_samples;
meta_info.codewords_per_slot = defs.codewords_per_slot_default;
meta_info.payload_bytes_per_pkt = defs.payload_bytes_per_pkt;

% ---- Pack META TLV into binary ----
meta_bytes = pack_meta_tlv(meta_info, name_bytes, ext_bytes, defs);

% ---- Concatenate META + file data ----
business_bytes = [meta_bytes(:); raw_bytes(:)];

fprintf('[CONTAINER] file=%s | type=%d | size=%d | meta=%d | total=%d bytes\n', ...
    name, meta_info.file_type, meta_info.file_size, length(meta_bytes), length(business_bytes));
end

function meta_bytes = pack_meta_tlv(info, name_bytes, ext_bytes, defs)
% Pack META info into TLV binary format
% Layout:
%   magic(4B) + proto_ver(1B) + file_type(1B) + fec_k(1B) + fec_r(1B)
%   + file_size(4B) + source_packet_num(4B) + parity_packet_num(4B)
%   + total_group_num(2B) + file_crc32(4B) + hop_seed(4B)
%   + slot_len_samples(4B) + codewords_per_slot(2B)
%   + name_len(1B) + ext_len(1B) + name(...) + ext(...)

% Compute packet counts
payload_bytes_per_pkt = defs.payload_bytes_per_pkt;
file_only_bytes = info.file_size;
total_business = 0;  % Will be computed

% Estimate META size before packing
est_meta_size = 4 + 1 + 1 + 1 + 1 + 4 + 4 + 4 + 2 + 4 + 4 + 4 + 2 + 1 + 1 + length(name_bytes) + length(ext_bytes);
total_business = est_meta_size + file_only_bytes;

source_packet_num = ceil(total_business / payload_bytes_per_pkt);
K = info.fec_k;
R = info.fec_r;
N = K + R;
num_groups = ceil(source_packet_num / K);
parity_packet_num = num_groups * R;

meta_parts = { ...
    defs.meta_magic(:); ...                          % magic 4B
    uint8(info.proto_ver); ...                       % proto_ver 1B
    uint8(info.file_type); ...                       % file_type 1B
    uint8(K); ...                                    % fec_k 1B
    uint8(R); ...                                    % fec_r 1B
    typecast(uint32(info.file_size), 'uint8')'; ... % file_size 4B
    typecast(uint32(source_packet_num), 'uint8')'; ... % source_packet_num 4B
    typecast(uint32(parity_packet_num), 'uint8')'; ... % parity_packet_num 4B
    typecast(uint16(num_groups), 'uint8')'; ...      % total_group_num 2B
    typecast(uint32(info.file_crc32), 'uint8')'; ...% file_crc32 4B
    typecast(uint32(info.hop_seed), 'uint8')'; ...   % hop_seed 4B
    typecast(uint32(info.slot_len_samples), 'uint8')'; ... % slot_len_samples 4B
    typecast(uint16(info.codewords_per_slot), 'uint8')'; ... % codewords_per_slot 2B
    uint8(length(name_bytes)); ...                   % name_len 1B
    uint8(length(ext_bytes)); ...                    % ext_len 1B
    name_bytes(:); ...                               % name
    ext_bytes(:); ...                                % ext
    };

meta_bytes = [];
for i = 1:length(meta_parts)
    meta_bytes = [meta_bytes; meta_parts{i}(:)];
end
end
