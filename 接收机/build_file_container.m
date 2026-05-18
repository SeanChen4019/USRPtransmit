function [business_bytes, meta_info] = build_file_container(file_path, cfg)
% BUILD_FILE_CONTAINER Unified file encapsulation for text/image/video
%   (same as transmitter version; see that file for full documentation)
defs = link_phy_defs();
if nargin < 2, cfg = struct(); end
if ~isfield(cfg, 'demo_video_mode'), cfg.demo_video_mode = 0; end
if ~isfield(cfg, 'jpeg_quality'), cfg.jpeg_quality = 60; end
if ~isfield(cfg, 'video_frame_rate'), cfg.video_frame_rate = 8; end
if ~isfield(cfg, 'video_duration'), cfg.video_duration = 10; end
if ~isfield(cfg, 'hop_seed'), cfg.hop_seed = randi(65535); end
if ~isfield(cfg, 'fec_k'), cfg.fec_k = defs.fec_k_default; end
if ~isfield(cfg, 'fec_r'), cfg.fec_r = defs.fec_r_default; end

meta_info = struct();
if isempty(file_path) && isfield(cfg, 'text_content')
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
    if fid == -1, error('build_file_container: cannot open file: %s', file_path); end
    raw_bytes = fread(fid, inf, 'uint8=>uint8');
    fclose(fid);
end

name_bytes = uint8(unicode2native(name, 'UTF-8'));
ext_bytes = uint8(unicode2native(ext, 'UTF-8'));
if length(name_bytes) > 255, name_bytes = name_bytes(1:255); end
if length(ext_bytes) > 255, ext_bytes = ext_bytes(1:255); end

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

meta_bytes = pack_meta_tlv_local(meta_info, name_bytes, ext_bytes, defs);
business_bytes = [meta_bytes(:); raw_bytes(:)];
fprintf('[CONTAINER] file=%s | type=%d | size=%d | meta=%d | total=%d bytes\n', ...
    name, meta_info.file_type, meta_info.file_size, length(meta_bytes), length(business_bytes));
end

function meta_bytes = pack_meta_tlv_local(info, name_bytes, ext_bytes, defs)
payload_bytes_per_pkt = defs.payload_bytes_per_pkt;
file_only_bytes = info.file_size;
est_meta_size = 4+1+1+1+1+4+4+4+2+4+4+4+2+1+1+length(name_bytes)+length(ext_bytes);
total_business = est_meta_size + file_only_bytes;
source_packet_num = ceil(total_business / payload_bytes_per_pkt);
K = info.fec_k; R = info.fec_r;
num_groups = ceil(source_packet_num / K);
parity_packet_num = num_groups * R;

meta_parts = { ...
    defs.meta_magic(:); ...
    uint8(info.proto_ver); ...
    uint8(info.file_type); ...
    uint8(K); ...
    uint8(R); ...
    typecast(uint32(info.file_size), 'uint8')'; ...
    typecast(uint32(source_packet_num), 'uint8')'; ...
    typecast(uint32(parity_packet_num), 'uint8')'; ...
    typecast(uint16(num_groups), 'uint8')'; ...
    typecast(uint32(info.file_crc32), 'uint8')'; ...
    typecast(uint32(info.hop_seed), 'uint8')'; ...
    typecast(uint32(info.slot_len_samples), 'uint8')'; ...
    typecast(uint16(info.codewords_per_slot), 'uint8')'; ...
    uint8(length(name_bytes)); ...
    uint8(length(ext_bytes)); ...
    name_bytes(:); ...
    ext_bytes(:)};

meta_bytes = [];
for i = 1:length(meta_parts), meta_bytes = [meta_bytes; meta_parts{i}(:)]; end
end
