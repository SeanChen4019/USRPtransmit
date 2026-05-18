function crc_val = crc32_bytes(data_bytes)
% CRC32_BYTES Compute CRC32 over a uint8 byte array (file-level)
%   Uses the same polynomial as link_phy_defs:
%   z^32 + z^26 + z^23 + z^22 + z^16 + z^12 + z^11 + z^10 + z^8 + z^7 + z^5 + z^4 + z^2 + z + 1
%
%   Returns a uint32 value.

defs = link_phy_defs();

% Parse polynomial: 'z^32 + z^26 + z^23 + ...'
poly_str = defs.poly;
exponents = regexp(poly_str, 'z\^(\d+)', 'tokens');
poly_terms = zeros(1, length(exponents));
for i = 1:length(exponents)
    poly_terms(i) = str2double(exponents{i}{1});
end

% Build CRC32 polynomial value
poly = uint32(0);
for i = 1:length(poly_terms)
    poly = bitor(poly, bitshift(uint32(1), poly_terms(i)));
end

crc = uint32(hex2dec('FFFFFFFF'));  % initial value

for i = 1:length(data_bytes)
    crc = bitxor(crc, uint32(data_bytes(i)));
    for j = 1:8
        if bitand(crc, uint32(1))
            crc = bitxor(bitshift(crc, -1), poly);
        else
            crc = bitshift(crc, -1);
        end
    end
end

crc_val = bitxor(crc, uint32(hex2dec('FFFFFFFF')));  % final XOR
end
