function bit_array = pack_bits(fields)
% PACK_BITS Pack a cell array of {value, width} into a single bit column vector
%   bit_array = pack_bits({{v1,w1}, {v2,w2}, ...})
%   Each value is converted to width bits (MSB first), then concatenated.
bit_array = zeros(0, 1);
for i = 1:length(fields)
    v = fields{i}{1};
    w = fields{i}{2};
    bits = double(dec2bin(max(0, v), w) == '1').';
    bit_array = [bit_array; bits];
end
end
