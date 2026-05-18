function vals = unpack_bits(bit_array, widths)
% UNPACK_BITS Unpack a bit column vector into a struct of integer values
%   vals = unpack_bits(bit_array, {field_name1, width1; field_name2, width2; ...})
%   Returns a struct with fields named field_name1, field_name2, ...

if iscell(widths) && size(widths, 2) == 2
    % Cell array of {name, width} pairs
    vals = struct();
    offset = 0;
    for i = 1:size(widths, 1)
        name = widths{i, 1};
        w = widths{i, 2};
        bits = bit_array(offset + 1 : offset + w);
        vals.(name) = bits_to_int(bits);
        offset = offset + w;
    end
else
    error('widths must be an Nx2 cell array of {name, width}');
end
end

function v = bits_to_int(bits)
v = (2.^(length(bits)-1:-1:0)) * bits(:);
end
