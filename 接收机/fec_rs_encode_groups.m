function encoded_groups = fec_rs_encode_groups(src_packets, K, R)
% FEC_RS_ENCODE_GROUPS Reed-Solomon erasure coding over GF(256)
%   encoded_groups = fec_rs_encode_groups(src_packets, K, R)
%
%   src_packets: cell array of uint8 column vectors, each of length payload_len
%   K: number of source packets per group
%   R: number of parity packets per group
%
%   Returns encoded_groups: struct array with fields:
%       .group_id
%       .packets: Nxpayload_len uint8 matrix (rows 1..K are source, K+1..N are parity)
%       .K, .R, .N

num_src = length(src_packets);
if num_src == 0
    encoded_groups = struct('group_id', {}, 'packets', {}, 'K', {}, 'R', {}, 'N', {});
    return;
end

payload_len = length(src_packets{1});
N = K + R;

% Ensure all packets have same length, pad if needed
for i = 1:num_src
    if length(src_packets{i}) < payload_len
        src_packets{i}(end+1:payload_len) = 0;
    end
end

num_groups = ceil(num_src / K);
encoded_groups = struct('group_id', {}, 'packets', {}, 'K', {}, 'R', {}, 'N', {});

% GF(256) primitive polynomial: x^8 + x^4 + x^3 + x^2 + 1
[exp_table, log_table] = gf256_tables();

for g = 1:num_groups
    src_start = (g-1)*K + 1;
    src_end = min(g*K, num_src);
    actual_K = src_end - src_start + 1;

    % Build source matrix: K x payload_len
    group_src = zeros(K, payload_len, 'uint8');
    for k = 1:actual_K
        group_src(k, :) = src_packets{src_start + k - 1}(:).';
    end

    % Encode: N x payload_len
    encoded = zeros(N, payload_len, 'uint8');
    encoded(1:K, :) = group_src;

    % Generate Vandermonde parity for each column
    for col = 1:payload_len
        src_col = double(group_src(:, col));
        parity = gf256_rs_encode_col(src_col, K, R, exp_table, log_table);
        encoded(K+1:N, col) = parity;
    end

    encoded_groups(end+1).group_id = g; %#ok<AGROW>
    encoded_groups(end).packets = encoded;
    encoded_groups(end).K = K;
    encoded_groups(end).R = R;
    encoded_groups(end).N = N;
end
end

function parity = gf256_rs_encode_col(src_col, K, R, exp_table, log_table)
parity = zeros(R, 1, 'uint8');
for i = 1:R
    sum = 0;
    for j = 1:K
        if src_col(j) ~= 0
            power = mod((i-1) * (j-1), 255);
            prod = gf256_mul(src_col(j), exp_table(power + 1), exp_table, log_table);
            sum = bitxor(sum, prod);
        end
    end
    parity(i) = sum;
end
end

function c = gf256_mul(a, b, exp_table, log_table)
if a == 0 || b == 0
    c = 0;
else
    sum_log = mod(log_table(a + 1) + log_table(b + 1), 255);
    c = exp_table(sum_log + 1);
end
end

function [exp_table, log_table] = gf256_tables()
exp_table = zeros(1, 256, 'uint8');
log_table = zeros(1, 256, 'uint8');
x = 1;
for i = 0:254
    exp_table(i + 1) = x;
    log_table(x + 1) = i;
    x = bitshift(x, 1);
    if bitand(x, 256)
        x = bitxor(x, 285);
    end
    x = bitand(x, 255);
end
exp_table(256) = exp_table(1);
end
