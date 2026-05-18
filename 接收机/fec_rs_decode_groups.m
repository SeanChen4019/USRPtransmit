function recovered = fec_rs_decode_groups(encoded_group, received_map, payload_len)
% FEC_RS_DECODE_GROUPS Reed-Solomon erasure decoding over GF(256)
%   recovered = fec_rs_decode_groups(encoded_group, received_map, payload_len)
%
%   encoded_group: struct with fields .packets (N x payload_len uint8), .K, .R, .N
%   received_map: 1 x N logical vector, true = packet received (CRC ok)
%   payload_len: expected payload length per packet
%
%   Returns recovered: K x payload_len uint8 matrix of recovered source packets,
%                      or empty if not enough packets received

K = encoded_group.K;
N = encoded_group.N;
num_received = sum(received_map);

if num_received < K
    recovered = [];
    return;  % Not enough packets to recover
end

% Build GF(256) tables
[exp_table, log_table] = gf256_tables_local();

% Build the decoding matrix
% For systematic encoding: first K rows are identity (source packets)
% The parity rows use Vandermonde: P(i,j) = j^(i-1), i=1..R, j=1..K

% We need to solve for the K source columns from the K received rows
% received_idx: which rows we have (1-based)
received_idx = find(received_map);
received_idx = received_idx(1:K);  % Take first K received
received_rows = encoded_group.packets(received_idx, :);

% Build the K x K submatrix of the generator matrix
% Generator G: first K rows = I_K, next R rows = Vandermonde
% We need G_received (K x K), then source = G_received^(-1) * received
G_sub = zeros(K, K, 'uint8');
for i = 1:K
    row_idx = received_idx(i);
    if row_idx <= K
        % Identity row
        G_sub(i, row_idx) = 1;
    else
        % Parity row: row p = row_idx - K
        p = row_idx - K;
        for j = 1:K
            power = mod(p * (j-1), 255);
            G_sub(i, j) = exp_table(power + 1);
        end
    end
end

% Solve: for each column, solve G_sub * x = received_col over GF(256)
recovered = zeros(K, payload_len, 'uint8');
for col = 1:payload_len
    b = double(received_rows(:, col));
    x = gf256_solve(G_sub, b, exp_table, log_table);
    if ~isempty(x)
        recovered(:, col) = x;
    else
        recovered = [];
        return;
    end
end
end

function x = gf256_solve(A, b, exp_table, log_table)
% Solve A*x = b over GF(256) using Gaussian elimination
% A: K x K uint8, b: K x 1 double
% Returns x: K x 1 uint8, or empty on failure

K = size(A, 1);
Aug = [double(A), b(:)];

for col = 1:K
    % Find pivot
    pivot = 0;
    pivot_row = -1;
    for row = col:K
        if Aug(row, col) ~= 0
            pivot = Aug(row, col);
            pivot_row = row;
            break;
        end
    end
    if pivot_row == -1
        x = [];
        return;  % Singular matrix
    end

    % Swap rows
    if pivot_row ~= col
        temp = Aug(col, :);
        Aug(col, :) = Aug(pivot_row, :);
        Aug(pivot_row, :) = temp;
        pivot = Aug(col, col);
    end

    % Normalize pivot row
    inv_pivot = gf256_inv_local(pivot, exp_table, log_table);
    for j = col:K+1
        if Aug(col, j) ~= 0
            Aug(col, j) = gf256_mul_local(Aug(col, j), inv_pivot, exp_table, log_table);
        end
    end

    % Eliminate other rows
    for row = 1:K
        if row ~= col && Aug(row, col) ~= 0
            factor = Aug(row, col);
            for j = col:K+1
                if Aug(col, j) ~= 0
                    prod = gf256_mul_local(factor, Aug(col, j), exp_table, log_table);
                    Aug(row, j) = bitxor(Aug(row, j), prod);
                end
            end
        end
    end
end

x = uint8(Aug(:, K+1));
end

function c = gf256_mul_local(a, b, exp_table, log_table)
if a == 0 || b == 0
    c = 0;
else
    sum_log = mod(log_table(a + 1) + log_table(b + 1), 255);
    c = exp_table(sum_log + 1);
end
end

function inv = gf256_inv_local(a, exp_table, log_table)
if a == 0
    inv = 0;
else
    inv = exp_table(mod(255 - log_table(a + 1), 255) + 1);
end
end

function [exp_table, log_table] = gf256_tables_local()
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
