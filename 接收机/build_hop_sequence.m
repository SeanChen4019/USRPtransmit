function hop_seq = build_hop_sequence(hop_seed, total_slots, num_carriers)
% BUILD_HOP_SEQUENCE Generate pseudo-random frequency hopping sequence
%   hop_seq = build_hop_sequence(hop_seed, total_slots, num_carriers)
%   Returns a total_slots x 1 vector of carrier indices (1..num_carriers).
%   Ensures no two consecutive slots use the same frequency.

rng(hop_seed, 'twister');
hop_seq = randi(num_carriers, total_slots, 1);

% Constraint: no two consecutive slots on same frequency
for i = 2:total_slots
    if hop_seq(i) == hop_seq(i-1)
        hop_seq(i) = mod(hop_seq(i), num_carriers) + 1;
    end
end
end
