function T = diagnose_address5(seeds)
%DIAGNOSE_ADDRESS5  Why the circling rover is the hardest address.
%
%   Address 5 is the only pad that TURNS. The tracker has to estimate its
%   turn rate from a velocity and an acceleration that are themselves
%   differentiated from centimetre-noisy vision fixes, and that estimate
%   sometimes drops to zero. This prints, for each seed, what the tracker
%   believed the turn rate was at the instant the drone committed to the
%   touchdown, beside the miss that followed. The true value is 0.60 rad/s.
if nargin < 1, seeds = 0:11; end
C = droneconfig(); p = C.PADS(5);
w = zeros(numel(seeds),1); miss = w;
fprintf('%s  seed   turn rate at commit   miss%s', newline, newline);
for i = 1:numel(seeds)
    r = simulate_landing(5, seeds(i));
    w(i) = r.turnRateAtCommit; miss(i) = 100*r.error_m;
    fprintf('   %2d        %+5.2f rad/s      %5.1f cm%s', seeds(i), w(i), miss(i), newline);
end
live = abs(w) > 0.5*p.omega;
fprintf('%s  true turn rate            %.2f rad/s%s', newline, p.omega, newline);
fprintf('  estimate alive at commit  %d of %d   mean miss %5.1f cm%s', sum(live), numel(w), mean(miss(live)), newline);
fprintf('  estimate dropped out      %d of %d   mean miss %5.1f cm%s', sum(~live), numel(w), mean(miss(~live)), newline);
T = table(seeds(:), w, miss, 'VariableNames', {'seed','turnRateAtCommit','miss_cm'});
writetable(T, 'address5_diagnostic.csv');
end
