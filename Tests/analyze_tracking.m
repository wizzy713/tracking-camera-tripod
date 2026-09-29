%% analyze_tracking.m -- TripodTracker run analysis
%
% Analyses every TrackingLog_*.csv in this folder (the logs exported from the
% app's Experiment tab) and, for each run, plots what the tracking actually did
% against what we want it to do (the TARGETS below). Then compares all runs
% side by side, so you can change one setting, log a new run, re-run this
% script and see whether the change helped.
%
% Each log already records its own settings (prediction horizon, Kalman R and
% sigma_a, KP/KI/KD, max speed, deadzone) in its "# key: value" header, so you
% don't have to note them by hand -- but put WHAT YOU CHANGED in the app's
% "Test notes" field, it shows up in the summary table.
%
% Outputs (in ./results):
%   <log name>.png       per-run dashboard
%   comparison.png       all runs vs targets
%   runs_summary.csv     one row per run: settings + metrics + pass/fail
%
% Terms used below:
%   ball offset  = where the ball actually is in the frame, as a fraction of
%                  half the frame (-1 = left/top edge, 0 = centred, +1 = right/
%                  bottom edge). This is the real measure of tracking quality:
%                  a perfect tripod keeps it at 0.
%   sent error   = the ErrX/ErrY actually sent to the tripod. It is computed
%                  from the Kalman PREDICTION (position + velocity * horizon),
%                  not the raw position, so it can differ from the ball offset.

clear; close all;

%% ---- Config -------------------------------------------------------------
logDir = fileparts(mfilename('fullpath'));
if isempty(logDir), logDir = pwd; end
outDir = fullfile(logDir, 'results');
defaultFrameSize = [480 640];   % [W H] fallback if it can't be inferred from the log

% Targets: what a good run looks like. Edit these to tighten/relax the goal.
T.offsetRms      = 0.15;   % RMS ball offset (X), fraction of half-frame   (<=)
T.band           = 0.20;   % "on target" band: |ball offset| within this ...
T.inBandPct      = 90;     % ... for at least this % of frames              (>=)
T.detectPct      = 95;     % frames with a detection                        (>=)
T.fps            = 28;     % mean analysis frame rate                       (>=)
T.saturatedPct   = 2;      % frames with |sent error| >= 0.99               (<=)
T.wrongSignPct   = 5;      % sent error points the opposite way to the ball (<=)
T.multiDetPct    = 1;      % frames with more than one detection            (<=)
T.frameMs        = 1000/30;% ideal frame interval (30 fps camera)

% Metric table: field, label, target, direction ('<=' or '>='), display format
M = {
    'offsetRmsX',   'Ball offset RMS (X)',       T.offsetRms,    '<=', '%.3f'
    'inBandPctX',   sprintf('Time within ±%.2f (X)', T.band), T.inBandPct, '>=', '%.1f%%'
    'detectPct',    'Detection rate',            T.detectPct,    '>=', '%.1f%%'
    'fps',          'Frame rate',                T.fps,          '>=', '%.1f fps'
    'saturatedPct', 'Saturated commands (X)',    T.saturatedPct, '<=', '%.1f%%'
    'wrongSignPct', 'Wrong-direction commands',  T.wrongSignPct, '<=', '%.1f%%'
    'multiDetPct',  'Multi-detection frames',    T.multiDetPct,  '<=', '%.1f%%'
};

%% ---- Load + analyse every run ---------------------------------------------
files = dir(fullfile(logDir, 'TrackingLog_*.csv'));
if isempty(files)
    error('No TrackingLog_*.csv files found in %s', logDir);
end
[~, order] = sort({files.name});   % filenames carry the date, so this is chronological
files = files(order);
if ~exist(outDir, 'dir'), mkdir(outDir); end

runs = struct([]);
for k = 1:numel(files)
    f = fullfile(files(k).folder, files(k).name);
    try
        [data, meta] = readTrackingLog(f);
    catch err
        warning('Skipping %s: %s', files(k).name, err.message);
        continue
    end
    r = analyseRun(data, meta, defaultFrameSize, T);
    r.file = files(k).name;
    r.label = sprintf('%d: %s', numel(runs) + 1, r.meta.test_name);
    r = scoreRun(r, M);
    runs = [runs, r]; %#ok<AGROW>

    fig = plotRun(r, T, M);
    [~, base] = fileparts(files(k).name);
    exportgraphics(fig, fullfile(outDir, [base '.png']), 'Resolution', 130);
    printRun(r, M);
end

%% ---- Cross-run comparison -------------------------------------------------
if ~isempty(runs)
    fig = plotComparison(runs, M);
    exportgraphics(fig, fullfile(outDir, 'comparison.png'), 'Resolution', 130);
    summary = buildSummary(runs, M);
    writetable(summary, fullfile(outDir, 'runs_summary.csv'));
    fprintf('\n==== Summary (%d runs) -> %s\n', numel(runs), fullfile(outDir, 'runs_summary.csv'));
    disp(summary(:, {'run', 'test_name', 'horizon_s', 'R', 'sigma_a', 'KP', 'KI', 'KD', 'MS', ...
        'offsetRmsX', 'inBandPctX', 'fps', 'wrongSignPct', 'passed'}));
end

%% ==========================================================================
%  Local functions
%% ==========================================================================

function [d, meta] = readTrackingLog(file)
    lines = splitlines(string(fileread(file)));
    isMeta = startsWith(lines, '#');

    meta = struct('test_name', "", 'test_notes', "", 'detection_mode', "", ...
        'prediction_horizon_s', NaN, 'kalman_measurement_noise', NaN, ...
        'kalman_acceleration_noise', NaN, 'tripod_kp', NaN, 'tripod_ki', NaN, ...
        'tripod_kd', NaN, 'tripod_max_speed_offset_us', NaN, 'tripod_deadzone', NaN);
    for ln = lines(isMeta)'
        kv = regexp(char(ln), '^#\s*([^:]+):\s*(.*)$', 'tokens', 'once');
        if isempty(kv), continue; end
        key = matlab.lang.makeValidName(strtrim(kv{1}));
        val = strtrim(string(kv{2}));
        num = str2double(val);
        if isfield(meta, key) && isnumeric(meta.(key)) || (~isfield(meta, key) && ~isnan(num))
            meta.(key) = num;
        else
            meta.(key) = val;
        end
    end
    if strlength(meta.test_name) == 0
        [~, meta.test_name] = fileparts(file);
        meta.test_name = string(meta.test_name);
    end

    hdrIdx = find(~isMeta & strlength(strtrim(lines)) > 0, 1);
    names = strtrim(split(lines(hdrIdx), ','))';
    body = lines(hdrIdx + 1:end);
    body = body(strlength(strtrim(body)) > 0);
    % First column is a wall-clock timestamp string (skipped); the rest are numeric.
    C = textscan(char(strjoin(body, newline)), ['%*s' repmat('%f', 1, numel(names) - 1)], ...
        'Delimiter', ',', 'Whitespace', '', 'EndOfLine', '\n');
    d = struct();
    for i = 2:numel(names)
        d.(char(names(i))) = C{i - 1};
    end
    need = {'FrameTimestampNanos', 'DetectionCount', 'RawX', 'RawY', 'FilteredX', ...
        'FilteredY', 'VelocityX', 'VelocityY', 'ErrX', 'ErrY'};
    missing = need(~isfield(d, need));
    if ~isempty(missing)
        error('missing columns: %s', strjoin(missing, ', '));
    end
end

function r = analyseRun(d, meta, defaultFrameSize, T)
    r.meta = meta;
    h = meta.prediction_horizon_s;
    if isnan(h), h = 0; end
    r.h = h;
    dz = meta.tripod_deadzone;
    if isnan(dz), dz = 0.03; end
    r.dz = dz;

    t = (d.FrameTimestampNanos - d.FrameTimestampNanos(1)) / 1e9;
    r.t = t;
    r.d = d;
    r.duration = t(end);
    r.n = numel(t);

    % Frame size: ErrX = (predX - W/2) / (W/2), so W = 2*predX / (ErrX + 1).
    % Solve it from the log itself, ignoring clamped (|err| ~ 1) rows.
    r.W = inferSize(d.FilteredX + d.VelocityX * h, d.ErrX, defaultFrameSize(1));
    r.H = inferSize(d.FilteredY + d.VelocityY * h, d.ErrY, defaultFrameSize(2));

    ok = ~isnan(d.RawX) & ~isnan(d.RawY);
    r.ok = ok;
    r.offX = (d.RawX - r.W / 2) / (r.W / 2);
    r.offY = (d.RawY - r.H / 2) / (r.H / 2);
    r.predX = d.FilteredX + d.VelocityX * h;

    dt = diff(t);
    r.dtMs = dt * 1000;
    r.fps = 1 / mean(dt);
    r.slowPct = 100 * mean(dt > 0.050);
    r.detectPct = 100 * mean(ok);
    r.multiDetPct = 100 * mean(d.DetectionCount > 1);

    r.offsetRmsX = sqrt(mean(r.offX(ok) .^ 2));
    r.offsetRmsY = sqrt(mean(r.offY(ok) .^ 2));
    r.inBandPctX = 100 * mean(abs(r.offX(ok)) <= T.band);
    r.inBandPctY = 100 * mean(abs(r.offY(ok)) <= T.band);
    r.sentRmsX = sqrt(mean(d.ErrX .^ 2));
    r.sentRmsY = sqrt(mean(d.ErrY .^ 2));
    r.saturatedPct = 100 * mean(abs(d.ErrX) >= 0.99);
    % Wrong direction: ball clearly on one side, command clearly the other way.
    m = ok & abs(r.offX) > dz & abs(d.ErrX) > dz;
    r.wrongSignPct = 100 * mean(sign(r.offX(m)) ~= sign(d.ErrX(m)));

    % Offline horizon sweep: how well filtered + velocity*h predicts where the
    % ball is actually seen h seconds later. Camera-frame positions, so camera
    % motion during the run skews it -- read it as indicative, not exact.
    r.hs = 0:0.025:0.5;
    r.hsRms = nan(size(r.hs));
    [tu, iu] = unique(t(ok));
    rx = d.RawX(ok);
    rx = rx(iu);
    for i = 1:numel(r.hs)
        fut = interp1(tu, rx, t + r.hs(i), 'linear', NaN);
        e = d.FilteredX + d.VelocityX * r.hs(i) - fut;
        e = e(ok & ~isnan(e));
        if ~isempty(e), r.hsRms(i) = sqrt(mean(e .^ 2)); end
    end
    [~, ib] = min(r.hsRms);
    r.bestHorizon = r.hs(ib);
end

function s = inferSize(pred, err, fallback)
    m = abs(err) < 0.95 & isfinite(pred) & abs(err + 1) > 1e-3;
    s = median(2 * pred(m) ./ (err(m) + 1));
    if isempty(s) || ~isfinite(s) || s <= 0
        s = fallback;
    else
        s = 2 * round(s / 2);
    end
end

function r = scoreRun(r, M)
    r.pass = false(size(M, 1), 1);
    for i = 1:size(M, 1)
        v = r.(M{i, 1});
        if strcmp(M{i, 4}, '<='), r.pass(i) = v <= M{i, 3}; else, r.pass(i) = v >= M{i, 3}; end
    end
end

function c = colours()
    c.target = [0.20 0.65 0.30];   % green: what we want
    c.actual = [0.00 0.35 0.75];   % blue: what the ball actually did
    c.sent   = [0.90 0.45 0.10];   % orange: what was sent to the tripod
    c.miss   = [0.85 0.15 0.15];   % red: missed detections / fails
    c.grey   = [0.55 0.55 0.55];
end

function fig = plotRun(r, T, M)
    c = colours();
    d = r.d;
    fig = figure('Name', r.file, 'Color', 'w', 'Position', [50 50 1500 950]);
    lightTheme(fig);
    tl = tiledlayout(fig, 3, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
    title(tl, sprintf('%s  (%s)', r.meta.test_name, r.file), 'Interpreter', 'none', 'FontWeight', 'bold');
    subtitle(tl, settingsLine(r), 'Interpreter', 'none');

    % --- X and Y offset vs time, against the target band --------------------
    axisPlot(nexttile(tl, [1 2]), r, r.offX, d.ErrX, 'Pan (X)', T, c);
    axisPlot(nexttile(tl, 4, [1 2]), r, r.offY, d.ErrY, 'Tilt (Y)', T, c);

    % --- CDF of |ball offset|: actual vs target -------------------------------
    ax = nexttile(tl, 3);
    hold(ax, 'on'); grid(ax, 'on'); box(ax, 'on');
    x = sort(abs(r.offX(r.ok)));
    y = 100 * (1:numel(x))' / numel(x);
    % Target: the curve must pass through the shaded box (>= inBandPct % of
    % frames within the band). The dashed curve is what an on-target run looks
    % like: Gaussian-distributed offset just meeting that requirement.
    fill(ax, [T.band 1 1 T.band], [T.inBandPct T.inBandPct 100 100], c.target, ...
        'FaceAlpha', 0.15, 'EdgeColor', 'none', 'DisplayName', ...
        sprintf('target: >= %d%% within ±%.2f', T.inBandPct, T.band));
    sig = T.band / (sqrt(2) * erfinv(T.inBandPct / 100));
    xi = linspace(0, 1, 200);
    plot(ax, xi, 100 * erf(xi / (sig * sqrt(2))), '--', 'Color', c.target, 'LineWidth', 1.5, ...
        'DisplayName', 'ideal run');
    plot(ax, T.band, T.inBandPct, 'o', 'Color', c.target, 'MarkerFaceColor', c.target, 'HandleVisibility', 'off');
    plot(ax, x, y, 'Color', c.actual, 'LineWidth', 2, 'DisplayName', 'actual (X)');
    xline(ax, T.band, ':', 'Color', c.grey, 'HandleVisibility', 'off');
    xlim(ax, [0 1]); ylim(ax, [0 100]);
    xlabel(ax, '|ball offset X| (fraction of half-frame)');
    ylabel(ax, '% of frames at or below');
    title(ax, 'How centred the ball stayed');
    legend(ax, 'Location', 'southeast');

    % --- Horizon sweep ----------------------------------------------------------
    ax = nexttile(tl, 6);
    hold(ax, 'on'); grid(ax, 'on'); box(ax, 'on');
    plot(ax, r.hs, r.hsRms, '-o', 'Color', c.actual, 'MarkerSize', 3, 'LineWidth', 1.5, ...
        'DisplayName', 'prediction error');
    xline(ax, r.h, '-', sprintf('used %.2fs', r.h), 'Color', c.sent, 'LineWidth', 1.5, ...
        'LabelVerticalAlignment', 'bottom', 'HandleVisibility', 'off');
    xline(ax, r.bestHorizon, '--', sprintf('best %.2fs', r.bestHorizon), 'Color', c.target, ...
        'LineWidth', 1.5, 'LabelVerticalAlignment', 'top', 'HandleVisibility', 'off');
    xlabel(ax, 'prediction horizon (s)'); ylabel(ax, 'RMS error vs later position (px)');
    title(ax, 'Prediction horizon sweep (X, indicative)');

    % --- Pixel trajectory: raw vs filtered vs predicted -------------------------
    ax = nexttile(tl, 7);
    hold(ax, 'on'); grid(ax, 'on'); box(ax, 'on');
    yline(ax, r.W / 2, '-', 'Color', c.target, 'LineWidth', 2, 'DisplayName', 'target (centre)');
    plot(ax, r.t, d.RawX, '.', 'Color', c.actual, 'MarkerSize', 6, 'DisplayName', 'raw detection');
    plot(ax, r.t, d.FilteredX, '-', 'Color', c.grey, 'LineWidth', 1, 'DisplayName', 'Kalman filtered');
    plot(ax, r.t, r.predX, '-', 'Color', c.sent, 'LineWidth', 1, 'DisplayName', sprintf('predicted +%.2fs', r.h));
    yline(ax, [0 r.W], ':', 'Color', c.grey, 'HandleVisibility', 'off');
    ylim(ax, [-0.25 1.25] * r.W);
    xlabel(ax, 'time (s)'); ylabel(ax, 'X (px)');
    title(ax, 'Ball X in frame (pixels)');
    legend(ax, 'Location', 'best', 'FontSize', 7);

    % --- Frame interval -----------------------------------------------------------
    ax = nexttile(tl, 8);
    hold(ax, 'on'); grid(ax, 'on'); box(ax, 'on');
    plot(ax, r.t(2:end), r.dtMs, '-', 'Color', c.actual, 'DisplayName', 'actual');
    yline(ax, T.frameMs, '-', 'Color', c.target, 'LineWidth', 2, 'DisplayName', sprintf('target %.0f ms (30 fps)', T.frameMs));
    miss = ~r.ok(2:end);
    plot(ax, r.t([false; miss]), r.dtMs(miss), 'x', 'Color', c.miss, 'DisplayName', 'no detection');
    ylim(ax, [0 max(250, min(max(r.dtMs) * 1.05, 800))]);
    xlabel(ax, 'time (s)'); ylabel(ax, 'frame interval (ms)');
    title(ax, sprintf('Frame timing: %.1f fps mean, %.1f%% frames > 50 ms', r.fps, r.slowPct));
    legend(ax, 'Location', 'northeast', 'FontSize', 7);

    % --- Scorecard ------------------------------------------------------------
    ax = nexttile(tl, 9);
    axis(ax, 'off'); xlim(ax, [0 1]); ylim(ax, [0 size(M, 1) + 1.5]);
    title(ax, sprintf('Scorecard: %d / %d targets met', sum(r.pass), numel(r.pass)));
    text(ax, 0.00, size(M, 1) + 1, 'metric', 'FontWeight', 'bold');
    text(ax, 0.55, size(M, 1) + 1, 'actual', 'FontWeight', 'bold');
    text(ax, 0.78, size(M, 1) + 1, 'target', 'FontWeight', 'bold');
    for i = 1:size(M, 1)
        yy = size(M, 1) + 1 - i;
        col = c.target; if ~r.pass(i), col = c.miss; end
        text(ax, 0.00, yy, M{i, 2}, 'FontSize', 9);
        text(ax, 0.55, yy, sprintf(M{i, 5}, r.(M{i, 1})), 'Color', col, 'FontWeight', 'bold', 'FontSize', 9);
        text(ax, 0.78, yy, [M{i, 4} ' ' sprintf(M{i, 5}, M{i, 3})], 'FontSize', 9, 'Color', c.grey);
    end
end

function axisPlot(ax, r, off, sent, name, T, c)
    hold(ax, 'on'); grid(ax, 'on'); box(ax, 'on');
    tt = r.t;
    fill(ax, [tt(1) tt(end) tt(end) tt(1)], [-T.band -T.band T.band T.band], c.target, ...
        'FaceAlpha', 0.15, 'EdgeColor', 'none', 'DisplayName', sprintf('target band ±%.2f', T.band));
    yline(ax, 0, '-', 'Color', c.target, 'LineWidth', 2, 'DisplayName', 'ideal (centred)');
    yline(ax, [-r.dz r.dz], ':', 'Color', c.target, 'HandleVisibility', 'off');
    plot(ax, tt, sent, '-', 'Color', [c.sent 0.8], 'LineWidth', 1, 'DisplayName', 'sent to tripod');
    plot(ax, tt, off, '-', 'Color', c.actual, 'LineWidth', 1.5, 'DisplayName', 'actual ball offset');
    miss = ~r.ok;
    if any(miss)
        plot(ax, tt(miss), -1.08 * ones(nnz(miss), 1), '|', 'Color', c.miss, 'MarkerSize', 8, ...
            'DisplayName', 'no detection');
    end
    ylim(ax, [-1.15 1.15]); xlim(ax, [tt(1) tt(end)]);
    xlabel(ax, 'time (s)'); ylabel(ax, 'offset (fraction of half-frame)');
    if name(1) == 'P'
        rms = r.offsetRmsX; inb = r.inBandPctX; srms = r.sentRmsX;
    else
        rms = r.offsetRmsY; inb = r.inBandPctY; srms = r.sentRmsY;
    end
    title(ax, sprintf('%s: ball offset RMS %.3f (target <= %.2f), %.0f%% in band, sent RMS %.3f', ...
        name, rms, T.offsetRms, inb, srms));
    legend(ax, 'Location', 'eastoutside', 'FontSize', 8);
end

function lightTheme(fig)
    % MATLAB R2025a+ follows the desktop's dark theme; force light so the
    % exported PNGs are readable. Older releases have no theme and are light.
    try
        theme(fig, 'light');
    catch
    end
end

function s = settingsLine(r)
    m = r.meta;
    s = sprintf(['mode %s | horizon %.3fs | Kalman R %.0f, sigma_a %.0f | KP %.1f  KI %.1f  KD %.2f | ' ...
        'max speed %.0f us | deadzone %.3f | %.1fs, %d frames, frame %dx%d'], ...
        m.detection_mode, r.h, m.kalman_measurement_noise, m.kalman_acceleration_noise, ...
        m.tripod_kp, m.tripod_ki, m.tripod_kd, m.tripod_max_speed_offset_us, r.dz, ...
        r.duration, r.n, r.W, r.H);
    if strlength(m.test_notes) > 0
        s = sprintf('%s\nnotes: %s', s, m.test_notes);
    end
end

function printRun(r, M)
    fprintf('\n==== %s  (%s)\n%s\n', r.meta.test_name, r.file, settingsLine(r));
    for i = 1:size(M, 1)
        st = 'PASS'; if ~r.pass(i), st = 'FAIL'; end
        fprintf('  %-28s %12s   target %s %-10s  %s\n', M{i, 2}, sprintf(M{i, 5}, r.(M{i, 1})), ...
            M{i, 4}, sprintf(M{i, 5}, M{i, 3}), st);
    end
    fprintf('  %-28s %12.3f\n', 'Ball offset RMS (Y)', r.offsetRmsY);
    fprintf('  %-28s %12.2f s  (used %.2f s)\n', 'Best horizon (indicative)', r.bestHorizon, r.h);
end

function fig = plotComparison(runs, M)
    c = colours();
    n = numel(runs);
    fig = figure('Name', 'Run comparison', 'Color', 'w', 'Position', [80 80 1500 850]);
    lightTheme(fig);
    tl = tiledlayout(fig, 2, 4, 'TileSpacing', 'compact', 'Padding', 'compact');
    title(tl, 'All runs vs targets (green = target met, red = missed; dashed line = target)', 'FontWeight', 'bold');
    labels = categorical({runs.label});
    labels = reordercats(labels, {runs.label});
    for i = 1:size(M, 1)
        ax = nexttile(tl);
        hold(ax, 'on'); grid(ax, 'on'); box(ax, 'on');
        v = arrayfun(@(r) r.(M{i, 1}), runs);
        p = arrayfun(@(r) r.pass(i), runs);
        b = bar(ax, labels, v, 'FaceColor', 'flat');
        b.CData = repmat(c.miss, n, 1);
        b.CData(p, :) = repmat(c.target, nnz(p), 1);
        yline(ax, M{i, 3}, '--', 'Color', 'k', 'LineWidth', 1.5);
        title(ax, sprintf('%s (%s %s)', M{i, 2}, M{i, 4}, strtrim(sprintf(M{i, 5}, M{i, 3}))), 'FontSize', 9);
        ax.TickLabelInterpreter = 'none';
    end
    % Settings that changed between runs, for reading the bars above.
    ax = nexttile(tl);
    axis(ax, 'off');
    rows = strings(n, 1);
    for k = 1:n
        m = runs(k).meta;
        rows(k) = sprintf('%s: h=%.2f R=%.0f sa=%.0f KP=%.0f KI=%.0f KD=%.1f MS=%.0f  [%d/%d]', ...
            runs(k).label, runs(k).h, m.kalman_measurement_noise, m.kalman_acceleration_noise, ...
            m.tripod_kp, m.tripod_ki, m.tripod_kd, m.tripod_max_speed_offset_us, ...
            sum(runs(k).pass), numel(runs(k).pass));
    end
    text(ax, 0, 1, ["Settings per run:"; rows], 'VerticalAlignment', 'top', ...
        'FontName', 'monospaced', 'FontSize', 8, 'Interpreter', 'none');
end

function S = buildSummary(runs, M)
    n = numel(runs);
    S = table((1:n)', string({runs.file})', 'VariableNames', {'run', 'file'});
    S.test_name = arrayfun(@(r) string(r.meta.test_name), runs)';
    S.notes     = arrayfun(@(r) string(r.meta.test_notes), runs)';
    S.mode      = arrayfun(@(r) string(r.meta.detection_mode), runs)';
    S.horizon_s = arrayfun(@(r) r.h, runs)';
    S.R         = arrayfun(@(r) r.meta.kalman_measurement_noise, runs)';
    S.sigma_a   = arrayfun(@(r) r.meta.kalman_acceleration_noise, runs)';
    S.KP        = arrayfun(@(r) r.meta.tripod_kp, runs)';
    S.KI        = arrayfun(@(r) r.meta.tripod_ki, runs)';
    S.KD        = arrayfun(@(r) r.meta.tripod_kd, runs)';
    S.MS        = arrayfun(@(r) r.meta.tripod_max_speed_offset_us, runs)';
    S.deadzone  = arrayfun(@(r) r.dz, runs)';
    S.duration_s = arrayfun(@(r) r.duration, runs)';
    for i = 1:size(M, 1)
        S.(M{i, 1}) = arrayfun(@(r) r.(M{i, 1}), runs)';
    end
    S.offsetRmsY  = arrayfun(@(r) r.offsetRmsY, runs)';
    S.sentRmsX    = arrayfun(@(r) r.sentRmsX, runs)';
    S.bestHorizon = arrayfun(@(r) r.bestHorizon, runs)';
    S.passed      = arrayfun(@(r) sprintf("%d/%d", sum(r.pass), numel(r.pass)), runs)';
end
