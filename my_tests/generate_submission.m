function generate_submission(my_script, out_dir, env_json, sched_json, config_json, utility_file, background_image)
% generate_submission.m
% Build a Robotarium upload folder from your local script that uses JSON.
% - Reads my_script.m, keeps only the top script (before first 'function')
% - Removes full-line comments and trailing spaces
% - Rewrites JSON/image loads to load MAT files and background.png
% - Saves env.mat, schedule.mat, config.mat
% - Copies utility.m and background image

    % ---------- defaults ----------
    if nargin < 1 || isempty(my_script),        my_script        = 'local_script_test1.m'; end
    if nargin < 2 || isempty(out_dir),          out_dir          = 'robotarium_upload'; end
    if nargin < 3 || isempty(env_json),         env_json         = 'env.json'; end
    if nargin < 4 || isempty(sched_json),       sched_json       = 'schedule.json'; end
    if nargin < 5 || isempty(config_json),      config_json      = 'config.json'; end
    if nargin < 6 || isempty(utility_file),     utility_file     = 'utility.m'; end
    if nargin < 7 || isempty(background_image), background_image = 'warehouse_image.png'; end

    % ---------- preflight ----------
    assert(exist(my_script,'file')==2,   'Missing my_script: %s', my_script);
    assert(exist(env_json,'file')==2,    'Missing env_json: %s', env_json);
    assert(exist(sched_json,'file')==2,  'Missing sched_json: %s', sched_json);
    assert(exist(config_json,'file')==2, 'Missing config_json: %s', config_json);
    if ~exist(out_dir,'dir'), mkdir(out_dir); end

    % ---------- JSON -> MAT ----------
    env      = jsondecode(fileread(env_json));
    schedule = jsondecode(fileread(sched_json));
    config   = jsondecode(fileread(config_json));
    save(fullfile(out_dir,'env.mat'),      'env',      '-v7');
    save(fullfile(out_dir,'schedule.mat'), 'schedule', '-v7');
    save(fullfile(out_dir,'config.mat'),   'config',   '-v7');

    % ---------- Copy utility (if present) ----------
    if exist(utility_file,'file')==2
        copyfile(utility_file, fullfile(out_dir,'utility.m'));
    end

    % ---------- Copy background image (optional) ----------
    if exist(background_image,'file')==2
        copyfile(background_image, fullfile(out_dir,'background.png'));
    end

    % ---------- Build main.m from my_script.m (top-section only) ----------
    raw = fileread(my_script);

    % Split into lines (preserve order)
    lines = regexp(raw, '\r\n|\n|\r', 'split');

    % Find first local function line (start of function block)
    funcIdx = [];
    for i = 1:numel(lines)
        if ~isempty(regexp(lines{i}, '^\s*function\b', 'once'))
            funcIdx = i;
            break;
        end
    end
    if isempty(funcIdx)
        topLines = lines;                 % no local functions; keep all
    else
        topLines = lines(1:funcIdx-1);    % keep only the script portion
    end

    % Remove full-line comments (lines starting with % after optional spaces)
    keep = true(size(topLines));
    for i = 1:numel(topLines)
        if ~isempty(regexp(topLines{i}, '^\s*%.*$', 'once'))
            keep(i) = false;
        end
    end
    topLines = topLines(keep);

    % Trim trailing spaces
    for i = 1:numel(topLines)
        topLines{i} = regexprep(topLines{i}, '[ \t]+$', '');
    end

    % Join back to a single string for targeted replacements
    T = strjoin(topLines, newline);

    % ---- JSON -> MAT replacements (script_dir must remain defined) ----
    T = regexprep(T, ...
        'env\s*=\s*load_json_data\(\s*script_dir\s*,\s*''env\.json''\s*\)\s*;', ...
        "S=load(fullfile(script_dir,'env.mat')); env=S.env;", 'once');

    T = regexprep(T, ...
        'schedule\s*=\s*load_json_data\(\s*script_dir\s*,\s*''schedule\.json''\s*\)\s*;', ...
        "S=load(fullfile(script_dir,'schedule.mat')); schedule=S.schedule;", 'once');

    T = regexprep(T, ...
        'config\s*=\s*load_json_data\(\s*script_dir\s*,\s*''config\.json''\s*\)\s*;', ...
        "S=load(fullfile(script_dir,'config.mat')); config=S.config;", 'once');

    T = regexprep(T, ...
        'img\s*=\s*load_background\(\s*script_dir\s*,\s*''[^'']*''\s*\)\s*;', ...
        "fp=fullfile(script_dir,'background.png'); if exist(fp,'file'), img=imread(fp); else, img=[]; end", 'once');

    % ---- SAFETY: strip any function blocks that somehow slipped through ----
    % Remove complete "function ... end" blocks (multiline, greedy-min)
    T = regexprep(T, '(?ms)^\s*function[^\n]*\n.*?^\s*end\s*$', '', 'once');
    % If *any* function header remains, cut from that header to EOF
    firstFuncPos = regexp(T, '(?m)^\s*function\b', 'once');
    if ~isempty(firstFuncPos)
        T = strtrim(T(1:firstFuncPos-1));
    end

    % Normalize extra blank lines
    T = regexprep(T, '\n{3,}', sprintf('\n\n'));
    T = strtrim(T);

    % Write main.m
    fid = fopen(fullfile(out_dir,'main.m'),'w');
    assert(fid>0,'Cannot open main.m for writing.');
    fwrite(fid, T, 'char');
    fclose(fid);

    % ---------- Done ----------
    fprintf(1,'\nPackaged to %s\n', out_dir);
    fprintf(1,'Includes:\n  main.m\n  env.mat\n  schedule.mat\n  config.mat\n');
    if exist(fullfile(out_dir,'utility.m'),'file'), fprintf(1,'  utility.m\n'); end
    if exist(fullfile(out_dir,'background.png'),'file'), fprintf(1,'  background.png\n'); end
end