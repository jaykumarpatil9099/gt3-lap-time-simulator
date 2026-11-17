function plan = buildfile
%BUILDFILE  MATLAB build plan (R2024a+).
%   buildtool   -> static code analysis + charter regression test
import matlab.buildtool.tasks.CodeIssuesTask
import matlab.buildtool.tasks.TestTask

plan = buildplan(localfunctions);

plan("check") = CodeIssuesTask(analysedFiles(["03_models", "04_correlation", "tests"]));
plan("test")  = TestTask("tests");

plan.DefaultTasks = ["check", "test"];
end

function files = analysedFiles(folders)
% All .m files under the given folders, except archived versions whose dated
% names (e.g. lap_sim_v05_retired_2026-04-21.m) are kept for the logbook trail
% but are not valid MATLAB identifiers.
files = strings(0, 1);
for f = folders
    d = dir(fullfile(f, "**", "*.m"));
    for k = 1:numel(d)
        [~, name] = fileparts(d(k).name);
        if isvarname(name)
            files(end+1, 1) = string(fullfile(d(k).folder, d(k).name)); %#ok<AGROW>
        end
    end
end
end
