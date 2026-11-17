function plan = buildfile
%BUILDFILE  MATLAB build plan (R2024a+).
%   buildtool   -> static code analysis + charter regression test
import matlab.buildtool.tasks.CodeIssuesTask
import matlab.buildtool.tasks.TestTask

plan = buildplan(localfunctions);

plan("check") = CodeIssuesTask(["03_models", "04_correlation", "tests"]);
plan("test")  = TestTask("tests");

plan.DefaultTasks = ["check", "test"];
end
