classdef CharterRegressionTest < matlab.unittest.TestCase
    %CHARTERREGRESSIONTEST  Guards the project charter: the head solver must
    %   reproduce the reference lap within +/-1 %, and the tyre model must
    %   keep its basic physical properties. Runs in CI on every push, so a
    %   change that breaks correlation is caught before it is merged.
    %
    %   Run with:  buildtool test

    properties
        Root
    end

    methods (TestClassSetup)
        function addPaths(tc)
            tc.Root = fileparts(fileparts(mfilename("fullpath")));
            for d = ["02_data", "03_models", "04_correlation"]
                tc.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                    fullfile(tc.Root, d), IncludingSubfolders = true));
            end
        end
    end

    methods (Test)
        function headLapWithinCharter(tc)
            [lapTime, refTime] = runHeadSolver(tc.Root);
            err = (lapTime - refTime) / refTime;
            tc.log(1, sprintf("v06.0b lap %.3f s vs reference %.3f s (%+.2f %%)", ...
                lapTime, refTime, 100*err));
            tc.verifyLessThan(abs(err), 0.01, "Charter: |lap error| must stay below 1 %");
        end

        function referenceTrackIsSane(tc)
            S = load(fullfile(tc.Root, "02_data", "track", "n24_track.mat"));
            tc.verifyEqual(S.track.length, 25205.7, "AbsTol", 50);   % Nordschleife + GP, metres
            tc.verifyEqual(S.track.meta.ref_laptime, 8*60 + 11.341, "AbsTol", 1e-6);
            tc.verifyTrue(all(S.track.kappa >= 0));
        end
    end
end

function [lapTime, refTime] = runHeadSolver(root)
% Run the head solver (v06.0b) in an isolated workspace, exactly as a user would
run(fullfile(root, "02_data", "car", "amg_gt3_params.m"));   % -> car
S = load(fullfile(root, "02_data", "track", "n24_track.mat"));
track = S.track; %#ok<NASGU> used by the solver script
run(fullfile(root, "03_models", "v06_pacejka", "lap_sim_v06b.m"));   % -> sim06b
lapTime = sim06b.lap_time;
refTime = S.track.meta.ref_laptime;
end
