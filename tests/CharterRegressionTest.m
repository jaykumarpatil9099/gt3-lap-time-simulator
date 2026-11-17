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
            r = runHeadSolver(tc.Root);
            for v = ["v06b", "v07"]
                err = (r.(v) - r.ref) / r.ref;
                tc.log(1, sprintf("%s lap %.3f s vs reference %.3f s (%+.3f s, %+.2f %%)", ...
                    v, r.(v), r.ref, r.(v) - r.ref, 100*err));
                tc.verifyLessThan(abs(err), 0.01, "Charter (" + v + "): |lap error| must stay below 1 %");
            end
        end

        function slipBuildUpCostsTime(tc)
            % v07 adds tyre relaxation on top of v06b: grip can only be lost, never gained
            r = runHeadSolver(tc.Root);
            tc.verifyGreaterThanOrEqual(r.v07, r.v06b - 1e-6);
        end

        function referenceTrackIsSane(tc)
            S = load(fullfile(tc.Root, "02_data", "track", "n24_track.mat"));
            tc.verifyEqual(S.track.length, 25205.7, "AbsTol", 50);   % Nordschleife + GP, metres
            tc.verifyEqual(S.track.meta.ref_laptime, 8*60 + 11.341, "AbsTol", 1e-6);
            tc.verifyTrue(all(S.track.kappa >= 0));
        end
    end
end

function r = runHeadSolver(root)
% Run the head chain (v06.0b -> v07.0) once per session, in an isolated
% workspace, exactly as a user would. Cached so both tests share one run.
persistent cache
if ~isempty(cache), r = cache; return; end
run(fullfile(root, "02_data", "car", "amg_gt3_params.m"));   % -> car
S = load(fullfile(root, "02_data", "track", "n24_track.mat"));
track = S.track; %#ok<NASGU> used by the solver scripts
run(fullfile(root, "03_models", "v06_pacejka", "lap_sim_v06b.m"));        % -> sim06b
run(fullfile(root, "03_models", "v07_slip_dynamics", "lap_sim_v07.m"));    % -> sim07
r = struct("v06b", sim06b.lap_time, "v07", sim07.lap_time, "ref", S.track.meta.ref_laptime);
cache = r;
end
