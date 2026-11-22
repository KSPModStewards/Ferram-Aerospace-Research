#!/bin/bash
# FAR's steps in KspTest's in-game workflow (tests.yml, and testing.yml through KspTest's test-release.yml:
# chambem42/KspTest/.github/workflows/ingame.yml), run in the repository with the game set up (KSPTEST_GAME,
# KSPTEST_WORK) and the build job's artifact in ci/. A test release's tests have its merge checked out already:
#   ci.sh prepare   the tests' FAR the build job's (FarGameData: ci/GameData/FerramAerospaceResearch, which the coverage
#                   leg instruments in place) and KspTest this repository's submodule (KSPTEST_ROOT: the harness FAR's
#                   tests are written against, not the actions' own copy)
#   ci.sh unit      the unit tests (.NET Framework 4.8: Windows), against the game's own assemblies (all of them run, as
#                   the public stripped ones do not let a third of them) with KSPBurst's Unity packages (ci/kspburst),
#                   and FAR as the tests have it (instrumented, when the leg measures coverage: their counts as
#                   $COV/unit.xml join the in-game ones). Results in $KSPTEST_WORK/unit-trx and the job summary; fails
#                   when a test does
set -eo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
win() { if command -v cygpath > /dev/null; then cygpath -m "$1"; else echo "$1"; fi; }

prepare() {
  {
    echo "FarGameData=$(win "$PWD/ci/GameData/FerramAerospaceResearch")"
    echo "KSPTEST_ROOT=$(win "$PWD/KspTests/KspTest")"
  } >> "$GITHUB_ENV"
}

unit() {
  local k="$KSPTEST_WORK/unit-ksp" u=KspTests/FerramAerospaceResearch.UnitTests/bin/Debug/net48
  local far="$PWD/ci/GameData/FerramAerospaceResearch/Plugins" status=0
  mkdir -p "$k/KSP_x64_Data" "$k/GameData/000_KSPBurst/Plugins"
  cp -r "$KSPTEST_GAME/KSP_x64_Data/Managed" "$k/KSP_x64_Data/"
  cp ci/kspburst/Unity.*.dll "$k/GameData/000_KSPBurst/Plugins/"
  dotnet build KspTests/FerramAerospaceResearch.UnitTests "-p:KspDir=$k" "-p:FarBin=$(win "$far")" -v q -nologo
  # instrumented (KspTest's coverage.sh, in place): the build copies only the FAR assemblies it references, so the
  # recorder, FastVisit and the report by hand
  if [ -n "$KSPTEST_COVERAGE_REPORT" ]; then
    cp "$far/coverage.xml" "$far/AltCover.Recorder.g.dll" "$far/KspTest.Coverage.FastVisit.dll" "$u/"
  fi
  dotnet test KspTests/FerramAerospaceResearch.UnitTests --no-build --logger "console;verbosity=normal" \
    --logger "trx;LogFileName=unit.trx" --results-directory "$KSPTEST_WORK/unit-trx" || status=$?
  if [ -n "$KSPTEST_COVERAGE_REPORT" ]; then cp "$u/coverage.xml" "$COV/unit.xml"; fi
  python -I "$HERE/test_summary.py" "Unit tests" "$KSPTEST_WORK/unit-trx" >> "$GITHUB_STEP_SUMMARY" ||
    echo "::warning::no unit test summary"
  return $status
}

case "$1" in
  prepare) prepare ;;
  unit) unit ;;
  *) echo "usage: ci.sh prepare | unit" >&2; exit 2 ;;
esac
