#!/bin/bash
# FAR's line coverage in the Windows in-game jobs (tests.yml): AltCover instruments the build job's FAR, the unit and
# in-game tests run against it, and their counts (one per sequence point) merge into a report.
#   coverage.sh instrument   the instrumented FAR: an overlay for the in-game tests ($COV/overlay, switched to through
#                            GITHUB_ENV only once it is complete, so a failure here leaves the tests on the plain build)
#                            and the assemblies the unit tests build against ($COV/farbin)
#   coverage.sh unit         the unit tests, against the instrumented FAR (else the plain build), their results in
#                            $WORK/unit-trx and their counts in $COV/unit.xml; fails when a test does
#   coverage.sh collect TAG  each KSP session's counts, from its results, as $COV/ingame-TAG-<n>.xml (TAG: the job's, so
#                            the jobs' files can go in one folder)
#   coverage.sh report       every count in $COV merged into $COV/report.md and the job summary (on any runner)
# Uses WORK, COV, KSPTEST_GAME and KSPTEST_RESULTS (forward-slash Windows paths) and ALTCOVER (the AltCover version).
set -eo pipefail
FAR="$WORK/far/FerramAerospaceResearch"
OUT="$COV/out"
REPORT_REL=GameData/FerramAerospaceResearch/Plugins/coverage.xml

instrument() {
  curl -fsSL -o "$WORK/altcover.nupkg" "https://www.nuget.org/api/v2/package/altcover/$ALTCOVER"
  7z x -y -bso0 -bsp0 "-o$WORK/altcover" "$WORK/altcover.nupkg" 'tools/net472/*'
  mkdir -p "$COV/in" "$COV/farbin" "$COV/overlay"
  cp "$FAR/Plugins"/FerramAerospaceResearch*.dll "$FAR/Plugins"/FerramAerospaceResearch*.pdb "$COV/in/"
  # --linecover: sequence points only (the report counts lines); the branch probes cost about a fifth of the run.
  # KSP's own assemblies resolve FAR's references (it needs nothing from the mods it talks to)
  local deps=() f
  for f in "$(cygpath -u "$KSPTEST_GAME")"/KSP_x64_Data/Managed/*.dll; do deps+=("-d=$(cygpath -m "$f")"); done
  "$WORK/altcover/tools/net472/AltCover.exe" -i="$COV/in" -o="$OUT" --save --portable --linecover \
    -a=ExcludeFromCodeCoverage "${deps[@]}" > "$COV/instrument.log" 2>&1 || { tail -20 "$COV/instrument.log"; return 1; }
  # every visit call goes to FastVisit, which passes only a point's first visit on to AltCover's recorder (see FastVisit.cs)
  dotnet build KspTests/Coverage/FarCoverage.FastVisit -c Release -v q -nologo -o "$COV/fastvisit"
  dotnet run -c Release --project KspTests/Coverage/CovPatch -- "$OUT/FerramAerospaceResearch.dll" "$OUT/FerramAerospaceResearch.Base.dll"
  cp "$COV/fastvisit/FarCoverage.FastVisit.dll" "$OUT/"
  local run=("$OUT"/FerramAerospaceResearch*.dll "$OUT/AltCover.Recorder.g.dll" "$OUT/FarCoverage.FastVisit.dll")
  # the in-game tests' FAR: the build's GameData with the instrumented assemblies, the recorder and its report
  cp -r "$FAR" "$COV/overlay/"; rm -f "$COV/overlay/FerramAerospaceResearch/Plugins/"*.pdb
  cp "${run[@]}" "$OUT/coverage.xml" "$COV/overlay/FerramAerospaceResearch/Plugins/"
  # the unit tests' FAR
  cp "${run[@]}" "$FAR/Plugins/ferramGraph.dll" "$COV/farbin/"
  # each KSP session's counts, copied into its results once KSP exits (the next session's deploy into the same
  # instance writes over them)
  {
    echo "FarGameData=$COV/overlay/FerramAerospaceResearch"
    echo "KSPTEST_COLLECT=$REPORT_REL"
  } >> "$GITHUB_ENV"
}

# against the game's own assemblies, so the tests that need Unity's code run too (stripped ones, as the public KSPLibs
# zip has, skip a third of them), with KSPBurst's Unity packages from the build job ($WORK/kspburst). A folder of their
# own: the game's stays as extracted for the instances
unit() {
  local k="$WORK/unit-ksp" u=KspTests/FerramAerospaceResearch.UnitTests/bin/Debug/net48 farbin="$COV/farbin" status=0
  # FAR not instrumented (instrument failed): the plain build, without counts
  [ -f "$farbin/FerramAerospaceResearch.dll" ] || farbin="$FAR/Plugins"
  mkdir -p "$k/KSP_x64_Data" "$k/GameData/000_KSPBurst/Plugins"
  cp -r "$KSPTEST_GAME/KSP_x64_Data/Managed" "$k/KSP_x64_Data/"
  cp "$WORK/kspburst"/Unity.*.dll "$k/GameData/000_KSPBurst/Plugins/"
  dotnet build KspTests/FerramAerospaceResearch.UnitTests "-p:KspDir=$k" "-p:FarBin=$farbin" -v q -nologo
  # the build copies only the FAR assemblies it references: the recorder, FastVisit and the report by hand
  if [ "$farbin" = "$COV/farbin" ]; then cp "$OUT/coverage.xml" "$OUT/AltCover.Recorder.g.dll" "$OUT/FarCoverage.FastVisit.dll" "$u/"; fi
  dotnet test KspTests/FerramAerospaceResearch.UnitTests --no-build --logger "console;verbosity=normal" \
    --logger "trx;LogFileName=unit.trx" --results-directory "$WORK/unit-trx" || status=$?
  if [ "$farbin" = "$COV/farbin" ]; then cp "$u/coverage.xml" "$COV/unit.xml"; fi
  return $status
}

# the counts of every session that got as far as quitting KSP, failed tests or not
collect() {
  local tag=${1:?coverage.sh collect TAG} i=0 f
  mkdir -p "$COV"
  for f in "$(cygpath -u "$KSPTEST_RESULTS")"/*/collected/$REPORT_REL; do
    [ -f "$f" ] || continue
    i=$((i + 1)); cp "$f" "$COV/ingame-$tag-$i.xml"
  done
  echo "$i KSP session report(s)"
}

report() {
  local reports=() f
  for f in "$COV"/unit.xml "$COV"/ingame-*.xml; do [ -f "$f" ] && reports+=("$f"); done
  echo "${#reports[@]} report(s)"
  if [ ${#reports[@]} -eq 0 ]; then echo "::warning::no coverage counts to report"; return 0; fi
  python -I .github/scripts/coverage_report.py "${GITHUB_REPOSITORY#*/}/" "$COV/report.md" "${reports[@]}"
  cat "$COV/report.md" >> "$GITHUB_STEP_SUMMARY"
}

case "$1" in
  instrument | unit | collect | report) "$@" ;;
  *) echo "coverage.sh instrument|unit|collect TAG|report"; exit 2 ;;
esac
