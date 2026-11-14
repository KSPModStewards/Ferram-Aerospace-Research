#!/usr/bin/env bash
# testing.yml's collect job: merges the open pull requests labelled $LABEL onto the checked-out master, lower numbers
# first, and writes what the build and publish jobs need to $OUT: merged.bundle (the merge commits; none without pull
# requests) and notes.md (the release notes). Outputs: build (false: nothing to publish), master, commit, version, tag.
# Nothing of the pull requests runs here: git merges them, that is all.
#
# Env: REPO (owner/name), LABEL, BASE (the default branch), OUT; from the event ACTION, PR, MERGED (all optional);
# FORCE=true publishes an unchanged set. Local dry run, which takes no labels off and posts no comments:
#   DRY_RUN=1 REPO=KSPModStewards/Ferram-Aerospace-Research LABEL=ready-for-testing BASE=master OUT=/tmp/t \
#     bash .github/scripts/testing_merge.sh        (on a scratch checkout of master: it merges into HEAD)
set -euo pipefail

: "${REPO:?}" "${LABEL:?}" "${BASE:?}" "${OUT:?}"
GITHUB_OUTPUT=${GITHUB_OUTPUT:-/dev/stdout}
GITHUB_STEP_SUMMARY=${GITHUB_STEP_SUMMARY:-/dev/stderr}
mkdir -p "$OUT"

output() { echo "$1=$2" >> "$GITHUB_OUTPUT"; }
write() { if [ -n "${DRY_RUN:-}" ]; then echo "(dry run) gh $*" >&2; else gh "$@"; fi; }
unlabel() { write api -X DELETE "repos/$REPO/issues/$1/labels/$LABEL" --silent || echo "::warning::could not take $LABEL off #$1"; }
comment() { write api "repos/$REPO/issues/$1/comments" -f body="$2" --silent || echo "::warning::could not comment on #$1"; }

# Lookups that fail end the run (explicit exits: set -e does not reach into functions called from an if). Read as "no",
# they would take a maintainer's label off
fail() { echo "::error::$1"; exit 1; }

declare -A permission
writer() {
  if [ -z "${permission[$1]+set}" ]; then
    permission[$1]=$(gh api "repos/$REPO/collaborators/$1/permission" --jq .permission) ||
      fail "could not look up @$1's permission on $REPO"
  fi
  case "${permission[$1]}" in
    admin|maintain|write) return 0 ;;
    *) return 1 ;;
  esac
}

if [ "${ACTION:-}" = closed ]; then
  unlabel "$PR"
  if [ "${MERGED:-}" = true ]; then
    echo "::notice::#$PR was merged: master has it now, the testing build stays as it is"
    output build false
    exit 0
  fi
fi

master=$(git rev-parse HEAD)
output master "$master"

# --- the labelled pull requests, those that count ---------------------------------------------------------------------

# into a variable first: from a process substitution its failure would read as no pull requests, and publish master
listing=$(gh pr list -R "$REPO" --base "$BASE" --state open --label "$LABEL" --limit 100 \
            --json number,headRefOid,author,title \
            --jq 'sort_by(.number)[] | [.number, .headRefOid, .author.login, .title] | @tsv') ||
  fail "could not list the pull requests labelled $LABEL"

nums=() shas=() authors=() titles=()
while IFS=$'\t' read -r n sha author title; do
  [ -n "$n" ] || continue
  # whoever put the label on last
  labeller=$(gh api --paginate "repos/$REPO/issues/$n/events" \
    --jq '.[] | select(.event == "labeled" and .label.name == env.LABEL) | .actor.login' | tail -n 1)
  reason=""
  if [ -z "$labeller" ] || ! writer "$labeller"; then
    reason="@$labeller labelled it $LABEL, but only someone with write access puts a pull request in the testing build."
  # as run-ingame in tests.yml: what the labeller read is what goes in (a review records its commit), not dismissed
  elif ! writer "$author"; then
    reviewed=$(L=$labeller S=$sha gh api --paginate "repos/$REPO/pulls/$n/reviews" \
      --jq '.[] | select(.user.login == env.L and .commit_id == env.S and .state != "PENDING" and .state != "DISMISSED") | .id') ||
      fail "could not read the reviews of #$n"
    [ -n "$reviewed" ] ||
      reason="no review by @$labeller of the latest commit ${sha::7}. Submit a review of it (a comment is enough), then label it $LABEL again."
  fi
  if [ -n "$reason" ]; then
    echo "::warning::#$n left out: $reason"
    unlabel "$n"
    comment "$n" "Left out of the testing build: $reason"
    continue
  fi
  nums+=("$n") shas+=("$sha") authors+=("$author") titles+=("$title")
done <<< "$listing"

if [ ${#nums[@]} -gt 0 ]; then
  git fetch -q --no-tags origin $(printf 'refs/pull/%s/head ' "${nums[@]}")
  # pushed to since the listing: the listed commit is the one checked
  for sha in "${shas[@]}"; do
    git cat-file -e "$sha^{commit}" 2> /dev/null || git fetch -q --no-tags origin "$sha"
  done
fi

# Left out: pull requests already in master, and those whose commits are all in another one, which brings them. Two
# conflicting pull requests go in together this way, through a third merging both
keep=() skipped=()
for i in "${!nums[@]}"; do
  within=""
  if git merge-base --is-ancestor "${shas[i]}" "$master"; then
    within=master
  else
    for j in "${!nums[@]}"; do
      [ "$i" != "$j" ] || continue
      if [ "${shas[i]}" = "${shas[j]}" ]; then
        [ "$j" -lt "$i" ] && within="#${nums[j]}" && break
      elif git merge-base --is-ancestor "${shas[i]}" "${shas[j]}"; then
        within="#${nums[j]}" && break
      fi
    done
  fi
  if [ -n "$within" ]; then
    skipped+=("- #${nums[i]}: its commits are in $within")
  else
    keep+=("$i")
  fi
done

# --- merge -------------------------------------------------------------------------------------------------------------

git config user.name "github-actions[bot]"
git config user.email "41898282+github-actions[bot]@users.noreply.github.com"

merged=()
for i in "${keep[@]}"; do
  n=${nums[i]}
  if git merge -q --no-ff --no-edit -m "Merge #$n: ${titles[i]}" "${shas[i]}" > /dev/null 2>&1; then
    merged+=("$i")
    continue
  fi
  files=$(git diff --name-only --diff-filter=U | sed 's/.*/- `&`/')
  git merge --abort

  # what it conflicts with: master alone, or each pull request merged before it on top of master
  with=() marker="${shas[i]}"
  if ! git merge-tree --write-tree --name-only "$master" "${shas[i]}" > /dev/null; then
    with=(master)
  else
    for p in "${merged[@]}"; do
      tree=$(git merge-tree --write-tree "$master" "${shas[p]}" | head -n 1) || continue
      probe=$(git commit-tree "$tree" -p "$master" -p "${shas[p]}" -m probe)
      if ! git merge-tree --write-tree --name-only "$probe" "${shas[i]}" > /dev/null; then
        with+=("#${nums[p]}") marker="$marker ${shas[p]}"
      fi
    done
  fi
  if [ "${with[*]}" = master ]; then
    why="it conflicts with master. Merge master into it, or rebase it."
  else
    if [ ${#with[@]} -eq 0 ]; then
      # with none of them alone: with several together
      what="the labelled pull requests merged before it, together"
    else
      what=$(printf '%s, ' "${with[@]}" | sed 's/, $//')
    fi
    why="it conflicts with $what (lower numbers merge first). To test them together, open a pull request merging both branches with the conflict resolved (a draft is fine) and label it too: a pull request whose commits are all in another labelled one is left out. Or take the label off one."
  fi
  body="The testing build stopped at this pull request: $why

Conflicting files:
$files

<!-- testing-conflict $marker -->"
  echo "::error::#$n: $why"
  printf '### Testing build stopped at #%s\n\n%s\n' "$n" "$body" >> "$GITHUB_STEP_SUMMARY"
  # once per conflict, not on every run it stops
  bodies=$(gh api --paginate "repos/$REPO/issues/$n/comments" --jq '.[].body')
  grep -qF -- "<!-- testing-conflict $marker -->" <<< "$bodies" || comment "$n" "$body"
  exit 1
done

# --- unchanged since the last testing build? ---------------------------------------------------------------------------

set_line="master=$master"
for i in "${merged[@]}"; do set_line="$set_line #${nums[i]}=${shas[i]}"; done
set_marker="<!-- testing-set: $set_line -->"

# the last published: GitHub lists releases by their commit's date, and a master-only build tags an older commit
latest=$(M=$set_marker gh api "repos/$REPO/releases?per_page=30" \
  --jq 'map(select(.draft | not)) | (max_by(.published_at) // {}) |
        "\(.prerelease // false) \((.body // "") | contains(env.M))"') ||
  fail "could not read the releases"
read -r latest_pre latest_same <<< "$latest"
if [ "${FORCE:-}" != true ]; then
  if [ "$latest_pre" = true ] && [ "$latest_same" = true ]; then
    echo "::notice::the labelled pull requests and master are what the last testing build has: nothing to publish"
    output build false
    exit 0
  fi
  if [ ${#merged[@]} -eq 0 ] && [ "$latest_pre" != true ]; then
    echo "::notice::no pull requests labelled $LABEL, and the latest release is a stable one: nothing to publish"
    output build false
    exit 0
  fi
fi

# --- version: the latest stable release's, its fourth number counting the testing builds on it --------------------------
# Stable releases end in .0 (package.yml enforces it), so 0.17.0.0 < testing 0.17.0.1, 0.17.0.2... < stable 0.17.1.0

stable=$(gh api "repos/$REPO/releases/latest" --jq .tag_name)
base=$(sed -nE 's/^v([0-9]+\.[0-9]+\.[0-9]+)\.[0-9]+_.*/\1/p' <<< "$stable")
stable_revision=$(sed -nE 's/^v[0-9]+\.[0-9]+\.[0-9]+\.([0-9]+)_.*/\1/p' <<< "$stable")
if [ -z "$base" ]; then
  echo "::error::no version in the latest release's tag, $stable"
  exit 1
fi
last=$(git ls-remote --tags origin "v$base.*" |
  sed -nE "s#.*refs/tags/v${base//./\\.}\\.([0-9]+)_testing\$#\\1#p" | sort -n | tail -n 1)
# above the stable release's own fourth number too, should an old one not end in .0
revision=$(( $( printf '%s\n' "${last:-0}" "${stable_revision:-0}" | sort -n | tail -n 1 ) + 1 ))
if [ "$revision" -gt 255 ]; then
  echo "::error::$base.$revision: Version.cs keeps the fourth number in a byte. Time for a stable release"
  exit 1
fi
version="$base.$revision"
tag="v${version}_testing"

# --- notes and bundle --------------------------------------------------------------------------------------------------

{
  echo "Testing build: master at $master with the pull requests labelled \`$LABEL\` merged in. For trying them out before they are merged; expect bugs, and report them on the pull request concerned."
  echo
  if [ ${#merged[@]} -gt 0 ]; then
    for i in "${merged[@]}"; do echo "- #${nums[i]} ${titles[i]} (${authors[i]}, ${shas[i]::7})"; done
  else
    echo "No pull requests labelled: master alone, replacing the previous testing build."
  fi
  if [ ${#skipped[@]} -gt 0 ]; then
    echo
    echo "Labelled, in through another:"
    printf '%s\n' "${skipped[@]}"
  fi
  echo
  echo "CKAN offers it to those whose stability tolerance for FAR (or for all mods) is Testing; the next stable release replaces it. The tag $tag is the merged source."
  echo
  echo "$set_marker"
} > "$OUT/notes.md"

if [ ${#merged[@]} -gt 0 ]; then
  git bundle create "$OUT/merged.bundle" HEAD "^$master" 2> /dev/null
fi

output build true
output commit "$(git rev-parse HEAD)"
output version "$version"
output tag "$tag"
cat "$OUT/notes.md" >> "$GITHUB_STEP_SUMMARY"
