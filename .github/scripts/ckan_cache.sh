#!/bin/bash
# CKAN's download cache kept between CI runs (tests.yml, package.yml): the folder ckan-downloads in the workspace, an
# actions/cache entry. The mods come mostly from GitHub release assets, whose downloads fail at times; with the cache a
# run downloads only the mods that changed since the entry was saved.
#   ckan_cache.sh use CKAN...   points CKAN (the command, e.g. "ckan" or "mono ckan.exe") at the folder and notes what
#                               it holds, for key and new
#   ckan_cache.sh snapshot      only notes what the folder holds
#   ckan_cache.sh key           when the folder gained or changed files since, trims it to $LIMIT_MB and sets the step
#                               output key to a new entry's key, named by its files; else sets none
#   ckan_cache.sh new DIR       copies the files the folder gained or changed since into DIR (none: DIR left empty)
set -eo pipefail
DIR=ckan-downloads
LIST="$RUNNER_TEMP/ckan-downloads.txt"
LIMIT_MB=${LIMIT_MB:-1536}

# a line a file: name<TAB>size
files() { find "$DIR" -maxdepth 1 -type f -printf '%f\t%s\n' 2>/dev/null | sort; }
snapshot() { mkdir -p "$DIR"; files > "$LIST"; echo "CKAN download cache: $(wc -l < "$LIST") files, $(du -sh "$DIR" | cut -f1)"; }
added() { [ -f "$LIST" ] || return 0; files | comm -13 "$LIST" - | cut -f1; }

# the files in the order the trim drops them: first those of a mod a newer file is here of (an older version), then
# the rest, oldest first each. CKAN names them <url hash>-<identifier>-<version>.zip; a name read wrong only puts its
# file with the rest
trim_order() {
  find "$DIR" -maxdepth 1 -type f -printf '%T@\t%f\n' | sort -rn | awk -F'\t' '
    { mod = $2; sub(/^[^-]*-/, "", mod); sub(/-v?[0-9].*$/, "", mod)
      if (mod in newest) old[++n] = $2; else { newest[mod] = 1; rest[++m] = $2 } }
    END { for (i = n; i >= 1; i--) print old[i]; for (i = m; i >= 1; i--) print rest[i] }'
}

case "$1" in
  use)
    shift
    snapshot
    path="$PWD/$DIR"
    command -v cygpath > /dev/null && path=$(cygpath -m "$path")
    "$@" cache set --headless "$path"
    ;;
  snapshot)
    snapshot
    ;;
  key)
    [ -n "$(added)" ] || { echo "no new downloads: the entry stays"; exit 0; }
    # only the downloads themselves (not CKAN's folder of unfinished ones)
    find "$DIR" -mindepth 1 -maxdepth 1 ! -type f -exec rm -rf {} +
    mapfile -t order < <(trim_order)
    for f in "${order[@]}"; do
      [ "$(du -sm "$DIR" | cut -f1)" -le "$LIMIT_MB" ] && break
      echo "over ${LIMIT_MB} MB, dropping $f"; rm -f "$DIR/$f"
    done
    key="ckan-downloads-$(files | sha256sum | cut -c1-16)"
    echo "new entry $key: $(files | wc -l) files, $(du -sh "$DIR" | cut -f1)"
    echo "key=$key" >> "$GITHUB_OUTPUT"
    ;;
  new)
    mkdir -p "$2"
    mapfile -t new < <(added)
    for f in "${new[@]}"; do cp "$DIR/$f" "$2/"; done
    echo "$(find "$2" -type f | wc -l) new CKAN downloads"
    ;;
  *)
    echo "usage: ckan_cache.sh use CKAN... | snapshot | key | new DIR" >&2; exit 2
    ;;
esac
