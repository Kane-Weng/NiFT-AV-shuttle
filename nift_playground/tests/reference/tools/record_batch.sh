#!/usr/bin/env bash
# Record several reference runs, a few containers at a time.
# Usage: record_batch.sh <workspace dir> <bag dir> <parallel> <label>:<route>[:<extra launch args>] ...
#   e.g. record_batch.sh ws bags 3 direct_01:01 ekf_04:04:localization:=ekf
set -uo pipefail
WS=$1; BAGS=$2; PAR=$3; shift 3
HERE=$(cd "$(dirname "$0")" && pwd)
printf '%s\n' "$@" | xargs -P "$PAR" -I{} bash -c '
  IFS=: read -r label route extra <<< "$1"
  if [ -n "$extra" ]; then
    "$0/record_reference.sh" "$2" "$3" "$label" "$route" --extra "\"$extra\""
  else
    "$0/record_reference.sh" "$2" "$3" "$label" "$route"
  fi' "$HERE" {} "$WS" "$BAGS"
