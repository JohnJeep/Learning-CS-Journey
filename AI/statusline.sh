#!/bin/bash
###
 # @Author: JohnJeep
 # @Date: 2026-09-30 14:26:47
 # @LastEditors: JohnJeep
 # @LastEditTime: 2026-09-30 16:30:57
 # @Description:
 # Copyright (c) 2026 by John Jeep, All Rights Reserved.
###
# Read JSON data that Claude Code sends to stdin
input=$(cat)

# Extract fields using jq, "// 0" provides a fallback if the field is null
read -r MODEL DIR SESSION_ID PCT COST DURATION_MS < <(
  jq -r '[
    (.model.display_name // ""),
    (.workspace.current_dir // ""),
    (.session_id // ""),
    ((.context_window.used_percentage // 0) | floor),
    (.cost.total_cost_usd // 0),
    (.cost.total_duration_ms // 0)
  ] | @tsv' <<<"$input"
)

# ANSI escape codes for terminal colors: green, yellow, red, cyan, and reset
CYAN='\033[36m'; GREEN='\033[32m'; YELLOW='\033[33m'; RED='\033[31m'; RESET='\033[0m'

# The cache filename needs to be stable across status line invocations within a
# session, but unique across sessions so concurrent sessions in different
# repositories don't read each other's cached git state. Process-based
# identifiers like $$ change on every invocation and defeat the cache. Use the
# session_id from the JSON input instead.
CACHE_FILE="/tmp/statusline-git-cache-${SESSION_ID:-default}"
CACHE_MAX_AGE=5  # seconds

cache_is_stale() {
  [ ! -f "$CACHE_FILE" ] && return 0
  # stat -c %Y (Linux) or stat -f %m (macOS) prints the file's last-modified
  # time. The Linux form must run first: on Linux, the macOS form prints a
  # filesystem report to stdout before failing, and that output would be
  # captured by the command substitution and break the arithmetic.
  local mtime now
  mtime=$(stat -c %Y "$CACHE_FILE" 2>/dev/null \
       || stat -f %m "$CACHE_FILE" 2>/dev/null \
       || echo 0)
  now=$(date +%s)
  [ $(( now - mtime )) -gt "$CACHE_MAX_AGE" ]
}

# Check if the cache file is missing or older than 5 seconds before running
# git commands, since commands like git status or git diff can be slow,
# especially in large repositories
if cache_is_stale; then
  if git rev-parse --git-dir >/dev/null 2>&1; then
    BRANCH=$(git branch --show-current 2>/dev/null)
    STAGED=$(git diff --cached --numstat 2>/dev/null | wc -l | tr -d ' ')
    MODIFIED=$(git diff --numstat 2>/dev/null | wc -l | tr -d ' ')
  else
    BRANCH=""; STAGED=0; MODIFIED=0
  fi
  printf '%s|%s|%s\n' "$BRANCH" "$STAGED" "$MODIFIED" > "$CACHE_FILE"
fi

IFS='|' read -r BRANCH STAGED MODIFIED < "$CACHE_FILE"

# Pick bar color based on context usage: green under 70%, yellow 70-89%,
# red 90%+
if   (( PCT >= 90 )); then BAR_COLOR="$RED"
elif (( PCT >= 70 )); then BAR_COLOR="$YELLOW"
else                       BAR_COLOR="$GREEN"
fi

# Build progress bar: printf -v creates a run of spaces, then
# ${var// /█} replaces each space with a block character
BAR_WIDTH=10
FILLED=$(( PCT * BAR_WIDTH / 100 ))
EMPTY=$(( BAR_WIDTH - FILLED ))
BAR=""
(( FILLED > 0 )) && printf -v FILL "%${FILLED}s" && BAR="${FILL// /█}"
(( EMPTY  > 0 )) && printf -v PAD  "%${EMPTY}s"  && BAR+="${PAD// /░}"

# Format cost as currency and convert milliseconds to minutes and seconds
COST_FMT=$(printf '$%.2f' "$COST")
MINS=$(( DURATION_MS / 60000 ))
SECS=$(( (DURATION_MS % 60000) / 1000 ))

# Build git segment with color-coded indicators for staged and modified files
GIT_SEG=""
if [ -n "$BRANCH" ]; then
  GIT_SEG=" | 🌿 ${BRANCH}"
  (( STAGED   > 0 )) && GIT_SEG+=" ${GREEN}+${STAGED}${RESET}"
  (( MODIFIED > 0 )) && GIT_SEG+=" ${YELLOW}~${MODIFIED}${RESET}"
fi

# Output multiple lines to create a richer display. Each print statement
# creates a separate row. printf '%b' interprets backslash escapes more
# reliably than echo -e across different shells
printf '%b\n' "${CYAN}[$MODEL]${RESET} 📁 ${DIR##*/}${GIT_SEG}"
printf '%b\n' "${BAR_COLOR}${BAR}${RESET} ${PCT}% | ${YELLOW}${COST_FMT}${RESET} | ⏱️ ${MINS}m ${SECS}s"
