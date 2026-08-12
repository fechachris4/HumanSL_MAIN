#!/usr/bin/env bash
# Mechanical capture for the intent record: append the user's prompt
# verbatim to docs/intent/raw-prompt-log.md. This script must stay dumb —
# no filtering, no interpretation — so the ground-truth layer cannot drift.
# Interpretation lives in docs/intent/story.md.
set -u

# The log lives at the repository root, two levels above this script
# (<repo>/.claude/hooks/), so the hook works no matter which directory
# the Claude session was launched from.
repo_root=$(cd "$(dirname "$0")/../.." && pwd)
log="$repo_root/docs/intent/raw-prompt-log.md"

prompt=$(jq -r '.prompt // empty')
[ -z "$prompt" ] && exit 0

# Claude Code injects background-task notifications as user-turn prompts.
# They are machine output, not Christian's words, so they do not belong in
# ground truth. This is a fixed mechanical rule, not interpretation.
case "$prompt" in
  "<task-notification>"*) exit 0 ;;
esac

{
  printf '\n## %s\n\n' "$(date '+%Y-%m-%d %H:%M:%S %Z')"
  printf '%s\n' "$prompt"
} >> "$log"

exit 0
