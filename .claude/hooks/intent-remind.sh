#!/usr/bin/env bash
# Stop: if the session captured prompts but left the story untouched, say
# so on the way out.
#
# The design asks for Christian to review DIFFS of the story at session
# boundaries. Whether that review happens cannot be left to the model
# remembering, so the boundary itself asks. Silent when the story is
# current or already being edited, and never blocks the session ending —
# an unreviewed story is a debt to note, not an error to enforce.
set -u

source "$(dirname "$0")/intent-common.sh"

# Already edited this session: the debt is being paid, so stay quiet.
intent_story_dirty && exit 0

stale=$(intent_stale_count)
[ "$stale" -eq 0 ] && exit 0

last=$(intent_story_commit_date)

printf 'intent: %s prompt(s) captured since story.md was last updated (%s); the story was not touched this session.\n' \
    "$stale" "$last"
printf '  fold them in with /intent-sync, or check the record against ground truth with /intent-redteam\n'
printf '  review what changed:  git diff docs/intent/story.md\n'

exit 0
