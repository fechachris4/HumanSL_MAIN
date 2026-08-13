#!/usr/bin/env bash
# SessionStart: say how far the interpretation layer has fallen behind
# ground truth, before any work starts.
#
# This exists because the CLAUDE.md instruction to read story.md at the
# start of substantive work is model behaviour, and on 2026-08-13 the model
# simply skipped it with nothing to catch the omission. A number injected
# into the session cannot be skipped the same way.
#
# It reports. It never blocks, and it never edits the story.
set -u

source "$(dirname "$0")/intent-common.sh"

stale=$(intent_stale_count)
[ "$stale" -eq 0 ] && exit 0

last=$(intent_story_commit_date)

if [ "$(intent_story_commit_epoch)" -eq 0 ]; then
    printf 'intent: story.md has never been committed, and %s prompt(s) are captured. Until it is in git there is no audit trail to diff against.\n' \
        "$stale"
elif intent_story_dirty; then
    printf 'intent: story.md has uncommitted edits covering %s new prompt(s) (last commit %s). Commit it separately from code.\n' \
        "$stale" "$last"
else
    printf 'intent: story.md is %s prompt(s) stale (last updated %s). Read docs/intent/story.md before substantive work; /intent-sync folds the new prompts in.\n' \
        "$stale" "$last"
fi

exit 0
