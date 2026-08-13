#!/usr/bin/env bash
# Shared staleness arithmetic for the intent hooks.
#
# One definition of "stale", used by both intent-status.sh (session start)
# and intent-remind.sh (session end), so the two can never disagree about
# what they are reporting. Like append-prompt.sh this stays dumb: it counts
# and compares, it never interprets.
set -u

intent_repo_root() {
    cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd
}

INTENT_ROOT=$(intent_repo_root)
INTENT_LOG="$INTENT_ROOT/docs/intent/raw-prompt-log.md"
INTENT_STORY="$INTENT_ROOT/docs/intent/story.md"

# Epoch seconds of story.md's last commit, or 0 if it has never been
# committed. Commit time rather than file mtime on purpose: the design makes
# git history the audit trail, so a story edit only counts once it is
# recorded there.
intent_story_commit_epoch() {
    local epoch
    epoch=$(git -C "$INTENT_ROOT" log -1 --format=%at -- "$INTENT_STORY" 2>/dev/null)
    [ -n "$epoch" ] && printf '%s' "$epoch" || printf '0'
}

# Human-readable date of that commit, for the reminder text.
intent_story_commit_date() {
    local date
    date=$(git -C "$INTENT_ROOT" log -1 --format=%ad --date=short -- "$INTENT_STORY" 2>/dev/null)
    [ -n "$date" ] && printf '%s' "$date" || printf 'never'
}

# True when story.md has uncommitted edits. Such a story is being worked on
# right now, so neither hook should nag about it.
intent_story_dirty() {
    [ -n "$(git -C "$INTENT_ROOT" status --porcelain -- "$INTENT_STORY" 2>/dev/null)" ]
}

# How many captured prompts postdate that commit.
#
# The log is chronological, so this walks it backwards and stops at the
# first entry old enough to predate the story. That keeps the cost
# proportional to what is new rather than to the whole log, which only ever
# grows.
intent_stale_count() {
    local story_epoch count line stamp entry_epoch
    story_epoch=$(intent_story_commit_epoch)
    count=0
    [ -r "$INTENT_LOG" ] || { printf '0'; return; }

    while IFS= read -r line; do
        stamp=${line#\#\# }
        entry_epoch=$(date -d "$stamp" +%s 2>/dev/null) || continue
        [ -n "$entry_epoch" ] || continue
        [ "$entry_epoch" -le "$story_epoch" ] && break
        count=$((count + 1))
    done < <(grep '^## 20' "$INTENT_LOG" 2>/dev/null | tac)

    printf '%s' "$count"
}
