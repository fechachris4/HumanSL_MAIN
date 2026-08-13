#!/usr/bin/env bash
# PreToolUse(AskUserQuestion): fire at the exact moment a prediction is
# supposed to be written — just before Christian is asked something.
#
# The design tests understanding by prediction: write down the expected
# answer BEFORE asking, so a mismatch becomes a calibration signal rather
# than a forgotten surprise. Writing the prediction is irreducibly model
# behaviour and no hook can perform it. What a hook can do is make the
# moment impossible to pass through unnoticed, which is all this does.
#
# Deliberately unconditional: a nudge that decides for itself when it is
# needed is interpretation, and interpretation drifts. This one is dumb.
set -u

read -r -d '' note <<'NOTE' || true
Before asking: log your prediction in docs/intent/predictions.md — the question, what you expect Christian to choose, and why. Record his actual answer afterwards and mark it a hit or a miss. A prediction written after seeing the answer is worthless.
NOTE

# Documented PreToolUse channel for feeding text back to the model without
# touching the permission decision. Plain stdout is the fallback if a
# harness version does not parse it.
if command -v jq >/dev/null 2>&1; then
    jq -n --arg note "$note" \
        '{hookSpecificOutput: {hookEventName: "PreToolUse", additionalContext: $note}}'
else
    printf '%s\n' "$note"
fi

exit 0
