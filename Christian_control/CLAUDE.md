# CLAUDE.md — Christian_control

Read and follow `AGENTS.md` in this directory: it holds the durable rules
(project purpose, architecture constraints, safety, coding rules, build
command). Detailed documentation lives in `docs/`. Do not append lessons or
change history to either file — see "Keeping these files short" in AGENTS.md.

## Working style

- Keep changes minimal: no speculative features, abstractions, or defensive
  code beyond what the task requires (full rules: "Scope discipline" in
  AGENTS.md).
- Delegate independent subtasks to subagents and keep working while they run.
  Intervene if a subagent goes off track or is missing relevant context.
- If Christian's instruction is unclear or ambiguous, don't assume: gather
  context first, and ask — ideally by stating your interpretation ("I read
  this as X, so I'll do Y — correct me if wrong") so he can confirm or fix
  it. Questions ("if I want to…", "how would…") want explanation, not
  implementation.

## Final summaries

Terse shorthand is fine between tool calls (that's thinking out loud). The
final summary is different: it's for a reader who didn't see any of that.

- After working a while without Christian watching (overnight, many tool
  calls, since he last spoke), the final message is his first look at any of
  it. Write it as a re-grounding, not a continuation of the working thread:
  outcome first, then the one or two things needed from him, each explained
  as if new.
- Drop the working shorthand: complete sentences, spell out terms, no arrow
  chains, hyphen-stacked compounds, or labels invented mid-session. The
  vocabulary built up while working stays behind unless re-introduced.
- Mention files, commits, flags, and other identifiers each in their own
  plain-language clause.
- Open with one sentence on what happened or what was found, then the
  supporting detail. If it's a choice between short and clear, choose clear.
