# Lean-Code Multi-Agent Workflow Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make every future production-code change pass a read-only minimality review that removes unjustified protective code, test bloat and replacement leftovers.

**Architecture:** Extend the existing `AGENTS.md` working protocol instead of adding tooling or another workflow document. One coordinator scopes the change, one implementer writes, one minimality reviewer returns a constrained verdict, and the same implementer performs any cleanup.

**Tech Stack:** Markdown repository instructions and Git diff inspection.

**Spec:** `docs/superpowers/specs/2026-08-22-lean-code-multi-agent-workflow-design.md`

## Global Constraints

- Apply the gate to every production-code change; documentation-only changes remain lightweight.
- Exactly one agent writes at a time; the minimality reviewer is read-only.
- The reviewer may request deletion or consolidation but may not demand new guards, abstractions, compatibility code or tests.
- Keep robot-safety analysis separate; existing binding safety requirements remain unchanged.
- Add no workflow program, manager, service, registry, generated ledger or review framework.
- Preserve all unrelated working-tree changes.
- Do not commit without Christian's explicit authorization.

---

### Task 1: Install the lean-code gate in the repository instructions

**Files:**
- Modify: `AGENTS.md`, under `## Working protocol`

**Interfaces:**
- Consumes: the approved workflow design and the existing working protocol.
- Produces: standing instructions inherited by future implementation agents.

- [ ] **Step 1: Confirm the edit boundary**

Read `AGENTS.md` and confirm that the only intended edit is the existing
`### Code and test discipline` subsection. Check `git diff -- AGENTS.md` so the
previously approved uncommitted bullets are preserved through replacement.

- [ ] **Step 2: Replace the brief discipline subsection with the approved gate**

The replacement must state, concisely:

1. Every production-code change uses coordinator, sole implementer and
   read-only minimality-reviewer roles.
2. Before editing, the coordinator records requested behaviour, non-goals,
   expected files, the old path expected to disappear and proportionate
   verification.
3. The reviewer returns only `CLEAN` or `CLEANUP REQUIRED` with cited findings
   for speculative handling, repeated validation, unnecessary branches/files/
   abstractions, compatibility layers, redundant tests or replacement residue.
4. The reviewer may request deletion or consolidation, never additional
   protective machinery.
5. The same implementer performs one cleanup pass and the reviewer rechecks
   only its findings; unresolved evidence gaps return to Christian.
6. Tests require requested behaviour, a reproduced regression or an existing
   binding invariant; replacements remove superseded tests and call paths.
7. Completion reports concept/test additions and removals, old-path status,
   the minimality verdict, exact verification and whether hardware ran.

Do not duplicate the surrounding inspection/implementation protocol or the
separate robotics safety workflow.

- [ ] **Step 3: Verify instruction quality**

Run:

```bash
git diff --check -- AGENTS.md
rg -n "Coordinator|implementer|minimality reviewer|CLEANUP REQUIRED|protective|Old path|Hardware executed" AGENTS.md
git diff -- AGENTS.md
```

Expected: no whitespace errors; every required gate term appears; the diff is
limited to the working-protocol subsection and contains no new framework,
script or production-code change.

- [ ] **Step 4: Apply independent minimality review**

Give a read-only reviewer Christian's request, the approved spec and the
complete `AGENTS.md` diff. The reviewer must return `CLEAN` or cited
`CLEANUP REQUIRED` findings and must not propose additional process machinery.
If cleanup is required, return only confirmed findings to the same implementer,
then recheck those findings once.

- [ ] **Step 5: Report without committing**

Report the exact file changed, verification output, review verdict and that no
production code, tests, safety workflow, hardware command or commit was
involved.
