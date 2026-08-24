# Lean-Code Multi-Agent Workflow Design

**Status:** Approved by Christian on 2026-08-22.

## Purpose

Prevent speculative protective code, duplicated validation, unnecessary
branching, compatibility scaffolding, abstractions and tests from accumulating
in the repository.

This is a code-minimality workflow, not a robot-safety workflow. Existing
binding safety requirements remain in force, but they do not justify new code
without a concrete requirement or hazard.

## Scope

Apply the workflow to every production-code change. Documentation-only edits
do not require the full loop.

Exactly one agent writes at a time. Review agents remain read-only. Independent
read-only investigation may run in parallel, but implementation does not.

## Roles

### Coordinator

The primary agent records the requested behaviour, explicit non-goals,
expected files, anything being replaced and the proportionate verification.
It controls scope, supplies the original request and complete diff to the
reviewer, resolves evidence-backed disagreements and reports unresolved intent
questions to Christian.

### Implementer

The implementer is the only writer. It makes the smallest complete change,
removes the superseded path and runs the agreed targeted hardware-free checks.
If review finds confirmed bloat, the same implementer performs the cleanup
pass.

### Minimality reviewer

The reviewer is read-only. It receives Christian's original request, the
declared non-goals and the complete diff. It reviews only for:

- speculative failure handling;
- repeated validation;
- unnecessary branches, helpers, files or abstractions;
- unrequested compatibility layers;
- redundant or implementation-coupled tests;
- production code, tests, configuration, includes, call paths or concepts
  left behind by a replacement.

The reviewer may recommend deletion or consolidation. It must not demand more
guards, abstractions, compatibility code or tests. Missing requested behaviour
is reported separately as a specification issue.

## Per-change flow

1. The coordinator records:
   - requested behaviour;
   - explicit non-goals;
   - expected files;
   - the old path expected to disappear, where applicable;
   - proportionate verification.
2. One implementer edits and self-reviews the complete diff.
3. The minimality reviewer returns exactly one verdict:
   - `CLEAN`; or
   - `CLEANUP REQUIRED`, with each finding citing the file and symbol and the
     code or concept to remove or consolidate.
4. The same implementer performs one focused cleanup pass.
5. The reviewer rechecks only its findings.
6. The coordinator runs the agreed verification and a scoped leftover search
   for replacements.

There are no parallel writers and no open-ended review rounds. A confirmed
residual finding blocks completion. A disputed finding is decided against the
requested behaviour and repository evidence; if those do not decide it,
Christian decides.

## Protective-code rule

Every new check, fallback or branch must trace to at least one of:

- behaviour Christian requested;
- a reproduced failure;
- an existing binding requirement;
- a concrete hazard with an identified owner and response.

"Just in case", imagined future reuse and agent caution are not
justifications. Calling code safety-related is not an exemption: the cited
requirement or hazard must already exist, or the decision returns to Christian.

## Replacement rule

A replacement includes removal of the former approach's production code,
tests, build registration, configuration, includes, call sites and dead
abstractions. A scoped repository search distinguishes active leftovers from
historical records. Any retained compatibility or old path requires an
explicit Christian-approved reason.

## Test discipline

Add a test only for requested observable behaviour, a reproduced regression or
an existing binding invariant. Prefer an existing test or characterization
check over a new one.

Do not add tests merely for imaginable inputs. Avoid tests coupled to private
helpers, source layout, exact internal call counts or incidental branching.
When replacing code, delete tests of the old behaviour and adapt only tests
that independently prove behaviour still required afterward.

Use targeted hardware-free checks for normal changes. Reserve broad suites for
cross-cutting changes and milestones. The reviewer does not request tests
automatically; every proposed test must name the real regression it would
catch and why existing evidence does not cover it.

## Completion report

Every production-code change ends with:

```text
Production concepts: +N / -N
Tests: +N / -N
Old path: removed / retained with Christian-approved reason
Minimality review: CLEAN
Verification: exact checks and results
Hardware executed: no / exact Christian-authorized command
```

## Implementation shape

Implement this workflow by updating `AGENTS.md` so future agents receive the
role boundaries, gate and completion conditions automatically. Do not add a
workflow program, manager, service, registry, generated ledger or additional
review framework.
