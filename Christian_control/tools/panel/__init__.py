"""Control panel package — the browser front-end's Python side.

One file per job (mirrors how src/ is organised):

    paths.py         where things are on disk
    config_file.py   whitelisted Config.h read/write, plus read-only constants
    build.py         cmake invocation and binary-vs-source freshness
    session.py       start, stop, status and re-attach for a supervised run
    telemetry.py     tail and parse the per-arm CSV; replay a recorded run
    plan.py          goal.yaml, solving via planner_bridge, validation report
    runs.py          run history and per-run summaries
    server.py        HTTP routing, static files, SSE. Nothing else.

The rule the whole package serves: the browser observes and commands; the
controller decides whether motion is safe. Nothing here re-implements a
safety decision — where a check already exists (run_session.sh's freshness
gate, ProcessLock, the controller's stop path) this package mirrors or
triggers it rather than replacing it.
"""
