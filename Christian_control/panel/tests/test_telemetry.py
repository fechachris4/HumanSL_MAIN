from Christian_control.panel.telemetry import Source


def test_live_source_ignores_existing_rows_and_emits_a_later_append(tmp_path):
    """The live panel must not replay an already-running controller log."""
    path = tmp_path / "loop_log_right.csv"
    path.write_text("time_s,value\n1.0,existing\n2.0,part")

    source = Source("right", path=path, mode="live", poll_interval_s=0.0)
    frames = source.frames()

    kind, payload = next(frames)

    assert kind == "heartbeat"
    assert source.rows_seen == 0

    with path.open("a") as handle:
        handle.write("ial\n3.0,appended\n")

    kind, payload = next(frames)

    assert kind == "telemetry"
    assert payload["row"]["time_s"] == 3.0
    assert payload["row"]["value"] == "appended"
    assert payload["rows_seen"] == 1
