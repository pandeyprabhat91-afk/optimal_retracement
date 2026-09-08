def test_run_replay_importable():
    from replay_retrace import run_replay

    assert callable(run_replay)
