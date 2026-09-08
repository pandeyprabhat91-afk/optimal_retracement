def test_plots_importable():
    import plots

    assert hasattr(plots, "plan_view")
