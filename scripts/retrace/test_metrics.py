def test_compute_ate_zero_on_identical():
    import numpy as np
    from metrics import compute_ate

    a = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]])
    assert compute_ate(a, a) == 0.0
