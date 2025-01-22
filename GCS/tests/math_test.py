import pytest
from GCS.src.math_functions import Math


def test_add():
    test = Math()
    assert test.add(2, 3) == 5
    assert test.add(-1, 1) == 0
    assert test.add(0, 0) == 0
