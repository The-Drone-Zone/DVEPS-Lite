import pytest
from src import Math


def test_add_positive():
    test = Math()
    assert test.add(2, 3) == 5


def test_add_negative():
    test = Math()
    assert test.add(-1, 1) == 0


def test_add_zero():
    test = Math()
    assert test.add(0, 0) == 0
