from datetime import datetime

import pytest

from depthai_pybind11_tests import chrono_operator_lookup


@pytest.mark.parametrize("microsecond", [0, 123456])
def test_system_clock_roundtrip_with_operator_bindings(microsecond):
    value = datetime(2026, 1, 15, 12, 34, 56, microsecond)
    assert chrono_operator_lookup.roundtrip(value) == value
