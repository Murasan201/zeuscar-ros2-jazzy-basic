# Copyright 2026 Murasan201
#
# Licensed under the MIT License

import pytest
from ament_pep257.main import main


@pytest.mark.linter
def test_pep257():
    """Test pep257 docstring compliance."""
    rc = main(argv=['.', 'test'])
    assert rc == 0, 'Found code style errors / warnings'
