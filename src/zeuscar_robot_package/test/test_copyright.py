# Copyright 2026 Murasan201
#
# Licensed under the MIT License

import pytest
from ament_copyright.main import main


@pytest.mark.copyright
@pytest.mark.linter
def test_copyright():
    """Test copyright compliance."""
    rc = main(argv=['.', 'test'])
    assert rc == 0, 'Found errors'
