import json
import sys
from pathlib import Path

import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))


@pytest.fixture(scope='session')
def vectors() -> dict:
    return json.loads((Path(__file__).parent / 'vectors.json').read_text())
