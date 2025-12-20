import sys
import os

# Add the backend/src directory to the Python path so imports work correctly
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

# Import the necessary modules to ensure they're available to tests
import asyncio
import pytest
from unittest.mock import AsyncMock


# This is a workaround for pytest-asyncio issues
@pytest.fixture(scope="session")
def event_loop():
    """Create an instance of the default event loop for the test session."""
    loop = asyncio.get_event_loop_policy().new_event_loop()
    yield loop
    loop.close()