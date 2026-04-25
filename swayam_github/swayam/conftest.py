"""pytest configuration for Swayam tests."""
import sys
import os

# Make sure src/ is always on the path when running pytest from project root
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "src"))
