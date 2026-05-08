# conftest.py — adds src/ and each subpackage to sys.path
import sys
import os

# make sure the repo root is on path so `src.swayam.*` resolves
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

# also expose flat names for backward compat (so we don't have to rewrite every test)
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src", "swayam", "core"))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src", "swayam", "control"))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src", "swayam", "comms"))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src", "swayam", "hardware"))
