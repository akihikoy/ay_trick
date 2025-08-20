# src/ay_trick/core_tool.py
# Thin loader: load the real module from scripts/core_tool.py
import os
import sys
import importlib.util

# Import ../../scripts/core_tool.py
_pkg_dir = os.path.dirname(__file__)
_root    = os.path.dirname(_pkg_dir)
_scripts = os.path.join(os.path.dirname(_root), 'scripts')
_core_py = os.path.realpath(os.path.join(_scripts, 'core_tool.py'))

_spec = importlib.util.spec_from_file_location(__name__, _core_py)
_mod  = importlib.util.module_from_spec(_spec)

sys.modules[__name__] = _mod
_spec.loader.exec_module(_mod)

# Add ../../scripts to the path.
sys.path.insert(0, _scripts)

try:
  __all__ = _mod.__all__
except Exception:
  pass
