"""Invalidate object files when Make command-line configuration changes."""
from pathlib import Path
import sys
path = Path(sys.argv[1])
content = '\n'.join(sys.argv[2:]) + '\n'
if not path.exists() or path.read_text() != content:
    path.write_text(content)
