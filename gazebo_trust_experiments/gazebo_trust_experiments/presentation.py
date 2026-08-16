from __future__ import annotations

import sys

from .runner import main as runner_main


def main() -> None:
    # Presentation runs always use the GUI. The runner always selects OGRE for GUI rendering.
    sys.argv.extend(['--headless', 'false'])
    runner_main()
