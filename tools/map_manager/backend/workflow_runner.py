from __future__ import annotations

import ctypes
import os
import signal
import sys
from pathlib import Path


def main() -> None:
    if len(sys.argv) != 2:
        raise SystemExit("usage: workflow_runner.py <script>")
    script = Path(sys.argv[1]).resolve()
    if not script.is_file():
        raise SystemExit(f"workflow script not found: {script}")

    libc = ctypes.CDLL("libc.so.6", use_errno=True)
    if libc.prctl(1, signal.SIGTERM) != 0:  # PR_SET_PDEATHSIG
        raise OSError(ctypes.get_errno(), "prctl(PR_SET_PDEATHSIG) failed")
    if os.getppid() == 1:
        raise SystemExit("workflow parent exited before startup")
    os.execv("/bin/bash", ["/bin/bash", str(script)])


if __name__ == "__main__":
    main()
