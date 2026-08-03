from __future__ import annotations

import ctypes
import os
import signal
import sys


def main() -> None:
    if len(sys.argv) < 2:
        raise SystemExit("usage: process_runner.py <command> [args...]")
    libc = ctypes.CDLL("libc.so.6", use_errno=True)
    if libc.prctl(1, signal.SIGTERM) != 0:  # PR_SET_PDEATHSIG
        raise OSError(ctypes.get_errno(), "prctl(PR_SET_PDEATHSIG) failed")
    if os.getppid() == 1:
        raise SystemExit("process parent exited before startup")
    os.execvp(sys.argv[1], sys.argv[1:])


if __name__ == "__main__":
    main()
