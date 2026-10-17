import sys


COLORS = {
    "STATE": "\033[94m",
    "RUNNING": "\033[36m",
    "SUCCESS": "\033[32m",
    "FAILED": "\033[31m",
    "RETRYING": "\033[33m",
    "SKIPPED": "\033[34m",
    "TIMEOUT": "\033[91m",
    "DEGRADED": "\033[95m",
    "CONTEXT": "\033[96m",
    "INFO": "\033[37m",
}
RESET = "\033[0m"


def log(message, status=None):
    color = COLORS.get(status)
    if color and sys.stdout.isatty():
        print(f"{color}{message}{RESET}")
    else:
        print(message)
