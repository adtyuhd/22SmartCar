import sys


COLORS = {
    "SUCCESS": "\033[32m",
    "FAILED": "\033[31m",
    "RETRY": "\033[33m",
    "SKIPPED": "\033[34m",
    "TIMEOUT": "\033[31m",
}

RESET = "\033[0m"


def log(message, status=None):
    if status in COLORS and sys.stdout.isatty():
        print(f"{COLORS[status]}{message}{RESET}")
    else:
        print(message)