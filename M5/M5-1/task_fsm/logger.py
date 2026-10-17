import sys


COLORS = {
    "IDLE": "\033[90m",
    "STATE": "\033[94m",
    "RUNNING": "\033[36m",
    "SUCCESS": "\033[32m",
    "FAILED": "\033[31m",
    "SKIPPED": "\033[34m",
    "RETRYING": "\033[33m",
    "RETRY": "\033[33m",
    "TIMEOUT": "\033[91m",
    "DEGRADED": "\033[95m",
    "CONTEXT": "\033[96m",
    "INFO": "\033[37m",
}

RESET = "\033[0m"


def log(message, status=None):
    """仅在交互式终端中使用 ANSI 颜色输出日志。"""
    color = COLORS.get(status)

    if color and sys.stdout.isatty():
        print(f"{color}{message}{RESET}")
    else:
        print(message)