"""SEDSnet 6 network-stamp helpers for host fixtures."""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def header():
    """Core/Inc/network_stamp.h for fixtures that strip project includes."""
    return (ROOT / 'Core/Inc/network_stamp.h').read_text()
