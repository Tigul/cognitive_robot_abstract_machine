"""
A launcher that keeps running until it is interrupted, as a component serving its
clients does.
"""

import signal

# %% running until interrupted

if __name__ == "__main__":
    signal.pause()
