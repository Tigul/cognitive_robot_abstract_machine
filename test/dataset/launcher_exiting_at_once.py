"""
A launcher that fails right away, as a component crashing during start-up does.
"""

import sys

# %% failing start-up

if __name__ == "__main__":
    sys.exit(1)
