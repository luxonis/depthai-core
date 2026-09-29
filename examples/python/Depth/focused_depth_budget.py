#!/usr/bin/env python3
"""Process a requested number of available regions per frame, using one selected model."""
from focused_depth import main

if __name__ == "__main__":
    main(defaultMode="detector", budget=True)
