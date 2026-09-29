#!/usr/bin/env python3
"""Process a fixed ROI count with one model, or budget automatically across multiple models."""
from focused_depth import main

if __name__ == "__main__":
    main(defaultMode="detector", budget=True)
