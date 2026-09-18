#!/usr/bin/env python3
"""Best-effort crop budget; this does not guarantee the requested FPS."""
from focused_depth import main

if __name__ == "__main__":
    main(defaultMode="detector", budget=True)
