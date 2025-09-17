#!/bin/bash

# Set up a Python virtual environment
python3 -m venv venv
source venv/bin/activate

# Install required Python packages
pip install hil-framework --upgrade --index-url "https://__token__:$PAT_TOKEN@gitlab.luxonis.com/api/v4/projects/213/packages/pypi/simple" > /dev/null 2>&1
