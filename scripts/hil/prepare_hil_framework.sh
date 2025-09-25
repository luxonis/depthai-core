#!/bin/bash

# Check if PAT token argument is provided
if [ -z "$1" ]; then
  echo "Usage: $0 <PAT_TOKEN>"
  exit 1
fi

PAT_TOKEN=$1

# Set up a Python virtual environment
python3 -m venv venv
source venv/bin/activate

# Install required Python packages
pip install hil-framework --upgrade --no-cache-dir \
  --index-url "https://__token__:$PAT_TOKEN@gitlab.luxonis.com/api/v4/projects/213/packages/pypi/simple" \
  > /dev/null 2>&1
