#!/usr/bin/env python

import os
import subprocess

# Path to your shell script
script_path = os.path.join(os.path.dirname(__file__), 'bringup_xbot_dummy.sh')

# Execute the shell script
subprocess.call(['bash', script_path])
