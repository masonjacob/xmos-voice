#!/bin/bash
# Basic entrypoint to change default terminal directory

# Add cd to .bashrc:
echo "cd /opt/xmos" >> ~/.bashrc


# Execute the command passed into this entrypoint
exec $@
