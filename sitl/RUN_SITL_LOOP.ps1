# Autonomous SITL retrace loop — single entry point. Run from repo root.
# Usage:
#   .\sitl\RUN_SITL_LOOP.ps1                     # full loop: params -> fly -> replay -> score
#   .\sitl\RUN_SITL_LOOP.ps1 -From replay -Bag flight_sitl6   # resume from stage
# Requires: docker container gpsdnav up, gz+PX4 running, .venv present (see README).
param([string]$From = "params", [string]$Bag = "flight_sitl_loop")
& "$PSScriptRoot\..\.venv\Scripts\python.exe" -X utf8 -u "$PSScriptRoot\..\scripts\retrace\sitl_loop.py" --from $From --bag $Bag
