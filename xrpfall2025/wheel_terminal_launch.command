#!/bin/bash
# Written by NetworkReceiver.gd on every launch; edits here are overwritten
export PYTHONUNBUFFERED=1
cd '/Users/hilarioesparza/Desktop/Cornell/CUP/xrp-fall-ar-kart-2026/xrpfall2025/'
echo '=== starting wheel bridge ===' | tee -a '/Users/hilarioesparza/Desktop/Cornell/CUP/xrp-fall-ar-kart-2026/xrpfall2025/wheel_bridge.log'
'python3' '-u' '/Users/hilarioesparza/Desktop/Cornell/CUP/xrp-fall-ar-kart-2026/xrpfall2025/Scripts/wheel_bridge.py' 2>&1 | tee -a '/Users/hilarioesparza/Desktop/Cornell/CUP/xrp-fall-ar-kart-2026/xrpfall2025/wheel_bridge.log'
echo '=== wheel bridge exited ===' | tee -a '/Users/hilarioesparza/Desktop/Cornell/CUP/xrp-fall-ar-kart-2026/xrpfall2025/wheel_bridge.log'
