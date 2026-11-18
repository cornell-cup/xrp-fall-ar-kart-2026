#!/bin/bash
# Written by NetworkReceiver.gd on every launch; edits here are overwritten
export PYTHONUNBUFFERED=1
cd '/Users/hilarioesparza/Desktop/Cornell/CUP/xrp-fall-ar-kart-2026/xrpfall2025/'
echo '=== starting fusion: single-file build ===' | tee -a '/Users/hilarioesparza/Desktop/Cornell/CUP/xrp-fall-ar-kart-2026/xrpfall2025/fusion.log'
'/Users/hilarioesparza/Desktop/Cornell/CUP/xrp-fall-ar-kart-2026/xrpfall2025/laptop_fusion' '192.168.0.138' 2>&1 | tee -a '/Users/hilarioesparza/Desktop/Cornell/CUP/xrp-fall-ar-kart-2026/xrpfall2025/fusion.log'
echo '=== fusion exited ===' | tee -a '/Users/hilarioesparza/Desktop/Cornell/CUP/xrp-fall-ar-kart-2026/xrpfall2025/fusion.log'
