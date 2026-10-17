# xrpfall2025

This repo contains the code to the racing game that will be integrated with an XRP robot for an AR experience.

### Start: September 9, 2025

### People
- Camille Lach
- Max Horton
- Jamie Zheng

### Art Credit
- Road, Rocks, Finish line, Traffic Light, Wooden and Metal Rails, Grass, and Barrels: Jake Adams
- UI Elements: Camille Lach
- Trees: https://www.fab.com/listings/053c6fcb-e8cf-403d-b596-b30635a574db
- City Models: https://sketchfab.com/3d-models/low-poly-accommodations-buildings-ab0ebe012b0c47af9d7cf2443c979715#download

### Technical
- Engine: Godot
- Languages: C# and GDScript

### How to Set Up Godot 

# Python Environment(if you don't already have one):
```bash
conda create -n ar-kart python=3.11 -y
conda activate ar-kart
```

# Install Packages
```bash
pip install numpy opencv-python-headless pupil-apriltags pyinstaller pygame
python -c "import cv2, numpy, pupil_apriltags, PyInstaller; print('OK')"
```

# generate and move script
```bash
cd xrpfall2025/Scripts
pyinstaller --onefile --clean --collect-all pupil_apriltags --name laptop_fusion laptop_fusion.py
cd ..
mv Scripts/dist/laptop_fusion . 
```
