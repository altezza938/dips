# FAA Rock Slope Kinematic Analysis

Desktop + web app implementing the **Extended Facet Amalgamation Approach (FAA)**  
from GEO Technical Note TN 4/2024 — K.K. Chan, CEDD, HKSAR Government.

---

## Desktop App (Windows / macOS / Linux)

### Windows — quickest start

1. Install **Python 3.11** from [python.org](https://www.python.org/downloads/)  
   *(tick "Add Python to PATH" during install)*
2. Download or clone this repository
3. Double-click **`run.bat`**

`run.bat` automatically creates a virtual environment, installs all dependencies on first run, then launches the app. Subsequent launches skip the install step and open immediately.

> **Python version note:** open3d requires Python 3.8–3.12. Python 3.13 is not yet supported.

### macOS / Linux

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
python faa_gui.py
```

### Supported input formats

| Type | Formats |
|------|---------|
| Mesh | PLY, OBJ, STL, FBX, OFF, GLTF, GLB |
| Point cloud | PLY, XYZ, XYZN, XYZRGB, PCD, PTS |
| LiDAR (GeoSLAM etc.) | **LAS, LAZ** |

### Workflow

1. Open a LiDAR scan or mesh file
2. Set slope orientation (dip / dip direction) — or click **Fit Plane from Data**
3. Set geotechnical parameters (friction angle, lateral limits, KNN)
4. Click **Sliding**, **Toppling**, **Wedge**, or **Run All**
5. Review coloured results in 3-D view and stereonet
6. Export result points as PLY or CSV

---

## Web App

Live at **[altezza938.github.io/dips](https://altezza938.github.io/dips)**

- Drag-and-drop LAS, LAZ, PLY, OBJ, or XYZ files
- LAZ decompressed in-browser via WebAssembly (laz-perf)
- No installation required

---

## Dependencies

```
numpy>=1.24
scipy>=1.10
PyQt5>=5.15
matplotlib>=3.7
open3d>=0.17
laspy[lazrs]>=2.0
```

---

## Reference

Chan, K.K. (2024). *Digital Mapping and Kinematic Analysis of Rock Slopes by Extended Facet Amalgamation Approach (FAA)*. GEO Technical Note TN 4/2024, Planning and Development Division, Civil Engineering and Development Department, HKSAR Government.
