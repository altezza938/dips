==============================================================================
 FAA Rock Slope Kinematic Analysis  —  Test Package
 Extended Facet Amalgamation Approach (GEO Technical Note TN 4/2024)
==============================================================================

This package contains TWO ways to run the tool:

  A) A single-file WEB APP  (no install — just open in a browser)
  B) A PYTHON DESKTOP APP   (handles large datasets, full mesh analysis)

Both implement the same three kinematic checks: planar SLIDING, flexural
TOPPLING, and WEDGE failure. Results are shown in a 3-D view and on a
lower-hemisphere stereonet.

------------------------------------------------------------------------------
 QUICK START — easiest option (Web app)
------------------------------------------------------------------------------
1. Double-click  FAA_RockSlope.html  (opens in Chrome / Edge / Firefox).
   Needs an internet connection the first time (it loads the 3-D library
   from a CDN). No installation, no server.
2. Drag  sample_slope.xyzn  onto the drop zone (or click to browse).
3. Enter the suggested parameters (see below) and click "Run All".
4. Switch to the "Stereonet" tab to see sliding (red) and toppling (green)
   poles plotted on the net.

Supported web formats: LAS, PLY, OBJ, XYZ, XYZN, PTS.
(LAZ is NOT supported in the browser — convert to LAS first.)

------------------------------------------------------------------------------
 FILE-BY-FILE GUIDE
------------------------------------------------------------------------------
FAA_RockSlope.html
    The complete web app in ONE file. All styling and code (3-D viewer,
    stereonet, FAA algorithms, file loaders) are inlined. Share this single
    file with anyone — they just open it in a browser.

sample_slope.xyzn
    Ready-to-test synthetic rock slope (~5,100 points) with per-point
    normals already included. Contains a main slope face plus three joint
    sets engineered to produce clear sliding, toppling and wedge results.
    Plain text: each line is  "x y z nx ny nz".

generate_sample_xyzn.py
    Regenerates sample_slope.xyzn. Pure Python, NO libraries required.
    Run:  python generate_sample_xyzn.py

----- Python desktop app (option B) ------------------------------------------
faa_gui.py
    The desktop application's main window and user interface. This is the
    file you run to launch the desktop app.
        Run:  python faa_gui.py

faa_core.py
    The FAA algorithms (sliding / toppling / wedge checks and the vector /
    stereonet maths). Directly ported from TN 4/2024, Appendix A. Shared
    "engine" used by the desktop GUI.

faa_io.py
    File loading for the desktop app — reads point clouds and triangle
    meshes (PLY, OBJ, STL, XYZ, PCD, etc.) and estimates normals when a
    file has none.

generate_sample.py
    Generates a richer sample point cloud (sample_slope.ply) using open3d.
    Use this if you want a coloured PLY for the desktop app instead of the
    plain .xyzn file.

requirements.txt
    Python packages the desktop app needs.
        Install:  pip install -r requirements.txt

run.bat
    Windows convenience launcher for the desktop app (sets up and starts it).

README_PACKAGE.txt
    This file.

------------------------------------------------------------------------------
 PYTHON DESKTOP APP — setup
------------------------------------------------------------------------------
Requires Python 3.8 - 3.12.
    1.  pip install -r requirements.txt
    2.  python faa_gui.py
   (On Windows you can instead double-click run.bat.)

------------------------------------------------------------------------------
 SUGGESTED PARAMETERS for sample_slope.xyzn
------------------------------------------------------------------------------
    Slope Dip ................. 70
    Slope Dip Direction ....... 140
    Friction Angle ............ 30
    Lateral Limit (Sliding) ... 20
    Lateral Limit (Toppling) .. 20
    Min. Angular Diff (Wedge) . 30
    k Nearest Neighbours ...... 16

 With these values the sample yields roughly:
    ~1,500 sliding points,  ~700 toppling points,  ~650 wedge intersections.
==============================================================================
