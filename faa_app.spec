# -*- mode: python ; coding: utf-8 -*-
"""
PyInstaller spec for FAA Rock Slope Analysis — standalone Windows .exe
Build:  pyinstaller faa_app.spec
Output: dist/FAA_RockSlope/FAA_RockSlope.exe  (one-folder bundle)
"""

from PyInstaller.utils.hooks import collect_all, collect_submodules

# Collect open3d (large; has native libs and shaders)
open3d_datas, open3d_binaries, open3d_hiddenimports = collect_all('open3d')

# scipy submodules not always auto-detected
scipy_hidden = collect_submodules('scipy')

a = Analysis(
    ['faa_gui.py'],
    pathex=['.'],
    binaries=open3d_binaries,
    datas=open3d_datas + [
        ('faa_core.py', '.'),
        ('faa_io.py',   '.'),
    ],
    hiddenimports=(
        open3d_hiddenimports
        + scipy_hidden
        + [
            'PyQt5',
            'PyQt5.QtCore',
            'PyQt5.QtGui',
            'PyQt5.QtWidgets',
            'PyQt5.sip',
            'matplotlib',
            'matplotlib.backends.backend_qt5agg',
            'matplotlib.backends.backend_agg',
            'matplotlib.figure',
            'laspy',
            'laspy.lasio',
            'lazrs',
        ]
    ),
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=['tkinter', 'test', 'unittest'],
    noarchive=False,
)

pyz = PYZ(a.pure, a.zipped_data)

exe = EXE(
    pyz,
    a.scripts,
    [],
    exclude_binaries=True,
    name='FAA_RockSlope',
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=False,
    console=False,          # no black CMD window
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
)

coll = COLLECT(
    exe,
    a.binaries,
    a.zipfiles,
    a.datas,
    strip=False,
    upx=False,
    upx_exclude=[],
    name='FAA_RockSlope',
)
