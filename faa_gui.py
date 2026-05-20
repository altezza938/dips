"""
Extended FAA Rock Slope Kinematic Analysis — Desktop GUI
Based on GEO Technical Note TN 4/2024

Run:  python faa_gui.py
"""

import sys
import os
import numpy as np

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QSplitter,
    QVBoxLayout, QHBoxLayout, QGridLayout, QGroupBox,
    QLabel, QDoubleSpinBox, QSpinBox, QPushButton,
    QFileDialog, QCheckBox, QMessageBox, QProgressBar,
    QScrollArea, QFrame, QColorDialog, QTabWidget,
    QStatusBar, QAction, QToolBar, QSizePolicy,
)
from PyQt5.QtCore import Qt, QThread, pyqtSignal, QTimer
from PyQt5.QtGui import QFont, QIcon, QColor, QPalette

import matplotlib
matplotlib.use('Qt5Agg')
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

import faa_core
import faa_io


# ── colour constants ──────────────────────────────────────────────────────────
COL_BASE     = (0.75, 0.75, 0.75)   # grey – no failure
COL_SLIDING  = (1.0,  0.18, 0.18)   # red
COL_TOPPLING = (0.18, 0.80, 0.18)   # green
COL_WEDGE    = (0.18, 0.75, 1.0)    # cyan


# ── worker thread ─────────────────────────────────────────────────────────────
class AnalysisWorker(QThread):
    finished = pyqtSignal(str, object)   # mode, result
    error    = pyqtSignal(str)

    def __init__(self, mode, data_type, geometry, params):
        super().__init__()
        self.mode      = mode
        self.data_type = data_type
        self.geometry  = geometry
        self.params    = params

    def run(self):
        try:
            p = self.params
            if self.data_type == 'mesh':
                normals = self.geometry['normals']
            else:
                normals = self.geometry['normals']

            if self.mode == 'sliding':
                result = faa_core.sliding_check(
                    normals,
                    p['slope_dip'], p['slope_dip_dir'],
                    p['friction_angle'], p['lateral_sliding'])
                self.finished.emit('sliding', result)

            elif self.mode == 'toppling':
                result = faa_core.toppling_check(
                    normals,
                    p['slope_dip'], p['slope_dip_dir'],
                    p['friction_angle'], p['lateral_toppling'])
                self.finished.emit('toppling', result)

            elif self.mode == 'wedge':
                if self.data_type == 'mesh':
                    result = faa_core.wedge_check_triangular_mesh(
                        self.geometry['triangles'],
                        self.geometry['normals'],
                        self.geometry['vertices'],
                        p['slope_dip'], p['slope_dip_dir'],
                        p['friction_angle'], p['min_angle_diff'])
                else:
                    result = faa_core.wedge_check_point(
                        self.geometry['points'],
                        self.geometry['normals'],
                        p['slope_dip'], p['slope_dip_dir'],
                        p['friction_angle'], p['min_angle_diff'],
                        p['k_neighbours'])
                self.finished.emit('wedge', result)

        except Exception as exc:
            self.error.emit(str(exc))


# ── 3-D canvas ────────────────────────────────────────────────────────────────
class Canvas3D(FigureCanvas):
    def __init__(self, parent=None):
        self.fig = Figure(figsize=(8, 6), facecolor='#1a1a2e')
        super().__init__(self.fig)
        self.setParent(parent)
        self.ax = self.fig.add_subplot(111, projection='3d')
        self._style_ax()
        self._scatter = {}   # mode -> artist
        self._base    = None

    def _style_ax(self):
        self.ax.set_facecolor('#1a1a2e')
        self.ax.tick_params(colors='#aaaacc', labelsize=7)
        for pane in [self.ax.xaxis.pane, self.ax.yaxis.pane, self.ax.zaxis.pane]:
            pane.fill = False
            pane.set_edgecolor('#334')
        self.ax.grid(True, color='#334', linewidth=0.4)
        self.ax.set_xlabel('X (E)', color='#aaaacc', fontsize=8)
        self.ax.set_ylabel('Y (N)', color='#aaaacc', fontsize=8)
        self.ax.set_zlabel('Z', color='#aaaacc', fontsize=8)

    def clear_all(self):
        self.ax.cla()
        self._style_ax()
        self._scatter = {}
        self._base    = None
        self.draw_idle()

    def plot_base(self, data_type, geometry, max_pts=50_000):
        self.ax.cla()
        self._style_ax()
        self._scatter = {}

        if data_type == 'mesh':
            pts = geometry['vertices']
        else:
            pts = geometry['points']

        # Sub-sample for speed
        if len(pts) > max_pts:
            idx = np.random.choice(len(pts), max_pts, replace=False)
            pts_show = pts[idx]
        else:
            pts_show = pts

        self._base = self.ax.scatter(
            pts_show[:, 0], pts_show[:, 1], pts_show[:, 2],
            c=[COL_BASE], s=1.0, alpha=0.4, linewidths=0, label='Base')

        self._set_equal_aspect(pts)
        self.draw_idle()

    def plot_result(self, mode, data_type, geometry, result,
                    colour, visible=True, max_pts=30_000):
        # Remove previous result for this mode
        if mode in self._scatter:
            self._scatter[mode].remove()
            del self._scatter[mode]

        if result is None or not visible:
            self.draw_idle()
            return

        if mode in ('sliding', 'toppling'):
            # result is a boolean mask over normals / triangles
            if data_type == 'mesh':
                tris = geometry['triangles'][result]
                pts  = geometry['vertices'][tris].mean(axis=1)
            else:
                pts = geometry['points'][result]

        elif mode == 'wedge':
            if data_type == 'mesh':
                edge_idx, edge_verts = result
                if edge_idx is None:
                    self.draw_idle()
                    return
                pts = edge_verts
            else:
                if result is None:
                    self.draw_idle()
                    return
                pts = result

        if len(pts) == 0:
            self.draw_idle()
            return

        if len(pts) > max_pts:
            idx = np.random.choice(len(pts), max_pts, replace=False)
            pts = pts[idx]

        sc = self.ax.scatter(
            pts[:, 0], pts[:, 1], pts[:, 2],
            c=[colour], s=3.0, alpha=0.85, linewidths=0,
            label=mode.capitalize(), zorder=5)
        self._scatter[mode] = sc
        self.draw_idle()

    def set_visible(self, mode, visible):
        if mode in self._scatter:
            self._scatter[mode].set_visible(visible)
            self.draw_idle()

    def _set_equal_aspect(self, pts):
        mn, mx = pts.min(axis=0), pts.max(axis=0)
        rng = (mx - mn).max() / 2
        mid = (mn + mx) / 2
        self.ax.set_xlim(mid[0] - rng, mid[0] + rng)
        self.ax.set_ylim(mid[1] - rng, mid[1] + rng)
        self.ax.set_zlim(mid[2] - rng, mid[2] + rng)


# ── stereonet canvas ─────────────────────────────────────────────────────────
class StereoCanvas(FigureCanvas):
    def __init__(self, parent=None):
        self.fig = Figure(figsize=(4, 4), facecolor='#1a1a2e')
        super().__init__(self.fig)
        self.ax = self.fig.add_subplot(111, aspect='equal')
        self._style()

    def _style(self):
        self.ax.set_facecolor('#0d0d1a')
        self.ax.set_xlim(-1.1, 1.1)
        self.ax.set_ylim(-1.1, 1.1)
        self.ax.axis('off')
        # Outer circle
        circle = plt.Circle((0, 0), 1, fill=False, color='#aaaacc', lw=1)
        self.ax.add_patch(circle)
        # Cardinal labels
        for txt, xy in [('N', (0, 1.06)), ('S', (0, -1.10)),
                         ('E', (1.06, 0)), ('W', (-1.10, 0))]:
            self.ax.text(*xy, txt, ha='center', va='center',
                         fontsize=8, color='#aaaacc')

    def plot_poles(self, normals, slope_dip, slope_dip_dir,
                   friction_angle, lateral_sliding, lateral_toppling, min_angle_diff,
                   sliding_mask=None, toppling_mask=None):
        self.ax.cla()
        self._style()

        # Convert normals to poles
        n = normals.copy()
        n[n[:, 2] > 0] *= -1
        n_norm = np.linalg.norm(n, axis=1, keepdims=True)
        valid = (n_norm[:, 0] > 0)
        n[valid] /= n_norm[valid]

        pt = faa_core.vector_to_plunge_and_trend(n[valid])
        plunge_rad = np.radians(pt[:, 0])
        trend_rad  = np.radians(pt[:, 1])

        r = np.cos(plunge_rad) / (1 + np.sin(plunge_rad))
        x = r * np.sin(trend_rad)
        y = r * np.cos(trend_rad)

        colours = np.array([[*COL_BASE, 0.15]] * len(x))
        if sliding_mask is not None:
            colours[sliding_mask[valid]] = [*COL_SLIDING, 0.7]
        if toppling_mask is not None:
            colours[toppling_mask[valid]] = [*COL_TOPPLING, 0.7]

        self.ax.scatter(x, y, c=colours, s=2, linewidths=0)

        # Draw slope great circle
        self._draw_failure_envelopes(slope_dip, slope_dip_dir,
                                     friction_angle, lateral_sliding, lateral_toppling)

        self.ax.set_title('Lower Hemisphere\nStereographic Projection',
                          color='#aaaacc', fontsize=8, pad=4)
        self.draw_idle()

    def _draw_failure_envelopes(self, slope_dip, slope_dip_dir,
                                friction_angle, lat_slide, lat_topple):
        # Slope great circle
        dip_rad  = np.radians(slope_dip)
        dir_rad  = np.radians(slope_dip_dir)
        phi_rad  = np.radians(friction_angle)

        # Approximate failure envelope arcs as coloured fans
        alpha = np.linspace(0, 2 * np.pi, 360)
        r_slope = np.cos(dip_rad) / (1 + np.sin(dip_rad))
        x_c = r_slope * np.sin(dir_rad)
        y_c = r_slope * np.cos(dir_rad)
        self.ax.plot(x_c, y_c, 'w+', ms=6, lw=1)
        self.ax.annotate('Slope', (x_c, y_c), color='w', fontsize=7,
                         xytext=(5, 5), textcoords='offset points')


# ── main window ───────────────────────────────────────────────────────────────
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Extended FAA – Rock Slope Kinematic Analysis  |  GEO TN 4/2024')
        self.resize(1400, 840)

        self._data_type = None
        self._geometry  = None
        self._results   = {}      # mode -> result
        self._colours   = {
            'sliding':  list(COL_SLIDING),
            'toppling': list(COL_TOPPLING),
            'wedge':    list(COL_WEDGE),
        }
        self._worker = None

        self._build_ui()
        self._apply_dark_theme()

    # ── UI construction ───────────────────────────────────────────────────────
    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QHBoxLayout(central)
        main_layout.setContentsMargins(6, 6, 6, 6)
        main_layout.setSpacing(6)

        # ── Left: viewer tabs ──
        tabs = QTabWidget()
        tabs.setMinimumWidth(700)

        self.canvas3d = Canvas3D()
        nav3d = NavigationToolbar(self.canvas3d, self)
        w3d = QWidget()
        lv3d = QVBoxLayout(w3d)
        lv3d.setContentsMargins(0, 0, 0, 0)
        lv3d.addWidget(nav3d)
        lv3d.addWidget(self.canvas3d)
        tabs.addTab(w3d, '3D View')

        self.stereo = StereoCanvas()
        nav_st = NavigationToolbar(self.stereo, self)
        w_st = QWidget()
        lv_st = QVBoxLayout(w_st)
        lv_st.setContentsMargins(0, 0, 0, 0)
        lv_st.addWidget(nav_st)
        lv_st.addWidget(self.stereo)
        tabs.addTab(w_st, 'Stereonet')

        main_layout.addWidget(tabs, stretch=3)

        # ── Right: control panel ──
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setMaximumWidth(380)
        scroll.setMinimumWidth(320)

        panel = QWidget()
        panel_layout = QVBoxLayout(panel)
        panel_layout.setContentsMargins(8, 8, 8, 8)
        panel_layout.setSpacing(10)

        panel_layout.addWidget(self._section_open_file())
        panel_layout.addWidget(self._section_parameters())
        panel_layout.addWidget(self._section_analysis())
        panel_layout.addWidget(self._section_results())
        panel_layout.addStretch()

        scroll.setWidget(panel)
        main_layout.addWidget(scroll, stretch=0)

        # ── Status bar ──
        self.status = QStatusBar()
        self.setStatusBar(self.status)
        self.status.showMessage('Ready — open a LiDAR or mesh file to begin.')

        self.progress = QProgressBar()
        self.progress.setMaximumWidth(180)
        self.progress.setVisible(False)
        self.status.addPermanentWidget(self.progress)

    # ── section: Open File ────────────────────────────────────────────────────
    def _section_open_file(self):
        grp = QGroupBox('1. Open File')
        layout = QVBoxLayout(grp)

        row = QHBoxLayout()
        self.btn_open = QPushButton('Select…')
        self.btn_open.clicked.connect(self._open_file)
        self.chk_show = QCheckBox('Show input model')
        self.chk_show.setChecked(True)
        self.chk_flip = QCheckBox('Flip normals (for wedge)')
        row.addWidget(self.btn_open)
        layout.addLayout(row)
        layout.addWidget(self.chk_show)
        layout.addWidget(self.chk_flip)

        self.lbl_file = QLabel('No file loaded')
        self.lbl_file.setWordWrap(True)
        self.lbl_file.setStyleSheet('color:#88aacc; font-size:10px;')
        layout.addWidget(self.lbl_file)
        return grp

    # ── section: Parameters ───────────────────────────────────────────────────
    def _section_parameters(self):
        grp = QGroupBox('2. Set Parameters')
        layout = QGridLayout(grp)
        layout.setColumnStretch(1, 1)

        def dbl(v, mn=0.0, mx=360.0, dec=1, step=1.0):
            sb = QDoubleSpinBox()
            sb.setRange(mn, mx)
            sb.setDecimals(dec)
            sb.setSingleStep(step)
            sb.setValue(v)
            return sb

        def integer(v, mn=1, mx=100):
            sb = QSpinBox()
            sb.setRange(mn, mx)
            sb.setValue(v)
            return sb

        row = 0
        layout.addWidget(QLabel("Slope Dip (°):"),      row, 0)
        self.sb_dip = dbl(70.0, 0, 90); layout.addWidget(self.sb_dip, row, 1); row+=1

        layout.addWidget(QLabel("Slope Dip Direction (°):"), row, 0)
        self.sb_dir = dbl(140.0, 0, 360); layout.addWidget(self.sb_dir, row, 1); row+=1

        fit_btn = QPushButton('Fit Plane from Data')
        fit_btn.setToolTip('Estimate slope orientation automatically from the loaded model')
        fit_btn.clicked.connect(self._fit_plane)
        layout.addWidget(fit_btn, row, 0, 1, 2); row+=1

        layout.addWidget(_hsep(), row, 0, 1, 2); row+=1

        layout.addWidget(QLabel("Friction Angle (°):"), row, 0)
        self.sb_phi = dbl(30.0, 0, 90); layout.addWidget(self.sb_phi, row, 1); row+=1

        layout.addWidget(QLabel("Lateral Limit – Sliding ±(°):"), row, 0)
        self.sb_lat_s = dbl(20.0, 0, 90); layout.addWidget(self.sb_lat_s, row, 1); row+=1

        layout.addWidget(QLabel("Lateral Limit – Toppling ±(°):"), row, 0)
        self.sb_lat_t = dbl(10.0, 0, 90); layout.addWidget(self.sb_lat_t, row, 1); row+=1

        layout.addWidget(QLabel("Min. angular diff. for wedge (°):"), row, 0)
        self.sb_ang = dbl(30.0, 0, 90); layout.addWidget(self.sb_ang, row, 1); row+=1

        layout.addWidget(QLabel("KNN for wedge (point cloud):"), row, 0)
        self.sb_k = integer(16); layout.addWidget(self.sb_k, row, 1); row+=1

        return grp

    # ── section: Run Analysis ─────────────────────────────────────────────────
    def _section_analysis(self):
        grp = QGroupBox('3. Run Analysis')
        layout = QHBoxLayout(grp)

        self.btn_slide = QPushButton('Sliding')
        self.btn_slide.setStyleSheet(f'background:{_rgb(COL_SLIDING)}; color:white; font-weight:bold;')
        self.btn_slide.clicked.connect(lambda: self._run('sliding'))

        self.btn_topple = QPushButton('Toppling')
        self.btn_topple.setStyleSheet(f'background:{_rgb(COL_TOPPLING)}; color:white; font-weight:bold;')
        self.btn_topple.clicked.connect(lambda: self._run('toppling'))

        self.btn_wedge = QPushButton('Wedge')
        self.btn_wedge.setStyleSheet(f'background:{_rgb(COL_WEDGE)}; color:white; font-weight:bold;')
        self.btn_wedge.clicked.connect(lambda: self._run('wedge'))

        self.btn_all = QPushButton('Run All')
        self.btn_all.clicked.connect(self._run_all)

        for b in [self.btn_slide, self.btn_topple, self.btn_wedge, self.btn_all]:
            layout.addWidget(b)

        return grp

    # ── section: View & Save Results ─────────────────────────────────────────
    def _section_results(self):
        grp = QGroupBox('4. View and Save Results')
        layout = QGridLayout(grp)

        self._result_rows = {}
        for r, (mode, col) in enumerate([
            ('sliding',  COL_SLIDING),
            ('toppling', COL_TOPPLING),
            ('wedge',    COL_WEDGE),
        ]):
            chk = QCheckBox(mode.capitalize())
            chk.setChecked(True)
            chk.stateChanged.connect(lambda st, m=mode: self._toggle_visibility(m, st))

            lbl = QLabel('—')
            lbl.setStyleSheet('color:#88aacc; font-size:10px;')

            btn_col = QPushButton()
            btn_col.setFixedSize(20, 20)
            btn_col.setStyleSheet(f'background:{_rgb(col)}; border:none; border-radius:3px;')
            btn_col.clicked.connect(lambda _, m=mode: self._pick_colour(m))

            btn_save = QPushButton('Save')
            btn_save.setFixedWidth(50)
            btn_save.clicked.connect(lambda _, m=mode: self._save_result(m))

            layout.addWidget(chk,     r, 0)
            layout.addWidget(btn_col, r, 1)
            layout.addWidget(lbl,     r, 2)
            layout.addWidget(btn_save, r, 3)
            self._result_rows[mode] = (chk, lbl, btn_col)

        return grp

    # ── actions ───────────────────────────────────────────────────────────────
    def _open_file(self):
        path, _ = QFileDialog.getOpenFileName(
            self, 'Open 3D File', '',
            'All supported (*.ply *.obj *.stl *.fbx *.off *.gltf *.glb '
            '*.xyz *.xyzn *.xyzrgb *.pcd *.pts);; All files (*)')
        if not path:
            return

        self.status.showMessage(f'Loading {os.path.basename(path)} …')
        self.progress.setVisible(True)
        self.progress.setRange(0, 0)
        QApplication.processEvents()

        try:
            data_type, geometry = faa_io.load_file(path)
            self._data_type = data_type
            self._geometry  = geometry
            self._results   = {}

            if self.chk_flip.isChecked():
                geometry['normals'] *= -1

            n_elem = (len(geometry['triangles']) if data_type == 'mesh'
                      else len(geometry['points']))
            self.lbl_file.setText(
                f'{os.path.basename(path)}\n'
                f'Type: {data_type}   |   Elements: {n_elem:,}')

            self.canvas3d.plot_base(data_type, geometry)
            self.status.showMessage(
                f'Loaded: {os.path.basename(path)}  ({n_elem:,} elements)')

        except Exception as exc:
            QMessageBox.critical(self, 'Load Error', str(exc))
            self.status.showMessage('Load failed.')
        finally:
            self.progress.setVisible(False)
            self.progress.setRange(0, 1)

    def _fit_plane(self):
        if self._geometry is None:
            QMessageBox.warning(self, 'No data', 'Please load a file first.')
            return
        pts = (self._geometry['vertices'] if self._data_type == 'mesh'
               else self._geometry['points'])
        dip, dip_dir = faa_core.fit_plane_to_points(pts)
        self.sb_dip.setValue(dip)
        self.sb_dir.setValue(dip_dir)
        self.status.showMessage(f'Fitted slope: dip {dip:.1f}° / dip dir {dip_dir:.1f}°')

    def _params(self):
        return {
            'slope_dip':     self.sb_dip.value(),
            'slope_dip_dir': self.sb_dir.value(),
            'friction_angle': self.sb_phi.value(),
            'lateral_sliding': self.sb_lat_s.value(),
            'lateral_toppling': self.sb_lat_t.value(),
            'min_angle_diff': self.sb_ang.value(),
            'k_neighbours':  self.sb_k.value(),
        }

    def _run(self, mode):
        if self._geometry is None:
            QMessageBox.warning(self, 'No data', 'Please load a file first.')
            return
        if self._worker and self._worker.isRunning():
            return

        self.status.showMessage(f'Running {mode} analysis…')
        self.progress.setVisible(True)
        self.progress.setRange(0, 0)
        for b in [self.btn_slide, self.btn_topple, self.btn_wedge, self.btn_all]:
            b.setEnabled(False)

        self._worker = AnalysisWorker(mode, self._data_type, self._geometry, self._params())
        self._worker.finished.connect(self._on_analysis_done)
        self._worker.error.connect(self._on_analysis_error)
        self._worker.start()

    def _run_all(self):
        for mode in ('sliding', 'toppling', 'wedge'):
            self._run(mode)

    def _on_analysis_done(self, mode, result):
        self._results[mode] = result

        # Count affected elements
        count = self._count_result(mode, result)
        chk, lbl, _ = self._result_rows[mode]
        lbl.setText(f'{count:,} pts' if count else '0 — stable')

        # Update 3D view
        self.canvas3d.plot_result(
            mode, self._data_type, self._geometry, result,
            self._colours[mode], visible=chk.isChecked())

        # Update stereonet for sliding/toppling
        self._refresh_stereonet()

        self.progress.setVisible(False)
        self.progress.setRange(0, 1)
        for b in [self.btn_slide, self.btn_topple, self.btn_wedge, self.btn_all]:
            b.setEnabled(True)
        self.status.showMessage(
            f'{mode.capitalize()} analysis done — {count:,} potentially unstable elements.')

    def _on_analysis_error(self, msg):
        QMessageBox.critical(self, 'Analysis Error', msg)
        self.progress.setVisible(False)
        self.progress.setRange(0, 1)
        for b in [self.btn_slide, self.btn_topple, self.btn_wedge, self.btn_all]:
            b.setEnabled(True)
        self.status.showMessage('Analysis failed.')

    def _count_result(self, mode, result):
        if result is None:
            return 0
        if mode in ('sliding', 'toppling'):
            return int(result.sum()) if hasattr(result, 'sum') else 0
        elif mode == 'wedge':
            if self._data_type == 'mesh':
                idx, _ = result
                return len(idx) if idx is not None else 0
            else:
                return len(result) if result is not None else 0
        return 0

    def _refresh_stereonet(self):
        if self._geometry is None:
            return
        normals = self._geometry['normals']
        p = self._params()
        self.stereo.plot_poles(
            normals,
            p['slope_dip'], p['slope_dip_dir'],
            p['friction_angle'], p['lateral_sliding'],
            p['lateral_toppling'], p['min_angle_diff'],
            sliding_mask=self._results.get('sliding'),
            toppling_mask=self._results.get('toppling'),
        )

    def _toggle_visibility(self, mode, state):
        self.canvas3d.set_visible(mode, bool(state))

    def _pick_colour(self, mode):
        init = QColor(*[int(c * 255) for c in self._colours[mode]])
        col = QColorDialog.getColor(init, self)
        if col.isValid():
            self._colours[mode] = [col.redF(), col.greenF(), col.blueF()]
            chk, lbl, btn_col = self._result_rows[mode]
            btn_col.setStyleSheet(
                f'background:rgb({col.red()},{col.green()},{col.blue()});'
                f'border:none; border-radius:3px;')
            if mode in self._results:
                self.canvas3d.plot_result(
                    mode, self._data_type, self._geometry,
                    self._results[mode], self._colours[mode],
                    visible=chk.isChecked())

    def _save_result(self, mode):
        if mode not in self._results or self._results[mode] is None:
            QMessageBox.information(self, 'No result', f'No {mode} result to save.')
            return

        path, _ = QFileDialog.getSaveFileName(
            self, f'Save {mode.capitalize()} Result', f'{mode}_result.ply',
            'PLY (*.ply);;CSV (*.csv)')
        if not path:
            return

        try:
            result = self._results[mode]
            if mode in ('sliding', 'toppling'):
                if self._data_type == 'mesh':
                    tris = self._geometry['triangles'][result]
                    pts  = self._geometry['vertices'][tris].mean(axis=1)
                else:
                    pts = self._geometry['points'][result]
            else:
                if self._data_type == 'mesh':
                    _, verts = result
                    pts = verts if verts is not None else np.empty((0, 3))
                else:
                    pts = result if result is not None else np.empty((0, 3))

            if path.endswith('.csv'):
                np.savetxt(path, pts, delimiter=',', header='x,y,z', comments='')
            else:
                cols = np.tile(self._colours[mode], (len(pts), 1))
                faa_io.save_point_cloud(path, pts, cols)
            self.status.showMessage(f'Saved {mode} result → {path}')
        except Exception as exc:
            QMessageBox.critical(self, 'Save Error', str(exc))

    # ── theme ─────────────────────────────────────────────────────────────────
    def _apply_dark_theme(self):
        self.setStyleSheet("""
            QMainWindow, QWidget { background:#1a1a2e; color:#ccccee; }
            QGroupBox {
                border:1px solid #334466; border-radius:6px;
                margin-top:14px; font-weight:bold; color:#aaaaff;
            }
            QGroupBox::title { subcontrol-origin:margin; left:10px; padding:0 4px; }
            QPushButton {
                background:#2a2a4e; color:#ccccee; border:1px solid #445;
                border-radius:4px; padding:4px 8px;
            }
            QPushButton:hover  { background:#3a3a6e; }
            QPushButton:pressed{ background:#1a1a3e; }
            QDoubleSpinBox, QSpinBox {
                background:#12122a; color:#ccccee; border:1px solid #334;
                border-radius:3px; padding:2px;
            }
            QCheckBox { color:#ccccee; }
            QLabel     { color:#ccccee; }
            QScrollArea, QScrollBar { background:#1a1a2e; }
            QTabWidget::pane { border:1px solid #334466; }
            QTabBar::tab {
                background:#12122a; color:#aaaacc; padding:6px 14px;
                border-top-left-radius:4px; border-top-right-radius:4px;
            }
            QTabBar::tab:selected { background:#2a2a4e; color:white; }
            QStatusBar { background:#0d0d1a; color:#88aacc; }
        """)


# ── helpers ───────────────────────────────────────────────────────────────────
def _rgb(col):
    r, g, b = [int(c * 255) for c in col]
    return f'rgb({r},{g},{b})'


def _hsep():
    f = QFrame()
    f.setFrameShape(QFrame.HLine)
    f.setStyleSheet('color:#334466;')
    return f


# ── entry point ───────────────────────────────────────────────────────────────
def main():
    app = QApplication(sys.argv)
    app.setApplicationName('FAA Rock Slope Analysis')
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
