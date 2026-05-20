/**
 * FAA Web App — Main Application
 * 3-D viewer powered by Three.js  |  Analysis by faa_core.js
 * LAS/LAZ parser + normal estimation by las_loader.js
 */

// ── Constants ─────────────────────────────────────────────────────────────────
const MAX_DISPLAY_PTS  = 300000;   // max points rendered in 3-D view
const MAX_ANALYSIS_PTS = 500000;   // max points used in FAA analysis (LAS sub-sampling)

// ── Three.js globals ──────────────────────────────────────────────────────────
let scene, camera, renderer, controls;
let baseCloud = null, slidingCloud = null, toppingCloud = null, wedgeCloud = null;

// ── App state ─────────────────────────────────────────────────────────────────
let pointPositions = null;   // Float32Array, flat xyz — used for analysis
let pointNormals   = null;   // Float32Array, flat xyz
let pointColors    = null;   // Float32Array, flat rgb [0–1], or null if no colour data
let dataLoaded     = false;
let resultCounts   = { sliding: 0, toppling: 0, wedge: 0 };

// ── Init ──────────────────────────────────────────────────────────────────────
document.addEventListener('DOMContentLoaded', () => {
  initThree();
  initStereonet();
  bindUI();
  loadSampleData();
  animate();
});

// ── Three.js setup ────────────────────────────────────────────────────────────
function initThree() {
  const wrap   = document.getElementById('viewer3d');
  const canvas = document.getElementById('canvas3d');

  scene    = new THREE.Scene();
  scene.background = new THREE.Color(0x0d0d1e);

  camera   = new THREE.PerspectiveCamera(55, wrap.clientWidth / wrap.clientHeight, 0.01, 10000);
  camera.position.set(30, -50, 30);
  camera.up.set(0, 0, 1);

  renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
  renderer.setPixelRatio(window.devicePixelRatio);
  renderer.setSize(wrap.clientWidth, wrap.clientHeight);

  controls = new THREE.OrbitControls(camera, canvas);
  controls.enableDamping = true;
  controls.dampingFactor = 0.08;

  scene.add(new THREE.AmbientLight(0xaaaacc, 0.8));
  const dl = new THREE.DirectionalLight(0xffffff, 0.6);
  dl.position.set(1, -1, 2);
  scene.add(dl);

  const grid = new THREE.GridHelper(100, 20, 0x223344, 0x1a2233);
  grid.rotation.x = Math.PI / 2;
  scene.add(grid);

  window.addEventListener('resize', onResize);
}

function onResize() {
  const wrap = document.getElementById('viewer3d');
  if (!wrap) return;
  camera.aspect = wrap.clientWidth / wrap.clientHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(wrap.clientWidth, wrap.clientHeight);
}

function animate() {
  requestAnimationFrame(animate);
  controls.update();
  renderer.render(scene, camera);
}

// ── Point cloud helpers ───────────────────────────────────────────────────────
/**
 * Build a Three.js Points object.
 * colorOrColors: hex number for flat colour, or Float32Array for per-vertex RGB [0–1].
 */
function makePointCloud(positions, colorOrColors, size) {
  size = size || 0.08;
  const geo  = new THREE.BufferGeometry();
  const pos32 = positions instanceof Float32Array ? positions : new Float32Array(positions);
  geo.setAttribute('position', new THREE.BufferAttribute(pos32, 3));

  let colArr;
  if (colorOrColors instanceof Float32Array) {
    colArr = colorOrColors;
  } else {
    const c = new THREE.Color(colorOrColors);
    colArr  = new Float32Array(pos32.length);
    for (let i = 0, n3 = pos32.length; i < n3; i += 3) {
      colArr[i] = c.r; colArr[i+1] = c.g; colArr[i+2] = c.b;
    }
  }
  geo.setAttribute('color', new THREE.BufferAttribute(colArr, 3));

  const mat = new THREE.PointsMaterial({ size, vertexColors: true, sizeAttenuation: true });
  return new THREE.Points(geo, mat);
}

function addOrReplaceCloud(existingRef, positions, colorOrColors, size) {
  if (existingRef && existingRef.parent) scene.remove(existingRef);
  if (!positions || positions.length === 0) return null;
  const cloud = makePointCloud(positions, colorOrColors, size);
  scene.add(cloud);
  return cloud;
}

function clearResultClouds() {
  [slidingCloud, toppingCloud, wedgeCloud].forEach(c => {
    if (c && c.parent) scene.remove(c);
  });
  slidingCloud = toppingCloud = wedgeCloud = null;
}

function centreCamera(positions) {
  let sx = 0, sy = 0, sz = 0;
  let mn = [Infinity,Infinity,Infinity], mx = [-Infinity,-Infinity,-Infinity];
  const n = positions.length / 3;
  for (let i = 0; i < n; i++) {
    const x = positions[i*3], y = positions[i*3+1], z = positions[i*3+2];
    sx += x; sy += y; sz += z;
    mn[0] = Math.min(mn[0], x); mn[1] = Math.min(mn[1], y); mn[2] = Math.min(mn[2], z);
    mx[0] = Math.max(mx[0], x); mx[1] = Math.max(mx[1], y); mx[2] = Math.max(mx[2], z);
  }
  const cx = sx/n, cy = sy/n, cz = sz/n;
  const span = Math.max(mx[0]-mn[0], mx[1]-mn[1], mx[2]-mn[2]);
  controls.target.set(cx, cy, cz);
  camera.position.set(cx + span, cy - span, cz + span * 0.5);
  camera.lookAt(cx, cy, cz);
}

// ── Sample data generator ─────────────────────────────────────────────────────
function loadSampleData() {
  setStatus('Generating sample slope…');

  const N_SLOPE = 3000, N_J1 = 600, N_J2 = 400, N_J3 = 300;
  const total   = N_SLOPE + N_J1 + N_J2 + N_J3;

  pointPositions = new Float32Array(total * 3);
  pointNormals   = new Float32Array(total * 3);
  pointColors    = null;

  let off = 0;

  function addJoint(n, dip, dip_dir, xr, yr, zr) {
    const dr = dip * Math.PI / 180, tr = dip_dir * Math.PI / 180;
    const nx = -Math.sin(dr)*Math.sin(tr);
    const ny = -Math.sin(dr)*Math.cos(tr);
    const nz =  Math.cos(dr);
    for (let i = 0; i < n; i++) {
      const u = xr[0] + Math.random()*(xr[1]-xr[0]);
      const v = yr[0] + Math.random()*(yr[1]-yr[0]);
      pointPositions[off]   = u;
      pointPositions[off+1] = v;
      pointPositions[off+2] = zr[0] + Math.random()*(zr[1]-zr[0])
                              + Math.tan(dr)*(u*Math.sin(tr)+v*Math.cos(tr))
                              + (Math.random()-.5)*.1;
      const rn = .05, mag = Math.sqrt((nx+rn)**2+(ny+rn)**2+(nz)**2);
      pointNormals[off]   = (nx+(Math.random()-.5)*rn)/mag;
      pointNormals[off+1] = (ny+(Math.random()-.5)*rn)/mag;
      pointNormals[off+2] = (nz+(Math.random()-.5)*rn)/mag;
      off += 3;
    }
  }

  // Background slope (70° / 140°)
  {
    const dr = 70*Math.PI/180, tr = 140*Math.PI/180;
    const nx = -Math.sin(dr)*Math.sin(tr);
    const ny = -Math.sin(dr)*Math.cos(tr);
    const nz =  Math.cos(dr);
    for (let i = 0; i < N_SLOPE; i++) {
      const u = (Math.random()-.5)*20, v = Math.random()*20;
      pointPositions[off]   = u*Math.cos(tr) + v*Math.sin(tr)*Math.cos(dr);
      pointPositions[off+1] = -u*Math.sin(tr)+ v*Math.cos(tr)*Math.cos(dr);
      pointPositions[off+2] = v*Math.sin(dr)+(Math.random()-.5)*.1;
      pointNormals[off]   = nx + (Math.random()-.5)*.05;
      pointNormals[off+1] = ny + (Math.random()-.5)*.05;
      pointNormals[off+2] = nz + (Math.random()-.5)*.05;
      const mag = Math.sqrt(pointNormals[off]**2+pointNormals[off+1]**2+pointNormals[off+2]**2);
      pointNormals[off]/=mag; pointNormals[off+1]/=mag; pointNormals[off+2]/=mag;
      off += 3;
    }
  }

  addJoint(N_J1, 27, 144, [-8,8], [2,18], [0,15]);   // J1 – sliding
  addJoint(N_J2, 87,   8, [-6,6], [4,16], [0,15]);   // J2 – toppling
  addJoint(N_J3, 81, 254, [-5,5], [6,14], [0,15]);   // J3 – wedge partner

  baseCloud = addOrReplaceCloud(baseCloud, pointPositions, 0xaaaacc, 0.07);
  centreCamera(pointPositions);
  dataLoaded = true;

  document.getElementById('fileInfo').textContent =
    `Sample slope  |  ${total.toLocaleString()} points`;
  setStatus('Sample data loaded. Set parameters and run analysis.');
  updateStereonet();
}

// ── File loading ──────────────────────────────────────────────────────────────
function handleFileLoad(file) {
  const ext = file.name.split('.').pop().toLowerCase();
  setStatus(`Loading ${file.name}…`);
  showProgress(true);

  const reader = new FileReader();

  if (ext === 'xyz' || ext === 'pts' || ext === 'xyzn' || ext === 'xyzrgb') {
    reader.onload = e => {
      try { loadXYZ(e.target.result, file.name); }
      catch (err) { setStatus('Error: ' + err.message); }
      showProgress(false);
    };
    reader.readAsText(file);

  } else if (ext === 'ply') {
    reader.onload = e => {
      try { loadPLY(e.target.result, file.name); }
      catch (err) { setStatus('Error: ' + err.message); }
      showProgress(false);
    };
    reader.readAsArrayBuffer(file);

  } else if (ext === 'obj') {
    reader.onload = e => {
      try { loadOBJ(e.target.result, file.name); }
      catch (err) { setStatus('Error: ' + err.message); }
      showProgress(false);
    };
    reader.readAsText(file);

  } else if (ext === 'las' || ext === 'laz') {
    reader.onload = e => {
      // showProgress(false) is called inside loadLAS after async normal estimation
      loadLAS(e.target.result, file.name);
    };
    reader.readAsArrayBuffer(file);

  } else {
    setStatus(`Format .${ext} not supported. Use LAS, PLY, OBJ, XYZ, or PTS.`);
    showProgress(false);
  }
}

// ── LAS/LAZ loader ────────────────────────────────────────────────────────────
async function loadLAS(buffer, name) {
  // Detect LAZ by the compression bit (format ID ≥ 128) in the header
  const fmtByte = new DataView(buffer).getUint8(104);
  const isLAZ   = fmtByte >= 128;

  let parsed;
  try {
    if (isLAZ) {
      setStatus('Initialising LAZ decompressor (WASM)…');
      parsed = await LASLoader.readLAZ(buffer);
    } else {
      parsed = LASLoader.read(buffer);
    }
  } catch (err) {
    setStatus((isLAZ ? 'LAZ' : 'LAS') + ' error: ' + err.message);
    console.error(err);
    showProgress(false);
    return;
  }

  const { points: pts64, colors: lasColors, numPoints } = parsed;

  // Centre around centroid so Float32 precision is adequate for display
  let cx = 0, cy = 0, cz = 0;
  for (let i = 0; i < numPoints; i++) {
    cx += pts64[i*3]; cy += pts64[i*3+1]; cz += pts64[i*3+2];
  }
  cx /= numPoints; cy /= numPoints; cz /= numPoints;

  // Sub-sample if cloud is too large for browser analysis
  let n       = numPoints;
  let indices = null;
  if (numPoints > MAX_ANALYSIS_PTS) {
    setStatus(`${numPoints.toLocaleString()} pts — sub-sampling to ${MAX_ANALYSIS_PTS.toLocaleString()} for analysis…`);
    indices = uniformSubsample(numPoints, MAX_ANALYSIS_PTS);
    n = MAX_ANALYSIS_PTS;
  }

  const pts32 = new Float32Array(n * 3);
  const col32 = lasColors ? new Float32Array(n * 3) : null;

  for (let i = 0; i < n; i++) {
    const src     = indices ? indices[i] : i;
    pts32[i*3]    = pts64[src*3]     - cx;
    pts32[i*3+1]  = pts64[src*3 + 1] - cy;
    pts32[i*3+2]  = pts64[src*3 + 2] - cz;
    if (col32 && lasColors) {
      col32[i*3]   = lasColors[src*3];
      col32[i*3+1] = lasColors[src*3+1];
      col32[i*3+2] = lasColors[src*3+2];
    }
  }

  setStatus(`Estimating normals for ${n.toLocaleString()} pts… (may take a moment)`);

  // Defer heavy computation to keep the UI responsive
  setTimeout(() => {
    try {
      const nrm = LASLoader.estimateNormals(pts32);
      pointPositions = pts32;
      pointNormals   = nrm;
      pointColors    = col32;
      const label = numPoints > MAX_ANALYSIS_PTS
        ? `${name}  |  ${numPoints.toLocaleString()} pts (sub-sampled to ${n.toLocaleString()})`
        : `${name}  |  ${numPoints.toLocaleString()} pts`;
      finaliseLoad(label);
    } catch (err) {
      setStatus('Normal estimation failed: ' + err.message);
      console.error(err);
    }
    showProgress(false);
  }, 30);
}

// ── XYZ / PTS / XYZN loader ───────────────────────────────────────────────────
function loadXYZ(text, name) {
  const lines = text.trim().split('\n').filter(l => l.trim() && !l.startsWith('#'));
  const pts = [], nrm = [];
  for (const line of lines) {
    const parts = line.trim().split(/[\s,]+/);
    if (parts.length < 3) continue;
    pts.push(+parts[0], +parts[1], +parts[2]);
    if (parts.length >= 6) nrm.push(+parts[3], +parts[4], +parts[5]);
  }
  pointPositions = new Float32Array(pts);
  pointNormals   = nrm.length === pts.length
    ? new Float32Array(nrm)
    : LASLoader.estimateNormals(pointPositions);
  pointColors = null;
  finaliseLoad(name);
}

// ── OBJ loader ────────────────────────────────────────────────────────────────
function loadOBJ(text, name) {
  const verts = [], normals_raw = [];
  for (const line of text.split('\n')) {
    const p = line.trim().split(/\s+/);
    if (p[0] === 'v')  verts.push(+p[1], +p[2], +p[3]);
    if (p[0] === 'vn') normals_raw.push(+p[1], +p[2], +p[3]);
  }
  pointPositions = new Float32Array(verts);
  pointNormals   = normals_raw.length
    ? new Float32Array(normals_raw)
    : LASLoader.estimateNormals(pointPositions);
  pointColors = null;
  finaliseLoad(name);
}

// ── PLY loader (ASCII) ────────────────────────────────────────────────────────
function loadPLY(buffer, name) {
  const text = new TextDecoder().decode(buffer);
  if (!text.startsWith('ply')) { setStatus('Invalid PLY file.'); return; }

  const lines = text.split('\n');
  let headerEnd = 0, numVertices = 0;
  const propOrder = [];

  for (let i = 0; i < lines.length; i++) {
    const l = lines[i].trim();
    if (l.startsWith('element vertex')) numVertices = parseInt(l.split(' ')[2]);
    if (l.startsWith('property') && !l.includes('list')) propOrder.push(l.split(' ')[2]);
    if (l === 'end_header') { headerEnd = i + 1; break; }
  }

  const xi = propOrder.indexOf('x'),  yi = propOrder.indexOf('y'),  zi = propOrder.indexOf('z');
  const nxi = propOrder.indexOf('nx'), nyi = propOrder.indexOf('ny'), nzi = propOrder.indexOf('nz');
  const ri  = propOrder.indexOf('red'), gi = propOrder.indexOf('green'), bi = propOrder.indexOf('blue');
  const hasNormals = nxi >= 0;
  const hasColors  = ri >= 0;

  const pts = [], nrm = [], col = [];
  for (let i = headerEnd; i < headerEnd + numVertices; i++) {
    const p = lines[i].trim().split(/\s+/);
    if (p.length <= zi) continue;
    pts.push(+p[xi], +p[yi], +p[zi]);
    if (hasNormals) nrm.push(+p[nxi], +p[nyi], +p[nzi]);
    if (hasColors)  col.push(+p[ri]/255, +p[gi]/255, +p[bi]/255);
  }

  pointPositions = new Float32Array(pts);
  pointNormals   = hasNormals ? new Float32Array(nrm) : LASLoader.estimateNormals(pointPositions);
  pointColors    = hasColors  ? new Float32Array(col) : null;
  finaliseLoad(name);
}

// ── Finalise load (common for all formats) ────────────────────────────────────
function finaliseLoad(label) {
  clearResultClouds();
  resultCounts = { sliding: 0, toppling: 0, wedge: 0 };
  updateCounts();

  if (baseCloud && baseCloud.parent) scene.remove(baseCloud);

  const n = pointPositions.length / 3;

  // Sub-sample display if cloud is large (analysis always uses full pointPositions)
  let dispPos = pointPositions;
  let dispCol = pointColors;
  if (n > MAX_DISPLAY_PTS) {
    const idx = uniformSubsample(n, MAX_DISPLAY_PTS);
    dispPos = new Float32Array(MAX_DISPLAY_PTS * 3);
    if (pointColors) dispCol = new Float32Array(MAX_DISPLAY_PTS * 3);
    for (let i = 0; i < MAX_DISPLAY_PTS; i++) {
      const s = idx[i];
      dispPos[i*3]   = pointPositions[s*3];
      dispPos[i*3+1] = pointPositions[s*3+1];
      dispPos[i*3+2] = pointPositions[s*3+2];
      if (dispCol && pointColors) {
        dispCol[i*3]   = pointColors[s*3];
        dispCol[i*3+1] = pointColors[s*3+1];
        dispCol[i*3+2] = pointColors[s*3+2];
      }
    }
  }

  baseCloud = addOrReplaceCloud(null, dispPos, dispCol || 0xaaaacc, 0.07);
  centreCamera(pointPositions);
  dataLoaded = true;

  document.getElementById('fileInfo').textContent = label;
  setStatus(`Loaded ${label.split('|')[0].trim()}. Ready for analysis.`);
  updateStereonet();
}

// ── Helper: uniform random subsample ─────────────────────────────────────────
function uniformSubsample(n, k) {
  const step = n / k;
  const idx  = new Uint32Array(k);
  for (let i = 0; i < k; i++) {
    idx[i] = Math.min(Math.floor(i * step + Math.random() * step), n - 1);
  }
  return idx;
}

// ── Analysis ──────────────────────────────────────────────────────────────────
function getParams() {
  return {
    slope_dip:        +document.getElementById('slope_dip').value,
    slope_dip_dir:    +document.getElementById('slope_dip_dir').value,
    friction_angle:   +document.getElementById('friction_angle').value,
    lateral_sliding:  +document.getElementById('lateral_sliding').value,
    lateral_toppling: +document.getElementById('lateral_toppling').value,
    min_angle_diff:   +document.getElementById('min_angle_diff').value,
    k_neighbours:     +document.getElementById('k_neighbours').value,
  };
}

function runAnalysis(mode) {
  if (!dataLoaded) { setStatus('Load a file first.'); return; }
  showProgress(true);
  setStatus(`Running ${mode} analysis…`);

  setTimeout(() => {
    try {
      const params = getParams();
      let mask, resultPts;

      if (mode === 'sliding') {
        mask      = FAA.analyseSliding(pointNormals, params);
        resultPts = maskedPositions(pointPositions, mask);
        slidingCloud = addOrReplaceCloud(slidingCloud, resultPts, 0xff3333, 0.10);
        resultCounts.sliding = countOnes(mask);

      } else if (mode === 'toppling') {
        mask      = FAA.analyseToppling(pointNormals, params);
        resultPts = maskedPositions(pointPositions, mask);
        toppingCloud = addOrReplaceCloud(toppingCloud, resultPts, 0x33cc55, 0.10);
        resultCounts.toppling = countOnes(mask);

      } else if (mode === 'wedge') {
        // Limit to 5k points for browser KNN performance
        const limit   = Math.min(pointPositions.length / 3, 5000);
        const pts_sub = pointPositions.slice(0, limit * 3);
        const nrm_sub = pointNormals.slice(0, limit * 3);
        const wpts    = FAA.analyseWedge(pts_sub, nrm_sub, params);
        const flat    = new Float32Array(wpts.flat());
        wedgeCloud    = addOrReplaceCloud(wedgeCloud, flat, 0x22ccff, 0.12);
        resultCounts.wedge = wpts.length;
      }

      updateCounts();
      updateStereonet(
        mode === 'sliding'  ? mask : null,
        mode === 'toppling' ? mask : null
      );
      setStatus(`${capitalize(mode)} analysis done — ${resultCounts[mode].toLocaleString()} potentially unstable elements.`);
    } catch (e) {
      setStatus('Analysis error: ' + e.message);
      console.error(e);
    }
    showProgress(false);
  }, 30);
}

function runAll() {
  ['sliding', 'toppling', 'wedge'].forEach((m, i) => {
    setTimeout(() => runAnalysis(m), i * 80);
  });
}

function maskedPositions(positions, mask) {
  const n = positions.length / 3;
  const out = [];
  for (let i = 0; i < n; i++) {
    if (mask[i]) out.push(positions[i*3], positions[i*3+1], positions[i*3+2]);
  }
  return new Float32Array(out);
}

function countOnes(mask) {
  let c = 0;
  for (let i = 0; i < mask.length; i++) c += mask[i];
  return c;
}

// ── Stereonet ─────────────────────────────────────────────────────────────────
let stereoCtx = null;

function initStereonet() {
  const canvas = document.getElementById('stereoCanvas');
  stereoCtx = canvas.getContext('2d');
  resizeStereonet();
  window.addEventListener('resize', resizeStereonet);
}

function resizeStereonet() {
  const canvas = document.getElementById('stereoCanvas');
  const wrap   = canvas.parentElement;
  canvas.width  = wrap.clientWidth  * window.devicePixelRatio;
  canvas.height = wrap.clientHeight * window.devicePixelRatio;
  canvas.style.width  = wrap.clientWidth  + 'px';
  canvas.style.height = wrap.clientHeight + 'px';
  updateStereonet();
}

function updateStereonet(slidingMask, toppling_mask) {
  if (!stereoCtx) return;
  const W = stereoCtx.canvas.width, H = stereoCtx.canvas.height;
  const cx = W / 2, cy = H / 2;
  const R  = Math.min(W, H) * 0.42;

  stereoCtx.clearRect(0, 0, W, H);
  stereoCtx.fillStyle = '#0d0d1e';
  stereoCtx.fillRect(0, 0, W, H);

  stereoCtx.beginPath();
  stereoCtx.arc(cx, cy, R, 0, Math.PI * 2);
  stereoCtx.strokeStyle = '#aaaacc';
  stereoCtx.lineWidth   = 1.5 * window.devicePixelRatio;
  stereoCtx.stroke();

  const fs = 11 * window.devicePixelRatio;
  stereoCtx.fillStyle = '#aaaacc';
  stereoCtx.font      = `${fs}px sans-serif`;
  stereoCtx.textAlign = 'center';
  stereoCtx.textBaseline = 'middle';
  stereoCtx.fillText('N', cx, cy - R - 14*window.devicePixelRatio);
  stereoCtx.fillText('S', cx, cy + R + 14*window.devicePixelRatio);
  stereoCtx.fillText('E', cx + R + 14*window.devicePixelRatio, cy);
  stereoCtx.fillText('W', cx - R - 14*window.devicePixelRatio, cy);

  if (!dataLoaded || !pointNormals) return;

  const step = Math.max(1, Math.floor(pointNormals.length / 3 / 4000));
  const n    = pointNormals.length / 3;

  for (let i = 0; i < n; i += step) {
    const [sx, sy] = FAA.normalToStereo(
      pointNormals[i*3], pointNormals[i*3+1], pointNormals[i*3+2]);
    const px = cx + sx * R;
    const py = cy - sy * R;

    let colour = 'rgba(170,170,204,0.12)';
    if (slidingMask   && slidingMask[i])   colour = 'rgba(255,51,51,0.6)';
    if (toppling_mask && toppling_mask[i]) colour = 'rgba(51,204,85,0.6)';

    stereoCtx.beginPath();
    stereoCtx.arc(px, py, 1.8 * window.devicePixelRatio, 0, Math.PI * 2);
    stereoCtx.fillStyle = colour;
    stereoCtx.fill();
  }

  drawSlopeMarker(cx, cy, R);

  stereoCtx.fillStyle = '#aaaacc';
  stereoCtx.font      = `${fs}px sans-serif`;
  stereoCtx.fillText('Lower Hemisphere — Equal Angle', cx, 14 * window.devicePixelRatio);
}

function drawSlopeMarker(cx, cy, R) {
  const dip = +document.getElementById('slope_dip').value;
  const dir = +document.getElementById('slope_dip_dir').value;
  const [sp, st] = FAA.dip_to_pole(dip, dir);
  const [sx, sy] = FAA.poleToStereo(sp, st);
  const px = cx + sx * R, py = cy - sy * R;

  stereoCtx.beginPath();
  stereoCtx.arc(px, py, 6 * window.devicePixelRatio, 0, Math.PI * 2);
  stereoCtx.strokeStyle = '#ffffff';
  stereoCtx.lineWidth   = 2 * window.devicePixelRatio;
  stereoCtx.stroke();

  stereoCtx.fillStyle = '#ffffff';
  stereoCtx.font      = `${9 * window.devicePixelRatio}px sans-serif`;
  stereoCtx.fillText('Slope', px + 10 * window.devicePixelRatio, py);
}

// ── UI bindings ───────────────────────────────────────────────────────────────
function bindUI() {
  document.querySelectorAll('.tab-btn').forEach(btn => {
    btn.addEventListener('click', () => {
      document.querySelectorAll('.tab-btn').forEach(b => b.classList.remove('active'));
      document.querySelectorAll('.tab-panel').forEach(p => p.classList.remove('active'));
      btn.classList.add('active');
      document.getElementById(btn.dataset.tab).classList.add('active');
      if (btn.dataset.tab === 'stereo') setTimeout(resizeStereonet, 50);
      else setTimeout(onResize, 50);
    });
  });

  const dz = document.getElementById('dropzone');
  const fi = document.getElementById('fileInput');
  dz.addEventListener('dragover',  e => { e.preventDefault(); dz.classList.add('drag-over'); });
  dz.addEventListener('dragleave', ()  => dz.classList.remove('drag-over'));
  dz.addEventListener('drop', e => {
    e.preventDefault();
    dz.classList.remove('drag-over');
    if (e.dataTransfer.files[0]) handleFileLoad(e.dataTransfer.files[0]);
  });
  fi.addEventListener('change', () => { if (fi.files[0]) handleFileLoad(fi.files[0]); });

  document.getElementById('btnSliding') .addEventListener('click', () => runAnalysis('sliding'));
  document.getElementById('btnToppling').addEventListener('click', () => runAnalysis('toppling'));
  document.getElementById('btnWedge')   .addEventListener('click', () => runAnalysis('wedge'));
  document.getElementById('btnAll')     .addEventListener('click', runAll);

  [
    ['friction_angle',   'friction_angle_val'],
    ['lateral_sliding',  'lateral_sliding_val'],
    ['lateral_toppling', 'lateral_toppling_val'],
    ['min_angle_diff',   'min_angle_diff_val'],
  ].forEach(([id, vid]) => {
    const el  = document.getElementById(id);
    const val = document.getElementById(vid);
    if (!el || !val) return;
    el.addEventListener('input', () => { val.textContent = el.value + '°'; updateStereonet(); });
  });

  document.getElementById('chk_sliding') .addEventListener('change', e => toggleCloud(slidingCloud,  e.target.checked));
  document.getElementById('chk_toppling').addEventListener('change', e => toggleCloud(toppingCloud,  e.target.checked));
  document.getElementById('chk_wedge')   .addEventListener('change', e => toggleCloud(wedgeCloud,    e.target.checked));

  document.getElementById('btnExportSample').addEventListener('click', exportResultsCSV);
}

function toggleCloud(cloud, visible) {
  if (cloud) cloud.visible = visible;
}

function updateCounts() {
  document.getElementById('count_sliding') .textContent = resultCounts.sliding  ? `${resultCounts.sliding.toLocaleString()} pts`  : '—';
  document.getElementById('count_toppling').textContent = resultCounts.toppling ? `${resultCounts.toppling.toLocaleString()} pts` : '—';
  document.getElementById('count_wedge')   .textContent = resultCounts.wedge    ? `${resultCounts.wedge.toLocaleString()} pts`    : '—';
}

// ── Export ────────────────────────────────────────────────────────────────────
function exportResultsCSV() {
  if (!dataLoaded) return;
  const params = getParams();
  const mS = FAA.analyseSliding(pointNormals, params);
  const mT = FAA.analyseToppling(pointNormals, params);
  const n  = pointPositions.length / 3;
  const rows = ['x,y,z,sliding,toppling'];
  for (let i = 0; i < n; i++) {
    rows.push(
      `${pointPositions[i*3].toFixed(4)},` +
      `${pointPositions[i*3+1].toFixed(4)},` +
      `${pointPositions[i*3+2].toFixed(4)},` +
      `${mS[i]},${mT[i]}`
    );
  }
  const blob = new Blob([rows.join('\n')], { type: 'text/csv' });
  const a = document.createElement('a');
  a.href     = URL.createObjectURL(blob);
  a.download = 'faa_results.csv';
  a.click();
}

// ── Utilities ─────────────────────────────────────────────────────────────────
function setStatus(msg) {
  document.getElementById('statusMsg').textContent = msg;
}

function showProgress(on) {
  document.getElementById('progressWrap').classList.toggle('show', on);
  if (on) {
    let w = 0;
    const iv = setInterval(() => {
      w = Math.min(w + 5, 90);
      document.getElementById('progressBar').style.width = w + '%';
      if (!on) { clearInterval(iv); document.getElementById('progressBar').style.width = '100%'; }
    }, 60);
  } else {
    document.getElementById('progressBar').style.width = '100%';
    setTimeout(() => {
      document.getElementById('progressWrap').classList.remove('show');
      document.getElementById('progressBar').style.width = '0%';
    }, 300);
  }
}

function capitalize(s) { return s.charAt(0).toUpperCase() + s.slice(1); }
