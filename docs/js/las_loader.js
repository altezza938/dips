/**
 * LAS 1.0–1.4 binary parser for browser use.
 *
 * Public API
 *   LASLoader.read(ArrayBuffer)
 *     → { points: Float64Array (N×3), colors: Float32Array|null (N×3),
 *          format: number, numPoints: number }
 *
 *   LASLoader.estimateNormals(Float32Array pts, k = 12)
 *     → Float32Array normals (same length as pts)
 *       Grid-accelerated PCA using a Jacobi eigensolver.
 *
 * LAZ (compressed) is detected immediately and rejected with a conversion guide.
 * Supported point formats: 0–10.
 * RGB extracted from formats 2, 3, 5, 7, 8, 10 (16-bit → normalised 0–1).
 */
const LASLoader = (() => {

  // Byte offset of RGB triple within a point record, keyed by point format ID
  const RGB_BYTE_OFFSET = { 2: 20, 3: 28, 5: 28, 7: 30, 8: 30, 10: 30 };

  // ── Jacobi eigendecomposition for 3×3 symmetric matrix ─────────────────────
  // Returns the eigenvector associated with the smallest eigenvalue (= surface normal).
  function smallestEigenvec(m00, m01, m02, m11, m12, m22) {
    const A = [m00, m01, m02,
               m01, m11, m12,
               m02, m12, m22];
    const V = [1, 0, 0,
               0, 1, 0,
               0, 0, 1];

    for (let iter = 0; iter < 20; iter++) {
      // Find largest off-diagonal element
      let maxV = 0, p = 0, q = 1;
      for (let r = 0; r < 3; r++)
        for (let c = r + 1; c < 3; c++) {
          const v = Math.abs(A[r * 3 + c]);
          if (v > maxV) { maxV = v; p = r; q = c; }
        }
      if (maxV < 1e-12) break;

      // Givens rotation
      const apq = A[p * 3 + q];
      const tau  = (A[q * 3 + q] - A[p * 3 + p]) / (2 * apq);
      const t    = (tau >= 0 ? 1 : -1) / (Math.abs(tau) + Math.sqrt(1 + tau * tau));
      const c    = 1 / Math.sqrt(1 + t * t);
      const s    = t * c;

      const App = A[p * 3 + p], Aqq = A[q * 3 + q];
      A[p * 3 + p] = c * c * App - 2 * s * c * apq + s * s * Aqq;
      A[q * 3 + q] = s * s * App + 2 * s * c * apq + c * c * Aqq;
      A[p * 3 + q] = A[q * 3 + p] = 0;

      for (let r = 0; r < 3; r++) {
        if (r === p || r === q) continue;
        const Apr = A[p * 3 + r], Aqr = A[q * 3 + r];
        A[p * 3 + r] = A[r * 3 + p] = c * Apr - s * Aqr;
        A[q * 3 + r] = A[r * 3 + q] = s * Apr + c * Aqr;
      }
      for (let r = 0; r < 3; r++) {
        const Vrp = V[r * 3 + p], Vrq = V[r * 3 + q];
        V[r * 3 + p] = c * Vrp - s * Vrq;
        V[r * 3 + q] = s * Vrp + c * Vrq;
      }
    }

    // Column of V for smallest diagonal of A
    let minI = 0;
    if (A[4] < A[0]) minI = 1;
    if (A[8] < A[minI * 3 + minI]) minI = 2;
    return [V[minI], V[3 + minI], V[6 + minI]];
  }

  // ── Normal estimation (grid-accelerated PCA) ────────────────────────────────
  function estimateNormals(pts, k) {
    k = k || 12;
    const n   = pts.length / 3;
    const nrm = new Float32Array(n * 3);

    // Bounding box
    let x0 = Infinity,  y0 = Infinity,  z0 = Infinity;
    let x1 = -Infinity, y1 = -Infinity, z1 = -Infinity;
    for (let i = 0; i < n; i++) {
      const x = pts[i*3], y = pts[i*3+1], z = pts[i*3+2];
      if (x < x0) x0 = x; if (x > x1) x1 = x;
      if (y < y0) y0 = y; if (y > y1) y1 = y;
      if (z < z0) z0 = z; if (z > z1) z1 = z;
    }

    const span = Math.max(x1 - x0, y1 - y0, z1 - z0, 1e-6);
    const G    = Math.max(1, Math.ceil(Math.cbrt(n / 8)));
    const cs   = span / G;

    // Build voxel grid index
    const grid = new Map();
    for (let i = 0; i < n; i++) {
      const gx = Math.min(Math.floor((pts[i*3]     - x0) / cs), G - 1);
      const gy = Math.min(Math.floor((pts[i*3 + 1] - y0) / cs), G - 1);
      const gz = Math.min(Math.floor((pts[i*3 + 2] - z0) / cs), G - 1);
      const key = (gx * G + gy) * G + gz;
      let cell = grid.get(key);
      if (!cell) { cell = []; grid.set(key, cell); }
      cell.push(i);
    }

    const cands = [];
    for (let i = 0; i < n; i++) {
      const px = pts[i*3], py = pts[i*3+1], pz = pts[i*3+2];
      const gx = Math.min(Math.floor((px - x0) / cs), G - 1);
      const gy = Math.min(Math.floor((py - y0) / cs), G - 1);
      const gz = Math.min(Math.floor((pz - z0) / cs), G - 1);

      // Collect points from 3×3×3 surrounding voxels
      cands.length = 0;
      for (let dx = -1; dx <= 1; dx++) {
        const nx_ = gx + dx; if (nx_ < 0 || nx_ >= G) continue;
        for (let dy = -1; dy <= 1; dy++) {
          const ny_ = gy + dy; if (ny_ < 0 || ny_ >= G) continue;
          for (let dz = -1; dz <= 1; dz++) {
            const nz_ = gz + dz; if (nz_ < 0 || nz_ >= G) continue;
            const cell = grid.get((nx_ * G + ny_) * G + nz_);
            if (cell) for (const j of cell) if (j !== i) cands.push(j);
          }
        }
      }

      if (cands.length < 3) {
        nrm[i*3] = 0; nrm[i*3+1] = 0; nrm[i*3+2] = 1;
        continue;
      }

      // Sort by distance, keep k nearest
      cands.sort((a, b) => {
        const dxa = pts[a*3]-px, dya = pts[a*3+1]-py, dza = pts[a*3+2]-pz;
        const dxb = pts[b*3]-px, dyb = pts[b*3+1]-py, dzb = pts[b*3+2]-pz;
        return (dxa*dxa+dya*dya+dza*dza) - (dxb*dxb+dyb*dyb+dzb*dzb);
      });
      const nn = Math.min(k, cands.length);

      // Centroid of neighbourhood
      let cx = px, cy = py, cz = pz;
      for (let j = 0; j < nn; j++) {
        const idx = cands[j];
        cx += pts[idx*3]; cy += pts[idx*3+1]; cz += pts[idx*3+2];
      }
      cx /= (nn + 1); cy /= (nn + 1); cz /= (nn + 1);

      // 3×3 covariance (upper triangle)
      let m00=0, m01=0, m02=0, m11=0, m12=0, m22=0;
      for (let j = 0; j < nn; j++) {
        const idx = cands[j];
        const dx = pts[idx*3]-cx, dy = pts[idx*3+1]-cy, dz = pts[idx*3+2]-cz;
        m00+=dx*dx; m01+=dx*dy; m02+=dx*dz;
        m11+=dy*dy; m12+=dy*dz;
        m22+=dz*dz;
      }

      const [ex, ey, ez] = smallestEigenvec(m00, m01, m02, m11, m12, m22);

      // Orient normal downward (z ≤ 0) — lower hemisphere convention
      const sign = ez > 0 ? -1 : 1;
      nrm[i*3]     = ex * sign;
      nrm[i*3 + 1] = ey * sign;
      nrm[i*3 + 2] = ez * sign;
    }
    return nrm;
  }

  // ── LAS binary parser ───────────────────────────────────────────────────────
  function read(buffer) {
    const view = new DataView(buffer);

    // Verify file signature
    const sig = String.fromCharCode(
      view.getUint8(0), view.getUint8(1), view.getUint8(2), view.getUint8(3));
    if (sig !== 'LASF')
      throw new Error('Not a valid LAS file — wrong file signature.');

    const vMaj   = view.getUint8(24);
    const vMin   = view.getUint8(25);
    const fmtRaw = view.getUint8(104);

    // LAZ compressed format IDs have bit 7 set (≥ 128)
    if (fmtRaw >= 128) throw new Error(
      'LAZ (compressed) format detected.\n\n' +
      'The browser cannot decompress LAZ files.\n' +
      'Please re-export as uncompressed LAS:\n' +
      '  • GeoSLAM Hub → Export → LAS (uncompressed)\n' +
      '  • CloudCompare → File → Save As → LAS 1.3\n' +
      '  • LAStools:  las2las -i file.laz -o file.las');

    const format = fmtRaw;
    const recLen = view.getUint16(105, true);
    const offset = view.getUint32(96,  true);

    // Point count: legacy 32-bit at byte 107; LAS 1.4 adds 64-bit at byte 247
    let numPts = view.getUint32(107, true);
    if (numPts === 0 && vMaj === 1 && vMin >= 4) {
      const lo = view.getUint32(247, true);
      const hi = view.getUint32(251, true);
      if (hi > 0) throw new Error('Point cloud > 4 billion points — too large for browser.');
      numPts = lo;
    }
    if (numPts === 0) throw new Error('LAS file contains 0 point records.');

    const xScale = view.getFloat64(131, true), xOff = view.getFloat64(155, true);
    const yScale = view.getFloat64(139, true), yOff = view.getFloat64(163, true);
    const zScale = view.getFloat64(147, true), zOff = view.getFloat64(171, true);

    if (buffer.byteLength < offset + numPts * recLen)
      throw new Error('File appears truncated — cannot read all point records.');

    const rgbOff = RGB_BYTE_OFFSET[format] !== undefined ? RGB_BYTE_OFFSET[format] : null;
    const points = new Float64Array(numPts * 3);
    const colors = rgbOff !== null ? new Float32Array(numPts * 3) : null;

    let ptr = offset;
    for (let i = 0; i < numPts; i++) {
      points[i*3]     = view.getInt32(ptr,     true) * xScale + xOff;
      points[i*3 + 1] = view.getInt32(ptr + 4, true) * yScale + yOff;
      points[i*3 + 2] = view.getInt32(ptr + 8, true) * zScale + zOff;

      if (rgbOff !== null) {
        colors[i*3]     = view.getUint16(ptr + rgbOff,     true) / 65535;
        colors[i*3 + 1] = view.getUint16(ptr + rgbOff + 2, true) / 65535;
        colors[i*3 + 2] = view.getUint16(ptr + rgbOff + 4, true) / 65535;
      }

      ptr += recLen;
    }

    return { points, colors, format, numPoints: numPts };
  }

  // ── LAZ decompressor via laz-perf WASM ──────────────────────────────────────
  // laz-perf is loaded dynamically. CDN builds may expose:
  //   createLazPerf()  — named Emscripten factory
  //   LazPerf / LazPerfExports — UMD export
  //   Module           — bare Emscripten global (older 0.0.x)
  // We try several CDN URLs in sequence and detect the global automatically.

  const LAZ_CDN_URLS = [
    'https://cdn.jsdelivr.net/npm/laz-perf@0.0.7/lib/web/laz-perf.js',
    'https://unpkg.com/laz-perf@0.0.7/lib/web/laz-perf.js',
    'https://cdn.jsdelivr.net/npm/laz-perf@0.0.7/build/laz-perf.js',
    'https://unpkg.com/laz-perf@0.0.7/build/laz-perf.js',
  ];

  let _lazPerfPromise = null;

  function _resolveModule() {
    // Try every known global name that laz-perf builds may use
    for (const name of ['createLazPerf', 'LazPerf', 'LazPerfExports']) {
      if (typeof window[name] === 'function') return window[name]();
    }
    if (typeof Module !== 'undefined') {
      if (typeof Module._malloc === 'function' || Module.calledRun)
        return Promise.resolve(Module);
      return new Promise((resolve, reject) => {
        const t = setTimeout(() => reject(new Error('laz-perf init timeout')), 20000);
        const prev = Module.onRuntimeInitialized;
        Module.onRuntimeInitialized = () => {
          clearTimeout(t);
          if (typeof prev === 'function') prev();
          resolve(Module);
        };
      });
    }
    return null;
  }

  function _loadScript(url) {
    return new Promise((resolve, reject) => {
      const s = document.createElement('script');
      s.src = url;
      s.onload  = resolve;
      s.onerror = reject;
      document.head.appendChild(s);
    });
  }

  function _getLazPerf() {
    if (!_lazPerfPromise) {
      _lazPerfPromise = (async () => {
        // Fast path: CDN script already loaded by <script> tag in HTML
        const quick = _resolveModule();
        if (quick) return quick;

        // Slow path: dynamically try each CDN URL in sequence
        for (const url of LAZ_CDN_URLS) {
          try {
            await _loadScript(url);
            const m = _resolveModule();
            if (m) return m;
          } catch (_) { /* try next */ }
        }

        throw new Error(
          'LAZ decompression unavailable — all CDN sources failed.\n\n' +
          'Convert to uncompressed LAS first:\n' +
          '  • GeoSLAM Hub → Export → LAS (uncompressed)\n' +
          '  • CloudCompare → File → Save As → LAS 1.3\n' +
          '  • LAStools:  las2las -i file.laz -o file.las'
        );
      })();
    }
    return _lazPerfPromise;
  }

  /**
   * Decompress a LAZ file using laz-perf WASM.
   * Returns the same shape as read(): { points, colors, format, numPoints }.
   */
  async function readLAZ(buffer) {
    const lp   = await _getLazPerf();
    const view = new DataView(buffer);

    const vMaj   = view.getUint8(24);
    const vMin   = view.getUint8(25);
    const fmtRaw = view.getUint8(104);
    const format = fmtRaw & 0x7F;   // strip LAZ compression bit

    let numPts = view.getUint32(107, true);
    if (numPts === 0 && vMaj === 1 && vMin >= 4) {
      const lo = view.getUint32(247, true);
      const hi = view.getUint32(251, true);
      if (hi > 0) throw new Error('Point cloud > 4 billion points — too large for browser.');
      numPts = lo;
    }
    if (numPts === 0) throw new Error('LAZ file contains 0 points.');

    const xScale = view.getFloat64(131, true), xOff = view.getFloat64(155, true);
    const yScale = view.getFloat64(139, true), yOff = view.getFloat64(163, true);
    const zScale = view.getFloat64(147, true), zOff = view.getFloat64(171, true);

    const rgbOff = RGB_BYTE_OFFSET[format] !== undefined ? RGB_BYTE_OFFSET[format] : null;

    // Copy entire file to WASM heap
    const filePtr = lp._malloc(buffer.byteLength);
    lp.HEAPU8.set(new Uint8Array(buffer), filePtr);

    const laszip   = new lp.LASZip();
    laszip.open(filePtr, buffer.byteLength);

    const pointLen = laszip.getPointLength();
    const destPtr  = lp._malloc(pointLen);
    const heapView = new DataView(lp.HEAPU8.buffer);

    const points = new Float64Array(numPts * 3);
    const colors = rgbOff !== null ? new Float32Array(numPts * 3) : null;

    for (let i = 0; i < numPts; i++) {
      laszip.getPoint(destPtr);
      points[i*3]     = heapView.getInt32(destPtr,     true) * xScale + xOff;
      points[i*3 + 1] = heapView.getInt32(destPtr + 4, true) * yScale + yOff;
      points[i*3 + 2] = heapView.getInt32(destPtr + 8, true) * zScale + zOff;

      if (rgbOff !== null) {
        colors[i*3]     = heapView.getUint16(destPtr + rgbOff,     true) / 65535;
        colors[i*3 + 1] = heapView.getUint16(destPtr + rgbOff + 2, true) / 65535;
        colors[i*3 + 2] = heapView.getUint16(destPtr + rgbOff + 4, true) / 65535;
      }
    }

    laszip.delete();
    lp._free(destPtr);
    lp._free(filePtr);

    return { points, colors, format, numPoints: numPts };
  }

  return { read, readLAZ, estimateNormals };
})();
