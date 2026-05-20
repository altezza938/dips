/**
 * FAA Core Algorithms – JavaScript port
 * Based on GEO Technical Note TN 4/2024, Appendix A
 * Extended Facet Amalgamation Approach for Rock Slope Kinematic Analysis
 */

const FAA = (() => {

  // ── vector helpers ──────────────────────────────────────────────────────────
  function dot(a, b) { return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]; }
  function cross(a, b) {
    return [
      a[1]*b[2] - a[2]*b[1],
      a[2]*b[0] - a[0]*b[2],
      a[0]*b[1] - a[1]*b[0]
    ];
  }
  function norm(v) { return Math.sqrt(dot(v, v)); }
  function normalise(v) {
    const n = norm(v);
    return n > 0 ? v.map(x => x / n) : v;
  }
  function deg2rad(d) { return d * Math.PI / 180; }
  function rad2deg(r) { return r * 180 / Math.PI; }

  /**
   * Convert plunge & trend (degrees) to unit direction vector.
   * Coordinate system: x=East, y=North, z=Up
   */
  function plunge_trend_to_vec(plunge_deg, trend_deg) {
    const p = deg2rad(plunge_deg);
    const t = deg2rad(trend_deg);
    const z  = -Math.sin(p);
    const xy =  Math.cos(p);
    const x  =  Math.sin(t) * xy;
    const y  =  Math.cos(t) * xy;
    return [x, y, z];
  }

  /**
   * Convert unit direction vector to [plunge, trend] in degrees.
   */
  function vec_to_plunge_trend(v) {
    const plunge = rad2deg(-Math.asin(Math.max(-1, Math.min(1, v[2]))));
    let trend    = rad2deg(Math.atan2(v[0], v[1]));
    if (trend < 0) trend += 360;
    return [plunge, trend];
  }

  /**
   * Convert dip / dip-direction to pole [plunge, trend].
   */
  function dip_to_pole(dip_deg, dip_dir_deg) {
    return [90 - dip_deg, (dip_dir_deg + 180) % 360];
  }

  /**
   * Absolute angular difference between two bearings, handling wrap-around.
   */
  function angle_diff(a1, a2) {
    return 180 - Math.abs(Math.abs(a1 - a2) - 180);
  }

  /**
   * Ensure normal vector points downward (z < 0).
   */
  function flip_down(v) {
    return v[2] > 0 ? [-v[0], -v[1], -v[2]] : [...v];
  }

  // ── kinematic checks ────────────────────────────────────────────────────────

  /**
   * Sliding failure check for a single normal vector.
   * Returns true if the facet/point is potentially susceptible to planar sliding.
   */
  function check_sliding_single(nx, ny, nz,
                                  slope_dip, slope_dip_dir,
                                  friction_angle, lateral_limit) {
    if (slope_dip <= friction_angle) return false;

    // Ensure normal points into the slope (downward)
    const n = nz > 0 ? [-nx, -ny, -nz] : [nx, ny, nz];
    const nn = normalise(n);

    const [pole_plunge, pole_trend] = vec_to_plunge_trend(nn);
    const [sp_plunge, sp_trend]     = dip_to_pole(slope_dip, slope_dip_dir);

    // 1) Lateral limit
    if (angle_diff(pole_trend, sp_trend) > lateral_limit) return false;

    // 2) Friction cone – plunge of pole must be ≤ (90° − φ)
    if (pole_plunge > 90 - friction_angle) return false;

    // 3) Daylight: dip vector must point toward the slope
    const dip_pt = dip_to_pole(pole_plunge, pole_trend);
    const dip_vec = plunge_trend_to_vec(...dip_pt);
    const slope_pole_vec = plunge_trend_to_vec(sp_plunge, sp_trend);
    if (dot(slope_pole_vec, dip_vec) > 0) return false;   // not daylighting

    return true;
  }

  /**
   * Toppling failure check for a single normal vector.
   * Returns true if the facet/point is potentially susceptible to flexural toppling.
   */
  function check_toppling_single(nx, ny, nz,
                                   slope_dip, slope_dip_dir,
                                   friction_angle, lateral_limit) {
    if (slope_dip <= friction_angle) return false;

    const n  = nz > 0 ? [-nx, -ny, -nz] : [nx, ny, nz];
    const nn = normalise(n);
    const [pole_plunge, pole_trend] = vec_to_plunge_trend(nn);

    // 1) Lateral limit (pole of an into-slope joint trends toward the slope dip dir)
    if (angle_diff(pole_trend, slope_dip_dir) > lateral_limit) return false;

    // 2) Unstable zone: plunge of pole must be ≤ slope_dip − friction_angle
    const [sp_plunge] = dip_to_pole(slope_dip, slope_dip_dir);
    const unstable_plunge = sp_plunge + friction_angle;  // = 90 − slope_dip + φ
    const uz_vec = plunge_trend_to_vec(unstable_plunge, (slope_dip_dir + 180) % 360);
    if (dot(uz_vec, nn) >= 0) return false;

    return true;
  }

  /**
   * Wedge failure check for a pair of normals (point cloud mode).
   * Returns [plunge, trend] of line of intersection if wedge is unstable, else null.
   */
  function check_wedge_pair(n1, n2, p1, p2,
                              slope_dip, slope_dip_dir,
                              friction_angle, min_angle_diff) {
    if (slope_dip <= friction_angle) return null;

    const nn1 = normalise(n1.map((v, i) => n1[2] > 0 ? -v : v));
    const nn2 = normalise(n2.map((v, i) => n2[2] > 0 ? -v : v));

    // Angular difference between planes
    const cos_ang = Math.abs(dot(nn1, nn2));
    if (cos_ang > Math.cos(deg2rad(min_angle_diff))) return null;  // same joint set

    // Line of intersection = cross product of normals
    let e = cross(nn1, nn2);
    if (norm(e) < 1e-10) return null;  // parallel planes
    e = normalise(e);
    if (e[2] > 0) e = e.map(v => -v);  // point downward

    const [pt_plunge] = vec_to_plunge_trend(e);

    // 1) Friction cone
    if (pt_plunge < friction_angle) return null;

    // 2) Daylight
    const [sp_plunge, sp_trend] = dip_to_pole(slope_dip, slope_dip_dir);
    const slope_pole_vec = plunge_trend_to_vec(sp_plunge, sp_trend);
    if (dot(slope_pole_vec, e) > 0) return null;  // not daylighting

    // 3) Converging (not diverging)
    const d12 = normalise([p2[0]-p1[0], p2[1]-p1[1], p2[2]-p1[2]]);
    if (dot(nn1, d12) <= 0) return null;  // diverging

    return e;  // unstable
  }

  // ── batch analysis on arrays ────────────────────────────────────────────────

  /**
   * Run sliding analysis on an array of normals.
   * @param {Float32Array|Array} normals  Flat [nx,ny,nz, nx,ny,nz, …]
   * @param {Object} params
   * @returns {Uint8Array} mask – 1 = sliding failure
   */
  function analyseSliding(normals, params) {
    const n = normals.length / 3;
    const mask = new Uint8Array(n);
    for (let i = 0; i < n; i++) {
      const nx = normals[i*3], ny = normals[i*3+1], nz = normals[i*3+2];
      if (check_sliding_single(nx, ny, nz,
            params.slope_dip, params.slope_dip_dir,
            params.friction_angle, params.lateral_sliding)) {
        mask[i] = 1;
      }
    }
    return mask;
  }

  /**
   * Run toppling analysis on an array of normals.
   */
  function analyseToppling(normals, params) {
    const n = normals.length / 3;
    const mask = new Uint8Array(n);
    for (let i = 0; i < n; i++) {
      const nx = normals[i*3], ny = normals[i*3+1], nz = normals[i*3+2];
      if (check_toppling_single(nx, ny, nz,
            params.slope_dip, params.slope_dip_dir,
            params.friction_angle, params.lateral_toppling)) {
        mask[i] = 1;
      }
    }
    return mask;
  }

  /**
   * Run wedge analysis on a point cloud using k-nearest neighbours.
   * Returns array of midpoint positions [[x,y,z], …] of unstable wedge intersections.
   */
  function analyseWedge(points, normals, params) {
    const k   = params.k_neighbours || 16;
    const n   = points.length / 3;
    const pts = [];

    // Build simple kd-style brute-force search (fast enough for ≤ 50k pts in browser)
    const knn = buildKNN(points, n, k);

    const wedge_pts = [];
    const seen = new Set();

    for (let i = 0; i < n; i++) {
      const neighbours = knn[i];
      for (let j = 0; j < neighbours.length; j++) {
        const jj = neighbours[j];
        const key = i < jj ? `${i}_${jj}` : `${jj}_${i}`;
        if (seen.has(key)) continue;
        seen.add(key);

        const n1 = [normals[i*3], normals[i*3+1], normals[i*3+2]];
        const n2 = [normals[jj*3], normals[jj*3+1], normals[jj*3+2]];
        const p1 = [points[i*3],  points[i*3+1],  points[i*3+2]];
        const p2 = [points[jj*3], points[jj*3+1], points[jj*3+2]];

        const e = check_wedge_pair(n1, n2, p1, p2,
                    params.slope_dip, params.slope_dip_dir,
                    params.friction_angle, params.min_angle_diff);
        if (e) {
          // Store midpoint between the two points
          wedge_pts.push([(p1[0]+p2[0])/2, (p1[1]+p2[1])/2, (p1[2]+p2[2])/2]);
        }
      }
    }
    return wedge_pts;
  }

  /** Very simple KNN via distance sorting (fine for demo-sized datasets). */
  function buildKNN(points, n, k) {
    const knn = [];
    for (let i = 0; i < n; i++) {
      const xi = points[i*3], yi = points[i*3+1], zi = points[i*3+2];
      const dists = [];
      for (let j = 0; j < n; j++) {
        if (j === i) continue;
        const dx = xi - points[j*3], dy = yi - points[j*3+1], dz = zi - points[j*3+2];
        dists.push([dx*dx + dy*dy + dz*dz, j]);
      }
      dists.sort((a, b) => a[0] - b[0]);
      knn.push(dists.slice(0, k).map(d => d[1]));
    }
    return knn;
  }

  // ── stereonet helpers ───────────────────────────────────────────────────────

  /**
   * Convert pole (plunge, trend) to equal-angle lower-hemisphere projection (x, y).
   */
  function poleToStereo(plunge_deg, trend_deg) {
    const p = deg2rad(plunge_deg);
    const t = deg2rad(trend_deg);
    const r = Math.cos(p) / (1 + Math.sin(p));
    return [r * Math.sin(t), r * Math.cos(t)];
  }

  /**
   * Convert a normal vector to its lower-hemisphere stereonet projection.
   */
  function normalToStereo(nx, ny, nz) {
    let v = [nx, ny, nz];
    if (v[2] > 0) v = v.map(x => -x);
    const vn = normalise(v);
    const [plunge, trend] = vec_to_plunge_trend(vn);
    return poleToStereo(plunge, trend);
  }

  // ── public API ──────────────────────────────────────────────────────────────
  return {
    analyseSliding,
    analyseToppling,
    analyseWedge,
    normalToStereo,
    poleToStereo,
    dip_to_pole,
    plunge_trend_to_vec,
    deg2rad,
    rad2deg,
  };

})();
