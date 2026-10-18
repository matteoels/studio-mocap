// =====================================================================
// Studio Mocap — outils du cube suivi (géométrie, ajustement de pose, export GDevelop)
// Le suivi lui-même (marqueurs imprimés) est dans marqueurs.js. Aucun module externe.
// Repère caméra : x à droite, y vers le bas, z devant la caméra.
// =====================================================================

// Faces du cube (même ordre que les matériaux d'un BoxGeometry Three.js)
// n = normale sortante, u et v = axes de la face (u × v = n)
export const FACES = [
  { name: 'Droite (+X)', n: [1, 0, 0], u: [0, 1, 0], v: [0, 0, 1] },
  { name: 'Gauche (−X)', n: [-1, 0, 0], u: [0, 0, 1], v: [0, 1, 0] },
  { name: 'Dessus (+Y)', n: [0, 1, 0], u: [0, 0, 1], v: [1, 0, 0] },
  { name: 'Dessous (−Y)', n: [0, -1, 0], u: [1, 0, 0], v: [0, 0, 1] },
  { name: 'Avant (+Z)', n: [0, 0, 1], u: [1, 0, 0], v: [0, 1, 0] },
  { name: 'Arrière (−Z)', n: [0, 0, -1], u: [0, 1, 0], v: [1, 0, 0] },
];

// ---------- petites maths 3D (tableaux) ----------
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const norm = (a) => Math.hypot(a[0], a[1], a[2]);
const scale = (a, s) => [a[0] * s, a[1] * s, a[2] * s];
const add = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
// Matrices 3x3 en tableau de 9 (ligne par ligne)
const mulMV = (M, v) => [M[0] * v[0] + M[1] * v[1] + M[2] * v[2], M[3] * v[0] + M[4] * v[1] + M[5] * v[2], M[6] * v[0] + M[7] * v[1] + M[8] * v[2]];
const mulMM = (A, B) => {
  const C = new Array(9);
  for (let r = 0; r < 3; r++) for (let c = 0; c < 3; c++) C[r * 3 + c] = A[r * 3] * B[c] + A[r * 3 + 1] * B[3 + c] + A[r * 3 + 2] * B[6 + c];
  return C;
};
const transpose = (M) => [M[0], M[3], M[6], M[1], M[4], M[7], M[2], M[5], M[8]];
const fromCols = (a, b, c) => [a[0], b[0], c[0], a[1], b[1], c[1], a[2], b[2], c[2]];
const det3 = (M) => M[0] * (M[4] * M[8] - M[5] * M[7]) - M[1] * (M[3] * M[8] - M[5] * M[6]) + M[2] * (M[3] * M[7] - M[4] * M[6]);
const inv3 = (M) => {
  const d = det3(M);
  if (Math.abs(d) < 1e-12) return null;
  const i = 1 / d;
  return [
    (M[4] * M[8] - M[5] * M[7]) * i, (M[2] * M[7] - M[1] * M[8]) * i, (M[1] * M[5] - M[2] * M[4]) * i,
    (M[5] * M[6] - M[3] * M[8]) * i, (M[0] * M[8] - M[2] * M[6]) * i, (M[2] * M[3] - M[0] * M[5]) * i,
    (M[3] * M[7] - M[4] * M[6]) * i, (M[1] * M[6] - M[0] * M[7]) * i, (M[0] * M[4] - M[1] * M[3]) * i,
  ];
};
// Rotation la plus proche (itérations de Higham)
const orthonormalize = (M) => {
  let R = M.slice();
  for (let k = 0; k < 8; k++) {
    const iT = inv3(R);
    if (!iT) break;
    const t = transpose(iT);
    R = R.map((x, i) => 0.5 * (x + t[i]));
  }
  return R;
};
// Rotation d'un petit vecteur (formule de Rodrigues)
const expRot = (w) => {
  const th = norm(w);
  if (th < 1e-12) return [1, 0, 0, 0, 1, 0, 0, 0, 1];
  const k = scale(w, 1 / th), c = Math.cos(th), s = Math.sin(th), v = 1 - c;
  return [
    c + k[0] * k[0] * v, k[0] * k[1] * v - k[2] * s, k[0] * k[2] * v + k[1] * s,
    k[1] * k[0] * v + k[2] * s, c + k[1] * k[1] * v, k[1] * k[2] * v - k[0] * s,
    k[2] * k[0] * v - k[1] * s, k[2] * k[1] * v + k[0] * s, c + k[2] * k[2] * v,
  ];
};
export const rotationAngle = (A, B) => {
  const M = mulMM(A, transpose(B));
  return Math.acos(Math.max(-1, Math.min(1, (M[0] + M[4] + M[8] - 1) / 2)));
};
export const matToQuat = (m) => {
  // m ligne par ligne -> quaternion [x, y, z, w]
  const tr = m[0] + m[4] + m[8];
  let x, y, z, w;
  if (tr > 0) {
    const s = 0.5 / Math.sqrt(tr + 1);
    w = 0.25 / s; x = (m[7] - m[5]) * s; y = (m[2] - m[6]) * s; z = (m[3] - m[1]) * s;
  } else if (m[0] > m[4] && m[0] > m[8]) {
    const s = 2 * Math.sqrt(1 + m[0] - m[4] - m[8]);
    w = (m[7] - m[5]) / s; x = 0.25 * s; y = (m[1] + m[3]) / s; z = (m[2] + m[6]) / s;
  } else if (m[4] > m[8]) {
    const s = 2 * Math.sqrt(1 + m[4] - m[0] - m[8]);
    w = (m[2] - m[6]) / s; x = (m[1] + m[3]) / s; y = 0.25 * s; z = (m[5] + m[7]) / s;
  } else {
    const s = 2 * Math.sqrt(1 + m[8] - m[0] - m[4]);
    w = (m[3] - m[1]) / s; x = (m[2] + m[6]) / s; y = (m[5] + m[7]) / s; z = 0.25 * s;
  }
  const l = Math.hypot(x, y, z, w);
  return [x / l, y / l, z / l, w / l];
};

// ---------- géométrie 2D ----------
function convexHull(pts) {
  pts.sort((a, b) => a[0] - b[0] || a[1] - b[1]);
  const crossZ = (o, a, b) => (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
  const lower = [], upper = [];
  for (const p of pts) {
    while (lower.length >= 2 && crossZ(lower[lower.length - 2], lower[lower.length - 1], p) <= 0) lower.pop();
    lower.push(p);
  }
  for (let i = pts.length - 1; i >= 0; i--) {
    const p = pts[i];
    while (upper.length >= 2 && crossZ(upper[upper.length - 2], upper[upper.length - 1], p) <= 0) upper.pop();
    upper.push(p);
  }
  upper.pop(); lower.pop();
  return lower.concat(upper);
}
const polyArea = (p) => {
  let a = 0;
  for (let i = 0; i < p.length; i++) {
    const q = p[(i + 1) % p.length];
    a += p[i][0] * q[1] - q[0] * p[i][1];
  }
  return Math.abs(a) / 2;
};
// Quadrilatère d'aire maximale inscrit dans l'enveloppe convexe
function bestQuad(hull) {
  const n = hull.length;
  if (n < 4) return null;
  if (n > 40) {
    const step = n / 40, s = [];
    for (let i = 0; i < 40; i++) s.push(hull[Math.floor(i * step)]);
    hull = s;
  }
  const m = hull.length;
  let best = null, bestA = -1;
  const tri = (a, b, c) => Math.abs((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])) / 2;
  for (let i = 0; i < m; i++) {
    for (let k = i + 2; k < m; k++) {
      // meilleur j entre i et k, meilleur l entre k et i
      let bj = -1, aj = -1;
      for (let j = i + 1; j < k; j++) { const a = tri(hull[i], hull[j], hull[k]); if (a > aj) { aj = a; bj = j; } }
      let bl = -1, al = -1;
      for (let l = k + 1; l < m + i; l++) { const a = tri(hull[k], hull[l % m], hull[i]); if (a > al) { al = a; bl = l % m; } }
      if (bj < 0 || bl < 0) continue;
      if (aj + al > bestA) { bestA = aj + al; best = [hull[i], hull[bj], hull[k], hull[bl]]; }
    }
  }
  return best;
}
// Ajuste chaque côté sur les points du contour, puis recoupe les côtés (coins sous-pixel)
function refineQuad(quad, boundary) {
  const lines = [];
  for (let s = 0; s < 4; s++) {
    const a = quad[s], b = quad[(s + 1) % 4];
    const dx = b[0] - a[0], dy = b[1] - a[1], len = Math.hypot(dx, dy);
    if (len < 3) return quad;
    const ux = dx / len, uy = dy / len;
    let sx = 0, sy = 0, sxx = 0, syy = 0, sxy = 0, n = 0;
    const tol = Math.max(1.5, len * 0.04);
    for (const p of boundary) {
      const px = p[0] - a[0], py = p[1] - a[1];
      const along = px * ux + py * uy;
      if (along < len * 0.12 || along > len * 0.88) continue;
      if (Math.abs(px * uy - py * ux) > tol) continue;
      sx += p[0]; sy += p[1]; sxx += p[0] * p[0]; syy += p[1] * p[1]; sxy += p[0] * p[1]; n++;
    }
    if (n < 4) {
      lines.push({ p: a, d: [ux, uy] });
      continue;
    }
    const mx = sx / n, my = sy / n;
    const cxx = sxx / n - mx * mx, cyy = syy / n - my * my, cxy = sxy / n - mx * my;
    const ang = 0.5 * Math.atan2(2 * cxy, cxx - cyy);
    let d = [Math.cos(ang), Math.sin(ang)];
    if (d[0] * ux + d[1] * uy < 0) d = [-d[0], -d[1]];
    lines.push({ p: [mx, my], d });
  }
  const out = [];
  for (let s = 0; s < 4; s++) {
    const L1 = lines[(s + 3) % 4], L2 = lines[s];
    const den = L1.d[0] * L2.d[1] - L1.d[1] * L2.d[0];
    if (Math.abs(den) < 1e-6) { out.push(quad[s]); continue; }
    const t = ((L2.p[0] - L1.p[0]) * L2.d[1] - (L2.p[1] - L1.p[1]) * L2.d[0]) / den;
    const x = L1.p[0] + L1.d[0] * t, y = L1.p[1] + L1.d[1] * t;
    if (Math.hypot(x - quad[s][0], y - quad[s][1]) > Math.hypot(quad[s][0] - quad[(s + 2) % 4][0], quad[s][1] - quad[(s + 2) % 4][1]) * 0.25) out.push(quad[s]);
    else out.push([x, y]);
  }
  return out;
}

// Homographie de 4 correspondances (a,b) -> (x,y)
function homography(src, dst) {
  const A = [], B = [];
  for (let i = 0; i < 4; i++) {
    const [a, b] = src[i], [x, y] = dst[i];
    A.push([a, b, 1, 0, 0, 0, -a * x, -b * x]); B.push(x);
    A.push([0, 0, 0, a, b, 1, -a * y, -b * y]); B.push(y);
  }
  // élimination de Gauss
  const n = 8;
  for (let c = 0; c < n; c++) {
    let p = c;
    for (let r = c + 1; r < n; r++) if (Math.abs(A[r][c]) > Math.abs(A[p][c])) p = r;
    if (Math.abs(A[p][c]) < 1e-12) return null;
    [A[c], A[p]] = [A[p], A[c]]; [B[c], B[p]] = [B[p], B[c]];
    for (let r = 0; r < n; r++) {
      if (r === c) continue;
      const f = A[r][c] / A[c][c];
      if (!f) continue;
      for (let k = c; k < n; k++) A[r][k] -= f * A[c][k];
      B[r] -= f * B[c];
    }
  }
  const h = B.map((v, i) => v / A[i][i]);
  return [h[0], h[1], h[2], h[3], h[4], h[5], h[6], h[7], 1];
}

// Gauss-Newton robuste (poids de Huber) : ajuste R, t pour que les points 3D P
// se projettent sur les points image q. items : [{P, q}]
export function refinePose(K, R, t, items, iters = 8) {
  const residuals = (Rc, tc) => {
    const r = [];
    for (const it of items) {
      const X = add(mulMV(Rc, it.P), tc);
      if (X[2] <= 0.1) { r.push(1e3, 1e3); continue; }
      const p = [K.f * X[0] / X[2] + K.cx, K.f * X[1] / X[2] + K.cy];
      r.push(p[0] - it.q[0], p[1] - it.q[1]);
    }
    return r;
  };
  let cur = { R, t };
  let r = residuals(R, t);
  for (let iter = 0; iter < iters; iter++) {
    // poids de Huber (un coin caché par un doigt compte moins)
    const absr = r.map(Math.abs).sort((a, b) => a - b);
    const sigma = Math.max(0.6, 1.5 * absr[Math.floor(absr.length / 2)]);
    const wts = r.map((v) => (Math.abs(v) <= sigma ? 1 : sigma / Math.abs(v)));
    const J = [];
    const eps = [1e-4, 1e-4, 1e-4, 1e-3, 1e-3, 1e-3];
    for (let k = 0; k < 6; k++) {
      const d = [0, 0, 0, 0, 0, 0]; d[k] = eps[k];
      const Rk = mulMM(expRot([d[0], d[1], d[2]]), cur.R);
      const tk = add(cur.t, [d[3], d[4], d[5]]);
      const rk = residuals(Rk, tk);
      J.push(rk.map((v, i) => (v - r[i]) / eps[k]));
    }
    // (JᵀWJ + μI) δ = -JᵀWr
    const A = Array.from({ length: 6 }, () => new Array(6).fill(0)), g = new Array(6).fill(0);
    for (let i = 0; i < r.length; i++) {
      for (let a = 0; a < 6; a++) {
        g[a] += J[a][i] * wts[i] * r[i];
        for (let b = a; b < 6; b++) A[a][b] += J[a][i] * wts[i] * J[b][i];
      }
    }
    for (let a = 0; a < 6; a++) { for (let b = 0; b < a; b++) A[a][b] = A[b][a]; A[a][a] *= 1.001; A[a][a] += 1e-6; }
    const delta = solve6(A, g.map((v) => -v));
    if (!delta) break;
    const Rn = orthonormalize(mulMM(expRot([delta[0], delta[1], delta[2]]), cur.R));
    const tn = add(cur.t, [delta[3], delta[4], delta[5]]);
    const rn = residuals(Rn, tn);
    const cost = (rr) => rr.reduce((s, v, i) => s + wts[i] * v * v, 0);
    if (cost(rn) < cost(r)) { cur = { R: Rn, t: tn }; r = rn; } else break;
    if (Math.hypot(...delta) < 1e-6) break;
  }
  const rms = Math.sqrt(r.reduce((s, v) => s + v * v, 0) / r.length);
  return { R: cur.R, t: cur.t, rms };
}

function solve6(A, b) {
  const n = 6, M = A.map((row, i) => [...row, b[i]]);
  for (let c = 0; c < n; c++) {
    let p = c;
    for (let r = c + 1; r < n; r++) if (Math.abs(M[r][c]) > Math.abs(M[p][c])) p = r;
    if (Math.abs(M[p][c]) < 1e-14) return null;
    [M[c], M[p]] = [M[p], M[c]];
    for (let r = c + 1; r < n; r++) {
      const f = M[r][c] / M[c][c];
      for (let k = c; k <= n; k++) M[r][k] -= f * M[c][k];
    }
  }
  const x = new Array(n).fill(0);
  for (let i = n - 1; i >= 0; i--) {
    let s = M[i][n];
    for (let k = i + 1; k < n; k++) s -= M[i][k] * x[k];
    x[i] = s / M[i][i];
  }
  return x;
}

// ---------- Conversion vers le repère "monde" de l'export ----------
// Monde : X à droite, Y en s'éloignant de la webcam, Z vers le haut (cm).
// Caméra : x à droite, y vers le bas, z devant.  monde = B · caméra
const B = [1, 0, 0, 0, 0, 1, 0, -1, 0];
export function cameraPoseToWorld(R, t, mirror) {
  let Rw = mulMM(B, R), p = mulMV(B, t);
  if (mirror) {
    // effet miroir : gauche et droite inversées (comme dans un miroir)
    const M = [-1, 0, 0, 0, 1, 0, 0, 0, 1];
    Rw = mulMM(mulMM(M, Rw), M);
    p = mulMV(M, p);
  }
  return { R: Rw, p };
}
export const mat3 = { mulMM, transpose, mulMV };

// ---------- Texte pour GDevelop ----------
// SMOCAP1|30|<n>|x,y,z,qx,qy,qz,qw;...   (cm et quaternion, relatifs à la première image)
export function encodeForGDevelop(frames, fps = 30) {
  if (frames.length < 2) return '';
  const t0 = frames[0].t, dur = frames[frames.length - 1].t - t0;
  const n = Math.max(2, Math.round(dur * fps) + 1);
  const p0 = frames[0].p, R0T = transpose(frames[0].R);
  const out = [];
  let i = 0;
  for (let k = 0; k < n; k++) {
    const t = t0 + Math.min(dur, k / fps);
    while (i < frames.length - 2 && frames[i + 1].t <= t) i++;
    const a = frames[i], b = frames[Math.min(i + 1, frames.length - 1)];
    const span = b.t - a.t, s = span > 0 ? Math.min(1, Math.max(0, (t - a.t) / span)) : 0;
    const p = [0, 1, 2].map((c) => a.p[c] + (b.p[c] - a.p[c]) * s - p0[c]);
    const qa = matToQuat(mulMM(a.R, R0T)), qb = matToQuat(mulMM(b.R, R0T));
    const q = slerp(qa, qb, s);
    out.push(p.map((v) => v.toFixed(2)).join(',') + ',' + q.map((v) => v.toFixed(4)).join(','));
  }
  return `SMOCAP1|${fps}|${n}|` + out.join(';');
}
export function slerp(a, b, s) {
  let d = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
  let bb = b;
  if (d < 0) { d = -d; bb = b.map((v) => -v); }
  if (d > 0.9995) {
    const r = a.map((v, i) => v + (bb[i] - v) * s);
    const l = Math.hypot(...r);
    return r.map((v) => v / l);
  }
  const th = Math.acos(d), sn = Math.sin(th);
  const wa = Math.sin((1 - s) * th) / sn, wb = Math.sin(s * th) / sn;
  return a.map((v, i) => v * wa + bb[i] * wb);
}

// Supprime les images aberrantes isolées (un doigt qui trompe le suivi pendant 1 ou 2 images) :
// elles sont remplacées par l'interpolation de leurs voisines. frames : [{time, R, pos}]
export function removeSpikes(frames, maxAngle = 0.35, maxGap = 3) {
  const n = frames.length;
  const bad = new Array(n).fill(false);
  for (let i = 1; i < n - 1; i++) {
    for (let gap = 1; gap <= maxGap; gap++) {
      const j = i + gap;
      if (j >= n) break;
      const a = frames[i - 1], b = frames[j];
      if (rotationAngle(a.R, b.R) > maxAngle * 0.6) continue; // les voisines ne sont pas d'accord entre elles
      let spike = false;
      for (let k = i; k < j; k++) if (rotationAngle(frames[k].R, a.R) > maxAngle) spike = true;
      if (spike) {
        for (let k = i; k < j; k++) bad[k] = true;
        break;
      }
    }
  }
  for (let i = 0; i < n; i++) {
    if (!bad[i]) continue;
    let a = i - 1, b = i + 1;
    while (a >= 0 && bad[a]) a--;
    while (b < n && bad[b]) b++;
    if (a < 0 || b >= n) continue;
    const s = (frames[i].time - frames[a].time) / Math.max(1e-6, frames[b].time - frames[a].time);
    const q = slerp(matToQuat(frames[a].R), matToQuat(frames[b].R), s);
    frames[i].R = quatToMat(q);
    frames[i].pos = frames[a].pos.map((v, c) => v + (frames[b].pos[c] - v) * s);
    frames[i].fixed = true;
  }
  return frames;
}
export function quatToMat([x, y, z, w]) {
  return [
    1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w),
    2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w),
    2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y),
  ];
}

// Outils partagés avec le suivi par marqueurs (marqueurs.js)
export const geom = { dot, cross, norm, scale, add, sub, mulMV, mulMM, transpose, fromCols, orthonormalize, homography, convexHull, polyArea, bestQuad, refineQuad };
