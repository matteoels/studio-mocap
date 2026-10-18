// =====================================================================
// Studio Mocap — suivi d'un cube aux faces colorées (position + rotation)
// Chaque face a une couleur. On repère les faces visibles dans l'image,
// on trouve leurs 4 coins, puis on calcule la position et la rotation
// du cube par rapport à la webcam (en cm). Aucun module externe.
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
const CORNER_AB = [[1, 1], [-1, 1], [-1, -1], [1, -1]];

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

// ---------- couleurs ----------
const srgbToLinear = (c) => (c <= 0.04045 ? c / 12.92 : Math.pow((c + 0.055) / 1.055, 2.4));
export function rgbToLab(r, g, b) {
  const R = srgbToLinear(r / 255), G = srgbToLinear(g / 255), B = srgbToLinear(b / 255);
  let X = (0.4124 * R + 0.3576 * G + 0.1805 * B) / 0.95047;
  let Y = 0.2126 * R + 0.7152 * G + 0.0722 * B;
  let Z = (0.0193 * R + 0.1192 * G + 0.9505 * B) / 1.08883;
  const f = (t) => (t > 0.008856 ? Math.cbrt(t) : 7.787 * t + 16 / 116);
  X = f(X); Y = f(Y); Z = f(Z);
  return [116 * Y - 16, 500 * (X - Y), 200 * (Y - Z)];
}
let LAB_LUT = null; // RGB sur 5 bits par canal -> Lab
function labLut() {
  if (LAB_LUT) return LAB_LUT;
  LAB_LUT = new Float32Array(32768 * 3);
  for (let i = 0; i < 32768; i++) {
    const lab = rgbToLab(((i >> 10) & 31) * 8.226, ((i >> 5) & 31) * 8.226, (i & 31) * 8.226);
    LAB_LUT[i * 3] = lab[0]; LAB_LUT[i * 3 + 1] = lab[1]; LAB_LUT[i * 3 + 2] = lab[2];
  }
  return LAB_LUT;
}
// Couleur moyenne autour d'un point (pour la calibration)
export function sampleColor(img, x, y, radius = 4) {
  const { data, width, height } = img;
  let r = 0, g = 0, b = 0, n = 0;
  for (let j = Math.max(0, y - radius); j <= Math.min(height - 1, y + radius); j++) {
    for (let i = Math.max(0, x - radius); i <= Math.min(width - 1, x + radius); i++) {
      const k = (j * width + i) * 4;
      r += data[k]; g += data[k + 1]; b += data[k + 2]; n++;
    }
  }
  return n ? [Math.round(r / n), Math.round(g / n), Math.round(b / n)] : [0, 0, 0];
}

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

// =====================================================================
export class CubeTracker {
  constructor() {
    this.size = 10; // arête du cube en cm
    this.fov = 60; // champ de vision horizontal de la webcam (degrés)
    this.tolerance = 28; // tolérance de couleur (distance Lab)
    this.colors = new Array(6).fill(null); // [r,g,b] par face
    this.labs = new Array(6).fill(null);
    this.prev = null; // dernière pose {R, t, time}
    this.last = null; // dernières détections (pour l'affichage)
  }
  setColor(face, rgb) {
    this.colors[face] = rgb;
    this.labs[face] = rgb ? rgbToLab(rgb[0], rgb[1], rgb[2]) : null;
  }
  isCalibrated() {
    return this.labs.filter(Boolean).length >= 2;
  }
  reset() {
    this.prev = null;
  }

  // ---- 1. Pixels de chaque couleur ----
  segment(img) {
    const { data, width: w, height: h } = img;
    const N = w * h;
    if (!this._labels || this._labels.length !== N) {
      this._labels = new Uint8Array(N);
      this._visited = new Int32Array(N);
      this._stack = new Int32Array(N);
    }
    const labels = this._labels;
    const lut = labLut();
    const labs = this.labs;
    const tol2 = this.tolerance * this.tolerance;
    const active = [];
    for (let f = 0; f < 6; f++) if (labs[f]) active.push(f);
    for (let p = 0, k = 0; p < N; p++, k += 4) {
      const q = ((data[k] >> 3) << 10) | ((data[k + 1] >> 3) << 5) | (data[k + 2] >> 3);
      const L = lut[q * 3], A = lut[q * 3 + 1], B = lut[q * 3 + 2];
      let best = 255, d1 = Infinity, d2 = Infinity;
      for (const f of active) {
        const c = labs[f];
        // la luminosité varie d'une face à l'autre avec l'éclairage : elle compte moins
        const dl = (L - c[0]) * 0.45, da = A - c[1], db = B - c[2];
        const d = dl * dl + da * da + db * db;
        if (d < d1) { d2 = d1; d1 = d; best = f; } else if (d < d2) d2 = d;
      }
      labels[p] = d1 < tol2 && d1 < d2 * 0.55 ? best : 255;
    }
    return labels;
  }

  // ---- 2. La plus grande tache de chaque couleur, ses coins ----
  blobs(img) {
    const w = img.width, h = img.height, N = w * h;
    const labels = this._labels, visited = this._visited, stack = this._stack;
    visited.fill(0);
    const best = new Array(6).fill(null);
    const groups = new Array(6).fill(null);
    let stamp = 0;
    const minArea = Math.max(30, N * 0.0012);
    for (let p0 = 0; p0 < N; p0++) {
      const f = labels[p0];
      if (f === 255 || visited[p0]) continue;
      stamp++;
      let sp = 0, area = 0, sx = 0, sy = 0;
      stack[sp++] = p0; visited[p0] = stamp;
      const rowMin = new Map(), rowMax = new Map();
      let minx = w, maxx = 0, miny = h, maxy = 0;
      const pix = [];
      while (sp) {
        const p = stack[--sp];
        const x = p % w, y = (p - x) / w;
        area++; sx += x; sy += y;
        if (x < minx) minx = x; if (x > maxx) maxx = x; if (y < miny) miny = y; if (y > maxy) maxy = y;
        const rm = rowMin.get(y);
        if (rm === undefined || x < rm) rowMin.set(y, x);
        const rM = rowMax.get(y);
        if (rM === undefined || x > rM) rowMax.set(y, x);
        pix.push(p);
        if (x > 0 && !visited[p - 1] && labels[p - 1] === f) { visited[p - 1] = stamp; stack[sp++] = p - 1; }
        if (x < w - 1 && !visited[p + 1] && labels[p + 1] === f) { visited[p + 1] = stamp; stack[sp++] = p + 1; }
        if (y > 0 && !visited[p - w] && labels[p - w] === f) { visited[p - w] = stamp; stack[sp++] = p - w; }
        if (y < h - 1 && !visited[p + w] && labels[p + w] === f) { visited[p + w] = stamp; stack[sp++] = p + w; }
      }
      if (area < minArea / 3) continue;
      (groups[f] = groups[f] || []).push({ face: f, area, sx, sy, rowMin, rowMax, pix, box: [minx, miny, maxx, maxy] });
    }
    // Une face coupée en morceaux par les doigts : on réunit les morceaux proches de la plus grande tache
    for (let f = 0; f < 6; f++) {
      const g0 = groups[f];
      if (!g0) continue;
      const g = g0.filter((c) => {
        const touches = (c.box[0] <= 0) + (c.box[1] <= 0) + (c.box[2] >= w - 1) + (c.box[3] >= h - 1);
        return touches < 2 && c.area < N * 0.3;
      });
      if (!g.length) continue;
      g.sort((a, b) => b.area - a.area);
      const main = g[0];
      if (main.area < minArea) continue;
      const diag = Math.hypot(main.box[2] - main.box[0], main.box[3] - main.box[1]);
      const mcx = main.sx / main.area, mcy = main.sy / main.area;
      const merged = { face: f, area: 0, sx: 0, sy: 0, rowMin: new Map(), rowMax: new Map(), pix: [] };
      for (const c of g) {
        if (c !== main && Math.hypot(c.sx / c.area - mcx, c.sy / c.area - mcy) > diag * 0.9) continue;
        merged.area += c.area; merged.sx += c.sx; merged.sy += c.sy;
        for (const [y, x] of c.rowMin) { const v = merged.rowMin.get(y); if (v === undefined || x < v) merged.rowMin.set(y, x); }
        for (const [y, x] of c.rowMax) { const v = merged.rowMax.get(y); if (v === undefined || x > v) merged.rowMax.set(y, x); }
        for (const p of c.pix) merged.pix.push(p);
      }
      merged.cx = merged.sx / merged.area; merged.cy = merged.sy / merged.area;
      best[f] = merged;
    }
    const out = [];
    for (const b of best) {
      if (!b) continue;
      // contour : extrêmes de chaque ligne et de chaque colonne
      const boundary = [];
      for (const [y, x] of b.rowMin) boundary.push([x, y]);
      for (const [y, x] of b.rowMax) boundary.push([x + 1, y]);
      const colMin = new Map(), colMax = new Map();
      for (const p of b.pix) {
        const x = p % w, y = (p - x) / w;
        const cm = colMin.get(x);
        if (cm === undefined || y < cm) colMin.set(x, y);
        const cM = colMax.get(x);
        if (cM === undefined || y > cM) colMax.set(x, y);
      }
      for (const [x, y] of colMin) boundary.push([x, y]);
      for (const [x, y] of colMax) boundary.push([x, y + 1]);
      const hull = convexHull(boundary.map((p) => p.slice()));
      // Bande très fine : souvent le mélange de deux couleurs le long d'une arête (rouge + jaune = orange...)
      let diameter = 0;
      for (let i = 0; i < hull.length; i++) for (let j = i + 1; j < hull.length; j++) diameter = Math.max(diameter, Math.hypot(hull[i][0] - hull[j][0], hull[i][1] - hull[j][1]));
      if (b.area / Math.max(1, diameter) < 3) continue;
      let quad = bestQuad(hull);
      let corners = null;
      if (quad) {
        const hullArea = polyArea(hull);
        // une face vue de biais reste un quadrilatère convexe ; sinon, c'est un morceau caché par la main
        if (polyArea(quad) > hullArea * 0.85 && b.area > hullArea * 0.5) {
          quad = refineQuad(quad, boundary);
          // ordre des coins autour du centre
          const mx = quad.reduce((s, p) => s + p[0], 0) / 4, my = quad.reduce((s, p) => s + p[1], 0) / 4;
          quad.sort((p, q) => Math.atan2(p[1] - my, p[0] - mx) - Math.atan2(q[1] - my, q[0] - mx));
          corners = quad;
        }
      }
      out.push({ face: b.face, area: b.area, cx: b.cx, cy: b.cy, corners, hull });
    }
    return out;
  }

  // ---- 3. Pose ----
  _K(w) {
    const f = (w / 2) / Math.tan((this.fov * Math.PI) / 360);
    return { f, cx: 0, cy: 0 };
  }
  _project(K, P) {
    return [K.f * P[0] / P[2] + K.cx, K.f * P[1] / P[2] + K.cy];
  }
  _faceModel(f) {
    const F = FACES[f], hs = this.size / 2;
    const c = scale(F.n, hs);
    return { c, corners: CORNER_AB.map(([a, b]) => add(c, add(scale(F.u, a * hs), scale(F.v, b * hs)))) };
  }
  _poseFromFace(K, f, imgCorners) {
    const F = FACES[f], hs = this.size / 2;
    const src = CORNER_AB.map(([a, b]) => [a * hs, b * hs]);
    const dst = imgCorners.map(([x, y]) => [(x - K.cx) / K.f, (y - K.cy) / K.f]);
    const H = homography(src, dst);
    if (!H) return null;
    const m1 = [H[0], H[3], H[6]], m2 = [H[1], H[4], H[7]], m3 = [H[2], H[5], H[8]];
    let lam = 2 / (norm(m1) + norm(m2));
    if (m3[2] * lam < 0) lam = -lam; // le cube est devant la caméra
    const Ru = scale(m1, lam), Rv = scale(m2, lam);
    const Rn = cross(Ru, Rv);
    // R * [u v n] = [Ru Rv Rn]
    const A = fromCols(Ru, Rv, scale(Rn, 1 / Math.max(1e-9, norm(Rn)) * Math.sqrt(norm(Ru) * norm(Rv))));
    const UVN = fromCols(F.u, F.v, F.n);
    const R = orthonormalize(mulMM(A, transpose(UVN)));
    const c = scale(F.n, hs);
    const t = sub(scale(m3, lam), mulMV(R, c));
    if (t[2] <= 0) return null;
    return { R, t };
  }
  _visible(R, t, f) {
    const F = FACES[f];
    const n = mulMV(R, F.n);
    const c = add(mulMV(R, scale(F.n, this.size / 2)), t);
    return dot(n, c) < 0; // la normale pointe vers la caméra
  }
  // Correspondance des coins détectés avec ceux du modèle (meilleur décalage)
  _matchCorners(K, R, t, det) {
    const model = this._faceModel(det.face).corners.map((P) => this._project(K, add(mulMV(R, P), t)));
    let best = null, bestE = Infinity;
    for (let s = 0; s < 4; s++) {
      for (const dir of [1, -1]) {
        let e = 0;
        for (let i = 0; i < 4; i++) {
          const j = (s + dir * i + 8) % 4;
          e += Math.hypot(model[i][0] - det.corners[j][0], model[i][1] - det.corners[j][1]);
        }
        if (e < bestE) { bestE = e; best = { s, dir }; }
      }
    }
    return { ...best, err: bestE / 4 };
  }
  _score(K, R, t, dets, faceSize) {
    let score = 0;
    const center = this._project(K, t);
    const reach = 1.6 * K.f * this.size / t[2]; // taille du cube à l'écran (avec de la marge)
    const detected = new Set(dets.map((d) => d.face));
    let maxArea = 0;
    for (const d of dets) maxArea = Math.max(maxArea, d.area);
    for (const d of dets) {
      const far = Math.hypot(d.cx - center[0], d.cy - center[1]) > reach;
      if (far) { score += 0.3; continue; } // tache ailleurs dans l'image (fond, objet de même couleur)
      if (!this._visible(R, t, d.face)) { score += 3; continue; }
      const c = this._project(K, add(mulMV(R, scale(FACES[d.face].n, this.size / 2)), t));
      if (d.corners) score += this._matchCorners(K, R, t, d).err / faceSize;
      else score += Math.min(3, Math.hypot(c[0] - d.cx, c[1] - d.cy) / faceSize);
    }
    for (let f = 0; f < 6; f++) {
      if (detected.has(f) || !this.labs[f] || !this._visible(R, t, f)) continue;
      // face qui devrait se voir mais n'a pas été trouvée (peut-être cachée par la main)
      const n = mulMV(R, FACES[f].n);
      const c = add(mulMV(R, scale(FACES[f].n, this.size / 2)), t);
      const facing = -dot(n, c) / norm(c);
      if (facing > 0.35) score += 1.2 * facing;
    }
    return score;
  }
  _refine(K, R, t, dets) {
    // Gauss-Newton robuste sur tous les coins de toutes les faces visibles
    const items = [];
    const center = this._project(K, t), reach = 1.6 * K.f * this.size / t[2];
    for (const d of dets) {
      if (!d.corners || !this._visible(R, t, d.face)) continue;
      if (Math.hypot(d.cx - center[0], d.cy - center[1]) > reach) continue;
      const m = this._matchCorners(K, R, t, d);
      const model = this._faceModel(d.face).corners;
      for (let i = 0; i < 4; i++) items.push({ P: model[i], q: d.corners[(m.s + m.dir * i + 8) % 4] });
    }
    if (items.length < 4) return { R, t, rms: Infinity, n: 0 };
    const res = refinePose(K, R, t, items);
    return { ...res, n: items.length / 4 };
  }

  // Analyse une image (ImageData). time en secondes.
  process(img, time) {
    if (!this.isCalibrated()) return null;
    this.segment(img);
    const raw = this.blobs(img);
    // coordonnées centrées sur l'image
    const w = img.width, h = img.height;
    const dets = raw.map((d) => ({
      ...d,
      cx: d.cx - w / 2 + 0.5, cy: d.cy - h / 2 + 0.5,
      corners: d.corners ? d.corners.map(([x, y]) => [x - w / 2, y - h / 2]) : null,
    }));
    this.last = { dets: raw, ok: false, w, h };
    const withCorners = dets.filter((d) => d.corners);
    if (!withCorners.length) return null;
    const K = this._K(w);
    const prev = this.prev && time - this.prev.time < 0.5 ? this.prev : null;
    let best = null, bestScore = Infinity;
    this._cands = [];
    for (const d of withCorners) {
      const faceSize = Math.sqrt(d.area);
      for (let s = 0; s < 4; s++) {
        for (const dir of [1, -1]) {
          const order = [0, 1, 2, 3].map((i) => d.corners[(s + dir * i + 8) % 4]);
          const pose = this._poseFromFace(K, d.face, order);
          if (!pose || !this._visible(pose.R, pose.t, d.face)) continue;
          let score = this._score(K, pose.R, pose.t, dets, faceSize);
          if (prev) score += 0.5 * rotationAngle(pose.R, prev.R) + 0.2 * norm(sub(pose.t, prev.t)) / this.size;
          if (this.debug) (this._cands = this._cands || []).push({ score, R: pose.R, t: pose.t, face: d.face, s, dir });
          if (score < bestScore) { bestScore = score; best = pose; }
        }
      }
    }
    if (!best) return null;
    const ref = this._refine(K, best.R, best.t, dets);
    const scale0 = Math.sqrt(Math.max(...withCorners.map((d) => d.area)));
    // trop d'écart entre le modèle et l'image : on garde la pose précédente
    if (!(ref.rms < Math.max(2.5, scale0 * 0.12))) return null;
    // saut de rotation impossible en 1/30 s : probablement une erreur d'identification
    if (prev && rotationAngle(ref.R, prev.R) > 1.4 && ref.n < 2) return null;
    this.prev = { R: ref.R, t: ref.t, time };
    this.last.ok = true;
    this.last.pose = { R: ref.R, t: ref.t, K, rms: ref.rms, faces: ref.n };
    return { R: ref.R, t: ref.t, rms: ref.rms, faces: ref.n, ambiguous: ref.n < 2 };
  }
}

// Avec une seule face visible, le cube a 4 orientations possibles (quart de tour).
// Quand une 2e face apparaît et révèle un quart de tour, on corrige après coup
// les images ambiguës qui précèdent. frames : [{R, t, ambiguous}] (repère caméra)
export function fixAmbiguousFrames(frames) {
  let segStart = -1;
  for (let i = 0; i < frames.length; i++) {
    const f = frames[i];
    if (f.ambiguous) {
      if (segStart < 0) segStart = i;
      continue;
    }
    if (segStart >= 0 && i > 0) {
      const prev = frames[i - 1];
      // correction locale (symétrie du cube) entre la dernière image ambiguë et celle-ci
      const L = mulMM(transpose(prev.R), f.R);
      const ang = Math.acos(Math.max(-1, Math.min(1, (L[0] + L[4] + L[8] - 1) / 2)));
      if (ang > 1.0) {
        // arrondi au quart de tour le plus proche
        const S = L.map((v) => Math.round(v));
        if (Math.abs(det3(S) - 1) < 1e-6) for (let k = segStart; k < i; k++) frames[k].R = mulMM(frames[k].R, S);
      }
    }
    segStart = -1;
  }
  return frames;
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
