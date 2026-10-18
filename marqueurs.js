// =====================================================================
// Studio Mocap — suivi d'un cube avec 6 marqueurs ArUco imprimés (4x4, ID 0 à 5)
// Chaque marqueur est unique et a un sens : une seule face visible suffit
// pour connaître la position ET la rotation du cube. Aucun module externe.
// Repère caméra : x à droite, y vers le bas, z devant la caméra (cm).
// =====================================================================
import { geom, refinePose, rotationAngle } from './cube.js';

const { dot, cross, norm, scale, add, sub, mulMV, mulMM, transpose, fromCols, orthonormalize, homography, convexHull, polyArea, bestQuad, refineQuad } = geom;

// Dictionnaire ArUco DICT_4X4_50 (OpenCV) : 16 bits par marqueur, ligne par ligne, blanc = 1
export const DICT_4X4_50 = [46386, 3994, 13101, 39238, 21662, 31181, 40494, 50418, 65242, 53078, 63889, 4519, 3767, 10767, 9393, 9790, 18021, 26112, 27742, 30383, 34443, 45099, 52437, 56706, 65095, 38001, 44260, 42324, 8483, 13423, 17429, 22450, 40655, 61643, 2222, 2345, 6261, 1279, 3574, 7258, 5912, 10792, 12940, 14514, 9448, 12011, 11583, 19300, 20526, 20499];

// Où est collé chaque marqueur (face du cube, et vers quelle face pointe le haut du carré).
// Faces dans l'ordre de FACES (cube.js) : 0 Droite +X, 1 Gauche −X, 2 Dessus +Y, 3 Dessous −Y, 4 Avant +Z, 5 Arrière −Z
export const MARKERS = [
  { id: 0, face: 0, n: [1, 0, 0], up: [0, 1, 0], name: 'Droite', upName: 'Dessus' },
  { id: 1, face: 1, n: [-1, 0, 0], up: [0, 1, 0], name: 'Gauche', upName: 'Dessus' },
  { id: 2, face: 2, n: [0, 1, 0], up: [0, 0, -1], name: 'Dessus', upName: 'Arrière' },
  { id: 3, face: 3, n: [0, -1, 0], up: [0, 0, 1], name: 'Dessous', upName: 'Avant' },
  { id: 4, face: 4, n: [0, 0, 1], up: [0, 1, 0], name: 'Avant', upName: 'Dessus' },
  { id: 5, face: 5, n: [0, 0, -1], up: [0, 1, 0], name: 'Arrière', upName: 'Dessus' },
];
// Coins d'un marqueur dans l'ordre ArUco : haut-gauche, haut-droite, bas-droite, bas-gauche (vu de face)
const CORNER_RU = [[-1, 1], [1, 1], [1, -1], [-1, -1]];

const popcount16 = (x) => { let c = 0; while (x) { x &= x - 1; c++; } return c; };

// Valeur d'un pixel avec interpolation (le pixel i est centré en i)
function bilinear(g, w, h, x, y) {
  if (x < 0) x = 0; else if (x > w - 1.001) x = w - 1.001;
  if (y < 0) y = 0; else if (y > h - 1.001) y = h - 1.001;
  const x0 = x | 0, y0 = y | 0, fx = x - x0, fy = y - y0, k = y0 * w + x0;
  const a = g[k] + (g[k + 1] - g[k]) * fx, b = g[k + w] + (g[k + w + 1] - g[k + w]) * fx;
  return a + (b - a) * fy;
}
const applyH = (H, a, b) => {
  const z = H[6] * a + H[7] * b + H[8];
  return [(H[0] * a + H[1] * b + H[2]) / z, (H[3] * a + H[4] * b + H[5]) / z];
};

// Ajuste chaque côté sur le bord net (gradient) entre le noir du marqueur et le blanc autour
function refineEdges(g, w, h, quad) {
  const lines = [];
  for (let s = 0; s < 4; s++) {
    const a = quad[s], b = quad[(s + 1) % 4];
    const dx = b[0] - a[0], dy = b[1] - a[1], len = Math.hypot(dx, dy);
    if (len < 6) return null;
    const d = [dx / len, dy / len], nrm = [d[1], -d[0]]; // vers l'extérieur (coins dans le sens horaire à l'écran)
    const R = Math.max(2, Math.min(6, len * 0.06));
    const count = Math.max(6, Math.min(30, Math.round(len / 3)));
    const pts = [];
    for (let k = 0; k < count; k++) {
      const t = 0.15 + 0.7 * (k / (count - 1));
      const px = a[0] + dx * t, py = a[1] + dy * t;
      let best = 0, bo = 0;
      const prof = [];
      for (let o = -R; o <= R + 1e-9; o += 0.5) prof.push(bilinear(g, w, h, px + nrm[0] * o, py + nrm[1] * o));
      for (let i = 1; i < prof.length - 1; i++) {
        const gr = prof[i + 1] - prof[i - 1]; // sombre dedans -> clair dehors : gradient positif
        if (gr > best) { best = gr; bo = i; }
      }
      if (best < 12 || bo <= 1 || bo >= prof.length - 2) continue;
      const gm = prof[bo] - prof[bo - 2], gp = prof[bo + 2] - prof[bo];
      const den = gm - 2 * best + gp;
      const off = Math.abs(den) > 1e-6 ? 0.5 * (gm - gp) / den : 0;
      const o = -R + (bo + Math.max(-0.5, Math.min(0.5, off))) * 0.5;
      pts.push([px + nrm[0] * o, py + nrm[1] * o]);
    }
    if (pts.length < Math.max(5, count * 0.7)) return null; // bord caché ou flou : coins peu fiables
    let mx = 0, my = 0;
    for (const p of pts) { mx += p[0]; my += p[1]; }
    mx /= pts.length; my /= pts.length;
    let cxx = 0, cyy = 0, cxy = 0;
    for (const p of pts) { const ex = p[0] - mx, ey = p[1] - my; cxx += ex * ex; cyy += ey * ey; cxy += ex * ey; }
    const ang = 0.5 * Math.atan2(2 * cxy, cxx - cyy);
    let dd = [Math.cos(ang), Math.sin(ang)];
    if (dd[0] * d[0] + dd[1] * d[1] < 0) dd = [-dd[0], -dd[1]];
    // écart des points au bord ajusté : un côté courbé ou cassé (doigt) est refusé
    let e2 = 0;
    for (const p of pts) { const ex = p[0] - mx, ey = p[1] - my; const dist = ex * dd[1] - ey * dd[0]; e2 += dist * dist; }
    if (Math.sqrt(e2 / pts.length) > 0.6 + len * 0.004) return null;
    lines.push({ p: [mx, my], d: dd });
  }
  const out = [];
  for (let s = 0; s < 4; s++) {
    const L1 = lines[(s + 3) % 4], L2 = lines[s];
    const den = L1.d[0] * L2.d[1] - L1.d[1] * L2.d[0];
    if (Math.abs(den) < 1e-6) return null;
    const t = ((L2.p[0] - L1.p[0]) * L2.d[1] - (L2.p[1] - L1.p[1]) * L2.d[0]) / den;
    const x = L1.p[0] + L1.d[0] * t, y = L1.p[1] + L1.d[1] * t;
    const diag = Math.hypot(quad[s][0] - quad[(s + 2) % 4][0], quad[s][1] - quad[(s + 2) % 4][1]);
    if (!(Math.hypot(x - quad[s][0], y - quad[s][1]) < diag * 0.08 + 1.5)) return null;
    out.push([x, y]);
  }
  return out;
}

// =====================================================================
export class MarkerTracker {
  constructor() {
    this.size = 10; // arête du cube en cm
    this.markerSize = 8; // côté du carré noir imprimé, en cm
    this.fov = 60; // champ de vision horizontal de la webcam (degrés)
    this.ids = MARKERS.length; // seuls les ID 0 à 5 sont acceptés
    this.prev = null;
    this.last = null;
  }
  isCalibrated() { return true; } // rien à calibrer
  reset() { this.prev = null; }

  // ---- 1. Trouver les marqueurs dans l'image ----
  detect(img) {
    const { data, width: w, height: h } = img;
    const N = w * h;
    if (!this._gray || this._gray.length !== N) {
      this._gray = new Uint8Array(N);
      this._dark = new Uint8Array(N);
      this._visited = new Uint8Array(N);
      this._stack = new Int32Array(N);
      this._integral = new Uint32Array((w + 1) * (h + 1));
    }
    const g = this._gray, dark = this._dark, visited = this._visited, stack = this._stack, I = this._integral;
    for (let p = 0, k = 0; p < N; p++, k += 4) g[p] = (data[k] * 77 + data[k + 1] * 150 + data[k + 2] * 29) >> 8;
    // seuil adaptatif : plus sombre que la moyenne autour
    const W1 = w + 1;
    for (let y = 0; y < h; y++) {
      let row = 0;
      for (let x = 0; x < w; x++) {
        row += g[y * w + x];
        I[(y + 1) * W1 + x + 1] = I[y * W1 + x + 1] + row;
      }
    }
    const r = Math.max(4, Math.round(Math.min(w, h) / 40)), C = 7;
    for (let y = 0; y < h; y++) {
      const y0 = Math.max(0, y - r), y1 = Math.min(h, y + r + 1);
      for (let x = 0; x < w; x++) {
        const x0 = Math.max(0, x - r), x1 = Math.min(w, x + r + 1);
        const sum = I[y1 * W1 + x1] - I[y0 * W1 + x1] - I[y1 * W1 + x0] + I[y0 * W1 + x0];
        const mean = sum / ((x1 - x0) * (y1 - y0));
        dark[y * w + x] = g[y * w + x] < mean - C ? 1 : 0;
      }
    }
    visited.fill(0);
    const minSide = 12, maxSide = Math.max(w, h) * 0.95;
    const found = new Map();
    const rowMin = new Int32Array(h), rowMax = new Int32Array(h), colMin = new Int32Array(w), colMax = new Int32Array(w);
    for (let p0 = 0; p0 < N; p0++) {
      if (!dark[p0] || visited[p0]) continue;
      // tache sombre (le cadre noir d'un marqueur, avec les cases noires qui le touchent)
      let sp = 0, area = 0, minx = w, maxx = 0, miny = h, maxy = 0;
      const start = sp;
      stack[sp++] = p0; visited[p0] = 1;
      const pix = [];
      while (sp > start) {
        const p = stack[--sp];
        const x = p % w, y = (p - x) / w;
        area++; pix.push(p);
        if (x < minx) minx = x; if (x > maxx) maxx = x; if (y < miny) miny = y; if (y > maxy) maxy = y;
        if (x > 0 && dark[p - 1] && !visited[p - 1]) { visited[p - 1] = 1; stack[sp++] = p - 1; }
        if (x < w - 1 && dark[p + 1] && !visited[p + 1]) { visited[p + 1] = 1; stack[sp++] = p + 1; }
        if (y > 0 && dark[p - w] && !visited[p - w]) { visited[p - w] = 1; stack[sp++] = p - w; }
        if (y < h - 1 && dark[p + w] && !visited[p + w]) { visited[p + w] = 1; stack[sp++] = p + w; }
      }
      const bw = maxx - minx + 1, bh = maxy - miny + 1;
      if (bw < minSide || bh < minSide || bw > maxSide || bh > maxSide) continue;
      if (area < minSide * 4 || minx <= 0 || miny <= 0 || maxx >= w - 1 || maxy >= h - 1) continue;
      if (Math.max(bw, bh) > 5 * Math.min(bw, bh)) continue;
      // contour : extrêmes de chaque ligne et de chaque colonne
      for (let y = miny; y <= maxy; y++) { rowMin[y] = w; rowMax[y] = -1; }
      for (let x = minx; x <= maxx; x++) { colMin[x] = h; colMax[x] = -1; }
      for (const p of pix) {
        const x = p % w, y = (p - x) / w;
        if (x < rowMin[y]) rowMin[y] = x; if (x > rowMax[y]) rowMax[y] = x;
        if (y < colMin[x]) colMin[x] = y; if (y > colMax[x]) colMax[x] = y;
      }
      const boundary = [];
      for (let y = miny; y <= maxy; y++) if (rowMax[y] >= 0) { boundary.push([rowMin[y] - 0.5, y]); boundary.push([rowMax[y] + 0.5, y]); }
      for (let x = minx; x <= maxx; x++) if (colMax[x] >= 0) { boundary.push([x, colMin[x] - 0.5]); boundary.push([x, colMax[x] + 0.5]); }
      const hull = convexHull(boundary.map((q) => q.slice()));
      let quad = bestQuad(hull);
      if (!quad) continue;
      const qa = polyArea(quad);
      if (qa < polyArea(hull) * 0.88 || area < qa * 0.12) continue;
      quad = refineQuad(quad, boundary);
      const mx = (quad[0][0] + quad[1][0] + quad[2][0] + quad[3][0]) / 4, my = (quad[0][1] + quad[1][1] + quad[2][1] + quad[3][1]) / 4;
      quad.sort((a, b) => Math.atan2(a[1] - my, a[0] - mx) - Math.atan2(b[1] - my, b[0] - mx)); // sens horaire à l'écran
      let minEdge = Infinity, maxEdge = 0;
      for (let s = 0; s < 4; s++) {
        const e = Math.hypot(quad[(s + 1) % 4][0] - quad[s][0], quad[(s + 1) % 4][1] - quad[s][1]);
        minEdge = Math.min(minEdge, e); maxEdge = Math.max(maxEdge, e);
      }
      if (minEdge < 9 || maxEdge > 4 * minEdge) continue;
      const code = this._read(g, data, w, h, quad);
      if (!code) continue;
      const corners0 = [0, 1, 2, 3].map((i) => quad[(code.s + i) % 4]);
      const corners = refineEdges(g, w, h, corners0);
      if (!corners) continue;
      const prevF = found.get(code.id);
      if (prevF && prevF.area >= qa) continue; // le même ID deux fois : on garde le plus grand
      found.set(code.id, { id: code.id, corners, area: qa, side: Math.sqrt(qa), errors: code.err });
    }
    return [...found.values()];
  }

  // Lit les 4x4 cases du marqueur (avec son cadre noir : grille de 6x6)
  _read(g, rgba, w, h, quad) {
    const H = homography([[0, 0], [6, 0], [6, 6], [0, 6]], quad);
    if (!H) return null;
    const cells = new Float32Array(36);
    let lo = 255, hi = 0;
    for (let i = 0; i < 6; i++) {
      for (let j = 0; j < 6; j++) {
        let s = 0;
        for (const oy of [-0.22, 0, 0.22]) for (const ox of [-0.22, 0, 0.22]) {
          const [x, y] = applyH(H, j + 0.5 + ox, i + 0.5 + oy);
          s += bilinear(g, w, h, x, y);
        }
        const v = s / 9;
        cells[i * 6 + j] = v;
        if (v < lo) lo = v; if (v > hi) hi = v;
      }
    }
    if (hi - lo < 30) return null;
    const thr = (lo + hi) / 2;
    let borderErr = 0;
    for (let i = 0; i < 6; i++) for (let j = 0; j < 6; j++) {
      if ((i === 0 || i === 5 || j === 0 || j === 5) && cells[i * 6 + j] > thr) borderErr++;
    }
    if (borderErr > 1) return null;
    // la marge blanche tout autour doit être visible (sinon un doigt touche le marqueur : coins faux)
    let dimOut = 0;
    const ring = [[-0.4, -0.4], [6.4, -0.4], [6.4, 6.4], [-0.4, 6.4]];
    for (let k = 0; k < 6; k++) {
      const u = k + 0.5;
      ring.push([u, -0.4], [6.4, u], [u, 6.4], [-0.4, u]);
    }
    // blanc de référence : les cases blanches du marqueur
    let wsum = 0, wn = 0;
    for (let i = 1; i < 5; i++) for (let j = 1; j < 5; j++) if (cells[i * 6 + j] > thr) { wsum += cells[i * 6 + j]; wn++; }
    const white = wn ? wsum / wn : hi, ringMin = lo + (white - lo) * 0.65;
    for (const [a, b] of ring) {
      const [x, y] = applyH(H, a, b);
      if (bilinear(g, w, h, x, y) < ringMin) { dimOut++; continue; }
      // papier blanc = gris neutre ; un doigt (peau) est coloré
      const px = Math.round(Math.min(w - 1, Math.max(0, x))), py = Math.round(Math.min(h - 1, Math.max(0, y))), k = (py * w + px) * 4;
      const mx = Math.max(rgba[k], rgba[k + 1], rgba[k + 2]), mn = Math.min(rgba[k], rgba[k + 1], rgba[k + 2]);
      if (mx > 0 && (mx - mn) / mx > 0.3) dimOut++;
    }
    if (dimOut > 1) return null;
    // les 4 rotations possibles (le coin de départ s devient le haut-gauche)
    const bit = (i, j) => (cells[(i + 1) * 6 + j + 1] > thr ? 1 : 0);
    let best = null;
    for (let s = 0; s < 4; s++) {
      let code = 0;
      for (let i = 0; i < 4; i++) {
        for (let j = 0; j < 4; j++) {
          // case (i, j) du marqueur droit, quand le coin s de l'image est son haut-gauche
          let a = i, b = j;
          for (let k = 0; k < s; k++) { const t = a; a = b; b = 3 - t; }
          code = (code << 1) | bit(a, b);
        }
      }
      for (let id = 0; id < DICT_4X4_50.length; id++) {
        const e = popcount16(code ^ DICT_4X4_50[id]);
        if (e === 0) best = { id, s, err: e };
      }
    }
    if (!best || best.id >= this.ids) return null;
    return best;
  }

  // ---- 2. Position et rotation du cube ----
  _K(w) {
    const f = (w / 2) / Math.tan((this.fov * Math.PI) / 360);
    return { f, cx: 0, cy: 0 };
  }
  _model(id) {
    const M = MARKERS[id], hs = this.size / 2, m = this.markerSize / 2;
    const c = scale(M.n, hs), right = cross(M.up, M.n);
    return { c, right, up: M.up, n: M.n, corners: CORNER_RU.map(([a, b]) => add(c, add(scale(right, a * m), scale(M.up, b * m)))) };
  }
  _poseFromMarker(K, det) {
    const mod = this._model(det.id), m = this.markerSize / 2;
    const src = CORNER_RU.map(([a, b]) => [a * m, b * m]);
    const dst = det.corners.map(([x, y]) => [(x - K.cx) / K.f, (y - K.cy) / K.f]);
    const H = homography(src, dst);
    if (!H) return null;
    const m1 = [H[0], H[3], H[6]], m2 = [H[1], H[4], H[7]], m3 = [H[2], H[5], H[8]];
    let lam = 2 / (norm(m1) + norm(m2));
    if (m3[2] * lam < 0) lam = -lam;
    const Ru = scale(m1, lam), Rv = scale(m2, lam), Rn = cross(Ru, Rv);
    const A = fromCols(Ru, Rv, scale(Rn, Math.sqrt(norm(Ru) * norm(Rv)) / Math.max(1e-9, norm(Rn))));
    const R = orthonormalize(mulMM(A, transpose(fromCols(mod.right, mod.up, mod.n))));
    const t = sub(scale(m3, lam), mulMV(R, mod.c));
    if (t[2] <= 0) return null;
    return { R, t };
  }
  // L'autre pose possible d'un carré seul (ambiguïté classique) : normale symétrique par rapport à la ligne de vue
  _flipped(R, t, id) {
    const mod = this._model(id);
    const pc = add(mulMV(R, mod.c), t), v = scale(pc, 1 / norm(pc));
    const nc = mulMV(R, mod.n), nf = sub(scale(v, 2 * dot(nc, v)), nc);
    const axis = cross(nc, nf), s = norm(axis), c = dot(nc, nf);
    if (s < 1e-6) return null;
    const k = scale(axis, 1 / s), ang = Math.atan2(s, c), cs = Math.cos(ang), sn = Math.sin(ang), vv = 1 - cs;
    const Q = [
      cs + k[0] * k[0] * vv, k[0] * k[1] * vv - k[2] * sn, k[0] * k[2] * vv + k[1] * sn,
      k[1] * k[0] * vv + k[2] * sn, cs + k[1] * k[1] * vv, k[1] * k[2] * vv - k[0] * sn,
      k[2] * k[0] * vv - k[1] * sn, k[2] * k[1] * vv + k[0] * sn, cs + k[2] * k[2] * vv,
    ];
    const R2 = mulMM(Q, R);
    return { R: R2, t: sub(pc, mulMV(R2, mod.c)) };
  }
  _facing(R, t, id) {
    const mod = this._model(id);
    const n = mulMV(R, mod.n), c = add(mulMV(R, mod.c), t);
    return -dot(n, c) / norm(c); // > 0 : la face regarde la caméra
  }

  // Analyse une image (ImageData). time en secondes.
  process(img, time) {
    const w = img.width, h = img.height;
    const raw = this.detect(img);
    this.last = { dets: raw.map((d) => ({ ...d, face: MARKERS[d.id].face })), ok: false, w, h, markers: true };
    if (!raw.length) return null;
    const dets = raw.map((d) => ({ ...d, corners: d.corners.map(([x, y]) => [x - w / 2 + 0.5, y - h / 2 + 0.5]) }));
    const K = this._K(w);
    const items = [];
    for (const d of dets) {
      const mod = this._model(d.id);
      for (let i = 0; i < 4; i++) items.push({ P: mod.corners[i], q: d.corners[i] });
    }
    const prev = this.prev && time - this.prev.time < 0.5 ? this.prev : null;
    const cands = [];
    for (const d of dets) {
      const p = this._poseFromMarker(K, d);
      if (!p) continue;
      cands.push(p);
      const f = this._flipped(p.R, p.t, d.id);
      if (f && f.t[2] > 0) cands.push(f);
    }
    if (prev) cands.push({ R: prev.R, t: prev.t });
    const side = dets.reduce((s, d) => s + d.side, 0) / dets.length;
    let best = null, bestScore = Infinity;
    for (const c of cands) {
      const ref = refinePose(K, c.R, c.t, items, 12);
      if (!isFinite(ref.rms) || ref.t[2] <= 0) continue;
      let score = ref.rms / Math.max(1, side * 0.02);
      for (const d of dets) if (this._facing(ref.R, ref.t, d.id) < 0.02) score += 5; // face vue de dos : impossible
      if (prev) score += 0.6 * rotationAngle(ref.R, prev.R);
      if (score < bestScore) { bestScore = score; best = ref; }
    }
    if (!best) return null;
    // trop d'écart entre le modèle et l'image (mauvaise taille de marqueur, image floue…)
    if (!(best.rms < Math.max(1.5, side * 0.06))) return null;
    this.prev = { R: best.R, t: best.t, time };
    this.last.ok = true;
    this.last.pose = { R: best.R, t: best.t, K, rms: best.rms, faces: dets.length };
    return { R: best.R, t: best.t, rms: best.rms, faces: dets.length, ambiguous: false };
  }
}

// Dessine un marqueur (pour l'aperçu 3D) : carré blanc de côté px, marqueur de taille ratio
export function drawMarker(ctx, id, px, ratio = 0.8) {
  ctx.fillStyle = '#fff';
  ctx.fillRect(0, 0, px, px);
  const m = px * ratio, o = (px - m) / 2, c = m / 6, code = DICT_4X4_50[id];
  ctx.fillStyle = '#000';
  ctx.fillRect(o, o, m, m);
  ctx.fillStyle = '#fff';
  for (let i = 0; i < 4; i++) for (let j = 0; j < 4; j++) {
    if ((code >> (15 - (i * 4 + j))) & 1) ctx.fillRect(Math.floor(o + (j + 1) * c), Math.floor(o + (i + 1) * c), Math.ceil(c), Math.ceil(c));
  }
}
