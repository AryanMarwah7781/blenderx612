// John Deere 612R interactive viewer.
// Loads the Blender-generated GLB and spec/612r_spec.json, then drives the PIV_* pivot nodes
// with hydraulic rate limits, interlocks, vehicle kinematics and simple dynamics.
//
// Public API, for a RAG front-end embedding this page in an iframe:
//   window.sprayer.runProcedure(id, step?)  window.sprayer.highlight(meshNames[])
//   window.sprayer.setJoint(id, value)      window.sprayer.press(controlId, ms?)
//   window.sprayer.camera(name)             window.sprayer.getState()
// From a parent window:  iframe.contentWindow.postMessage({ type: "612r", cmd: "procedure", id: "op_fold_boom" }, "*")
// URL parameters:  ?procedure=op_fold_boom&step=2   ?highlight=SW_fold_L_in,BTN_park   ?camera=commandarm
//                  ?set=boom_height:2.4,fold_L:90   ?control=ca_fold_L_in

import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { GLTFLoader } from "three/addons/loaders/GLTFLoader.js";
import { RoomEnvironment } from "three/addons/environments/RoomEnvironment.js";

const BASE = new URL("..", import.meta.url);
const SPEC = await (await fetch(new URL("spec/612r_spec.json", BASE))).json();
const SPECS = Object.fromEntries(Object.entries(SPEC.specs).map(([k, v]) => [k, v.value]));
const JOINTS = Object.fromEntries(SPEC.joints.map(j => [j.id, j]));
const CONTROLS = Object.fromEntries(SPEC.controls.map(c => [c.id, c]));
const CONTROL_BY_MESH = Object.fromEntries(SPEC.controls.map(c => [c.mesh, c]));
const SERVICE_BY_MESH = Object.fromEntries(SPEC.service_points.map(s => [s.mesh, s]));
const PROCS = Object.fromEntries(SPEC.procedures.map(p => [p.id, p]));
const deg = THREE.MathUtils.degToRad;
const TIMESCALE = +(new URLSearchParams(location.search).get("timescale") || 1);   // >1 speeds up the sim (useful for tests)

// Blender (+Z up, +Y left) -> glTF/three (+Y up, -Z left)
const b2t = ([x, y, z]) => new THREE.Vector3(x, z, -y);
const AXIS = { x: new THREE.Vector3(1, 0, 0), y: new THREE.Vector3(0, 0, -1), z: new THREE.Vector3(0, 1, 0) };

// ------------------------------------------------------------------------------------ renderer / scene
const canvas = document.getElementById("c");
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
renderer.outputColorSpace = THREE.SRGBColorSpace;
renderer.toneMapping = THREE.ACESFilmicToneMapping;
const scene = new THREE.Scene();
scene.background = new THREE.Color(0xbfd3e6);
scene.fog = new THREE.Fog(0xbfd3e6, 120, 380);
const camera = new THREE.PerspectiveCamera(40, 1, 0.02, 1000);
const orbit = new OrbitControls(camera, canvas);
orbit.enableDamping = true;
orbit.maxPolarAngle = Math.PI * 0.495;
scene.environment = new THREE.PMREMGenerator(renderer).fromScene(new RoomEnvironment(), 0.04).texture;
scene.environmentIntensity = 0.6;
scene.add(new THREE.HemisphereLight(0xe8f0ff, 0x6b5a45, 1.4));
const sun = new THREE.DirectionalLight(0xffffff, 2.6);
sun.position.set(30, 50, 20);
sun.castShadow = true;
sun.shadow.mapSize.set(2048, 2048);
Object.assign(sun.shadow.camera, { left: -30, right: 30, top: 30, bottom: -30, near: 1, far: 150 });
scene.add(sun, sun.target);

// ground: crop rows on a heightfield (flat unless "rough field" is on)
let rough = false;
const groundH = (x, z) => !rough ? 0 :
  0.06 * Math.sin(x * 0.45) * Math.cos(z * 0.38) + 0.035 * Math.sin(x * 1.3 + z * 0.7) + 0.05 * Math.sin(z * 0.21 + 1.3);
const gtex = (() => {
  const c = document.createElement("canvas"); c.width = c.height = 256;
  const g = c.getContext("2d"); g.fillStyle = "#6b5338"; g.fillRect(0, 0, 256, 256);
  for (let i = 0; i < 8; i++) { g.fillStyle = "#4f7d2e"; g.fillRect(0, i * 32 + 10, 256, 12); }   // 30 in crop rows along X
  const t = new THREE.CanvasTexture(c); t.wrapS = t.wrapT = THREE.RepeatWrapping; t.repeat.set(66, 66);
  t.colorSpace = THREE.SRGBColorSpace; t.anisotropy = 8; return t;
})();
const groundGeo = new THREE.PlaneGeometry(400, 400, 200, 200).rotateX(-Math.PI / 2);
const ground = new THREE.Mesh(groundGeo, new THREE.MeshStandardMaterial({ map: gtex, roughness: 1 }));
ground.receiveShadow = true;
scene.add(ground);
function rebuildGround() {
  const p = groundGeo.attributes.position;
  for (let i = 0; i < p.count; i++) p.setY(i, groundH(p.getX(i), p.getZ(i)));
  p.needsUpdate = true; groundGeo.computeVertexNormals();
}

// ------------------------------------------------------------------------------------ load model
const gltf = await new GLTFLoader().loadAsync(new URL("build/john_deere_612r.glb", BASE).href);
const model = gltf.scene;
scene.add(model);
document.getElementById("loading").remove();
const N = {};
model.traverse(o => {
  N[o.name] = o;
  if (o.isMesh) { o.castShadow = true; o.receiveShadow = true; }
});
const ROOT = N["SPRAYER_612R"];
const REST = {};
for (const [name, o] of Object.entries(N)) if (name.startsWith("PIV_")) REST[name] = { p: o.position.clone(), q: o.quaternion.clone() };

// ------------------------------------------------------------------------------------ live display (canvas on DSP_screen)
const dspCanvas = document.createElement("canvas"); dspCanvas.width = 512; dspCanvas.height = 340;
const dspTex = new THREE.CanvasTexture(dspCanvas); dspTex.colorSpace = THREE.SRGBColorSpace; dspTex.flipY = false;
if (N["DSP_screen"]) N["DSP_screen"].material = new THREE.MeshBasicMaterial({ map: dspTex, toneMapped: false });

// ------------------------------------------------------------------------------------ state
const J = {}, T = {};                  // joint value / target
for (const j of SPEC.joints) { J[j.id] = j.default; T[j.id] = j.default; }
const st = {
  park: true, road_mode: false, spray_on: false, boomtrac: false, range: 3, throttle: 2,
  speed: 0,            // m/s
  x: 0, z: 0, yaw: 0,  // world pose of the rear-axle centre (three coordinates)
  heave: 0, pitch: 0, roll: 0, heaveV: 0, pitchV: 0, rollV: 0,
  sway: { L: [0, 0], R: [0, 0] }, bounce: { L: [0, 0], R: [0, 0] },
  wheelSpin: { FL: 0, FR: 0, RL: 0, RR: 0 },
};
const RANGE_CAP_KPH = [0, 16, 25, SPECS.max_spray_speed_kph];   // ESTIMATE range caps; road = max transport
const held = new Map();                 // controlId -> dir while a momentary switch is held
let warnMsg = "", warnT = 0;
const warn = (m) => { warnMsg = m; warnT = 3.5; };

// ------------------------------------------------------------------------------------ interlocks
const boomFolded = () => J.fold_L > 88 && J.fold_R > 88 && J.outer_L > 170 && J.outer_R > 170;
const boomUnfolded = () => J.fold_L < 1 && J.fold_R < 1 && J.outer_L < 1 && J.outer_R < 1;
function allowed(jid, dir) {
  const side = jid.slice(-1);
  if (jid === "fold_L" || jid === "fold_R") {
    if (J.boom_height < 2.1) return "Raise the boom above the tires (≥ 2.1 m) before you fold or unfold the main wings.";
    if (dir > 0 && J["outer_" + side] < 170) return "Fold the outer wing onto the inner wing first.";
  }
  if ((jid === "outer_L" || jid === "outer_R") && dir < 0 && J["fold_" + side] > 0.5) return "Unfold the main wing completely before you unfold the outer wing.";
  if ((jid === "outer_L" || jid === "outer_R") && dir > 0 && J.boom_height < 1.2) return "Raise the boom before you fold the outer wings.";
  if (jid === "tread") {
    const kph = Math.abs(st.speed) * 3.6;
    if (kph < 0.3 || kph > 8) return "Tread adjust moves only while the machine rolls slowly (0.3–8 km/h). The wheels must roll to slide.";
  }
  if (jid === "boom_height" && dir < 0 && boomFolded() && J.boom_height <= 2.2) return "The folded boom is resting in the cradles.";
  return null;
}

// ------------------------------------------------------------------------------------ kinematics: joints -> nodes
const q = new THREE.Quaternion();
function rot(name, axisKey, angRad, extra) {
  const n = N[name]; if (!n) return;
  const sign = axisKey.endsWith("-") ? -1 : 1;
  n.quaternion.copy(REST[name].q).multiply(q.setFromAxisAngle(AXIS[axisKey[0]], sign * angRad));
  if (extra) n.quaternion.multiply(extra);
}
function ackermann(steerDeg, tread) {
  const L = SPECS.wheelbase_m, s = deg(Math.abs(steerDeg));
  const inner = Math.atan(L * Math.tan(s) / (L - tread * Math.tan(s)));
  const sg = Math.sign(steerDeg);
  return { FL: steerDeg > 0 ? inner : deg(steerDeg), FR: steerDeg < 0 ? -inner : deg(steerDeg), sg };
}
const qa = new THREE.Quaternion(), qb = new THREE.Quaternion();
function applyJoints() {
  // tread: Blender ±Y (left) -> three ∓Z
  const half = (J.tread - SPECS.tread_min_m) / 2;
  for (const [n, s] of Object.entries(JOINTS.tread.nodes)) {
    N[n].position.copy(REST[n].p); N[n].position.z += (s.startsWith("+") ? -1 : 1) * half;
  }
  // suspension per corner (from dynamics) + wheel spin
  for (const c of ["FL", "FR", "RL", "RR"]) {
    const n = "PIV_susp_" + c; N[n].position.copy(REST[n].p); N[n].position.y += st.susp?.[c] ?? 0;
    rot("PIV_roll_" + c, "y", st.wheelSpin[c]);
  }
  const a = ackermann(J.steer, J.tread);
  rot("PIV_steer_FL", "z", a.FL); rot("PIV_steer_FR", "z", a.FR);
  const lk = JOINTS.boom_height.linkage;
  const th = Math.asin(THREE.MathUtils.clamp((J.boom_height - (lk.pivot_z - lk.nozzle_drop)) / lk.arm_length, -1, 1));
  rot("PIV_lift_upper", "y", th); rot("PIV_lift_lower", "y", th); rot("PIV_boom_center", "y", -th);
  for (const s of ["L", "R"]) {
    const sg = s === "L" ? 1 : -1;
    // boom dynamics: fore-aft sway (yaw at fold hinge) and vertical bounce (roll at tilt hinge)
    rot("PIV_tilt_" + s, JOINTS["tilt_" + s].nodes["PIV_tilt_" + s], deg(J["tilt_" + s]) + st.bounce[s][0]);
    rot("PIV_fold_" + s, JOINTS["fold_" + s].nodes["PIV_fold_" + s], deg(J["fold_" + s]) + st.sway[s][0]);
    rot("PIV_outer_" + s, JOINTS["outer_" + s].nodes["PIV_outer_" + s], deg(J["outer_" + s]));
    rot("PIV_tip_" + s, JOINTS["tip_" + s].nodes["PIV_tip_" + s], deg(J["tip_" + s]));
  }
  for (const id of ["ladder", "cab_door", "hood", "eductor", "hydro_handle", "steering_wheel"])
    for (const [n, ax] of Object.entries(JOINTS[id].nodes)) rot(n, ax, deg(J[id]));
  // chassis body motion
  const ch = N["PIV_chassis"];
  ch.position.set(0, st.heave, 0);
  ch.quaternion.setFromEuler(new THREE.Euler(st.roll, 0, st.pitch, "XYZ"));
  ROOT.position.set(st.x, 0, st.z);
  ROOT.quaternion.setFromAxisAngle(AXIS.z, st.yaw);
}

// ------------------------------------------------------------------------------------ simulation step
const springStep = (s, input, w, zeta, dt) => {   // 2nd-order oscillator [pos, vel]
  const acc = -w * w * (s[0] - input) - 2 * zeta * w * s[1];
  s[1] += acc * dt; s[0] += s[1] * dt;
};
function step(dt) {
  // hydraulic functions: rate-limited motion toward targets
  for (const [cid, dir] of held) {
    const c = CONTROLS[cid];
    const why = allowed(c.joint, dir);
    if (why) { warn(why); T[c.joint] = J[c.joint]; continue; }
    T[c.joint] = dir > 0 ? JOINTS[c.joint].max : JOINTS[c.joint].min;
  }
  for (const j of SPEC.joints) {
    if (j.id === "hydro_handle" || j.id === "steering_wheel") continue;
    const d = T[j.id] - J[j.id];
    if (Math.abs(d) < 1e-6) continue;
    if (!held.size || ![...held.keys()].some(k => CONTROLS[k].joint === j.id)) {
      // procedure-driven / toggled moves still honour physical interlocks
      const why = (j.id.startsWith("fold_") || j.id.startsWith("outer_") || j.id === "tread") ? allowed(j.id, Math.sign(d)) : null;
      if (why && !procRunning) { warn(why); T[j.id] = J[j.id]; continue; }
    }
    J[j.id] += Math.sign(d) * Math.min(Math.abs(d), j.rate * dt);
    if (j.spring_return && Math.abs(J[j.id]) > 0 && T[j.id] !== 0 && Math.abs(J[j.id] - T[j.id]) < 1e-3) T[j.id] = 0;
  }
  // hydro handle → commanded speed (hydrostatic: first-order with accel limit)
  if (st.park && Math.abs(J.hydro_handle) > 1) warn("Park is engaged. Take the machine out of Park (P) to drive.");
  if (!st.park && J.ladder < 75 && Math.abs(J.hydro_handle) > 1) warn("Raise the ladder before you drive.");
  const cap = (st.road_mode ? SPECS.max_transport_speed_kph : RANGE_CAP_KPH[st.range]) / 3.6;
  const frac = J.hydro_handle >= 0 ? J.hydro_handle / JOINTS.hydro_handle.max : -J.hydro_handle / JOINTS.hydro_handle.min * 0.4;
  const canDrive = !st.park && J.ladder >= 75;
  const vCmd = canDrive ? frac * cap : 0;
  const accel = THREE.MathUtils.clamp((vCmd - st.speed) * 1.2, -2.5, 1.5);
  const prevV = st.speed;
  st.speed += accel * dt;
  if (st.park) st.speed = 0;
  // steering wheel drives the steer joint (about 3 turns lock to lock)
  T.steer = THREE.MathUtils.clamp(J.steering_wheel / JOINTS.steering_wheel.max * JOINTS.steer.max, JOINTS.steer.min, JOINTS.steer.max);
  // bicycle kinematics about the rear-axle centre
  const L = SPECS.wheelbase_m, t = J.tread, r = SPECS.tire_diameter_m / 2;
  const s = deg(J.steer);
  let Rc = Infinity;
  if (Math.abs(s) > 1e-4) Rc = Math.sign(s) * (L / Math.tan(Math.abs(s)) - t / 2);   // the outer-wheel angle is the input
  const yawRate = isFinite(Rc) ? st.speed / Rc : 0;
  st.yaw += yawRate * dt;
  const fwd = new THREE.Vector3(Math.cos(st.yaw), 0, -Math.sin(st.yaw));
  st.x += fwd.x * st.speed * dt; st.z += fwd.z * st.speed * dt;
  // per-wheel rolling distance (each wheel follows its own radius)
  const wheelR = (c) => {
    if (!isFinite(Rc)) return 1;
    const y = (c[1] === "L" ? 1 : -1) * t / 2, x = c[0] === "F" ? L : 0;
    return Math.hypot(x, Rc - y) / Math.abs(Rc);
  };
  for (const c of ["FL", "FR", "RL", "RR"]) st.wheelSpin[c] += (st.speed * wheelR(c) / r) * dt;
  // suspension: ground height under each corner -> body heave/pitch/roll via spring-dampers
  const toWorld = (bx, by) => new THREE.Vector3(st.x + bx * Math.cos(st.yaw) - by * Math.sin(st.yaw), 0,
    st.z - bx * Math.sin(st.yaw) - by * Math.cos(st.yaw));
  const gh = {}; for (const c of ["FL", "FR", "RL", "RR"]) {
    const p = toWorld(c[0] === "F" ? L : 0, (c[1] === "L" ? 1 : -1) * t / 2); gh[c] = groundH(p.x, p.z);
  }
  const heaveT = (gh.FL + gh.FR + gh.RL + gh.RR) / 4;
  const pitchT = Math.atan2((gh.FL + gh.FR) / 2 - (gh.RL + gh.RR) / 2, L);
  const rollT = Math.atan2((gh.FL + gh.RL) / 2 - (gh.FR + gh.RR) / 2, t);
  const latAcc = st.speed * yawRate, lonAcc = (st.speed - prevV) / Math.max(dt, 1e-3);
  const hv = [st.heave, st.heaveV]; springStep(hv, heaveT, 7, 0.35, dt); [st.heave, st.heaveV] = hv;
  const pv = [st.pitch, st.pitchV]; springStep(pv, pitchT - lonAcc * 0.004, 6, 0.4, dt); [st.pitch, st.pitchV] = pv;
  const rv = [st.roll, st.rollV]; springStep(rv, rollT - latAcc * 0.006, 6, 0.4, dt); [st.roll, st.rollV] = rv;
  st.susp = {};
  for (const c of ["FL", "FR", "RL", "RR"]) {
    const bx = c[0] === "F" ? L : 0, by = (c[1] === "L" ? 1 : -1) * t / 2;
    const bodyZ = st.heave + Math.sin(st.pitch) * bx + Math.sin(st.roll) * by;
    st.susp[c] = THREE.MathUtils.clamp(gh[c] - bodyZ, JOINTS.suspension.min, JOINTS.suspension.max);
  }
  // boom wing dynamics (only meaningful when unfolded); BoomTrac damps the bounce
  const unf = boomUnfolded() ? 1 : 0;
  for (const sd of ["L", "R"]) {
    const sg = sd === "L" ? 1 : -1;
    springStep(st.sway[sd], unf * THREE.MathUtils.clamp(lonAcc * 0.004, -0.03, 0.03) * sg, 2.2, 0.08, dt);
    springStep(st.bounce[sd], unf * (st.roll * 1.5 * sg + st.rollV * 0.05 * sg), 1.8, st.boomtrac ? 0.6 : 0.12, dt);
  }
  // spray-on interlock (ASSUMED; check the OM)
  if (st.spray_on && (!boomUnfolded() || st.road_mode)) { st.spray_on = false; warn("Spray OFF: spraying requires the boom fully unfolded and Field mode."); }
  if (warnT > 0) warnT -= dt;
}

// ------------------------------------------------------------------------------------ controls
function pressControl(cid, down) {
  const c = CONTROLS[cid]; if (!c) return;
  if (c.kind === "momentary") { if (down) held.set(cid, c.dir); else { held.delete(cid); T[c.joint] = J[c.joint]; } return; }
  if (!down) return;
  switch (c.kind) {
    case "toggle":
      if (c.state === "road_mode" && !st.road_mode && !boomFolded()) return warn("Fold the boom completely into the cradles before you select Road mode.");
      if (c.state === "park" && st.park === false && Math.abs(st.speed) > 0.2) return warn("Stop the machine before you engage Park.");
      if (c.state === "spray_on" && !st.spray_on && !boomUnfolded()) return warn("Unfold the boom completely before you turn master spray on.");
      st[c.state] = !st[c.state]; if (c.state === "road_mode") st.range = 3; break;
    case "step": if (c.state === "range") st.range = THREE.MathUtils.clamp(st.range + c.dir, 1, 3);
                 else st.throttle = (st.throttle % 3) + 1; break;
    case "toggle_joint": { const j = JOINTS[c.joint]; T[c.joint] = (T[c.joint] > (j.min + j.max) / 2) ? j.min : j.max; break; }
    case "lever": T.hydro_handle = J.hydro_handle = J.hydro_handle > 1 ? 0 : 12; break;
    case "wheel": break;
  }
}

// ------------------------------------------------------------------------------------ highlight
const hl = new Map();   // mesh -> original material
function highlight(names = []) {
  for (const [m, orig] of hl) m.material = orig; hl.clear();
  for (const n of [].concat(names)) {
    const o = N[n]; if (!o) continue;
    o.traverse(m => {
      if (!m.isMesh || hl.has(m)) return;
      hl.set(m, m.material);
      m.material = Array.isArray(m.material) ? m.material.map(x => x.clone()) : m.material.clone();
    });
  }
}
function pulseHighlights(t) {
  const k = 0.5 + 0.5 * Math.sin(t * 6);
  for (const m of hl.keys()) for (const mat of [].concat(m.material)) if (mat.emissive) { mat.emissive.setRGB(1, 0.85 * k, 0); mat.emissiveIntensity = 0.4 + 0.8 * k; }
}

// ------------------------------------------------------------------------------------ camera tweening
let camTween = null;
function goCamera(name) {
  const c = SPEC.cameras[name]; if (!c) return;
  const toLocal = (v) => ROOT.localToWorld(b2t(v));
  ROOT.updateMatrixWorld(true);
  camTween = { t: 0, p0: camera.position.clone(), t0: orbit.target.clone(), p1: toLocal(c.pos), t1: toLocal(c.target), f0: camera.fov, f1: c.fov ?? 40 };
}
function stepCamera(dt) {
  if (!camTween) return;
  camTween.t = Math.min(1, camTween.t + dt / 1.2);
  const e = camTween.t < .5 ? 2 * camTween.t ** 2 : 1 - (-2 * camTween.t + 2) ** 2 / 2;
  camera.position.lerpVectors(camTween.p0, camTween.p1, e);
  orbit.target.lerpVectors(camTween.t0, camTween.t1, e);
  camera.fov = THREE.MathUtils.lerp(camTween.f0, camTween.f1, e); camera.updateProjectionMatrix();
  if (camTween.t >= 1) camTween = null;
}

// ------------------------------------------------------------------------------------ procedures
let proc = null, procStep = 0, procRunning = false, autoPlay = false;
const $ = (id) => document.getElementById(id);
function runProcedure(id, stepIdx = 0) {
  proc = PROCS[id]; if (!proc) return;
  $("step").style.display = "block"; showStep(stepIdx);
}
function showStep(i) {
  procStep = THREE.MathUtils.clamp(i, 0, proc.steps.length - 1);
  const s = proc.steps[procStep];
  $("s_n").textContent = `${proc.title} · ${proc.audience} · step ${procStep + 1} of ${proc.steps.length}`;
  $("s_t").textContent = s.text;
  if (s.camera) goCamera(s.camera);
  highlight(s.highlight ? [].concat(s.highlight) : []);
  if (s.state) for (const [k, v] of Object.entries(s.state)) { if (k === "speed_kph") { st.park = false; J.ladder = T.ladder = 80; T.hydro_handle = J.hydro_handle = v / RANGE_CAP_KPH[st.range] * JOINTS.hydro_handle.max; } else st[k] = v; }
  if (s.set) { procRunning = true; for (const [k, v] of Object.entries(s.set)) { T[k] = v; if (k === "hydro_handle") J[k] = v; } }
}
const settled = () => SPEC.joints.every(j => Math.abs(T[j.id] - J[j.id]) < 1e-3 || j.id === "steer");

// ------------------------------------------------------------------------------------ UI
function renderList(tab) {
  const L = $("list"); L.innerHTML = "";
  const add = (html, on) => { const b = document.createElement("button"); b.className = "item"; b.innerHTML = html; if (on) Object.assign(b, on); L.appendChild(b); return b; };
  const h = (t) => { const e = document.createElement("h3"); e.textContent = t; L.appendChild(e); };
  if (tab === "procedures") for (const aud of ["operator", "mechanic"]) {
    h(aud === "operator" ? "Operator" : "Mechanic / service");
    for (const p of SPEC.procedures.filter(p => p.audience === aud)) add(`<span>${p.title}</span><small>${p.steps.length} steps</small>`, { onclick: () => runProcedure(p.id) });
  }
  if (tab === "controls") for (const loc of ["hydro_handle", "commandarm", "cab", "exterior"]) {
    h(loc.replace("_", " "));
    for (const c of SPEC.controls.filter(c => c.location === loc)) {
      const b = add(`<span>${c.label}</span>${c.verified ? "" : '<span class="tag unv" title="Placement not yet checked against the OM">verify</span>'}`);
      b.onpointerdown = () => { highlight([c.mesh]); goCamera(loc === "exterior" ? "left_side" : "commandarm"); pressControl(c.id, true); };
      b.onpointerup = b.onpointerleave = () => pressControl(c.id, false);
    }
  }
  if (tab === "service") for (const aud of ["operator", "mechanic"]) {
    h(aud);
    for (const s of SPEC.service_points.filter(s => s.audience === aud))
      add(`<span>${s.label}</span><span class="tag">${s.src}</span>`, { onclick: () => showService(s) });
  }
  if (tab === "joints") for (const j of SPEC.joints) {
    const w = document.createElement("label"); w.style.display = "block"; w.style.margin = "6px 2px";
    w.innerHTML = `<div style="display:flex;justify-content:space-between"><span>${j.label}</span><span class="tag">${j.src}</span></div>`;
    const r = document.createElement("input"); Object.assign(r, { type: "range", min: j.min, max: j.max, step: (j.max - j.min) / 200, value: T[j.id] });
    r.style.width = "100%"; r.oninput = () => { T[j.id] = +r.value; if (j.id === "hydro_handle" || j.id === "steering_wheel") J[j.id] = +r.value; };
    w.appendChild(r); L.appendChild(w);
  }
}
function showService(s) {
  highlight([s.mesh]);
  if (s.requires?.hood) T.hood = JOINTS.hood.max;
  const o = N[s.mesh]; if (o) {
    const p = new THREE.Vector3(); o.getWorldPosition(p);
    camTween = { t: 0, p0: camera.position.clone(), t0: orbit.target.clone(), p1: p.clone().add(new THREE.Vector3(2.2, 1.6, p.z < ROOT.position.z ? 3.2 : -3.2).applyQuaternion(ROOT.quaternion)), t1: p, f0: camera.fov, f1: 40 };
  }
  info(s.label, `${s.audience} · position ${s.src}${s.requires ? " · requires the hood open" : ""}`, s.id);
}
function info(title, sub, id) {
  const e = $("info"); e.style.display = "block";
  e.innerHTML = `<h4>${title}</h4><p>${sub}</p><p><code>${id}</code></p>`;
}
document.querySelectorAll(".tabs button").forEach(b => b.onclick = () => {
  document.querySelectorAll(".tabs button").forEach(x => x.classList.toggle("on", x === b)); renderList(b.dataset.tab);
});
renderList("procedures");
const camBar = $("cams");
for (const name of Object.keys(SPEC.cameras)) { const b = document.createElement("button"); b.textContent = name.replace("_", " "); b.onclick = () => goCamera(name); camBar.appendChild(b); }
$("s_prev").onclick = () => showStep(procStep - 1);
$("s_next").onclick = () => showStep(procStep + 1);
$("s_close").onclick = () => { $("step").style.display = "none"; proc = null; autoPlay = false; highlight([]); procRunning = false; };
$("s_auto").onclick = () => { autoPlay = !autoPlay; $("s_auto").classList.toggle("on", autoPlay); };
$("rough").onclick = () => { rough = !rough; $("rough").textContent = rough ? "on" : "off"; rebuildGround(); };

// picking: mesh -> nearest ancestor that is a control or service point
const ray = new THREE.Raycaster(), ptr = new THREE.Vector2();
function pick(ev) {
  const r = canvas.getBoundingClientRect();
  ptr.set(((ev.clientX - r.left) / r.width) * 2 - 1, -((ev.clientY - r.top) / r.height) * 2 + 1);
  ray.setFromCamera(ptr, camera);
  for (const hit of ray.intersectObject(model, true)) {
    if (hit.object.material?.opacity < 0.5) continue;           // click through glass
    for (let o = hit.object; o; o = o.parent) {
      if (CONTROL_BY_MESH[o.name]) return { control: CONTROL_BY_MESH[o.name], o };
      if (SERVICE_BY_MESH[o.name]) return { service: SERVICE_BY_MESH[o.name], o };
    }
    return { part: hit.object };
  }
  return null;
}
let downPick = null;
canvas.addEventListener("pointerdown", (ev) => {
  const p = pick(ev); downPick = p;
  if (p?.control) { orbit.enabled = false; highlight([p.control.mesh]); info(p.control.label, `${p.control.location.replace("_", " ")} · ${p.control.kind}${p.control.verified ? "" : " · position not yet verified against the OM"}`, p.control.id); pressControl(p.control.id, true); }
  else if (p?.service) showService(p.service);
});
addEventListener("pointerup", () => { if (downPick?.control) pressControl(downPick.control.id, false); downPick = null; orbit.enabled = true; });
const tip = $("tip");
canvas.addEventListener("pointermove", (ev) => {
  const p = pick(ev);
  const label = p?.control?.label || p?.service?.label;
  canvas.style.cursor = label ? "pointer" : "grab";
  if (label) { tip.style.display = "block"; tip.style.left = ev.clientX + 14 + "px"; tip.style.top = ev.clientY + 10 + "px"; tip.textContent = label; }
  else tip.style.display = "none";
});
const keys = new Set();
addEventListener("keydown", (e) => {
  if (e.target.tagName === "INPUT") return;
  keys.add(e.key.toLowerCase());
  if (e.key === " ") { T.hydro_handle = J.hydro_handle = 0; st.speed *= 0.5; }
  if (e.key.toLowerCase() === "p") pressControl("ca_park", true);
  if (e.key.toLowerCase() === "b") { T.tip_L = JOINTS.tip_L.max; }
});
addEventListener("keyup", (e) => keys.delete(e.key.toLowerCase()));
function keyInput(dt) {
  if (keys.has("w")) J.hydro_handle = T.hydro_handle = Math.min(JOINTS.hydro_handle.max, J.hydro_handle + 15 * dt);
  if (keys.has("s")) J.hydro_handle = T.hydro_handle = Math.max(JOINTS.hydro_handle.min, J.hydro_handle - 15 * dt);
  if (keys.has("a")) J.steering_wheel = Math.min(JOINTS.steering_wheel.max, J.steering_wheel + 300 * dt);
  if (keys.has("d")) J.steering_wheel = Math.max(JOINTS.steering_wheel.min, J.steering_wheel - 300 * dt);
}

// ------------------------------------------------------------------------------------ HUD + display
function hud() {
  const kph = st.speed * 3.6;
  $("h_speed").textContent = `${kph.toFixed(1)} km/h (${(kph / 1.609).toFixed(1)} mph)`;
  $("h_mode").textContent = st.road_mode ? "Road (transport)" : `Field · range ${st.range}`;
  $("h_park").textContent = st.park ? "ON" : "off";
  $("h_boom").textContent = `${J.boom_height.toFixed(2)} m${st.boomtrac ? " · BoomTrac" : ""}`;
  $("h_fold").textContent = boomFolded() ? "Folded (transport)" : boomUnfolded() ? "Unfolded" : `Moving · main ${J.fold_L.toFixed(0)}/${J.fold_R.toFixed(0)}° outer ${J.outer_L.toFixed(0)}/${J.outer_R.toFixed(0)}°`;
  $("h_tread").textContent = `${J.tread.toFixed(2)} m (${(J.tread / 0.0254).toFixed(0)} in)`;
  $("h_steer").textContent = `${J.steer.toFixed(1)}° · R ${Math.abs(J.steer) > 0.2 ? (SPECS.wheelbase_m / Math.sin(deg(Math.abs(J.steer)))).toFixed(1) + " m" : "∞"}`;
  $("h_spray").textContent = st.spray_on ? "ON" : "OFF";
  $("warn").textContent = warnT > 0 ? warnMsg : "";
  const g = dspCanvas.getContext("2d");
  g.fillStyle = "#0e1a10"; g.fillRect(0, 0, 512, 340);
  g.fillStyle = "#367c2b"; g.fillRect(0, 0, 512, 38);
  g.fillStyle = "#fff"; g.font = "bold 20px system-ui"; g.fillText("612R  ·  " + (st.road_mode ? "ROAD" : "FIELD"), 14, 26);
  g.font = "bold 64px system-ui"; g.fillStyle = "#ffde00"; g.fillText(kph.toFixed(1), 18, 120);
  g.font = "18px system-ui"; g.fillStyle = "#cfe8c8"; g.fillText("km/h   range " + st.range + (st.park ? "   PARK" : ""), 20, 148);
  const rows = [["Boom", J.boom_height.toFixed(2) + " m"], ["Tread", (J.tread / 0.0254).toFixed(0) + " in"], ["Spray", st.spray_on ? "ON" : "OFF"], ["Fold", boomFolded() ? "Transport" : boomUnfolded() ? "Field" : "Moving"]];
  rows.forEach(([k, v], i) => { g.fillStyle = "#9fb89a"; g.fillText(k, 280, 80 + i * 32); g.fillStyle = "#fff"; g.fillText(v, 380, 80 + i * 32); });
  // boom section bar (11 sections on a 120 ft boom)
  for (let i = 0; i < 11; i++) { g.fillStyle = st.spray_on ? "#2e9e44" : "#3a463a"; g.fillRect(20 + i * 43, 200, 38, 22); }
  if (warnT > 0) { g.fillStyle = "#b3261e"; g.fillRect(0, 290, 512, 50); g.fillStyle = "#fff"; g.font = "15px system-ui"; g.fillText(warnMsg.slice(0, 60), 12, 320); }
  dspTex.needsUpdate = true;
}

// ------------------------------------------------------------------------------------ public API (RAG hook)
window.sprayer = {
  spec: SPEC, fps: () => fps,
  runProcedure, highlight, camera: goCamera,
  setJoint: (id, v) => { T[id] = v; },
  press: (cid, ms = 1500) => { pressControl(cid, true); if (CONTROLS[cid]?.kind === "momentary") setTimeout(() => pressControl(cid, false), ms); },
  getState: () => ({ joints: { ...J }, ...st, boomFolded: boomFolded(), boomUnfolded: boomUnfolded() }),
};
addEventListener("message", (e) => {
  const m = e.data; if (!m || m.type !== "612r") return;
  if (m.camera && m.cmd !== "camera") goCamera(m.camera);
  if (m.cmd === "press" && CONTROLS[m.control]) highlight([CONTROLS[m.control].mesh]);
  if (m.cmd === "procedure") runProcedure(m.id, m.step ?? 0);
  if (m.cmd === "highlight") highlight(m.meshes);
  if (m.cmd === "camera") goCamera(m.name);
  if (m.cmd === "set") for (const [k, v] of Object.entries(m.joints || {})) T[k] = v;
  if (m.cmd === "press") window.sprayer.press(m.control, m.ms);
});
const qs = new URLSearchParams(location.search);
goCamera(qs.get("camera") || "overview"); if (camTween) camTween.t = 0.999;
if (qs.get("set")) for (const kv of qs.get("set").split(",")) { const [k, v] = kv.split(":"); J[k] = T[k] = +v; }
if (qs.get("highlight")) highlight(qs.get("highlight").split(","));
if (qs.get("control")) { const c = CONTROLS[qs.get("control")]; if (c) { highlight([c.mesh]); goCamera("commandarm"); } }
if (qs.get("procedure")) runProcedure(qs.get("procedure"), +(qs.get("step") || 0));

// ------------------------------------------------------------------------------------ main loop
function resize() {
  const w = innerWidth, h = innerHeight;
  renderer.setSize(w, h, false); camera.aspect = w / h; camera.updateProjectionMatrix();
}
addEventListener("resize", resize); resize();
const clock = new THREE.Clock();
let acc = 0, autoWait = 0, fps = 0;
renderer.setAnimationLoop(() => {
  const raw = clock.getDelta(); fps = fps * 0.9 + 0.1 / Math.max(raw, 1e-3);
  const dt = Math.min(raw, 0.1) * TIMESCALE;
  keyInput(dt);
  acc += dt; while (acc > 1 / 120) { step(1 / 120); acc -= 1 / 120; }
  if (procRunning && settled()) procRunning = false;
  if (proc && autoPlay && !procRunning) { autoWait += dt; if (autoWait > 1.8) { autoWait = 0; if (procStep < proc.steps.length - 1) showStep(procStep + 1); else autoPlay = false; } }
  applyJoints();
  // follow the machine with the camera when it drives
  const before = new THREE.Vector3(st.x, 0, st.z);
  if (st._last) { const d = before.clone().sub(st._last); camera.position.add(d); orbit.target.add(d); }
  st._last = before;
  sun.position.set(st.x + 30, 50, st.z + 20); sun.target.position.set(st.x, 0, st.z);
  stepCamera(dt);
  orbit.update();
  pulseHighlights(clock.elapsedTime);
  hud();
  renderer.render(scene, camera);
});
