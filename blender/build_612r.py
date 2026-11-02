"""
Procedural Blender model of a John Deere 612R self-propelled sprayer.

Run (headless):   blender -b -P blender/build_612r.py -- --out build
            or:   python3 blender/build_612r.py --out build          (pip install bpy)
Optional:         --render   (photoreal Cycles previews -> docs/render_*.png)

Everything is generated from spec/612r_spec.json.
  * Every moving part hangs under an empty named PIV_<joint>, at the real pivot point.
  * The root object "SPRAYER_612R" carries one custom property per joint, and drivers
    connect each property to the pivots. In Blender: select SPRAYER_612R, open
    Object Properties > Custom Properties, and drag the values.
  * Clickable controls and service points are separate meshes named CTL_/BTN_/SW_/SVC_/DSP_*,
    so a web viewer (viewer/) or a RAG pipeline can refer to them by ID.

Layout follows photos of production 612R machines:
  * Engine at the FRONT under a tractor-style nose (black grille, tilting hood).
  * Cab over the front axle; exhaust stack at the left front cab corner.
  * Stainless 1,200 gal tank between the axles; black poly rinse tank on top.
  * Rear-mounted green steel boom on a parallel-link lift; folds forward alongside
    the machine with the tips reaching past the nose.
  * Hydraulic green ladder ahead of the left front wheel; black fenders / mud flaps.

Axes: +X forward, +Y left, +Z up. Origin = rear axle centre on the ground. Units: metres.
"""
import json
import math
import os
import sys

import bpy  # noqa: E402  (bpy must be imported before bmesh when used as a pip module)
import bmesh
from mathutils import Matrix, Vector

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)

argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else sys.argv[1:]
OUT = os.path.join(ROOT, argv[argv.index("--out") + 1]) if "--out" in argv else os.path.join(ROOT, "build")
DO_RENDER = "--render" in argv
os.makedirs(OUT, exist_ok=True)

with open(os.path.join(ROOT, "spec", "612r_spec.json")) as f:
    SPEC = json.load(f)
S = {k: v["value"] for k, v in SPEC["specs"].items()}
JOINTS = {j["id"]: j for j in SPEC["joints"]}

# ----------------------------------------------------------------------------- dimensions
WB = S["wheelbase_m"]                      # 4.30
TREAD0 = S["tread_min_m"]                  # 3.05 (modelled rest tread)
TIRE_R = S["tire_diameter_m"] / 2          # 1.034
TIRE_W = S["tire_width_m"]                 # 0.38
RIM_R = 0.635                              # 50 in rim
CLEAR = S["crop_clearance_m"]              # 1.47 underside of chassis
FRAME_BOT = CLEAR + 0.03
FRAME_TOP = 1.86
LEG_INSET = 0.36                           # leg centre is this far inboard of the wheel centre

# Cab interior/shell is authored in a local "cab frame" (x 4.20..6.30) and the whole
# cab_mount is shifted by CAB_DX so the cab sits over the front axle like the real machine.
CAB_X0, CAB_X1 = 4.20, 6.30
CAB_DX = -1.25                             # real cab: x 2.95 .. 5.05
CAB_HW = 0.95
CAB_FLOOR = 2.28
CAB_ROOF = 3.92
NOSE_X0, NOSE_X1 = CAB_X1 + CAB_DX, 6.75   # engine hood / nose

LINK = JOINTS["boom_height"]["linkage"]
ARM_L = LINK["arm_length"]
UPPER_PIV = Vector((-0.60, 0, LINK["pivot_z"]))
LOWER_PIV = Vector((-0.60, 0, LINK["pivot_z"] - 0.60))
NOZ_DROP = LINK["nozzle_drop"]
BOOM_X = UPPER_PIV.x - ARM_L               # centre-frame plane at rest (theta = 0)
NOZ_Z0 = UPPER_PIV.z - NOZ_DROP            # nozzle height at theta = 0
BL = SPEC["boom_layout_per_side_m"]
Y_HINGE = BL["center_half"]
Y_OUTER = Y_HINGE + BL["inner_wing"]
Y_TIP = Y_OUTER + BL["outer_wing"]
Y_END = Y_TIP + BL["breakaway"]
BOOM_CHORD_Z = NOZ_Z0 + 0.10               # bottom chord of the truss

# ----------------------------------------------------------------------------- scene reset
bpy.ops.wm.read_factory_settings(use_empty=True)
scene = bpy.context.scene
scene.unit_settings.system = "METRIC"
COLL = bpy.data.collections.new("612R")
scene.collection.children.link(COLL)
ENV = bpy.data.collections.new("Environment")
scene.collection.children.link(ENV)

# ----------------------------------------------------------------------------- materials
def _set(bsdf, name, value):
    if name in bsdf.inputs:
        bsdf.inputs[name].default_value = value

def mat(name, rgb, metal=0.0, rough=0.5, alpha=1.0, emit=None, coat=0.0, emit_strength=3.0):
    m = bpy.data.materials.new(name)
    try:
        m.use_nodes = True
    except Exception:
        pass
    bsdf = m.node_tree.nodes["Principled BSDF"]
    _set(bsdf, "Base Color", (*rgb, 1))
    _set(bsdf, "Metallic", metal)
    _set(bsdf, "Roughness", rough)
    if coat:
        _set(bsdf, "Coat Weight", coat)
        _set(bsdf, "Coat Roughness", 0.08)
    if alpha < 1:
        _set(bsdf, "Alpha", alpha)
        try:
            m.surface_render_method = "BLENDED"
        except AttributeError:
            m.blend_method = "BLEND"
    if emit:
        _set(bsdf, "Emission Color", (*emit, 1))
        _set(bsdf, "Emission Strength", emit_strength)
    m.diffuse_color = (*rgb, alpha)
    m.metallic, m.roughness = metal, rough
    return m

def srgb(h):
    c = [int(h[i:i + 2], 16) / 255 for i in (0, 2, 4)]
    return tuple(((x + 0.055) / 1.055) ** 2.4 if x > 0.04045 else x / 12.92 for x in c)

M = {
    "green":   mat("JD_Green_Paint", srgb("367C2B"), 0.0, 0.32, coat=0.7),
    "green_d": mat("JD_Green_Cast", srgb("2F6B26"), 0.0, 0.55, coat=0.3),
    "yellow":  mat("JD_Yellow_Paint", srgb("FFDE00"), 0.0, 0.35, coat=0.6),
    "black":   mat("Black_Plastic", srgb("151515"), 0.0, 0.55),
    "black_g": mat("Black_Gloss", srgb("0E0E0E"), 0.0, 0.18, coat=0.5),
    "tire":    mat("Tire_Rubber", srgb("1B1B1B"), 0.0, 0.88),
    "steel":   mat("Stainless", srgb("D2D5D8"), 1.0, 0.22),
    "dsteel":  mat("Dark_Steel", srgb("3E4246"), 0.7, 0.45),
    "glass":   mat("Cab_Glass", srgb("1E2A30"), 0.0, 0.03, alpha=0.30),
    "seat":    mat("Seat_Leather_Gray", srgb("B8B5AD"), 0.0, 0.55),
    "int_gray": mat("Interior_Gray", srgb("9D9A92"), 0.0, 0.6),
    "int_dark": mat("Interior_Charcoal", srgb("2E2F31"), 0.0, 0.5),
    "btn_o":   mat("JD_Orange_Trim", srgb("E2601E"), 0.0, 0.45),
    "int_pad": mat("Interior_Textured", srgb("8E8B83"), 0.0, 0.85),
    "glyph":   mat("Key_Legend", srgb("E8E8E8"), 0.0, 0.4, emit=srgb("FFFFFF"), emit_strength=0.3),
    "engine":  mat("Engine_Green", srgb("2E6A25"), 0.1, 0.5),
    "red":     mat("Hydraulic_Red", srgb("A0201A"), 0.1, 0.4),
    "tail":    mat("Tail_Light", srgb("C0101A"), 0.0, 0.15, emit=srgb("FF1010"), emit_strength=1.0),
    "smv":     mat("SMV_Orange", srgb("FF4A12"), 0.0, 0.3, emit=srgb("FF4A12"), emit_strength=0.4),
    "orange":  mat("Nozzle_Tip", srgb("E87722"), 0.0, 0.5),
    "blue":    mat("DEF_Blue", srgb("1F5FAF"), 0.0, 0.4),
    "white":   mat("Poly_White", srgb("E8E8E2"), 0.0, 0.5),
    "screen":  mat("Display", srgb("0B1F12"), 0.0, 0.2, emit=srgb("1D4D2A")),
    "btn_g":   mat("Btn_Green", srgb("2E9E44"), 0.0, 0.4),
    "btn_y":   mat("Btn_Yellow", srgb("F2C200"), 0.0, 0.4),
    "btn_r":   mat("Btn_Red", srgb("C8202A"), 0.0, 0.4),
    "btn_k":   mat("Btn_Charcoal", srgb("3B3D40"), 0.0, 0.45),
    "lens":    mat("LED_Lens", srgb("F5F5F0"), 0.0, 0.05, emit=srgb("FFFFFF"), emit_strength=1.5),
    "amber":   mat("Beacon_Amber", srgb("FF9A00"), 0.0, 0.2, emit=srgb("FF8800"), emit_strength=1.5),
    "hose":    mat("Hose_Black", srgb("101010"), 0.0, 0.7),
}

# ----------------------------------------------------------------------------- object helpers
WORLD = {}   # name -> world-space Vector of the object origin (all pivots are unrotated at rest)

def empty(name, world, parent=None, size=0.25):
    o = bpy.data.objects.new(name, None)
    o.empty_display_type = "ARROWS"
    o.empty_display_size = size
    COLL.objects.link(o)
    world = Vector(world)
    WORLD[name] = world
    if parent is not None:
        o.parent = parent
        o.location = world - WORLD[parent.name]
    else:
        o.location = world
    return o


class Builder:
    """Collects primitives (world coordinates) into one mesh object parented to a pivot."""

    def __init__(self, name, parent, origin=None, local=False):
        self.name, self.parent, self.local = name, parent, local
        self.origin = Vector(origin) if origin is not None else WORLD[parent.name]
        self.bm = bmesh.new()
        self.mats = []

    def _mi(self, m):
        if m not in self.mats:
            self.mats.append(m)
        return self.mats.index(m)

    def _tag(self, geom_verts, m, smooth=False):
        mi = self._mi(m)
        faces = {f for v in geom_verts for f in v.link_faces}
        for f in faces:
            f.material_index = mi
            f.smooth = smooth

    def _xf(self, p):
        return Vector(p) if self.local else Vector(p) - self.origin

    def box(self, center, size, m, rot=None):
        mtx = Matrix.Translation(self._xf(center))
        if rot is not None:
            mtx = mtx @ rot.to_4x4()
        mtx = mtx @ Matrix.Diagonal((*size, 1))
        r = bmesh.ops.create_cube(self.bm, size=1.0, matrix=mtx)
        self._tag(r["verts"], m)
        return self

    def cyl(self, center, radius, depth, m, axis="Z", segs=24, r2=None, rot=None, smooth=True):
        if rot is None:
            rot = {"X": Matrix.Rotation(math.pi / 2, 3, "Y"),
                   "Y": Matrix.Rotation(math.pi / 2, 3, "X"),
                   "Z": Matrix.Identity(3)}[axis]
        mtx = Matrix.Translation(self._xf(center)) @ rot.to_4x4()
        r = bmesh.ops.create_cone(self.bm, cap_ends=True, segments=segs, radius1=radius,
                                  radius2=radius if r2 is None else r2, depth=depth, matrix=mtx)
        self._tag(r["verts"], m, smooth and segs > 6)
        if smooth and segs > 6:   # keep the caps flat
            for f in {f for v in r["verts"] for f in v.link_faces}:
                if len(f.verts) > 4:
                    f.smooth = False
        return self

    def tube(self, p1, p2, radius, m, segs=8):
        p1, p2 = Vector(p1), Vector(p2)
        d = p2 - p1
        if d.length < 1e-6:
            return self
        rot = Vector((0, 0, 1)).rotation_difference(d.normalized()).to_matrix()
        return self.cyl((p1 + p2) / 2, radius, d.length, m, rot=rot, segs=segs)

    def bar(self, p1, p2, w, m):
        """Square steel section between two points (4-sided tube)."""
        p1, p2 = Vector(p1), Vector(p2)
        d = p2 - p1
        if d.length < 1e-6:
            return self
        rot = Vector((0, 0, 1)).rotation_difference(d.normalized()).to_matrix() @ Matrix.Rotation(math.pi / 4, 3, "Z")
        return self.cyl((p1 + p2) / 2, w * 0.7071, d.length, m, rot=rot, segs=4, smooth=False)

    def sphere(self, center, radius, m, segs=16, scale=(1, 1, 1)):
        mtx = Matrix.Translation(self._xf(center)) @ Matrix.Diagonal((*scale, 1))
        r = bmesh.ops.create_uvsphere(self.bm, u_segments=segs, v_segments=segs // 2, radius=radius, matrix=mtx)
        self._tag(r["verts"], m, True)
        return self

    def loft(self, rings, m, smooth=True, cap=True):
        """rings: list of lists of world-space points (same count, closed loops)."""
        vs = [[self.bm.verts.new(self._xf(p)) for p in ring] for ring in rings]
        new = [v for ring in vs for v in ring]
        n = len(rings[0])
        for a, b in zip(vs, vs[1:]):
            for i in range(n):
                self.bm.faces.new((a[i], a[(i + 1) % n], b[(i + 1) % n], b[i]))
        caps = []
        if cap:
            caps.append(self.bm.faces.new(list(reversed(vs[0]))))
            caps.append(self.bm.faces.new(vs[-1]))
        self._tag(new, m, smooth)
        for f in caps:
            f.smooth = False
        return self

    def lathe(self, center, profile, m, axis="Y", steps=64, smooth=True):
        """profile: list of (radius, offset-along-axis). Revolved 360 degrees about the axis."""
        c = Vector(self._xf(center))
        ax = {"X": Vector((1, 0, 0)), "Y": Vector((0, 1, 0)), "Z": Vector((0, 0, 1))}[axis]
        perp = Vector((0, 0, 1)) if axis != "Z" else Vector((1, 0, 0))
        rings = []
        for k in range(steps):
            q = Matrix.Rotation(2 * math.pi * k / steps, 3, ax)
            rings.append([self.bm.verts.new(c + q @ (perp * r + ax * o)) for r, o in profile])
        n = len(profile)
        for k in range(steps):
            a, b = rings[k], rings[(k + 1) % steps]
            for i in range(n - 1):
                self.bm.faces.new((a[i], a[i + 1], b[i + 1], b[i]))
        self._tag([v for r in rings for v in r], m, smooth)
        return self

    def done(self, bevel=0.0, smooth=False, props=None):
        me = bpy.data.meshes.new(self.name)
        bmesh.ops.recalc_face_normals(self.bm, faces=self.bm.faces)
        self.bm.to_mesh(me)
        self.bm.free()
        for m in self.mats:
            me.materials.append(m)
        if smooth:
            for p in me.polygons:
                p.use_smooth = True
        o = bpy.data.objects.new(self.name, me)
        COLL.objects.link(o)
        o.parent = self.parent
        o.location = self.origin - WORLD[self.parent.name] if not self.local else Vector()
        WORLD[self.name] = self.origin if not self.local else WORLD[self.parent.name]
        if bevel > 0:
            mod = o.modifiers.new("Bevel", "BEVEL")
            mod.width, mod.segments, mod.limit_method = bevel, 2, "ANGLE"
            mod.harden_normals = False
        for k, v in (props or {}).items():
            o[k] = v
        return o


def part(name, parent, fn, origin=None, **kw):
    b = Builder(name, parent, origin)
    fn(b)
    return b.done(**kw)


def tag(o, kind, ref_id, label):
    """Metadata exported as glTF 'extras', which the viewer and RAG pipeline read."""
    o["kind"] = kind
    o["ref_id"] = ref_id
    o["label"] = label


def rrect(x, w, z0, z1, r, n=5, y0=0.0):
    """Closed ring (in a plane x = const) of a box section with rounded top corners."""
    pts = [(x, y0 + w, z0)]
    for k in range(n + 1):                           # left top corner
        a = math.pi * 0.5 * k / n
        pts.append((x, y0 + w - r + r * math.cos(a), z1 - r + r * math.sin(a)))
    for k in range(n + 1):                           # right top corner
        a = math.pi * 0.5 + math.pi * 0.5 * k / n
        pts.append((x, y0 - w + r + r * math.cos(a), z1 - r + r * math.sin(a)))
    pts.append((x, y0 - w, z0))
    return [Vector(p) for p in pts]

# ============================================================================= ROOT + CHASSIS
root = empty("SPRAYER_612R", (0, 0, 0), size=1.0)
root["model"] = SPEC["model"]
root["spec_version"] = SPEC["spec_version"]

chassis = empty("PIV_chassis", (0, 0, 0), root)   # body heave/pitch/roll are applied here by the viewer

CRADLE_X = 3.05
def frame(b):
    for y in (0.42, -0.42):                                                   # twin main rails
        b.box((2.95, y, (FRAME_BOT + FRAME_TOP) / 2), (7.3, 0.18, FRAME_TOP - FRAME_BOT), M["green"])
    for x in (-0.55, 0.0, 1.3, 2.4, 3.5, 4.3, 5.6, 6.4):
        b.box((x, 0, FRAME_TOP - 0.12), (0.14, 0.84, 0.18), M["green"])
    for x in (0.0, WB):                                                       # axle carriers (outer telescopes)
        for s in (1, -1):
            b.box((x, s * 0.80, FRAME_TOP - 0.16), (0.32, 0.78, 0.28), M["green"])
    # rear boom mount tower behind the tank
    b.box((-0.52, 0, (LOWER_PIV.z + UPPER_PIV.z) / 2 - 0.15), (0.24, 1.10, 1.10), M["green"])
    b.box((-0.40, 0, FRAME_TOP + 0.05), (0.40, 1.10, 0.12), M["green"])
    # boom cradles (fold rests) beside the cab front corners
    for s in (1, -1):
        b.box((CRADLE_X, s * 1.30, 2.16), (0.14, 0.80, 0.12), M["green"])
        b.box((CRADLE_X, s * 1.78, 2.24), (0.30, 0.36, 0.06), M["black"])
        b.box((CRADLE_X, s * 1.78, 2.33), (0.30, 0.06, 0.18), M["black"])
        b.tube((CRADLE_X, s * 0.95, FRAME_TOP - 0.1), (CRADLE_X, s * 1.25, 2.12), 0.04, M["green"])
    # entry platform at the ladder top (left)
    b.box((5.30, 1.08, CAB_FLOOR - 0.08), (0.75, 0.34, 0.05), M["dsteel"])
    for x in (4.98, 5.62):
        b.tube((x, 1.24, CAB_FLOOR - 0.06), (x, 1.24, CAB_FLOOR + 0.85), 0.018, M["yellow"])
    b.tube((4.98, 1.24, CAB_FLOOR + 0.85), (5.62, 1.24, CAB_FLOOR + 0.85), 0.018, M["yellow"])
part("chassis_frame", chassis, frame, bevel=0.012)

# ============================================================================= WHEEL LEGS
def tire_profile():
    """Half-section of a 380/105R50 tire: (radius, lateral offset)."""
    w = TIRE_W / 2
    R = TIRE_R
    pts = [(RIM_R + 0.005, -w * 0.86), (RIM_R + 0.07, -w * 0.97), (RIM_R + 0.20, -w * 1.02),
           (R - 0.16, -w * 1.00), (R - 0.07, -w * 0.93), (R - 0.025, -w * 0.78), (R - 0.02, 0.0),
           (R - 0.025, w * 0.78), (R - 0.07, w * 0.93), (R - 0.16, w * 1.00), (RIM_R + 0.20, w * 1.02),
           (RIM_R + 0.07, w * 0.97), (RIM_R + 0.005, w * 0.86)]
    return pts

def build_leg(tag_, x, side, front):
    y_wheel = side * TREAD0 / 2
    y_leg = side * (TREAD0 / 2 - LEG_INSET)
    hub_z = TIRE_R
    tread_piv = empty(f"PIV_tread_{tag_}", (x, y_leg, FRAME_TOP - 0.16), chassis)

    def inner_tube(b):   # inner telescoping axle beam, slides with the tread
        b.box((x, y_leg - side * 0.55, FRAME_TOP - 0.16), (0.24, 1.10, 0.22), M["green"])
        b.box((x, y_leg, FRAME_TOP - 0.08), (0.42, 0.40, 0.40), M["green_d"])
    part(f"axle_beam_{tag_}", tread_piv, inner_tube, bevel=0.02)

    def strut_body(b):
        b.cyl((x, y_leg, FRAME_TOP - 0.42), 0.14, 0.55, M["green_d"])
        b.cyl((x, y_leg, FRAME_TOP + 0.20), 0.19, 0.22, M["black"], segs=32)   # air spring
    o = part(f"SVC_strut_{tag_}" if tag_ == "FL" else f"strut_body_{tag_}", tread_piv, strut_body)
    if tag_ == "FL":
        tag(o, "service_point", "sp_strut_FL", "Front-left air-ride strut")

    susp = empty(f"PIV_susp_{tag_}", (x, y_leg, FRAME_TOP - 0.16), tread_piv)

    def strut_rod(b):
        b.cyl((x, y_leg, hub_z + 0.55), 0.09, 0.75, M["steel"])
        b.cyl((x, y_leg, hub_z + 0.12), 0.18, 0.34, M["green_d"])   # cast leg knuckle
    part(f"strut_rod_{tag_}", susp, strut_rod)

    if not front:   # rear mud flap with tail light, like production machines
        def flap(b):
            fx = x - TIRE_R - 0.14
            b.box((fx, y_wheel, 0.95), (0.03, TIRE_W + 0.14, 1.0), M["black"])
            b.box((fx - 0.02, y_wheel, 0.62), (0.03, 0.22, 0.10), M["tail"])
            b.tube((fx + 0.02, y_wheel - side * 0.1, 1.45), (x - 0.25, y_leg, hub_z + 0.45), 0.035, M["black"])
            ring = []
            for k in range(13):                                   # top-rear arc over the wheel
                a = math.radians(95 + 85 * k / 12)
                ring.append((x + (TIRE_R + 0.10) * math.cos(a), hub_z + (TIRE_R + 0.10) * math.sin(a)))
            for (ax_, az), (bx_, bz) in zip(ring, ring[1:]):
                b.box(((ax_ + bx_) / 2, y_wheel, (az + bz) / 2), (math.hypot(bx_ - ax_, bz - az) + 0.01, TIRE_W + 0.14, 0.025),
                      M["black"], rot=Matrix.Rotation(-math.atan2(bz - az, bx_ - ax_), 3, "Y"))
        part(f"mudflap_{tag_}", susp, flap, smooth=False)

    steer_parent = susp
    if front:
        steer_parent = empty(f"PIV_steer_{tag_}", (x, y_leg, hub_z), susp)
        def fender(b):   # black front fender over the top-rear of the tyre, steers with the wheel
            pts = []
            for k in range(15):
                a = math.radians(40 + 120 * k / 14)
                pts.append((x + (TIRE_R + 0.09) * math.cos(a), hub_z + (TIRE_R + 0.09) * math.sin(a)))
            for (ax_, az), (bx_, bz) in zip(pts, pts[1:]):
                b.box(((ax_ + bx_) / 2, y_wheel, (az + bz) / 2), (math.hypot(bx_ - ax_, bz - az) + 0.01, TIRE_W + 0.16, 0.025),
                      M["black"], rot=Matrix.Rotation(-math.atan2(bz - az, bx_ - ax_), 3, "Y"))
            b.tube((x, y_leg, hub_z + 0.75), (x, y_wheel - side * 0.05, hub_z + TIRE_R + 0.08), 0.03, M["black"])
        part(f"fender_{tag_}", steer_parent, fender)

    def hub(b):
        b.cyl((x, (y_leg + y_wheel) / 2 - side * 0.02, hub_z), 0.27, abs(y_wheel - y_leg) - 0.05, M["green_d"], axis="Y", segs=32)
        b.cyl((x, y_leg, hub_z), 0.21, 0.26, M["green_d"], axis="Y", segs=32)
    o = part(f"SVC_wheel_motor_{tag_}" if tag_ == "FL" else f"wheel_motor_{tag_}", steer_parent, hub)
    if tag_ == "FL":
        tag(o, "service_point", "sp_wheel_motor_FL", "Front-left wheel motor / final drive")

    roll = empty(f"PIV_roll_{tag_}", (x, y_wheel, hub_z), steer_parent)

    def tire(b):
        b.lathe((x, y_wheel, hub_z), tire_profile(), M["tire"], steps=72)
        n = 34   # R1W bar lugs: two offset rows meeting near the centre (chevron)
        for i in range(n):
            for s2 in (1, -1):
                a = 2 * math.pi * (i + (0.5 if s2 < 0 else 0)) / n
                rr = TIRE_R - 0.005
                c = Vector((x + rr * math.cos(a), y_wheel + s2 * TIRE_W * 0.25, hub_z + rr * math.sin(a)))
                rot = Matrix.Rotation(-a + math.pi / 2, 3, "Y") @ Matrix.Rotation(s2 * 0.62, 3, "Z")
                b.box(c, (0.055, TIRE_W * 0.56, 0.055), M["tire"], rot=rot)
    part(f"tire_{tag_}", roll, tire)

    def rim(b):
        w = TIRE_W * 0.86
        b.lathe((x, y_wheel, hub_z), [(RIM_R + 0.025, -w / 2), (RIM_R, -w / 2 + 0.02), (RIM_R, w / 2 - 0.02), (RIM_R + 0.025, w / 2)],
                M["yellow"], steps=64)
        sgn = side
        b.lathe((x, y_wheel, hub_z), [(RIM_R - 0.005, sgn * 0.02), (0.42, sgn * 0.09), (0.30, sgn * 0.12), (0.0, sgn * 0.12)],
                M["yellow"], steps=48)
        b.cyl((x, y_wheel + sgn * 0.13, hub_z), 0.16, 0.06, M["yellow"], axis="Y", segs=32)
        for i in range(10):
            a = 2 * math.pi * i / 10
            b.cyl((x + 0.215 * math.cos(a), y_wheel + sgn * 0.13, hub_z + 0.215 * math.sin(a)), 0.024, 0.05, M["dsteel"], axis="Y", segs=6)
        for i in range(8):        # hand holes in the disc
            a = 2 * math.pi * (i + 0.5) / 8
            b.cyl((x + 0.50 * math.cos(a), y_wheel + sgn * 0.055, hub_z + 0.50 * math.sin(a)), 0.055, 0.03, M["black"], axis="Y", segs=16)
    part(f"rim_{tag_}", roll, rim)

for tag_, x, side, front in (("FL", WB, 1, True), ("FR", WB, -1, True), ("RL", 0.0, 1, False), ("RR", 0.0, -1, False)):
    build_leg(tag_, x, side, front)

# ============================================================================= ENGINE (front, under the nose)
def svc(name, ref, label, fn, parent=None):
    o = part(name, parent or chassis, fn)
    tag(o, "service_point", ref, label)
    return o

def engine(b):
    b.box((5.80, 0, 2.05), (1.15, 0.62, 0.62), M["engine"])          # 9.0 L PowerTech block
    b.box((5.80, 0, 2.45), (1.05, 0.40, 0.20), M["engine"])          # valve cover
    b.cyl((5.30, 0.32, 2.38), 0.13, 0.2, M["dsteel"], axis="Y")      # turbo
    b.box((5.15, 0, 1.95), (0.25, 0.70, 0.55), M["dsteel"])          # hydrostatic pump drive
    b.cyl((5.15, -0.42, 1.95), 0.12, 0.25, M["dsteel"], axis="Y")
    for y in (0.48, -0.48):
        b.box((5.85, y, 1.80), (1.5, 0.08, 0.30), M["green"])         # nose sub-frame
part("engine_block", chassis, engine, bevel=0.01)

svc("SVC_engine_oil", "sp_engine_oil", "Engine oil dipstick and fill",
    lambda b: b.cyl((6.05, 0.36, 2.55), 0.03, 0.24, M["yellow"]).cyl((6.05, 0.36, 2.68), 0.045, 0.04, M["yellow"]))
svc("SVC_air_filter", "sp_air_filter", "Engine air filter housing",
    lambda b: b.cyl((5.80, -0.38, 2.48), 0.15, 0.62, M["black"], axis="X").cyl((5.45, -0.38, 2.48), 0.05, 0.08, M["yellow"], axis="X"))
svc("SVC_fuel_filter", "sp_fuel_filter", "Fuel filters / water separator",
    lambda b: b.cyl((6.15, -0.40, 2.05), 0.055, 0.24, M["white"]).cyl((6.30, -0.40, 2.05), 0.055, 0.24, M["white"]))
svc("SVC_cooling", "sp_cooling", "Cooling package (radiator / charge-air cooler)",
    lambda b: b.box((6.52, 0, 2.25), (0.16, 0.95, 0.85), M["dsteel"]))
svc("SVC_hyd_reservoir", "sp_hyd_reservoir", "Hydraulic reservoir (98.4 L)",
    lambda b: b.box((3.35, -0.92, 1.95), (0.50, 0.32, 0.45), M["green"]).cyl((3.35, -0.92, 2.20), 0.05, 0.06, M["black"]).box((3.35, -1.085, 1.95), (0.12, 0.01, 0.2), M["glass"]))
svc("SVC_battery", "sp_battery", "Battery box / disconnect",
    lambda b: b.box((3.40, 0.86, 1.72), (0.45, 0.30, 0.30), M["black"]).cyl((3.40, 1.02, 1.78), 0.03, 0.05, M["red"], axis="Y"))
svc("SVC_air_tank", "sp_air_tank", "Onboard air compressor tank",
    lambda b: b.cyl((2.0, 0, FRAME_BOT + 0.12), 0.11, 0.9, M["dsteel"], axis="X"))

# exhaust stack at the left front corner of the cab
part("exhaust_stack", chassis, lambda b: b.cyl((NOSE_X0 + 0.10, 0.50, 3.35), 0.075, 1.30, M["black"])
     .cyl((NOSE_X0 + 0.10, 0.50, 4.03), 0.085, 0.08, M["black"]).box((NOSE_X0 + 0.10, 0.50, 2.80), (0.20, 0.20, 0.25), M["dsteel"]))

# fuel + DEF (right side, between the axles)
part("fuel_tank", chassis, lambda b: b.box((2.10, -0.98, 1.92), (1.55, 0.45, 0.62), M["black"]), bevel=0.04)
svc("SVC_fuel_fill", "sp_fuel_fill", "Fuel fill (565 L)", lambda b: b.cyl((2.60, -1.24, 2.15), 0.06, 0.08, M["yellow"], axis="Y"))
svc("SVC_def_fill", "sp_def_fill", "DEF fill (29.5 L)",
    lambda b: b.box((3.05, -0.98, 1.95), (0.30, 0.40, 0.40), M["white"]).cyl((3.05, -1.21, 2.08), 0.05, 0.06, M["blue"], axis="Y"))

# engine hood / nose: hinged at the cab, the front tilts up (grille goes with it)
hood_piv = empty("PIV_hood", (NOSE_X0 + 0.02, 0, 2.97), chassis)
def hood(b):
    secs = []
    for t, w, z0, z1, r in ((0.00, 0.58, 1.70, 2.97, 0.20), (0.45, 0.58, 1.70, 2.96, 0.22),
                            (0.85, 0.57, 1.72, 2.92, 0.22), (1.00, 0.55, 1.74, 2.86, 0.21)):
        x = NOSE_X0 + 0.02 + t * (NOSE_X1 - NOSE_X0 - 0.06)
        secs.append(rrect(x, w, z0, z1, r))
    b.loft(secs, M["green"])
    gx = NOSE_X1 - 0.02
    b.loft([rrect(gx, 0.50, 1.80, 2.62, 0.16), rrect(gx + 0.06, 0.48, 1.82, 2.60, 0.15)], M["black_g"])   # grille
    for i in range(7):
        b.box((gx + 0.065, 0, 1.92 + i * 0.1), (0.01, 0.86, 0.025), M["black"])
    b.box((gx + 0.07, 0, 2.48), (0.012, 0.16, 0.10), M["yellow"])                 # emblem plate
    for s in (1, -1):
        b.box((gx - 0.04, s * 0.36, 2.74), (0.08, 0.26, 0.07), M["lens"])         # LED headlight strips
        b.box((NOSE_X0 + 0.9, s * 0.585, 2.45), (0.75, 0.01, 0.30), M["black"])   # side louvres
        for i in range(6):
            b.box((NOSE_X0 + 0.58 + i * 0.12, s * 0.592, 2.45), (0.04, 0.008, 0.26), M["dsteel"])
part("engine_hood", hood_piv, hood, smooth=False)
o = part("CTL_hood_latch", hood_piv, lambda b: b.box((NOSE_X1 + 0.06, 0, 1.76), (0.05, 0.16, 0.05), M["yellow"]))
tag(o, "control", "ex_hood_latch", "Engine hood latch")

# ============================================================================= SOLUTION SYSTEM
TANK_X0, TANK_X1 = -0.25, 2.85
TANK_A, TANK_B = 0.85, 0.55                # 1,200 gal: pi*a*b*L = 4.55 m^3
TANK_ZC = 1.86 + TANK_B
def tank(b):
    rot = Matrix.Rotation(math.pi / 2, 3, "Y") @ Matrix.Diagonal((TANK_B / TANK_A, 1, 1))
    b.cyl(((TANK_X0 + TANK_X1) / 2, 0, TANK_ZC), TANK_A, TANK_X1 - TANK_X0, M["steel"], rot=rot, segs=56)
    for x in (TANK_X0 + 0.35, (TANK_X0 + TANK_X1) / 2, TANK_X1 - 0.35):        # green hold-down straps
        b.cyl((x, 0, TANK_ZC), TANK_A + 0.012, 0.08, M["green"], rot=rot, segs=56)
    for x in (TANK_X0 + 0.35, TANK_X1 - 0.35):
        b.box((x, 0, 1.92), (0.14, 1.5, 0.12), M["green"])
    for s in (1, -1):                                                             # top handrails
        b.tube((TANK_X0 + 0.2, s * 0.62, TANK_ZC + TANK_B + 0.35), (TANK_X1 - 0.2, s * 0.62, TANK_ZC + TANK_B + 0.35), 0.02, M["yellow"])
        for x in (TANK_X0 + 0.2, TANK_X1 - 0.2):
            b.tube((x, s * 0.62, TANK_ZC + TANK_B * 0.6), (x, s * 0.62, TANK_ZC + TANK_B + 0.35), 0.02, M["yellow"])
part("solution_tank", chassis, tank)
def tank_lid(b):
    b.cyl((2.25, 0.25, TANK_ZC + TANK_B), 0.22, 0.10, M["black"], segs=32)
    b.cyl((2.25, 0.25, TANK_ZC + TANK_B + 0.06), 0.18, 0.04, M["yellow"], segs=32)
svc("SVC_tank_lid", "sp_solution_tank", "Solution tank lid (4,542 L)", tank_lid)
def rinse(b):   # black poly rinse tank sitting on top of the solution tank (as on production units)
    secs = [rrect(x, 0.52, TANK_ZC + TANK_B - 0.06, TANK_ZC + TANK_B + 0.40, 0.14) for x in (0.10, 0.20, 1.60, 1.70)]
    secs[0] = rrect(0.10, 0.46, TANK_ZC + TANK_B - 0.02, TANK_ZC + TANK_B + 0.34, 0.12)
    secs[-1] = rrect(1.70, 0.46, TANK_ZC + TANK_B - 0.02, TANK_ZC + TANK_B + 0.34, 0.12)
    b.loft(secs, M["black"])
    b.cyl((1.2, 0, TANK_ZC + TANK_B + 0.42), 0.09, 0.06, M["dsteel"])
svc("SVC_rinse_tank", "sp_rinse_tank", "Rinse tank (646 L)", rinse)
part("handwash_tank", chassis, lambda b: b.cyl((-0.05, 0.40, TANK_ZC + TANK_B + 0.12), 0.17, 0.55, M["yellow"], axis="Y", segs=32))
svc("SVC_solution_pump", "sp_solution_pump", "Solution pump (hydraulically driven centrifugal)",
    lambda b: b.cyl((2.4, 0, FRAME_BOT + 0.08), 0.17, 0.22, M["dsteel"], axis="Y").cyl((2.4, 0.18, FRAME_BOT + 0.08), 0.09, 0.15, M["red"], axis="Y"))

# left-side fill station + plumbing panel
def plumbing(b):
    b.box((1.90, 1.00, 1.72), (1.35, 0.06, 0.42), M["green"])
    b.tube((1.2, 1.08, 1.60), (2.7, 1.08, 1.60), 0.035, M["steel"])
    b.tube((2.4, 0.4, FRAME_BOT + 0.08), (2.4, 1.0, 1.60), 0.04, M["steel"])
    b.tube((1.3, 1.08, 1.80), (2.6, 1.08, 1.80), 0.025, M["hose"])
part("plumbing_panel", chassis, plumbing)
svc("SVC_suction_strainer", "sp_suction_strainer", "Suction strainer (20 mesh)",
    lambda b: b.cyl((1.35, 1.15, 1.68), 0.08, 0.26, M["white"]).cyl((1.35, 1.15, 1.84), 0.09, 0.05, M["black"]))
svc("SVC_pressure_strainer", "sp_pressure_strainer", "Pressure strainer (50 mesh)",
    lambda b: b.cyl((1.70, 1.15, 1.68), 0.07, 0.24, M["white"]).cyl((1.70, 1.15, 1.83), 0.08, 0.05, M["red"]))
svc("SVC_quick_fill", "sp_quick_fill", "3 in quick-fill coupler (side fill)",
    lambda b: b.cyl((2.55, 1.16, 1.60), 0.055, 0.14, M["black"], axis="Y").cyl((2.55, 1.24, 1.60), 0.07, 0.03, M["yellow"], axis="Y"))
for i, x in enumerate((2.05, 2.25)):
    part(f"valve_handle_{i}", chassis, lambda b, x=x: b.box((x, 1.10, 1.80), (0.04, 0.06, 0.16), M["yellow"]))

# chemical eductor on a parallel-link arm (the hopper stays level)
EDU_PIV = Vector((2.30, 1.06, 1.45))
edu_piv = empty("PIV_eductor", EDU_PIV, chassis)
def edu_arm(b):
    b.tube(EDU_PIV, EDU_PIV + Vector((0, 0, 0.62)), 0.03, M["green"])
    b.tube(EDU_PIV + Vector((0.12, 0, 0)), EDU_PIV + Vector((0.12, 0, 0.62)), 0.03, M["green"])
part("eductor_arm", edu_piv, edu_arm)
edu_lvl = empty("PIV_eductor_level", EDU_PIV + Vector((0, 0, 0.62)), edu_piv)
def edu_hopper(b):
    c = EDU_PIV + Vector((0.06, 0.24, 0.62))
    b.cyl(c + Vector((0, 0, 0.18)), 0.22, 0.30, M["steel"], r2=0.24, segs=32)
    b.cyl(c, 0.05, 0.12, M["steel"], r2=0.2)
    b.cyl(c + Vector((0, 0, 0.35)), 0.25, 0.03, M["black"], segs=32)
part("eductor_hopper", edu_lvl, edu_hopper)
o = part("CTL_eductor_handle", edu_lvl, lambda b: b.box(EDU_PIV + Vector((0.06, 0.50, 0.75)), (0.05, 0.10, 0.03), M["yellow"]))
tag(o, "control", "ex_eductor", "Chemical eductor (lower/raise)")

# ============================================================================= CAB (authored in the cab frame, then shifted)
cab = empty("cab_mount", (0, 0, 0), chassis)      # WORLD = cab frame origin; location shifted at the end

def cab_shell(b):
    cx = (CAB_X0 + CAB_X1) / 2
    z0 = CAB_FLOOR - 0.22
    b.box((cx, 0, z0 + 0.10), (CAB_X1 - CAB_X0 + 0.06, 2 * CAB_HW + 0.06, 0.24), M["green"])          # floor pan
    b.box((cx - 0.05, 0, (FRAME_TOP + z0) / 2 + 0.05), (1.5, 1.2, z0 - FRAME_TOP), M["green"])         # cab mount
    b.box((cx, 0, CAB_FLOOR + 0.18), (CAB_X1 - CAB_X0, 2 * CAB_HW, 0.40), M["black"])                  # lower body
    # pillars (A, B on the door side, C)
    for x in (CAB_X0 + 0.05, CAB_X1 - 0.07):
        for y in (CAB_HW - 0.05, -CAB_HW + 0.05):
            b.box((x, y, (CAB_FLOOR + CAB_ROOF) / 2), (0.09, 0.09, CAB_ROOF - CAB_FLOOR), M["black_g"])
    b.box((CAB_X1 - 0.55, -CAB_HW + 0.04, (CAB_FLOOR + CAB_ROOF) / 2), (0.06, 0.06, CAB_ROOF - CAB_FLOOR), M["black_g"])
    b.box((CAB_X0 + 0.03, 0, CAB_FLOOR + 0.80), (0.05, 2 * CAB_HW, 0.85), M["black"])                 # rear lower wall
    # roof: rounded green cap with black underside
    rz0, rz1 = CAB_ROOF - 0.02, CAB_ROOF + 0.20
    secs = [rrect(x, w, rz0, rz1, 0.10) for x, w in ((CAB_X0 - 0.12, 1.02), (CAB_X0 - 0.06, 1.08), (CAB_X1 + 0.10, 1.08), (CAB_X1 + 0.22, 1.00))]
    b.loft(secs, M["green"])
    b.box((cx + 0.05, 0, rz0 - 0.03), (CAB_X1 - CAB_X0 + 0.30, 2 * CAB_HW + 0.14, 0.06), M["black"])
    for s in (1, -1):                                                                                 # roof LED lights
        for y in (0.25, 0.62):
            b.box((CAB_X1 + 0.22, s * y, CAB_ROOF + 0.07), (0.03, 0.26, 0.07), M["lens"])
        b.box((CAB_X0 - 0.13, s * 0.75, CAB_ROOF + 0.07), (0.03, 0.22, 0.07), M["lens"])
        b.cyl((CAB_X0 + 0.25, s * 0.82, CAB_ROOF + 0.30), 0.07, 0.16, M["amber"])                     # beacons
    # StarFire receiver
    b.cyl((cx + 0.35, 0, CAB_ROOF + 0.24), 0.17, 0.06, M["green"], segs=32)
    b.sphere((cx + 0.35, 0, CAB_ROOF + 0.28), 0.15, M["black_g"], segs=24, scale=(1, 1, 0.55))
    # mirrors on long arms from the front roof corners
    for s in (1, -1):
        p0 = Vector((CAB_X1 + 0.05, s * (CAB_HW - 0.02), CAB_ROOF - 0.10))
        p1 = Vector((CAB_X1 + 0.18, s * (CAB_HW + 0.55), CAB_ROOF - 0.25))
        b.tube(p0, p1, 0.022, M["black"])
        b.tube(p1, p1 + Vector((0, 0, -0.35)), 0.02, M["black"])
        b.box(p1 + Vector((0.03, s * 0.05, -0.12)), (0.10, 0.22, 0.42), M["green"])
        b.box(p1 + Vector((-0.025, s * 0.05, -0.12)), (0.01, 0.19, 0.38), M["steel"])
        b.box(p1 + Vector((0.03, s * 0.05, -0.47)), (0.08, 0.18, 0.18), M["green"])                   # convex mirror
part("cab_shell", cab, cab_shell, bevel=0.012)

def cab_glass(b):
    h = CAB_ROOF - CAB_FLOOR - 0.38
    zc = CAB_FLOOR + 0.38 + h / 2
    tilt = Matrix.Rotation(math.radians(-6), 3, "Y")
    b.box((CAB_X1 - 0.06, 0, zc), (0.02, 2 * CAB_HW - 0.12, h), M["glass"], rot=tilt)                 # windscreen (raked)
    b.box(((CAB_X0 + CAB_X1) / 2, -CAB_HW + 0.03, (CAB_FLOOR + CAB_ROOF) / 2), (CAB_X1 - CAB_X0 - 0.12, 0.02, CAB_ROOF - CAB_FLOOR - 0.04), M["glass"])
    b.box((CAB_X1 - 0.30, CAB_HW - 0.03, (CAB_FLOOR + CAB_ROOF) / 2), (0.36, 0.02, CAB_ROOF - CAB_FLOOR - 0.04), M["glass"])
    b.box((CAB_X0 + 0.02, 0, CAB_ROOF - 0.55), (0.02, 2 * CAB_HW - 0.12, 1.0), M["glass"])            # rear window
part("cab_glass", cab, cab_glass)

# door: left side, hinged on the rear-left post
DOOR_X0, DOOR_X1 = CAB_X0 + 0.10, CAB_X1 - 0.50
door_piv = empty("PIV_cab_door", (DOOR_X0, CAB_HW, CAB_FLOOR), cab)
def door(b):
    h = CAB_ROOF - CAB_FLOOR - 0.06
    b.box(((DOOR_X0 + DOOR_X1) / 2, CAB_HW, CAB_FLOOR + h / 2 + 0.02), (DOOR_X1 - DOOR_X0, 0.02, h), M["glass"])
    b.box(((DOOR_X0 + DOOR_X1) / 2, CAB_HW + 0.005, CAB_FLOOR + 0.20), (DOOR_X1 - DOOR_X0, 0.03, 0.36), M["black"])
    b.box((DOOR_X1 - 0.03, CAB_HW, CAB_FLOOR + h / 2), (0.05, 0.04, h), M["black_g"])
    b.box((DOOR_X0 + 0.02, CAB_HW, CAB_FLOOR + h / 2), (0.04, 0.04, h), M["black_g"])
part("cab_door", door_piv, door)
o = part("CTL_door_handle", door_piv, lambda b: b.box((DOOR_X1 - 0.10, CAB_HW + 0.04, CAB_FLOOR + 1.0), (0.16, 0.03, 0.04), M["dsteel"]))
tag(o, "control", "ex_cab_door", "Cab door handle")

# ---------------------------------------------------------------- interior
# Light-gray interior, matching photos of a production 612R cab (dealer gallery, Sloan Implement).
SEAT = Vector((4.95, 0.02, CAB_FLOOR))
def interior(b):
    b.box((CAB_X0 + 0.5, 0, CAB_FLOOR + 0.01), (1.8, 1.7, 0.02), M["int_dark"])        # floor mat
    b.box(SEAT + Vector((0, 0, 0.22)), (0.30, 0.30, 0.44), M["int_dark"])             # seat suspension + bellows
    # seat: light-gray leather with dark side bolsters
    b.loft([rrect(x, 0.27, CAB_FLOOR + 0.44, CAB_FLOOR + 0.58, 0.05, y0=0.02) for x in (4.70, 4.74, 5.20, 5.25)], M["seat"])
    for s in (1, -1):
        b.box(SEAT + Vector((0.02, s * 0.25, 0.55)), (0.48, 0.06, 0.10), M["int_dark"])
    back = Matrix.Rotation(math.radians(-12), 3, "Y")
    b.box(SEAT + Vector((-0.24, 0, 0.98)), (0.13, 0.50, 0.78), M["seat"], rot=back)
    b.box(SEAT + Vector((-0.21, 0, 1.10)), (0.06, 0.30, 0.42), M["int_dark"], rot=back)  # backrest insert
    b.box(SEAT + Vector((-0.31, 0, 1.47)), (0.10, 0.30, 0.16), M["seat"], rot=back)      # headrest
    b.box(SEAT + Vector((0.02, 0.33, 0.72)), (0.34, 0.08, 0.06), M["seat"])              # left armrest
    b.box(SEAT + Vector((0.14, 0.33, 0.70)), (0.05, 0.05, 0.03), M["btn_r"])
    b.box((CAB_X1 - 0.30, 0, CAB_FLOOR + 0.06), (0.30, 0.6, 0.08), M["int_dark"])       # pedal plate
    for y in (0.12, -0.12):
        b.box((CAB_X1 - 0.30, y, CAB_FLOOR + 0.16), (0.06, 0.1, 0.16), M["dsteel"])     # brake / decel pedals
    b.box((CAB_X0 + 0.28, -0.75, CAB_FLOOR + 0.75), (0.35, 0.32, 0.25), M["int_gray"])  # rear right console
    b.box((CAB_X1 - 0.10, -0.80, CAB_ROOF - 0.55), (0.06, 0.16, 0.26), M["int_gray"])   # corner-post display
    b.box((CAB_X1 - 0.135, -0.80, CAB_ROOF - 0.55), (0.005, 0.13, 0.21), M["screen"])
    b.box((CAB_X0 + 0.9, 0, CAB_ROOF - 0.08), (1.4, 1.6, 0.06), M["int_gray"])          # headliner
part("cab_interior", cab, interior, bevel=0.01)

# steering column (tilted, light gray with stalks) + steering wheel pivot
COL_BASE = Vector((5.62, 0.02, CAB_FLOOR + 0.05))
COL_TOP = Vector((5.42, 0.02, CAB_FLOOR + 0.95))
def column(b):
    b.tube(COL_BASE, COL_TOP, 0.07, M["int_gray"], segs=16)
    b.box(COL_BASE + Vector((0.02, 0, 0.30)), (0.20, 0.30, 0.58), M["int_gray"])
    for i in range(5):                                                                 # column grille
        b.box(COL_BASE + Vector((-0.085, 0, 0.12 + i * 0.05)), (0.01, 0.24, 0.02), M["int_dark"])
    stalk = COL_TOP + Vector((0.06, 0, -0.10))
    b.tube(stalk, stalk + Vector((-0.04, 0.20, -0.02)), 0.012, M["int_dark"])          # turn / wiper stalk (left)
    b.tube(stalk, stalk + Vector((-0.06, -0.17, 0.02)), 0.016, M["int_dark"])          # right stalk
part("steering_column", cab, column)
sw_tilt = empty("PIV_steering_tilt", COL_TOP, cab)
sw_tilt.rotation_euler = (0, -math.atan2(COL_BASE.x - COL_TOP.x, COL_TOP.z - COL_BASE.z), 0)
sw_piv = bpy.data.objects.new("PIV_steering_wheel", None); COLL.objects.link(sw_piv)
sw_piv.parent = sw_tilt; sw_piv.empty_display_size = 0.1
WORLD["PIV_steering_wheel"] = COL_TOP
bw = Builder("CTL_steering_wheel", sw_piv, local=True)
bw.lathe((0, 0, 0.02), [(0.185, -0.016), (0.205, -0.022), (0.222, 0.0), (0.205, 0.022), (0.185, 0.016), (0.185, -0.016)], M["int_dark"], axis="Z", steps=48)
for a in (math.pi / 2, math.pi * 1.15, -math.pi * 0.15):                              # three spokes
    bw.box((0.10 * math.cos(a), 0.10 * math.sin(a), 0.02), (0.20, 0.035, 0.018), M["int_dark"], rot=Matrix.Rotation(a, 3, "Z"))
bw.cyl((0, 0, 0.03), 0.065, 0.05, M["int_dark"], segs=24)
bw.box((0.0, 0, 0.058), (0.05, 0.05, 0.004), M["green"])                              # hub badge
bw.cyl((0.205, -0.07, 0.07), 0.028, 0.10, M["int_dark"], segs=20)                      # spinner knob
o = bw.done()
tag(o, "control", "ca_steering_wheel", "Steering wheel")

# ---------------------------------------------------------------- CommandARM
# Layout traced from photos of production 612R cabs (MachineFinder listings 11882541, 10321271,
# 10409271, 11690386: near top-down and over-the-shoulder views). Front to back:
#   display on a post at the front, with a 2-row key strip + rotary knob under it
#   hydro handle at the FRONT-INBOARD corner, moving fore/aft in a gate slot
#   front pocket + two thumbwheels; outboard column of 2x8 keys with a thumbwheel on top
#   centre: lock key, orange double key, boom decal, 5x3 radio/HVAC block
#   inboard: 2x3 keypad and the orange (P) park key; rear: large padded armrest
# The PHYSICAL layout follows the photos. Which key does which function is NOT confirmed from
# the operator's manual yet (only the (P) park key is identified by its printed symbol), so the
# controls stay "verified": false in the spec.
CA_Y = -0.40                  # armrest centre line (inboard = +y, towards the seat)
CA_Z = CAB_FLOOR + 0.70
TOP = CA_Z + 0.078            # console top surface

def commandarm(b):
    b.box((4.92, CA_Y, CA_Z - 0.26), (0.13, 0.13, 0.40), M["int_dark"])                      # mount post
    secs = [(4.56, 0.070, 0.030), (4.60, 0.088, 0.050), (4.98, 0.095, 0.060),               # padded armrest
            (5.03, 0.135, 0.075), (5.40, 0.140, 0.075), (5.46, 0.118, 0.055)]                  # console + nose
    b.loft([rrect(x, w, CA_Z - 0.10, CA_Z + h, 0.035, y0=CA_Y) for x, w, h in secs], M["int_gray"])
    b.loft([rrect(x, 0.080, CA_Z + 0.040, CA_Z + 0.072, 0.030, y0=CA_Y) for x in (4.60, 4.64, 4.95, 4.99)], M["int_pad"])
    b.box((4.86, CA_Y - 0.094, CA_Z + 0.00), (0.035, 0.012, 0.022), M["btn_o"])                # armrest adjust lever
    # dark key beds
    b.box((5.225, CA_Y - 0.105, TOP - 0.002), (0.30, 0.058, 0.006), M["int_dark"])             # outboard column
    b.box((5.135, CA_Y + 0.046, TOP - 0.002), (0.115, 0.064, 0.006), M["int_dark"])            # inboard keypad
    b.box((5.135, CA_Y - 0.038, TOP - 0.002), (0.090, 0.092, 0.006), M["int_dark"])            # radio / HVAC block
    b.box((5.385, CA_Y - 0.015, TOP - 0.004), (0.070, 0.10, 0.010), M["black"])                # front pocket
    b.box((5.335, CA_Y + 0.088, TOP - 0.003), (0.19, 0.030, 0.008), M["black"])                # hydro-handle gate slot
    b.box((5.245, CA_Y - 0.040, TOP - 0.001), (0.045, 0.085, 0.003), M["int_pad"])             # boom-function decal panel
    b.box((5.03, CA_Y + 0.135, CA_Z + 0.02), (0.10, 0.004, 0.05), M["int_pad"])                # shift-pattern decal (inboard wall)
    # display post
    b.tube((5.44, CA_Y + 0.0, CA_Z + 0.04), (5.535, CA_Y + 0.015, CA_Z + 0.30), 0.018, M["int_dark"])
part("commandarm_body", cab, commandarm, bevel=0.005)

# dash ledge to the right of the operator: cup holder, power sockets
def ledge(b):
    b.box((5.55, -0.72, CA_Z + 0.02), (0.55, 0.20, 0.10), M["int_gray"])
    b.cyl((5.42, -0.72, CA_Z + 0.05), 0.045, 0.06, M["int_dark"])
    for i in range(4):
        b.cyl((5.58 + i * 0.045, -0.72, CA_Z + 0.072), 0.013, 0.012, M["int_dark"], segs=12)
part("dash_ledge", cab, ledge, bevel=0.01)

# display: large landscape touchscreen angled at the operator, key strip + rotary knob under it
DSP_C = Vector((5.555, CA_Y + 0.025, CA_Z + 0.43))
DSP_ROT = Matrix.Rotation(math.radians(-22), 3, "Z") @ Matrix.Rotation(math.radians(-12), 3, "Y")
DSP_W, DSP_H = 0.33, 0.215
def display(b):
    b.box(DSP_C, (0.032, DSP_W, DSP_H), M["int_dark"], rot=DSP_ROT)
    b.box(DSP_C + DSP_ROT @ Vector((-0.017, 0, 0)), (0.002, DSP_W - 0.026, DSP_H - 0.026), M["screen"], rot=DSP_ROT)
    strip = DSP_C + DSP_ROT @ Vector((0.0, 0.0, -DSP_H / 2 - 0.040))
    b.box(strip, (0.030, 0.20, 0.06), M["int_dark"], rot=DSP_ROT)
    for r in range(2):
        for c in range(7):
            b.box(strip + DSP_ROT @ Vector((-0.016, -0.075 + c * 0.025, 0.012 - r * 0.026)), (0.004, 0.020, 0.020), M["btn_k"], rot=DSP_ROT)
    b.cyl(strip + DSP_ROT @ Vector((-0.02, 0.122, 0.0)), 0.022, 0.025, M["int_dark"], rot=DSP_ROT @ Matrix.Rotation(math.pi / 2, 3, "Y"))
o = part("DSP_commandcenter", cab, display)
tag(o, "control", "ca_display", "CommandCenter touchscreen")
# live screen: a UV-mapped quad facing the operator (the viewer paints a canvas onto it)
_basis = Matrix(((0, 0, -1), (-1, 0, 0), (0, 1, 0)))   # columns: grid X -> -Y, grid Y -> +Z, normal -> -X
_scr = Builder("DSP_screen", o, origin=DSP_C)
_scr.bm.loops.layers.uv.new("UVMap")
_m = Matrix.Translation(DSP_ROT @ Vector((-0.019, 0, 0))) @ (DSP_ROT @ _basis).to_4x4() @ Matrix.Diagonal(((DSP_W - 0.026) / 2, (DSP_H - 0.026) / 2, 1, 1))
_r = bmesh.ops.create_grid(_scr.bm, x_segments=1, y_segments=1, size=1.0, matrix=_m, calc_uvs=True)
_scr._tag(_r["verts"], mat("Display_Screen", srgb("0B1F12"), 0.0, 0.3, emit=srgb("1D4D2A")))
tag(_scr.done(), "control", "ca_display", "CommandCenter touchscreen")

def key(name, ref, label, pos, m, size=(0.018, 0.018, 0.008), parent=None, rot=None, shape="box", glyph=True):
    """A clickable key with a small white legend mark."""
    R = rot or Matrix.Identity(3)
    def fn(b):
        if shape == "round":
            b.cyl(pos, size[0] / 2, size[2], m, segs=20, rot=R)
        elif shape == "roller":
            b.cyl(pos, size[0] / 2, size[1], m, segs=20, rot=R @ Matrix.Rotation(math.pi / 2, 3, "X"))
            for k in range(10):   # knurl ridges
                a = 2 * math.pi * k / 10
                b.box(Vector(pos) + R @ Vector((size[0] / 2 * math.cos(a), 0, size[0] / 2 * math.sin(a))), (0.003, size[1], 0.003), m,
                      rot=R @ Matrix.Rotation(-a, 3, "Y"))
        else:
            b.box(pos, size, m, rot=R)
        if glyph and shape == "box":
            b.box(Vector(pos) + R @ Vector((0, 0, size[2] / 2 + 0.0004)), (size[0] * 0.45, size[1] * 0.16, 0.0008), M["glyph"], rot=R)
    o = part(name, parent or cab, fn, bevel=0.002 if shape == "box" else 0.0)
    if ref:
        tag(o, "control", ref, label)
    return o

def decor_keys(name, positions, m=None, size=(0.018, 0.018, 0.008)):
    def fn(b):
        for p in positions:
            b.box(p, size, m or M["btn_k"])
            b.box(Vector(p) + Vector((0, 0, size[2] / 2 + 0.0004)), (size[0] * 0.45, size[1] * 0.16, 0.0008), M["glyph"])
    part(name, cab, fn, bevel=0.0015)

KZ = TOP + 0.004
# outboard column: 2 x 8 keys; rows 0-3 carry the boom-fold functions (assignment unverified)
COL_X = [5.345 - r * 0.034 for r in range(8)]
COL_Y = [CA_Y - 0.092, CA_Y - 0.118]
fold_keys = [("SW_fold_L_out", "ca_fold_L_out", "Left main fold OUT"), ("SW_fold_R_out", "ca_fold_R_out", "Right main fold OUT"),
             ("SW_fold_L_in", "ca_fold_L_in", "Left main fold IN"), ("SW_fold_R_in", "ca_fold_R_in", "Right main fold IN"),
             ("SW_outer_L_out", "ca_outer_L_out", "Left outer fold OUT"), ("SW_outer_R_out", "ca_outer_R_out", "Right outer fold OUT"),
             ("SW_outer_L_in", "ca_outer_L_in", "Left outer fold IN"), ("SW_outer_R_in", "ca_outer_R_in", "Right outer fold IN")]
for i, (name, ref, label) in enumerate(fold_keys):
    key(name, ref, label, (COL_X[i // 2], COL_Y[i % 2], KZ), M["btn_k"], size=(0.026, 0.022, 0.008))
decor_keys("commandarm_keys_outboard", [(COL_X[r], COL_Y[c], KZ) for r in range(4, 8) for c in range(2)], size=(0.026, 0.022, 0.008))
key("CTL_throttle", "ca_throttle", "Engine throttle (thumbwheel)", (5.39, CA_Y - 0.105, TOP + 0.010), M["int_dark"],
    size=(0.026, 0.020, 0.0), shape="roller")
# front pocket thumbwheels (function unverified -> decor)
for i, y in enumerate((CA_Y - 0.035, CA_Y + 0.005)):
    key(f"commandarm_thumbwheel_{i}", None, "", (5.315, y, TOP + 0.008), M["int_dark"], size=(0.024, 0.018, 0.0), shape="roller")
# centre: lock key, orange double key, radio/HVAC block
decor_keys("commandarm_key_lock", [(5.262, CA_Y + 0.046, KZ)], size=(0.016, 0.016, 0.008))
key("BTN_transport", "ca_transport", "Field / Road mode", (5.222, CA_Y + 0.060, KZ), M["btn_o"], size=(0.034, 0.028, 0.010))
decor_keys("commandarm_key_orange", [(5.222, CA_Y + 0.030, KZ)], m=M["btn_o"], size=(0.034, 0.028, 0.010))
decor_keys("commandarm_keys_hvac", [(5.163 - r * 0.028, CA_Y - 0.072 + c * 0.017, KZ) for r in range(3) for c in range(5)],
           size=(0.024, 0.015, 0.007))
# inboard keypad (2 x 3) + park
ib = [(5.170, CA_Y + 0.062), (5.170, CA_Y + 0.031), (5.140, CA_Y + 0.062), (5.140, CA_Y + 0.031), (5.110, CA_Y + 0.062), (5.110, CA_Y + 0.031)]
key("SW_tread_narrow", "ca_tread_narrower", "Tread adjust NARROWER", (*ib[0], KZ), M["btn_k"], size=(0.026, 0.026, 0.008))
key("SW_tread_wide", "ca_tread_wider", "Tread adjust WIDER", (*ib[1], KZ), M["btn_k"], size=(0.026, 0.026, 0.008))
key("SW_ladder", "ca_ladder", "Ladder raise/lower", (*ib[2], KZ), M["btn_k"], size=(0.026, 0.026, 0.008))
decor_keys("commandarm_keys_inboard", [(*p, KZ) for p in ib[3:]], size=(0.026, 0.026, 0.008))
key("BTN_park", "ca_park", "Park (P)", (5.068, CA_Y + 0.046, KZ), M["btn_o"], size=(0.030, 0.056, 0.010))

# hydro handle at the front-inboard corner; lever swings fore/aft in the gate
HH_PIV = Vector((5.360, CA_Y + 0.088, TOP))
hh_piv = empty("PIV_hydro_handle", HH_PIV, cab, size=0.08)
HEAD_ROT = Matrix.Rotation(math.radians(-38), 3, "Y")      # key face tilted up/back toward the operator
HEAD_C = HH_PIV + Vector((-0.016, 0.0, 0.172))
def hh(b):
    b.cyl(HH_PIV + Vector((0, 0, 0.006)), 0.020, 0.012, M["black"], segs=16)                    # gate boot
    b.bar(HH_PIV, HH_PIV + Vector((0.0, 0, 0.12)), 0.026, M["black"])                            # stem
    rings = []                                                                                   # orange finger grip, flows into the shell
    for k in range(8):
        t = k / 7
        c = HH_PIV + Vector((-0.010 - 0.050 * t * t, 0, 0.045 + 0.105 * t))
        w, d = 0.020 + 0.022 * t, 0.016 + 0.020 * t
        rings.append([c + Vector((d * math.cos(a), w * math.sin(a), 0)) for a in [2 * math.pi * i / 16 for i in range(16)]])
    b.loft(rings, M["btn_o"])
    b.box(HEAD_C + HEAD_ROT @ Vector((-0.004, -0.002, -0.014)), (0.142, 0.100, 0.032), M["btn_o"], rot=HEAD_ROT)  # orange lower shell
    b.box(HEAD_C + HEAD_ROT @ Vector((0.004, 0.002, 0.008)), (0.130, 0.090, 0.028), M["black"], rot=HEAD_ROT)      # black key face
    b.box(HH_PIV + Vector((0.0, -0.016, 0.10)), (0.03, 0.006, 0.035), M["black"])               # side keypad bed
o = part("CTL_hydro_handle", hh_piv, hh, bevel=0.007)
tag(o, "control", "hh_lever", "Hydro handle")
def head(u, v):    # point on the head's key face (u forward, v inboard)
    return HEAD_C + HEAD_ROT @ Vector((u, v, 0.025))
hh_buttons = [
    ("BTN_master_spray", "hh_master_spray", "Master spray ON/OFF",  ( 0.046,  0.024), "btn_y", (0.026, 0.026, 0.008)),
    ("BTN_tilt_L_up",    "hh_tilt_L_up",    "Left wing tilt up",    ( 0.016,  0.024), "btn_k", (0.018, 0.018, 0.007)),
    ("BTN_boom_raise",   "hh_boom_raise",   "Boom raise (centre)",  ( 0.016,  0.002), "btn_k", (0.018, 0.018, 0.007)),
    ("BTN_tilt_R_up",    "hh_tilt_R_up",    "Right wing tilt up",   ( 0.016, -0.020), "btn_k", (0.018, 0.018, 0.007)),
    ("BTN_tilt_L_down",  "hh_tilt_L_down",  "Left wing tilt down",  (-0.007,  0.024), "btn_k", (0.018, 0.018, 0.007)),
    ("BTN_boom_lower",   "hh_boom_lower",   "Boom lower (centre)",  (-0.007,  0.002), "btn_k", (0.018, 0.018, 0.007)),
    ("BTN_tilt_R_down",  "hh_tilt_R_down",  "Right wing tilt down", (-0.007, -0.020), "btn_k", (0.018, 0.018, 0.007)),
    ("BTN_resume",       "hh_resume",       "Boom height resume",   (-0.034, -0.026), "btn_y", (0.016, 0.016, 0.007)),
]
for name, ref, label, (u, v), mm, size in hh_buttons:
    key(name, ref, label, head(u, v), M[mm], size=size, parent=hh_piv, rot=HEAD_ROT)
def hh_decor(b):
    for u, v in ((0.046, -0.004), (0.046, -0.028), (-0.034, -0.002)):                            # HMS, F, AUTO keys
        b.box(head(u, v), (0.018, 0.018, 0.007), M["btn_k"], rot=HEAD_ROT)
    b.cyl(head(-0.036, 0.024), 0.014, 0.008, M["btn_k"], segs=24, rot=HEAD_ROT)                 # round 'E' rocker
for name, ref, label, dz in (("BTN_range_up", "hh_range_up", "Speed range up", 0.112), ("BTN_range_down", "hh_range_down", "Speed range down", 0.090)):
    key(name, ref, label, HH_PIV + Vector((0.0, -0.020, dz)), M["btn_k"], size=(0.012, 0.004, 0.012), parent=hh_piv, glyph=False)
part("hydro_handle_keys", hh_piv, hh_decor, bevel=0.0015)

# shift the whole cab (shell, interior, controls) into its real position over the front axle
cab.location.x += CAB_DX

# ---------------------------------------------------------------- ladder (left front, hydraulic, green)
LAD_PIV = Vector((5.48, 1.20, CAB_FLOOR - 0.10))
LAD_BOT = Vector((6.05, 1.24, 0.30))
lad_piv = empty("PIV_ladder", LAD_PIV, chassis)
def ladder(b):
    for dy in (-0.24, 0.24):
        b.bar(LAD_PIV + Vector((0, dy, 0)), LAD_BOT + Vector((0, dy, 0)), 0.05, M["green"])
    n = 7
    for i in range(n):
        p = LAD_BOT.lerp(LAD_PIV, (i + 0.3) / n)
        b.box(p, (0.16, 0.46, 0.035), M["green"])
    rail_top = LAD_PIV + Vector((0.05, 0.27, 0.0))
    b.tube(LAD_BOT + Vector((0, 0.27, 0.95)), rail_top + Vector((0, 0, 0.55)), 0.02, M["green"])   # handrail
part("ladder", lad_piv, ladder)
o = part("SW_ladder_ground", chassis, lambda b: b.box((5.25, 1.12, 1.62), (0.06, 0.03, 0.08), M["btn_y"]))
tag(o, "control", "ex_ladder_switch", "Ground-level ladder switch")

# ============================================================================= BOOM (green steel)
lift_u = empty("PIV_lift_upper", UPPER_PIV, chassis)
lift_l = empty("PIV_lift_lower", LOWER_PIV, chassis)
for piv, nm in ((lift_u, "lift_arm_upper"), (lift_l, "lift_arm_lower")):
    p0 = WORLD[piv.name]
    def arms(b, p0=p0):
        for y in (0.45, -0.45):
            b.box(p0 + Vector((-ARM_L / 2, y, 0)), (ARM_L, 0.12, 0.16), M["green"])
            b.cyl(p0 + Vector((0, y, 0)), 0.08, 0.16, M["dsteel"], axis="Y")
            b.cyl(p0 + Vector((-ARM_L, y, 0)), 0.08, 0.16, M["dsteel"], axis="Y")
    part(nm, piv, arms, bevel=0.01)

center = empty("PIV_boom_center", (BOOM_X, 0, UPPER_PIV.z), lift_u)
CF_TOP = UPPER_PIV.z + 0.45
def center_frame(b):
    zb = BOOM_CHORD_Z
    xb = BOOM_X - 0.30
    for y in (Y_HINGE - 0.06, -Y_HINGE + 0.06, 0.55, -0.55):                     # uprights
        b.box((xb, y, (zb + CF_TOP) / 2), (0.14, 0.12, CF_TOP - zb), M["green"])
    for z in (zb + 0.06, zb + 0.70, CF_TOP - 0.06):                               # cross beams
        b.box((xb, 0, z), (0.16, 2 * Y_HINGE, 0.12), M["green"])
    b.box((BOOM_X - 0.08, 0, (LOWER_PIV.z + UPPER_PIV.z) / 2), (0.20, 1.10, 0.75), M["green"])   # lift mast
    for s in (1, -1):                                                             # diagonal braces
        b.bar((xb, s * 0.55, zb + 0.70), (xb, s * (Y_HINGE - 0.06), CF_TOP - 0.06), 0.07, M["green"])
    # SMV emblem + lights on the rear face
    b.cyl((xb - 0.09, 0, CF_TOP - 0.42), 0.30, 0.02, M["tail"], axis="X", segs=3, rot=Matrix.Rotation(math.pi / 2, 3, "Y") @ Matrix.Rotation(math.pi / 2, 3, "Z"), smooth=False)
    b.cyl((xb - 0.10, 0, CF_TOP - 0.42), 0.21, 0.02, M["smv"], segs=3, rot=Matrix.Rotation(math.pi / 2, 3, "Y") @ Matrix.Rotation(math.pi / 2, 3, "Z"), smooth=False)
    for s in (1, -1):
        b.box((xb - 0.09, s * 1.15, zb + 0.70), (0.03, 0.18, 0.10), M["tail"])
        b.box((xb - 0.09, s * 1.15, zb + 0.84), (0.03, 0.14, 0.06), M["amber"])
    b.box((xb, 0.75, CF_TOP + 0.12), (0.30, 0.45, 0.22), M["black"])               # control box
    # plumbing: spray pipe, section valves, hoses
    b.tube((xb - 0.12, Y_HINGE, zb - 0.02), (xb - 0.12, -Y_HINGE, zb - 0.02), 0.022, M["steel"])
    b.tube((xb + 0.05, 1.2, CF_TOP + 0.05), (xb + 0.05, -1.2, CF_TOP + 0.05), 0.03, M["hose"])
    for k in range(int(Y_HINGE / S["nozzle_spacing_m"] - 0.5) + 1):                # centre-section nozzles
        for sg in (1, -1):
            y = sg * (k + 0.5) * S["nozzle_spacing_m"]
            b.box((xb - 0.12, y, zb - 0.07), (0.05, 0.04, 0.08), M["black"])
            b.box((xb - 0.12, y, zb - 0.12), (0.025, 0.025, 0.03), M["orange"])
part("boom_center_frame", center, center_frame, bevel=0.01)
o = part("SVC_boom_strainer", center, lambda b: b.cyl((BOOM_X + 0.05, -0.30, BOOM_CHORD_Z + 0.30), 0.06, 0.22, M["white"]).cyl((BOOM_X + 0.05, -0.30, BOOM_CHORD_Z + 0.44), 0.07, 0.04, M["orange"]))
tag(o, "service_point", "sp_boom_strainer", "Boom strainer (80 mesh)")

NOZ_SP = S["nozzle_spacing_m"]

def truss(b, side, y0, y1, h0, h1, d0=0.48, d1=0.30, chord=0.07):
    """Four-chord green steel boom truss from |y| = y0 to y1 with ExactApply nozzle bodies."""
    def P(y, dx, dz):
        return Vector((BOOM_X + dx, side * y, BOOM_CHORD_Z + dz))
    L = y1 - y0
    n = max(2, int(L / 0.85))
    def at(t):
        return y0 + t * L, h0 + (h1 - h0) * t, d0 + (d1 - d0) * t
    g = M["green"]
    ya, ha, da = at(0); yb, hb, db = at(1)
    # chords: bottom front/rear, top front/rear (top narrower -> trapezoid section)
    for (dxa, dza), (dxb, dzb) in (((0, 0), (0, 0)), ((-da, 0), (-db, 0)),
                                   ((-da * 0.2, ha), (-db * 0.2, hb)), ((-da * 0.8, ha), (-db * 0.8, hb))):
        b.bar(P(ya, dxa, dza), P(yb, dxb, dzb), chord * (1 - 0.3 * (y0 / Y_END)), g)
    for i in range(n + 1):
        y, h, d = at(i / n)
        b.bar(P(y, 0, 0), P(y, -d * 0.2, h), 0.04, g)               # front vertical
        b.bar(P(y, -d, 0), P(y, -d * 0.8, h), 0.04, g)              # rear vertical
        b.bar(P(y, 0, 0), P(y, -d, 0), 0.035, g)                    # bottom cross
        b.bar(P(y, -d * 0.2, h), P(y, -d * 0.8, h), 0.035, g)       # top cross
        if i < n:
            y2, h2, d2 = at((i + 1) / n)
            if i % 2 == 0:
                b.bar(P(y, 0, 0), P(y2, -d2 * 0.2, h2), 0.03, g)
                b.bar(P(y, -d, 0), P(y2, -d2 * 0.8, h2), 0.03, g)
            else:
                b.bar(P(y, -d * 0.2, h), P(y2, 0, 0), 0.03, g)
                b.bar(P(y, -d * 0.8, h), P(y2, -d2, 0), 0.03, g)
    # plumbing: stainless spray pipe behind the rear chord + black hose bundle on top
    b.tube(P(ya, -da - 0.07, -0.02), P(yb, -db - 0.07, -0.02), 0.016, M["steel"])
    b.tube(P(ya, -da * 0.5, ha + 0.05), P(yb, -db * 0.5, hb + 0.05), 0.022, M["hose"])
    k = math.ceil(y0 / NOZ_SP - 0.5)
    while (k + 0.5) * NOZ_SP < y1:
        y = (k + 0.5) * NOZ_SP
        d = d0 + (d1 - d0) * (y - y0) / L
        b.box(P(y, -d - 0.07, -0.07), (0.05, 0.04, 0.08), M["black"])      # ExactApply body
        b.box(P(y, -d - 0.07, -0.12), (0.025, 0.025, 0.03), M["orange"])   # tip
        k += 1
    return b

OUTER_DX = -0.30          # outer-fold hinge sits behind the boom plane, so the folded wing nests outboard
for side, sfx in ((1, "L"), (-1, "R")):
    hinge = Vector((BOOM_X, side * Y_HINGE, BOOM_CHORD_Z + 0.35))
    tilt = empty(f"PIV_tilt_{sfx}", hinge, center)
    fold = empty(f"PIV_fold_{sfx}", hinge, tilt)
    def inner_fn(b, s=side):
        truss(b, s, Y_HINGE, Y_OUTER, 0.95, 0.62, 0.50, 0.36)
        b.box((BOOM_X - 0.25, s * (Y_HINGE + 0.10), BOOM_CHORD_Z + 0.45), (0.55, 0.20, 0.95), M["green"])   # fold knuckle
        b.cyl((BOOM_X + 0.10, s * (Y_HINGE + 0.6), BOOM_CHORD_Z + 0.75), 0.06, 1.0, M["steel"], axis="Y")    # fold cylinder rod
    part(f"boom_inner_{sfx}", fold, inner_fn)
    outer = empty(f"PIV_outer_{sfx}", (BOOM_X + OUTER_DX, side * Y_OUTER, BOOM_CHORD_Z + 0.3), fold)
    def outer_fn(b, s=side):
        truss(b, s, Y_OUTER, Y_TIP, 0.60, 0.42, 0.34, 0.26, chord=0.06)
        b.box((BOOM_X + OUTER_DX / 2, s * (Y_OUTER + 0.08), BOOM_CHORD_Z + 0.30), (0.40, 0.16, 0.62), M["green"])
    part(f"boom_outer_{sfx}", outer, outer_fn)
    tip = empty(f"PIV_tip_{sfx}", (BOOM_X - 0.13, side * Y_TIP, BOOM_CHORD_Z + 0.2), outer)
    def tip_fn(b, s=side):
        truss(b, s, Y_TIP, Y_END, 0.40, 0.24, 0.26, 0.18, chord=0.05)
        b.cyl((BOOM_X - 0.13, s * Y_TIP, BOOM_CHORD_Z + 0.25), 0.05, 0.14, M["yellow"])                     # breakaway spring can
        b.bar((BOOM_X - 0.05, s * (Y_END - 0.05), BOOM_CHORD_Z - 0.35), (BOOM_X - 0.05, s * (Y_END - 0.05), BOOM_CHORD_Z + 0.25), 0.06, M["yellow"])  # end guard
    part(f"boom_tip_{sfx}", tip, tip_fn)

# ============================================================================= DRIVERS (Blender-side posing)
def add_prop(name, j):
    root[name] = float(j["default"])
    ui = root.id_properties_ui(name)
    ui.update(min=float(j["min"]), max=float(j["max"]), soft_min=float(j["min"]), soft_max=float(j["max"]),
              description=j["label"] + (" — " + j.get("note", "") if j.get("note") else ""))

for j in SPEC["joints"]:
    add_prop(j["id"], j)

def drive(obj_name, path, idx, expr, var_names):
    o = bpy.data.objects[obj_name]
    fc = o.driver_add(path, idx)
    d = fc.driver
    d.type = "SCRIPTED"
    for vn in var_names:
        v = d.variables.new()
        v.name = vn
        v.targets[0].id = root
        v.targets[0].data_path = f'["{vn}"]'
    d.expression = expr

AX = {"x": 0, "y": 1, "z": 2}
for j in SPEC["joints"]:
    jid = j["id"]
    if jid == "steer":
        L = WB
        inner = f"(atan({L}*tan(radians(abs(steer)))/({L}-tread*tan(radians(abs(steer))))))"
        drive("PIV_steer_FL", "rotation_euler", 2, f"({inner} if steer>0 else radians(steer))", ["steer", "tread"])
        drive("PIV_steer_FR", "rotation_euler", 2, f"(-{inner} if steer<0 else radians(steer))", ["steer", "tread"])
    elif jid == "tread":
        for node, sign in j["nodes"].items():
            rest = bpy.data.objects[node].location.y
            s = 1 if sign.startswith("+") else -1
            drive(node, "location", 1, f"{rest:.5f}{'+' if s > 0 else '-'}(tread-{TREAD0})/2", ["tread"])
    elif jid == "suspension":
        for node in j["nodes"]:
            rest = bpy.data.objects[node].location.z
            drive(node, "location", 2, f"{rest:.5f}+suspension", ["suspension"])
    elif jid == "boom_height":
        th = f"asin(max(-1,min(1,(boom_height-{NOZ_Z0:.4f})/{ARM_L})))"
        drive("PIV_lift_upper", "rotation_euler", 1, th, ["boom_height"])
        drive("PIV_lift_lower", "rotation_euler", 1, th, ["boom_height"])
        drive("PIV_boom_center", "rotation_euler", 1, f"-{th}", ["boom_height"])
    elif jid == "steering_wheel":
        drive("PIV_steering_wheel", "rotation_euler", 2, "radians(steering_wheel)", ["steering_wheel"])
    else:
        for node, axs in j["nodes"].items():
            sign = -1 if axs.endswith("-") else 1
            drive(node, "rotation_euler", AX[axs[0]], f"{'-' if sign < 0 else ''}radians({jid})", [jid])

# ============================================================================= metadata on every mesh
for o in COLL.objects:
    if o.type == "MESH" and "kind" not in o:
        o["kind"] = "part"
        o["ref_id"] = o.name
        o["label"] = o.name.replace("_", " ")

# ============================================================================= environment (render only)
def grass_material():
    m = bpy.data.materials.new("Grass_Field")
    try:
        m.use_nodes = True
    except Exception:
        pass
    nt = m.node_tree
    bsdf = nt.nodes["Principled BSDF"]
    tc = nt.nodes.new("ShaderNodeTexCoord")
    noise = nt.nodes.new("ShaderNodeTexNoise")
    noise.inputs["Scale"].default_value = 0.6
    noise.inputs["Detail"].default_value = 10
    noise.inputs["Roughness"].default_value = 0.65
    ramp = nt.nodes.new("ShaderNodeValToRGB")
    e = ramp.color_ramp.elements
    e[0].position, e[0].color = 0.35, (*srgb("3F5A22"), 1)
    e[1].position, e[1].color = 0.65, (*srgb("6E8238"), 1)
    e.new(0.80).color = (*srgb("7D7048"), 1)
    fine = nt.nodes.new("ShaderNodeTexNoise")
    fine.inputs["Scale"].default_value = 120
    bump = nt.nodes.new("ShaderNodeBump"); bump.inputs["Strength"].default_value = 0.35
    nt.links.new(tc.outputs["Object"], noise.inputs["Vector"])
    nt.links.new(tc.outputs["Object"], fine.inputs["Vector"])
    nt.links.new(noise.outputs["Fac"], ramp.inputs["Fac"])
    nt.links.new(ramp.outputs["Color"], bsdf.inputs["Base Color"])
    nt.links.new(fine.outputs["Fac"], bump.inputs["Height"])
    nt.links.new(bump.outputs["Normal"], bsdf.inputs["Normal"])
    bsdf.inputs["Roughness"].default_value = 0.95
    m.diffuse_color = (*srgb("5E7A36"), 1)
    return m

bpy.ops.mesh.primitive_plane_add(size=400, location=(0, 0, 0))
g = bpy.context.active_object; g.name = "ENV_ground"; g.data.materials.append(grass_material())
for c in g.users_collection: c.objects.unlink(g)
ENV.objects.link(g)

# ============================================================================= save + export
bpy.context.view_layer.update()
blend_path = os.path.join(OUT, "john_deere_612r.blend")
bpy.ops.wm.save_as_mainfile(filepath=blend_path)

for o in bpy.data.objects:
    o.select_set(o.name in COLL.objects)
# Drivers are not exported. The viewer drives the PIV_* nodes from spec/612r_spec.json.
for o in COLL.objects:
    if o.animation_data:
        o.animation_data_clear()
for o in COLL.objects:
    if o.name == "PIV_steering_tilt":
        continue
    if o.type == "EMPTY":
        o.rotation_euler = (0, 0, 0)
glb_path = os.path.join(OUT, "john_deere_612r.glb")
bpy.ops.export_scene.gltf(filepath=glb_path, export_format="GLB", use_selection=True,
                          export_extras=True, export_apply=True, export_yup=True,
                          export_animations=False)
print("SAVED", blend_path)
print("EXPORTED", glb_path, os.path.getsize(glb_path) // 1024, "KB")

# ============================================================================= photoreal preview renders
if DO_RENDER:
    bpy.ops.wm.open_mainfile(filepath=blend_path)
    scene = bpy.context.scene
    root = bpy.data.objects["SPRAYER_612R"]
    scene.render.engine = "CYCLES"
    scene.cycles.samples = int(os.environ.get("SAMPLES", 96))
    scene.cycles.use_denoising = True
    scene.render.resolution_x, scene.render.resolution_y = 1600, 900
    try:
        scene.view_settings.view_transform = "AgX"
        scene.view_settings.look = "AgX - Medium High Contrast"
    except Exception:
        pass
    # physically based sky + matching sun
    world = bpy.data.worlds.new("Sky"); scene.world = world
    try:
        world.use_nodes = True
    except Exception:
        pass
    nt = world.node_tree
    sky = nt.nodes.new("ShaderNodeTexSky")
    for t in ("MULTIPLE_SCATTERING", "NISHITA", "HOSEK_WILKIE"):
        try:
            sky.sky_type = t
            break
        except Exception:
            continue
    SUN_EL, SUN_AZ = math.radians(38), math.radians(215)
    for attr, val in (("sun_elevation", SUN_EL), ("sun_rotation", SUN_AZ), ("sun_intensity", 0.6)):
        if hasattr(sky, attr):
            setattr(sky, attr, val)
    nt.links.new(sky.outputs["Color"], nt.nodes["Background"].inputs["Color"])
    nt.nodes["Background"].inputs["Strength"].default_value = 0.22
    scene.view_settings.exposure = -0.35
    sun = bpy.data.objects.new("Sun", bpy.data.lights.new("Sun", "SUN"))
    sun.data.energy = 3.2
    sun.data.angle = math.radians(0.6)
    sun.rotation_euler = (math.pi / 2 - SUN_EL, 0, SUN_AZ + math.pi / 2)
    scene.collection.objects.link(sun)
    cam = bpy.data.objects.new("Cam", bpy.data.cameras.new("Cam")); scene.collection.objects.link(cam)
    cam.data.clip_end = 1000
    scene.camera = cam

    def look(pos, tgt, lens=35):
        cam.location = pos
        if tgt is None:                       # straight down, machine-forward at the top of the frame
            cam.rotation_euler = (0, 0, -math.pi / 2)
        else:
            cam.rotation_euler = (Vector(tgt) - Vector(pos)).to_track_quat("-Z", "Y").to_euler()
        cam.data.lens = lens

    def pose(**kw):
        for j in SPEC["joints"]:
            root[j["id"]] = float(j["default"])
        for k, v in kw.items():
            root[k] = float(v)
        root.update_tag(); bpy.context.view_layer.update()

    docs = os.path.join(ROOT, "docs")
    os.makedirs(docs, exist_ok=True)
    TRANSPORT = dict(boom_height=2.2, outer_L=178, outer_R=178, fold_L=90, fold_R=90, ladder=0)
    shots = [
        ("render_transport.png", TRANSPORT, ((12.5, 9.5, 3.4), (2.6, 0.2, 2.0), 30)),
        ("render_side.png",      TRANSPORT, ((2.6, 20, 2.2), (2.6, 0, 2.0), 40)),
        ("render_rear.png",      TRANSPORT, ((-11, -4.5, 2.6), (0, 0, 2.0), 35)),
        ("render_field.png",     dict(boom_height=0.76), ((-13, 21, 6.5), (-1, 4, 1.2), 30)),
        ("render_commandarm.png", dict(boom_height=0.76, cab_door=0), ((3.72, 0.06, 3.50), (4.15, -0.38, 3.02), 26)),
        ("render_commandarm_top.png", dict(boom_height=0.76), ((3.93, -0.40, 3.80), None, 20)),
        ("render_service.png",   dict(hood=60, ladder=0, cab_door=75, eductor=85, tread=4.06, steer=25, boom_height=1.6), ((10.5, 7.5, 3.8), (3.8, 0.4, 2.0), 30)),
    ]
    only = os.environ.get("ONLY")
    for fn, p, (pos, tgt, lens) in shots:
        if only and only not in fn:
            continue
        pose(**p)
        look(pos, tgt, lens)
        scene.render.filepath = os.path.join(docs, fn)
        bpy.ops.render.render(write_still=True)
        print("RENDERED", fn)
