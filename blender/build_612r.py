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

    def text(self, body, size, m, matrix):
        """Append flat 3-D text. matrix maps text space (x = reading direction, y = up, z = out of the
        surface) to world space."""
        cu = bpy.data.curves.new("txt", "FONT")
        cu.body, cu.size, cu.extrude, cu.align_x, cu.align_y = body, size, 0.0008, "CENTER", "CENTER"
        cu.resolution_u = 2                      # legends are tiny: keep the glyph outlines light
        fp = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
        if os.path.exists(fp):
            cu.font = bpy.data.fonts.load(fp, check_existing=True)
        ob = bpy.data.objects.new("txt_tmp", cu)
        bpy.context.scene.collection.objects.link(ob)
        dg = bpy.context.evaluated_depsgraph_get()
        me = bpy.data.meshes.new_from_object(ob.evaluated_get(dg))
        mtx = matrix if self.local else Matrix.Translation(-self.origin) @ matrix
        me.transform(mtx)
        before = set(self.bm.faces)
        self.bm.from_mesh(me)
        mi = self._mi(m)
        for f in self.bm.faces:                  # tag exactly the faces the text added
            if f not in before:
                f.material_index = mi
                f.smooth = False
        bpy.data.objects.remove(ob); bpy.data.curves.remove(cu); bpy.data.meshes.remove(me)
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
    for x in (4.98, 5.30, 5.62):
        b.tube((x, 1.24, CAB_FLOOR - 0.06), (x, 1.24, CAB_FLOOR + 0.95), 0.02, M["green"])
    for z in (CAB_FLOOR + 0.50, CAB_FLOOR + 0.95):
        b.tube((4.98, 1.24, z), (5.62, 1.24, z), 0.02, M["green"])
    b.tube((5.62, 1.24, CAB_FLOOR + 0.95), (5.66, 0.92, CAB_FLOOR + 0.95), 0.02, M["green"])
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
                a = math.radians(28 + 142 * k / 14)
                pts.append((x + (TIRE_R + 0.11) * math.cos(a), hub_z + (TIRE_R + 0.11) * math.sin(a)))
            for (ax_, az), (bx_, bz) in zip(pts, pts[1:]):
                b.box(((ax_ + bx_) / 2, y_wheel, (az + bz) / 2), (math.hypot(bx_ - ax_, bz - az) + 0.01, TIRE_W + 0.24, 0.03),
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

# exhaust: large black muffler box + stack, outboard of the hood at the cab's left front corner
def exhaust(b):
    mx, my = NOSE_X0 + 0.08, 0.80
    b.loft([rrect(mx + dx, 0.13, 2.80, 3.55, 0.08, y0=my) for dx in (-0.14, -0.12, 0.12, 0.14)], M["black_g"])   # muffler
    b.cyl((mx, my, 3.85), 0.075, 0.62, M["black_g"])                                                            # stack
    b.cyl((mx, my, 4.17), 0.082, 0.05, M["black"])
    b.box((mx - 0.20, my - 0.02, 3.20), (0.26, 0.05, 0.06), M["black"])                                         # cab bracket
    b.box((mx - 0.20, my - 0.02, 2.95), (0.26, 0.05, 0.06), M["black"])
part("exhaust_stack", chassis, exhaust)

# fuel + DEF (right side, between the axles)
part("fuel_tank", chassis, lambda b: b.box((2.10, -0.98, 1.92), (1.55, 0.45, 0.62), M["black"]), bevel=0.04)
svc("SVC_fuel_fill", "sp_fuel_fill", "Fuel fill (565 L)", lambda b: b.cyl((2.60, -1.24, 2.15), 0.06, 0.08, M["yellow"], axis="Y"))
svc("SVC_def_fill", "sp_def_fill", "DEF fill (29.5 L)",
    lambda b: b.box((3.05, -0.98, 1.95), (0.30, 0.40, 0.40), M["white"]).cyl((3.05, -1.21, 2.08), 0.05, 0.06, M["blue"], axis="Y"))

# engine hood / nose: modern Deere styling traced from photos (wide rounded hood, full-face black grille,
# LED lights in the top corners, yellow stripe with the John Deere wordmark, vertical side louvres).
# Hinged at the cab; the front tilts up and the grille goes with it.
SE_N = 3.0                                     # superellipse exponent of the hood cross-section
def se_pt(x, hw, zc, hh, t):
    c, s_ = math.cos(t), math.sin(t)
    return Vector((x, hw * math.copysign(abs(c) ** (2 / SE_N), c), zc + hh * math.copysign(abs(s_) ** (2 / SE_N), s_)))
def se_ring(x, hw, z0, z1, k=40):
    zc, hh = (z0 + z1) / 2, (z1 - z0) / 2
    return [se_pt(x, hw, zc, hh, 2 * math.pi * i / k) for i in range(k)]
HOOD_SECS = [(NOSE_X0 + 0.02, 0.62, 1.60, 2.98), (NOSE_X0 + 0.60, 0.64, 1.60, 2.97), (6.30, 0.63, 1.62, 2.93),
             (6.66, 0.60, 1.64, 2.86), (6.78, 0.55, 1.68, 2.80)]
def hood_band(b, t0, t1, m, lift=0.004, xs=None):
    """A thin decal band lying on the hood surface between parametric angles t0..t1."""
    rings = []
    for (x, hw, z0, z1) in (xs or HOOD_SECS[1:4]):
        zc, hh = (z0 + z1) / 2, (z1 - z0) / 2
        ts = [t0 + (t1 - t0) * i / 4 for i in range(5)]
        outer = [se_pt(x, hw + lift, zc, hh + lift, t) for t in ts]
        inner = [se_pt(x, hw - 0.002, zc, hh - 0.002, t) for t in reversed(ts)]
        rings.append(outer + inner)
    b.loft(rings, m, smooth=True)

hood_piv = empty("PIV_hood", (NOSE_X0 + 0.02, 0, 2.97), chassis)
def hood(b):
    b.loft([se_ring(*sec) for sec in HOOD_SECS], M["green"])
    # grille: black, recessed, with mesh bars
    gx = HOOD_SECS[-1][0]
    b.loft([se_ring(gx - 0.01, 0.50, 1.70, 2.70), se_ring(gx + 0.035, 0.47, 1.73, 2.66)], M["black_g"])
    for i in range(22):                                                         # fine grille mesh
        z = 1.78 + i * 0.038
        b.box((gx + 0.038, 0, z), (0.006, 0.88 - abs(z - 2.2) * 0.28, 0.006), M["dsteel"])
    for j in range(-14, 15):
        b.box((gx + 0.040, j * 0.03, 2.18), (0.006, 0.005, 0.80), M["dsteel"])
    # LED headlights in the top corners of the grille
    for s_ in (1, -1):
        b.box((gx + 0.010, s_ * 0.31, 2.60), (0.08, 0.32, 0.14), M["black_g"])      # LED headlight cluster
        for k in range(3):
            b.box((gx + 0.052, s_ * (0.20 + k * 0.10), 2.60), (0.004, 0.085, 0.10), M["lens"])
    # badge (green field, yellow mark) on the left of the grille face
    b.box((gx + 0.05, 0.30, 2.30), (0.006, 0.12, 0.12), M["yellow"])
    b.box((gx + 0.054, 0.30, 2.30), (0.004, 0.09, 0.09), M["green"])
    # yellow stripe along each side, wordmark + model number in black
    STRIPE = [(x, hw, z0, z1) for (x, hw, z0, z1) in HOOD_SECS[:4]] + [(6.74, 0.575, 1.66, 2.83)]
    for s_, t0, t1 in ((1, 0.30, 0.52), (-1, math.pi - 0.52, math.pi - 0.30)):
        hood_band(b, t0, t1, M["yellow"], xs=STRIPE)
        x0, hw, z0, z1 = HOOD_SECS[1]                                             # widest section
        zc, hh = (z0 + z1) / 2, (z1 - z0) / 2
        tm = (t0 + t1) / 2
        p = se_pt(5.90, hw + 0.006, zc, hh + 0.006, tm)
        q = se_pt(5.95, hw, zc, hh, tm + 0.05 * s_)
        nrm = Vector((0, p.y, (p.z - zc) * (hw / hh) ** 2)).normalized()          # approx. surface normal
        read = Vector((-s_, 0, 0))                                                # reads front -> back on each side
        up = nrm.cross(read).normalized() * (-1 if s_ > 0 else 1)
        if up.z < 0:
            up = -up
        basis = Matrix((read, up, nrm)).transposed()
        b.text("JOHN DEERE", 0.088, M["black"], Matrix.Translation(p + nrm * 0.012) @ basis.to_4x4())
        p2 = se_pt(6.50, hw + 0.006, zc, hh + 0.006, tm + 0.30 * s_)
        b.text("612R", 0.06, M["black"], Matrix.Translation(p2 + nrm * 0.003) @ basis.to_4x4())
        # vertical side louvres near the front
        for i in range(3):
            xl = 6.30 + i * 0.09
            b.box((xl, s_ * (hw * 0.985), 2.30), (0.05, 0.012, 0.42), M["black"])
part("engine_hood", hood_piv, hood)
o = part("CTL_hood_latch", hood_piv, lambda b: b.box((HOOD_SECS[-1][0] + 0.05, 0, 1.76), (0.05, 0.16, 0.05), M["yellow"]))
tag(o, "control", "ex_hood_latch", "Engine hood latch")

# front bumper / frame nose under the grille, with the front-fill coupler
def bumper(b):
    b.box((6.62, 0, 1.58), (0.42, 1.05, 0.20), M["green"])
    b.box((6.86, 0, 1.52), (0.08, 0.70, 0.10), M["green_d"])
    b.cyl((6.92, 0.28, 1.55), 0.06, 0.16, M["black"], axis="X")
    b.cyl((7.01, 0.28, 1.55), 0.075, 0.03, M["yellow"], axis="X")
    b.box((6.95, 0.28, 1.66), (0.03, 0.03, 0.14), M["yellow"])
part("front_bumper", chassis, bumper, bevel=0.015)

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
    b.box((cx, 0, CAB_FLOOR + 0.18), (CAB_X1 - CAB_X0, 2 * CAB_HW, 0.40), M["green"])                  # lower body (green)
    b.box((cx, 0, CAB_FLOOR + 0.39), (CAB_X1 - CAB_X0 + 0.01, 2 * CAB_HW + 0.01, 0.03), M["black_g"])  # window-line trim
    b.box((cx, 0, CAB_FLOOR - 0.03), (CAB_X1 - CAB_X0 + 0.02, 2 * CAB_HW + 0.02, 0.06), M["black"])    # lower skirt
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
        b.box(p1 + Vector((0.03, s * 0.05, -0.12)), (0.10, 0.22, 0.42), M["black_g"])
        b.box(p1 + Vector((-0.025, s * 0.05, -0.12)), (0.01, 0.19, 0.38), M["steel"])
        b.box(p1 + Vector((0.03, s * 0.05, -0.47)), (0.08, 0.18, 0.18), M["black_g"])                 # convex mirror
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
# v4: traced from Aryan's close-up photos of a real 612R CommandARM (top-down console, display key strip,
# hydro handle from several angles), cross-checked with dealer photos. Key positions are measured off the
# top-down photo (P() maps photo pixels to metres). Legends are printed on the keys.
# Functions whose key legend makes them unambiguous carry "evidence" in the spec (tread, ladder, park,
# engine speed, spray, AUTO boom height, wing tilt). Everything stays "verified": false until checked
# against the operator's manual.
CA_Y = -0.40                  # armrest centre line (inboard = +y, towards the seat)
CA_Z = CAB_FLOOR + 0.70
TOP = CA_Z + 0.078            # console top surface
CON_X = 5.215                 # console centre (x) in the cab frame
PX_S = 0.000206               # metres per pixel of the reference top-down photo
def P(px, py, dz=0.0):
    """Reference-photo pixel (front up, seat side left) -> cab-frame point on the console top."""
    return Vector((CON_X - (py - 890) * PX_S, CA_Y - (px - 785) * PX_S, TOP + dz))
def SZ(wpx, hpx, t=0.008):
    return (hpx * PX_S, wpx * PX_S, t)

# key-top text: reads left->right for the seated operator (text x -> -y, text up -> +x)
TXT_BASIS = Matrix(((0, 1, 0), (-1, 0, 0), (0, 0, 1)))      # columns: (0,-1,0), (1,0,0), (0,0,1)

def legend(b, pos, text, size, m=None, rot=None, lift=0.0):
    R = rot or Matrix.Identity(3)
    b.text(text, size, m or M["glyph"], Matrix.Translation(Vector(pos) + R @ Vector((0, 0, lift))) @ (R @ TXT_BASIS).to_4x4())

def key(name, ref, label, pos, m, size=(0.018, 0.018, 0.008), parent=None, rot=None, shape="box", text=None, tcol=None, tsize=None):
    """A clickable key/roller with an optional printed legend. ref=None -> decorative."""
    R = rot or Matrix.Identity(3)
    pos = Vector(pos)
    def fn(b):
        if shape == "round":
            b.cyl(pos, size[0] / 2, size[2], m, segs=28, rot=R)
        elif shape == "roller":              # thumbwheel rolling fore/aft (axis across the console)
            b.cyl(pos, size[0] / 2, size[1], m, segs=24, rot=R @ Matrix.Rotation(math.pi / 2, 3, "X"))
            for k in range(14):
                a = 2 * math.pi * k / 14
                b.box(pos + R @ Vector((size[0] / 2 * math.cos(a), 0, size[0] / 2 * math.sin(a))), (0.0025, size[1], 0.0025), m,
                      rot=R @ Matrix.Rotation(-a, 3, "Y"))
        else:
            b.box(pos, size, m, rot=R)
        if text:
            legend(b, pos, text, tsize or min(size[0], size[1]) * 0.45, tcol, rot=R, lift=size[2] / 2 + 0.0006)
    o = part(name, parent or cab, fn, bevel=0.0018 if shape == "box" else 0.0)
    if ref:
        tag(o, "control", ref, label)
    return o

# ---- console body: armrest pad, keypad console, separate hydro-handle gate pod on the seat side
GATE_V = 0.170                          # gate slot lateral offset from the armrest centre line
def commandarm(b):
    b.box((4.92, CA_Y, CA_Z - 0.26), (0.13, 0.13, 0.40), M["int_dark"])                       # mount post
    secs = [(4.56, 0.070, 0.030), (4.60, 0.088, 0.050), (4.98, 0.095, 0.060),
            (5.03, 0.140, 0.075), (5.36, 0.145, 0.075), (5.42, 0.125, 0.055)]
    b.loft([rrect(x, w, CA_Z - 0.10, CA_Z + h, 0.035, y0=CA_Y) for x, w, h in secs], M["int_gray"])
    b.loft([rrect(x, 0.080, CA_Z + 0.040, CA_Z + 0.072, 0.030, y0=CA_Y) for x in (4.60, 4.64, 4.95, 4.99)], M["int_pad"])
    b.box((4.86, CA_Y - 0.094, CA_Z + 0.00), (0.035, 0.012, 0.022), M["btn_o"])                # armrest adjust lever
    # gate pod (seat side): raised, with the J-shaped hydro-handle slot
    b.loft([rrect(x, w, CA_Z - 0.06, CA_Z + h, 0.025, y0=CA_Y + GATE_V) for x, w, h in
            ((5.12, 0.030, 0.050), (5.16, 0.045, 0.070), (5.40, 0.048, 0.072), (5.45, 0.035, 0.055))], M["int_gray"])
    b.box((5.27, CA_Y + GATE_V, CA_Z + 0.072), (0.19, 0.022, 0.006), M["black"])               # slot (fore/aft)
    b.box((5.185, CA_Y + GATE_V - 0.016, CA_Z + 0.072), (0.030, 0.030, 0.006), M["black"])     # J notch
    b.box((5.08, CA_Y + GATE_V + 0.03, CA_Z + 0.03), (0.09, 0.004, 0.05), M["int_pad"])        # shift-pattern decal
    # keypad beds (black membrane panels), measured off the photo
    b.box(P(277, 1225, -0.002), SZ(290, 660, 0.006), M["int_dark"])                           # inboard column
    b.box(P(763, 1090, -0.002), SZ(690, 350, 0.006), M["int_dark"])                           # radio / climate block
    b.box(P(1230, 760, -0.002), SZ(260, 900, 0.006), M["int_dark"])                           # outboard column
    b.box(P(725, 425, -0.005), SZ(590, 270, 0.012), M["black"])                               # front storage pocket
    b.box(P(780, 785, -0.001), SZ(540, 125, 0.003), M["int_pad"])                             # boom thumbwheel decal
    # display post
    b.tube((5.44, CA_Y + 0.0, CA_Z + 0.04), (5.535, CA_Y + 0.015, CA_Z + 0.30), 0.018, M["int_dark"])
part("commandarm_body", cab, commandarm, bevel=0.005)
part("commandarm_decal", cab, lambda b: [legend(b, P(px, 785, 0.0012), t, 0.010, M["int_dark"]) for px, t in ((630, "± ‿‿⌒"), (900, "⌒‿‿ ±"))])

# dash ledge to the right of the operator: cup holder, power sockets
def ledge(b):
    b.box((5.55, -0.72, CA_Z + 0.02), (0.55, 0.20, 0.10), M["int_gray"])
    b.cyl((5.42, -0.72, CA_Z + 0.05), 0.045, 0.06, M["int_dark"])
    for i in range(4):
        b.cyl((5.58 + i * 0.045, -0.72, CA_Z + 0.072), 0.013, 0.012, M["int_dark"], segs=12)
part("dash_ledge", cab, ledge, bevel=0.01)

# ---- display + key strip (X knob, 2x3 group, 2x4 group)
DSP_C = Vector((5.555, CA_Y + 0.025, CA_Z + 0.43))
DSP_ROT = Matrix.Rotation(math.radians(-22), 3, "Z") @ Matrix.Rotation(math.radians(-12), 3, "Y")
DSP_W, DSP_H = 0.33, 0.215
def display(b):
    b.box(DSP_C, (0.032, DSP_W, DSP_H), M["int_dark"], rot=DSP_ROT)
    b.box(DSP_C + DSP_ROT @ Vector((-0.017, 0, 0)), (0.002, DSP_W - 0.026, DSP_H - 0.026), M["screen"], rot=DSP_ROT)
    strip = DSP_C + DSP_ROT @ Vector((0.0, 0.0, -DSP_H / 2 - 0.040))
    b.box(strip, (0.030, 0.24, 0.065), M["int_gray"], rot=DSP_ROT)
o = part("DSP_commandcenter", cab, display)
tag(o, "control", "ca_display", "CommandCenter touchscreen")
_basis = Matrix(((0, 0, -1), (-1, 0, 0), (0, 1, 0)))   # columns: grid X -> -Y, grid Y -> +Z, normal -> -X
_scr = Builder("DSP_screen", o, origin=DSP_C)
_scr.bm.loops.layers.uv.new("UVMap")
_m = Matrix.Translation(DSP_ROT @ Vector((-0.019, 0, 0))) @ (DSP_ROT @ _basis).to_4x4() @ Matrix.Diagonal(((DSP_W - 0.026) / 2, (DSP_H - 0.026) / 2, 1, 1))
_r = bmesh.ops.create_grid(_scr.bm, x_segments=1, y_segments=1, size=1.0, matrix=_m, calc_uvs=True)
_scr._tag(_r["verts"], mat("Display_Screen", srgb("0B1F12"), 0.0, 0.3, emit=srgb("1D4D2A")))
tag(_scr.done(), "control", "ca_display", "CommandCenter touchscreen")
# strip keys face the operator: key "up" is the strip's up, "out" is -X of the display frame
STRIP_ROT = DSP_ROT @ Matrix.Rotation(-math.pi / 2, 3, "Y")   # key +Z -> towards the operator
def strip_pt(c, r):
    return DSP_C + DSP_ROT @ Vector((-0.017, 0.098 - c * 0.027, -DSP_H / 2 - 0.026 - r * 0.027))
strip_keys = {   # (col, row) -> (mesh, control id, label, legend)
    (0, 0): ("strip_k00", None, "", "→"), (1, 0): ("strip_k10", None, "", "▯"), (2, 0): ("strip_k20", None, "", "⊼"),
    (0, 1): ("strip_k01", None, "", "▶"), (1, 1): ("strip_k11", None, "", ""), (2, 1): ("strip_k21", None, "", ""),
    (4, 0): ("SW_outer_L_out", "ca_outer_L_out", "Left outer fold OUT", "↶"), (5, 0): ("SW_outer_L_in", "ca_outer_L_in", "Left outer fold IN", "↷"),
    (6, 0): ("SW_outer_R_in", "ca_outer_R_in", "Right outer fold IN", "↶"), (7, 0): ("SW_outer_R_out", "ca_outer_R_out", "Right outer fold OUT", "↷"),
    (4, 1): ("strip_k41", None, "", "☀"), (5, 1): ("strip_k51", None, "", "≋"), (6, 1): ("strip_k61", None, "", "⇧"),
    (7, 1): ("BTN_transport", "ca_transport", "Field / Road mode", "⇆"),
}
for (c, r), (nm, ref, lab, txt) in strip_keys.items():
    key(nm, ref, lab, strip_pt(c + (0.6 if c >= 4 else 0), r), M["btn_k"], size=(0.020, 0.024, 0.008), rot=STRIP_ROT, text=txt or None, tsize=0.011)
key("strip_knob_X", None, "", DSP_C + DSP_ROT @ Vector((-0.02, 0.135, -DSP_H / 2 - 0.040)), M["int_dark"], size=(0.040, 0.040, 0.030),
    rot=STRIP_ROT, shape="round", text="X", tsize=0.012)

# ---- console keys, measured off the top-down photo: (mesh, control id, label, px, py, wpx, hpx, colour, legend, legend colour)
K, O = "btn_k", "btn_o"
console_keys = [
    # seat-side: lock key + orange engine-speed plate (turtle | n/min rocker | rabbit)
    ("ca_key_lock",       None, "", 265, 660, 105, 90, K, "⊡", None),
    ("ca_rpm_low",        None, "", 185, 815, 110, 140, O, "n−", None),
    ("CTL_throttle", "ca_throttle", "Engine speed (n/min) up/down", 300, 790, 105, 160, O, "▲▼", None),
    ("ca_rpm_high",       None, "", 420, 770, 110, 150, O, "n+", None),
    # inboard keypad column
    ("ca_key_hitch",      None, "", 255, 960, 245, 105, K, "⇡", None),
    ("SW_tread_narrow", "ca_tread_narrower", "Tread adjust NARROWER", 200, 1080, 110, 115, K, "→‖←", None),
    ("SW_tread_wide",   "ca_tread_wider",    "Tread adjust WIDER",    320, 1080, 110, 115, K, "←‖→", None),
    ("ca_key_boomlock",   None, "", 200, 1210, 110, 115, K, "⊘", None),
    ("SW_ladder",       "ca_ladder",         "Ladder raise/lower",    320, 1210, 110, 115, K, "⇅", None),
    ("ca_key_blank1",     None, "", 200, 1340, 110, 115, K, None, None),
    ("ca_key_blank2",     None, "", 320, 1340, 110, 115, K, None, None),
    ("BTN_park",        "ca_park",           "Park (P)",              295, 1490, 235, 125, O, "(P)", None),
    # radio / climate block: rockers + bottom light row
    ("ca_vol",   None, "", 490, 1075, 120, 210, K, "+ −", None), ("ca_mute", None, "", 560, 965, 90, 60, K, "⊘", None),
    ("ca_track", None, "", 640, 1070, 120, 200, K, "♫", None),   ("ca_phone", None, "", 765, 1060, 120, 200, K, "☎", None),
    ("ca_temp",  None, "", 895, 1050, 120, 200, K, "+ −", "red"), ("ca_air", None, "", 965, 950, 90, 60, K, "≋", None),
    ("ca_fan",   None, "", 1045, 1050, 120, 200, K, "✣", None),
    ("ca_hazard", None, "", 535, 1215, 120, 70, K, "⚠", "red"), ("ca_poslt", None, "", 665, 1215, 120, 70, K, "≡◐", None),
    ("ca_headlt", None, "", 795, 1215, 120, 70, K, "≡D", None), ("ca_worklt", None, "", 920, 1215, 120, 70, K, "☀", None),
    ("ca_beacon", None, "", 1045, 1215, 120, 70, K, "✺", None),
    # outboard column (two columns, drifting outboard towards the rear)
    ("ca_autotrac", None, "", 1085, 345, 110, 95, K, "⟲", None), ("ca_docs", None, "", 1195, 345, 110, 95, K, "▤", None),
    ("ca_rate_man", None, "", 1110, 425, 120, 80, K, "⊘", None), ("ca_rate1", None, "", 1225, 420, 120, 80, K, "1", None),
    ("ca_rate2",    None, "", 1120, 505, 120, 80, K, "2", None), ("ca_rate3", None, "", 1245, 500, 120, 80, K, "3", None),
    ("ca_noz_off",  None, "", 1165, 705, 120, 80, K, "⊘", None), ("ca_noz_auto", None, "", 1290, 700, 120, 80, K, "AUTO", None),
    ("ca_noz_a",    None, "", 1180, 800, 120, 80, K, "A", None),  ("ca_noz_b",    None, "", 1310, 795, 120, 80, K, "B", None),
    ("ca_cam1",     None, "", 1220, 925, 125, 95, K, "1▶", None), ("ca_cam2",     None, "", 1355, 925, 125, 95, K, "2▶", None),
    ("ca_lock2",    None, "", 1240, 1050, 120, 80, K, "⊡", None), ("ca_mem1",     None, "", 1380, 1050, 120, 80, K, "1", None),
    ("ca_mem2",     None, "", 1310, 1160, 200, 80, K, "‹2›", None),
]
for nm, ref, lab, px, py, w, h, col, txt, tcol in console_keys:
    key(nm, ref, lab, P(px, py, 0.004), M[col], size=SZ(w * 0.9, h * 0.9), text=txt,
        tcol=M["tail"] if tcol == "red" else None, tsize=min(w, h) * PX_S * 0.42)
# boom-wing keys with fold arcs (outboard column): front half = OUT, rear half = IN (assignment unverified)
for side, px, txt in (("L", 1150, "⌒‿‿"), ("R", 1270, "‿‿⌒")):
    nm = "Left" if side == "L" else "Right"
    key(f"SW_fold_{side}_out", f"ca_fold_{side}_out", f"{nm} main fold OUT", P(px, 585, 0.004), M[K], size=SZ(110, 44), text=txt, tsize=0.006)
    key(f"SW_fold_{side}_in",  f"ca_fold_{side}_in",  f"{nm} main fold IN",  P(px, 632, 0.004), M[K], size=SZ(110, 44))
# thumbwheels behind the pocket (decal: boom wing ±) and the outboard thumbwheel
for i, px in enumerate((610, 850)):
    key(f"ca_thumbwheel_{i}", None, "", P(px, 635, 0.008), M["int_dark"], size=(110 * PX_S, 70 * PX_S, 0.0), shape="roller")
key("ca_thumbwheel_out", None, "", P(1150, 250, 0.010), M["int_dark"], size=(0.028, 0.016, 0.0), shape="roller")

# ---- hydro handle on the gate pod: black teardrop head with key face tilted to the operator,
# orange shell on the seat side wrapping into the dimpled finger grip, A-B-C-D keypad on the side.
HH_PIV = Vector((5.335, CA_Y + GATE_V, CA_Z + 0.075))
hh_piv = empty("PIV_hydro_handle", HH_PIV, cab, size=0.08)
HEAD_ROT = Matrix.Rotation(math.radians(-34), 3, "Y")
HEAD_C = HH_PIV + Vector((-0.020, -0.004, 0.180))
def H(u, v, z=0.0):            # head frame: u forward, v towards the seat, z out of the key face
    return HEAD_C + HEAD_ROT @ Vector((u, v, z))
def head_ring(u, w, z0, z1, r, dv=0.0):
    return [H(p.x, p.y, p.z) for p in rrect(u, w, z0, z1, r, y0=dv)]
def hh(b):
    b.cyl(HH_PIV + Vector((0, 0, 0.004)), 0.019, 0.010, M["black"], segs=16)                    # gate boot
    # head body: teardrop (wide front, narrow rear)
    prof = [(-0.090, 0.030), (-0.080, 0.036), (-0.040, 0.044), (0.010, 0.050), (0.050, 0.052), (0.072, 0.046), (0.082, 0.032)]
    b.loft([head_ring(u, w, -0.034, 0.0, 0.014) for u, w in prof], M["black"])
    # orange shell on the seat side, below the key face
    b.loft([head_ring(u, w * 0.96, -0.046, -0.010, 0.012, dv=0.006) for u, w in prof[:-1]], M["btn_o"])
    # swan-neck column: one smooth sweep from the gate, bowing back toward the operator, into the head.
    # Orange body with a black front spine (as in the photos), finger dimples on the seat side.
    P0 = HH_PIV + Vector((0.0, 0.002, 0.012))
    P1 = HH_PIV + Vector((-0.070, 0.002, 0.075))
    P2 = H(-0.030, 0.002, -0.032)
    def neck(t):
        return P0 * (1 - t) ** 2 + P1 * 2 * t * (1 - t) + P2 * t ** 2
    def ring(c, w, d):
        return [c + Vector((d * math.cos(a), w * math.sin(a), 0)) for a in [2 * math.pi * i / 20 for i in range(20)]]
    b.loft([ring(neck(k / 10), 0.018 + 0.018 * (k / 10) ** 1.5, 0.016 + 0.012 * (k / 10) ** 1.5) for k in range(11)], M["btn_o"])
    b.loft([ring(neck(k / 10) + Vector((0.011 + 0.006 * k / 10, 0, 0)), 0.011, 0.009) for k in range(11)], M["black"])
    for t in (0.30, 0.48, 0.66):
        c = neck(t) + Vector((0, 0.018 + 0.018 * t ** 1.5, 0))
        b.sphere(c, 0.0042, M["btn_o"], segs=10, scale=(1, 0.45, 1))
    b.box(H(-0.060, -0.040, -0.016), (0.040, 0.006, 0.026), M["black"], rot=HEAD_ROT)          # side keypad bed (outboard)
o = part("CTL_hydro_handle", hh_piv, hh)
tag(o, "control", "hh_lever", "Hydro handle")
HK = 0.006   # key height above the face
hh_keys = [
    # (mesh, control id, label, u, v, size(u,v), colour, legend)
    ("BTN_tilt_L_up",    "hh_tilt_L_up",    "Left wing tilt up",           0.018,  0.011, (0.022, 0.020), "btn_k", "↖"),
    ("BTN_tilt_R_up",    "hh_tilt_R_up",    "Right wing tilt up",          0.018, -0.012, (0.022, 0.020), "btn_k", "↗"),
    ("BTN_tilt_L_down",  "hh_tilt_L_down",  "Left wing tilt down",        -0.007,  0.011, (0.022, 0.020), "btn_k", "↙"),
    ("BTN_tilt_R_down",  "hh_tilt_R_down",  "Right wing tilt down",       -0.007, -0.012, (0.022, 0.020), "btn_k", "↘"),
    ("BTN_master_spray", "hh_master_spray", "Master spray ON/OFF (spray key)", 0.002, -0.037, (0.022, 0.020), "btn_y", "✱"),
    ("BTN_resume",       "hh_resume",       "Boom height AUTO (BoomTrac)", -0.034,  0.000, (0.020, 0.020), "btn_k", "AUTO"),
    ("BTN_boom_raise",   "hh_boom_raise",   "Boom height raise (centre)",  -0.029, -0.024, (0.011, 0.020), "btn_k", "↑"),
    ("BTN_boom_lower",   "hh_boom_lower",   "Boom height lower (centre)",  -0.041, -0.024, (0.011, 0.020), "btn_k", "↓"),
]
for nm, ref, lab, u, v, (su, sv), col, txt in hh_keys:
    key(nm, ref, lab, H(u, v, HK / 2), M[col], size=(su, sv, HK), parent=hh_piv, rot=HEAD_ROT, text=txt,
        tcol=M["black"] if col == "btn_y" else None, tsize=0.0055 if len(txt) > 2 else 0.009)
# E rocker (round, seat side rear): top half = range up, bottom half = range down
key("BTN_range_up",   "hh_range_up",   "Speed range up (E rocker)",   H(-0.020, 0.032, HK / 2), M["btn_k"], size=(0.016, 0.032, HK), parent=hh_piv, rot=HEAD_ROT, text="E▲", tsize=0.006)
key("BTN_range_down", "hh_range_down", "Speed range down (E rocker)", H(-0.038, 0.032, HK / 2), M["btn_k"], size=(0.016, 0.032, HK), parent=hh_piv, rot=HEAD_ROT, text="▼", tsize=0.007)
def hh_decor(b):
    b.box(H(0.062, 0.034, 0.010), (0.024, 0.024, 0.020), M["btn_y"], rot=HEAD_ROT)                     # yellow cube (front, seat side)
    legend(b, H(0.062, 0.034, 0.0205), "△", 0.012, M["black"], rot=HEAD_ROT)
    for u, v, su, sv, t in ((0.045, 0.030, 0.020, 0.018, "✱"), (0.045, 0.004, 0.022, 0.030, "←✱→"), (0.035, -0.034, 0.040, 0.022, "HMS F")):
        b.box(H(u, v, HK / 2), (su, sv, HK), M["btn_k"], rot=HEAD_ROT)
        legend(b, H(u, v, HK + 0.0006), t, 0.0055, rot=HEAD_ROT)
    for i, t in enumerate("ABCD"):                                                                   # side keypad
        b.box(H(-0.052 - (i // 2) * 0.013, -0.044, -0.010 - (i % 2) * 0.012), (0.011, 0.004, 0.010), M["btn_k"], rot=HEAD_ROT)
part("hydro_handle_keys", hh_piv, hh_decor, bevel=0.0012)

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
        ("render_front.png",     TRANSPORT, ((11.5, 6.0, 2.4), (5.4, 0.2, 2.3), 38)),
        ("render_nose.png",      dict(boom_height=0.76, ladder=0), ((10.2, 4.6, 2.6), (5.6, 0.0, 2.35), 42)),
        ("render_side.png",      TRANSPORT, ((2.6, 20, 2.2), (2.6, 0, 2.0), 40)),
        ("render_rear.png",      TRANSPORT, ((-11, -4.5, 2.6), (0, 0, 2.0), 35)),
        ("render_field.png",     dict(boom_height=0.76), ((-13, 21, 6.5), (-1, 4, 1.2), 30)),
        ("render_commandarm.png", dict(boom_height=0.76, cab_door=0), ((3.72, 0.06, 3.50), (4.15, -0.38, 3.02), 26)),
        ("render_commandarm_top.png", dict(boom_height=0.76), ((3.93, -0.40, 3.80), None, 20)),
        ("render_hydro_handle.png", dict(boom_height=0.76), ((3.86, -0.02, 3.40), (4.075, -0.24, 3.22), 45)),
        ("render_hydro_side.png",   dict(boom_height=0.76), ((3.86, 0.20, 3.18), (4.06, -0.23, 3.15), 34)),
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
