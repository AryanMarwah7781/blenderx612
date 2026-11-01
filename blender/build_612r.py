"""
Procedural Blender model of a John Deere 612R self-propelled sprayer.

Run (headless):   blender -b -P blender/build_612r.py -- --out build
            or:   python3 blender/build_612r.py --out build          (pip install bpy)
Optional:         --render   (writes preview PNGs to docs/)

Everything is generated from spec/612r_spec.json.
  * Every moving part hangs under an empty named PIV_<joint>, at the real pivot point.
  * The root object "SPRAYER_612R" carries one custom property per joint, and drivers
    connect each property to the pivots. In Blender: select SPRAYER_612R, open
    Object Properties > Custom Properties, and drag the values. The machine moves
    with the right kinematics (Ackermann steering, parallel-link boom lift, fold chain, and so on).
  * Clickable controls and service points are separate meshes named CTL_/BTN_/SW_/SVC_/DSP_*,
    so a web viewer (viewer/) or a RAG pipeline can refer to them by ID.

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

CAB_X0, CAB_X1 = 4.20, 6.30
CAB_HW = 0.95
CAB_FLOOR = 2.28
CAB_ROOF = 3.92

LINK = JOINTS["boom_height"]["linkage"]
ARM_L = LINK["arm_length"]
UPPER_PIV = Vector((-0.90, 0, LINK["pivot_z"]))
LOWER_PIV = Vector((-0.90, 0, LINK["pivot_z"] - 0.60))
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
def mat(name, rgb, metal=0.0, rough=0.5, alpha=1.0, emit=None):
    m = bpy.data.materials.new(name)
    bsdf = m.node_tree.nodes["Principled BSDF"]
    bsdf.inputs["Base Color"].default_value = (*rgb, 1)
    bsdf.inputs["Metallic"].default_value = metal
    bsdf.inputs["Roughness"].default_value = rough
    if alpha < 1:
        bsdf.inputs["Alpha"].default_value = alpha
        try:
            m.surface_render_method = "BLENDED"
        except AttributeError:
            m.blend_method = "BLEND"
    if emit:
        bsdf.inputs["Emission Color"].default_value = (*emit, 1)
        bsdf.inputs["Emission Strength"].default_value = 2.0
    m.diffuse_color = (*rgb, alpha)
    return m

def srgb(h):
    c = [int(h[i:i + 2], 16) / 255 for i in (0, 2, 4)]
    return tuple(((x + 0.055) / 1.055) ** 2.4 if x > 0.04045 else x / 12.92 for x in c)

M = {
    "green":   mat("JD_Green", srgb("367C2B"), 0.1, 0.35),
    "yellow":  mat("JD_Yellow", srgb("FFDE00"), 0.1, 0.4),
    "black":   mat("Black_Plastic", srgb("1A1A1A"), 0.0, 0.6),
    "tire":    mat("Tire_Rubber", srgb("141414"), 0.0, 0.9),
    "steel":   mat("Stainless", srgb("C8CCD0"), 1.0, 0.25),
    "dsteel":  mat("Dark_Steel", srgb("4A4E52"), 0.8, 0.45),
    "boom":    mat("Boom_Steel", srgb("D9DCDF"), 0.6, 0.35),
    "glass":   mat("Cab_Glass", srgb("9DB8C8"), 0.0, 0.05, alpha=0.28),
    "seat":    mat("Seat_Fabric", srgb("2B2B2B"), 0.0, 0.9),
    "engine":  mat("Engine_Gray", srgb("5C5F62"), 0.6, 0.5),
    "red":     mat("Hydraulic_Red", srgb("B3261E"), 0.2, 0.4),
    "orange":  mat("Nozzle_Orange", srgb("E87722"), 0.0, 0.5),
    "blue":    mat("DEF_Blue", srgb("1F5FAF"), 0.0, 0.4),
    "white":   mat("Poly_White", srgb("E8E8E2"), 0.0, 0.5),
    "screen":  mat("Display", srgb("0B1F12"), 0.0, 0.2, emit=srgb("1D4D2A")),
    "btn_g":   mat("Btn_Green", srgb("2E9E44"), 0.0, 0.4),
    "btn_y":   mat("Btn_Yellow", srgb("F2C200"), 0.0, 0.4),
    "btn_r":   mat("Btn_Red", srgb("C8202A"), 0.0, 0.4),
    "btn_k":   mat("Btn_Gray", srgb("6B6E70"), 0.0, 0.5),
    "lens":    mat("Light_Lens", srgb("F5F2E0"), 0.0, 0.1, emit=srgb("FFF8D8")),
    "amber":   mat("Beacon_Amber", srgb("FF9A00"), 0.0, 0.2, emit=srgb("FF8800")),
    "ground":  mat("Soil", srgb("5B4632"), 0.0, 0.95),
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

    def _tag(self, geom_verts, m):
        mi = self._mi(m)
        faces = {f for v in geom_verts for f in v.link_faces}
        for f in faces:
            f.material_index = mi
            f.smooth = False

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

    def cyl(self, center, radius, depth, m, axis="Z", segs=24, r2=None, rot=None):
        if rot is None:
            rot = {"X": Matrix.Rotation(math.pi / 2, 3, "Y"),
                   "Y": Matrix.Rotation(math.pi / 2, 3, "X"),
                   "Z": Matrix.Identity(3)}[axis]
        mtx = Matrix.Translation(self._xf(center)) @ rot.to_4x4()
        r = bmesh.ops.create_cone(self.bm, cap_ends=True, segments=segs, radius1=radius,
                                  radius2=radius if r2 is None else r2, depth=depth, matrix=mtx)
        self._tag(r["verts"], m)
        return self

    def tube(self, p1, p2, radius, m, segs=8):
        p1, p2 = Vector(p1), Vector(p2)
        d = p2 - p1
        if d.length < 1e-6:
            return self
        rot = Vector((0, 0, 1)).rotation_difference(d.normalized()).to_matrix()
        return self.cyl((p1 + p2) / 2, radius, d.length, m, rot=rot, segs=segs)

    def sphere(self, center, radius, m, segs=16):
        mtx = Matrix.Translation(self._xf(center))
        r = bmesh.ops.create_uvsphere(self.bm, u_segments=segs, v_segments=segs // 2, radius=radius, matrix=mtx)
        self._tag(r["verts"], m)
        return self

    def done(self, bevel=0.0, smooth=False, props=None):
        me = bpy.data.meshes.new(self.name)
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

# ============================================================================= ROOT + CHASSIS
root = empty("SPRAYER_612R", (0, 0, 0), size=1.0)
root["model"] = SPEC["model"]
root["spec_version"] = SPEC["spec_version"]

chassis = empty("PIV_chassis", (0, 0, 0), root)   # body heave/pitch/roll are applied here by the viewer

def frame(b):
    # twin main rails from the boom mount to the cab
    for y in (0.42, -0.42):
        b.box((2.6, y, (FRAME_BOT + FRAME_TOP) / 2), (7.6, 0.16, FRAME_TOP - FRAME_BOT), M["green"])
    for x in (-0.95, 0.0, 1.3, 2.4, 3.5, 4.3, 5.8):
        b.box((x, 0, FRAME_TOP - 0.12), (0.14, 0.84, 0.18), M["green"])
    # axle carrier housings (outer tubes of the tread-adjust telescopes)
    for x in (0.0, WB):
        for s in (1, -1):
            b.box((x, s * 0.80, FRAME_TOP - 0.16), (0.30, 0.78, 0.26), M["green"])
    # rear boom mount tower
    b.box((-0.92, 0, (LOWER_PIV.z + UPPER_PIV.z) / 2), (0.22, 1.10, 0.95), M["green"])
    # boom cradles (fold rests) beside the cab
    for s in (1, -1):
        b.box((4.55, s * 1.25, 2.12), (0.16, 0.70, 0.10), M["black"])
        b.box((4.55, s * 1.70, 2.20), (0.30, 0.30, 0.12), M["black"])
        b.box((4.55, s * 1.25, 1.98), (0.12, 0.12, 0.28), M["black"])
    # side walkway (left) from ladder to door
    b.box((5.2, 1.08, CAB_FLOOR - 0.06), (1.9, 0.30, 0.04), M["dsteel"])
part("chassis_frame", chassis, frame, bevel=0.01)

# ============================================================================= WHEEL LEGS
def build_leg(tag_, x, side, front):
    y_wheel = side * TREAD0 / 2
    y_leg = side * (TREAD0 / 2 - LEG_INSET)
    hub_z = TIRE_R
    tread_piv = empty(f"PIV_tread_{tag_}", (x, y_leg, FRAME_TOP - 0.16), chassis)

    def inner_tube(b):   # inner telescoping axle beam, slides with the tread
        b.box((x, y_leg - side * 0.55, FRAME_TOP - 0.16), (0.22, 1.10, 0.20), M["green"])
        b.box((x, y_leg, FRAME_TOP - 0.10), (0.36, 0.36, 0.34), M["green"])
    part(f"axle_beam_{tag_}", tread_piv, inner_tube, bevel=0.01)

    def strut_body(b):
        b.cyl((x, y_leg, FRAME_TOP - 0.42), 0.13, 0.55, M["dsteel"])
        b.cyl((x, y_leg, FRAME_TOP - 0.12), 0.17, 0.12, M["black"])  # air spring
    o = part(f"SVC_strut_{tag_}" if tag_ == "FL" else f"strut_body_{tag_}", tread_piv, strut_body, smooth=True)
    if tag_ == "FL":
        tag(o, "service_point", "sp_strut_FL", "Front-left air-ride strut")

    susp = empty(f"PIV_susp_{tag_}", (x, y_leg, FRAME_TOP - 0.16), tread_piv)

    def strut_rod(b):
        b.cyl((x, y_leg, hub_z + 0.55), 0.085, 0.75, M["steel"])
        b.cyl((x, y_leg, hub_z + 0.10), 0.16, 0.30, M["green"])   # leg knuckle
    part(f"strut_rod_{tag_}", susp, strut_rod, smooth=True)

    steer_parent = susp
    if front:
        steer_parent = empty(f"PIV_steer_{tag_}", (x, y_leg, hub_z), susp)

    def hub(b):
        b.cyl((x, (y_leg + y_wheel) / 2 - side * 0.02, hub_z), 0.27, abs(y_wheel - y_leg) - 0.05, M["engine"], axis="Y")
        b.cyl((x, y_leg, hub_z), 0.2, 0.25, M["green"], axis="Y")
    o = part(f"SVC_wheel_motor_{tag_}" if tag_ == "FL" else f"wheel_motor_{tag_}", steer_parent, hub, smooth=True)
    if tag_ == "FL":
        tag(o, "service_point", "sp_wheel_motor_FL", "Front-left wheel motor / final drive")

    roll = empty(f"PIV_roll_{tag_}", (x, y_wheel, hub_z), steer_parent)

    def tire(b):
        b.cyl((x, y_wheel, hub_z), TIRE_R - 0.03, TIRE_W, M["tire"], axis="Y", segs=48)
        n = 36  # tread lugs (R1-W style chevrons)
        for i in range(n):
            a = 2 * math.pi * i / n
            for s2 in (1, -1):
                c = Vector((x + (TIRE_R - 0.02) * math.cos(a), y_wheel + s2 * TIRE_W * 0.24, hub_z + (TIRE_R - 0.02) * math.sin(a)))
                rot = Matrix.Rotation(-a, 3, "Y") @ Matrix.Rotation(s2 * 0.45, 3, "X")
                b.box(c, (0.035, TIRE_W * 0.5, 0.07), M["tire"], rot=rot)
    part(f"tire_{tag_}", roll, tire, bevel=0.0, smooth=False)

    def rim(b):
        b.cyl((x, y_wheel + side * 0.01, hub_z), RIM_R, TIRE_W * 0.92, M["yellow"], axis="Y", segs=40)
        b.cyl((x, y_wheel + side * TIRE_W * 0.47, hub_z), 0.30, 0.03, M["yellow"], axis="Y")
        for i in range(10):
            a = 2 * math.pi * i / 10
            b.cyl((x + 0.22 * math.cos(a), y_wheel + side * TIRE_W * 0.49, hub_z + 0.22 * math.sin(a)), 0.022, 0.03, M["dsteel"], axis="Y", segs=6)
    part(f"rim_{tag_}", roll, rim)

for tag_, x, side, front in (("FL", WB, 1, True), ("FR", WB, -1, True), ("RL", 0.0, 1, False), ("RR", 0.0, -1, False)):
    build_leg(tag_, x, side, front)

# ============================================================================= POWERTRAIN / REAR
ENG_X0, ENG_X1 = -0.82, 1.25
ENG_TOP = 2.92

def engine(b):
    b.box((0.25, 0, 2.15), (1.25, 0.62, 0.62), M["engine"])          # 9.0 L block
    b.box((0.25, 0, 2.55), (1.15, 0.38, 0.20), M["engine"])          # valve cover
    b.cyl((-0.25, 0.30, 2.45), 0.14, 0.2, M["dsteel"], axis="Y")     # turbo
    b.cyl((0.9, -0.36, 2.05), 0.12, 0.30, M["dsteel"], axis="Y")     # hydro pump drive
    b.box((0.95, 0, 2.05), (0.25, 0.55, 0.5), M["dsteel"])           # pump drive gearbox
part("engine_block", chassis, engine, bevel=0.01)

def svc(name, ref, label, fn):
    o = part(name, chassis, fn, smooth=True)
    tag(o, "service_point", ref, label)
    return o

svc("SVC_engine_oil", "sp_engine_oil", "Engine oil dipstick and fill",
    lambda b: b.cyl((0.75, 0.38, 2.62), 0.035, 0.22, M["yellow"]).cyl((0.75, 0.38, 2.75), 0.05, 0.04, M["yellow"]))
svc("SVC_air_filter", "sp_air_filter", "Engine air filter housing",
    lambda b: b.cyl((-0.30, -0.42, 2.48), 0.17, 0.62, M["black"], axis="X").cyl((-0.65, -0.42, 2.48), 0.05, 0.08, M["yellow"], axis="X"))
svc("SVC_fuel_filter", "sp_fuel_filter", "Fuel filters / water separator",
    lambda b: b.cyl((0.55, -0.44, 2.15), 0.06, 0.24, M["white"]).cyl((0.72, -0.44, 2.15), 0.06, 0.24, M["white"]))
svc("SVC_cooling", "sp_cooling", "Cooling package (radiator / charge-air cooler)",
    lambda b: b.box((-0.66, 0, 2.30), (0.18, 1.30, 0.95), M["dsteel"]))
svc("SVC_hyd_reservoir", "sp_hyd_reservoir", "Hydraulic reservoir (98.4 L)",
    lambda b: b.box((0.55, -0.95, 1.95), (0.55, 0.30, 0.45), M["green"]).cyl((0.55, -0.95, 2.20), 0.05, 0.06, M["black"]).box((0.55, -1.105, 1.95), (0.12, 0.01, 0.2), M["glass"]))
svc("SVC_battery", "sp_battery", "Battery box / disconnect",
    lambda b: b.box((3.75, 0.82, 1.70), (0.45, 0.32, 0.30), M["black"]).cyl((3.75, 0.99, 1.75), 0.03, 0.05, M["red"], axis="Y"))
svc("SVC_air_tank", "sp_air_tank", "Onboard air compressor tank",
    lambda b: b.cyl((2.0, 0, FRAME_BOT + 0.12), 0.11, 0.9, M["dsteel"], axis="X"))

# fuel + DEF (right side)
def fuel(b):
    b.box((2.15, -0.98, 1.92), (1.55, 0.45, 0.62), M["black"])    # 565 L
part("fuel_tank", chassis, fuel, bevel=0.03)
svc("SVC_fuel_fill", "sp_fuel_fill", "Fuel fill (565 L)", lambda b: b.cyl((2.70, -1.24, 2.15), 0.06, 0.08, M["yellow"], axis="Y"))
svc("SVC_def_fill", "sp_def_fill", "DEF fill (29.5 L)",
    lambda b: b.box((3.15, -0.98, 1.95), (0.32, 0.40, 0.40), M["white"]).cyl((3.15, -1.21, 2.08), 0.05, 0.06, M["blue"], axis="Y"))

# engine hood, pivots at its front top edge
hood_piv = empty("PIV_hood", (ENG_X1, 0, ENG_TOP), chassis)
def hood(b):
    b.box((0.22, 0, 2.68), (2.10, 1.50, 0.48), M["green"])
    b.box((0.22, 0.76, 2.25), (2.10, 0.03, 0.70), M["green"])
    b.box((0.22, -0.76, 2.25), (2.10, 0.03, 0.70), M["green"])
    b.box((ENG_X0 - 0.02, 0, 2.28), (0.04, 1.50, 0.95), M["black"])  # rear grille
    for i in range(9):
        b.box((ENG_X0 - 0.05, 0, 1.9 + i * 0.095), (0.02, 1.4, 0.02), M["dsteel"])
    b.cyl((-0.1, 0.45, 3.05), 0.07, 0.35, M["black"])               # exhaust stack
    b.box((-0.84, 0.55, 2.75), (0.04, 0.18, 0.08), M["lens"])
    b.box((-0.84, -0.55, 2.75), (0.04, 0.18, 0.08), M["lens"])
part("engine_hood", hood_piv, hood, bevel=0.02)
o = part("CTL_hood_latch", hood_piv, lambda b: b.box((ENG_X0 - 0.07, 0, 2.02), (0.06, 0.16, 0.05), M["yellow"]))
tag(o, "control", "ex_hood_latch", "Engine hood latch")

# ============================================================================= SOLUTION SYSTEM
TANK_X0, TANK_X1 = 1.32, 3.52
TANK_A, TANK_B = 0.90, 0.73
TANK_ZC = 1.88 + TANK_B
def tank(b):
    rot = Matrix.Rotation(math.pi / 2, 3, "Y") @ Matrix.Diagonal((TANK_B / TANK_A, 1, 1))
    b.cyl(((TANK_X0 + TANK_X1) / 2, 0, TANK_ZC), TANK_A, TANK_X1 - TANK_X0, M["steel"], rot=rot, segs=48)
    for x in (TANK_X0 + 0.25, TANK_X1 - 0.25):
        b.box((x, 0, 1.92), (0.12, 1.5, 0.12), M["green"])
    b.box(((TANK_X0 + TANK_X1) / 2, 0, TANK_ZC + TANK_B + 0.02), (1.6, 0.08, 0.04), M["dsteel"])  # top walkway rail
part("solution_tank", chassis, tank, smooth=True)
svc("SVC_tank_lid", "sp_solution_tank", "Solution tank lid (4,542 L)",
    lambda b: b.cyl((2.0, 0, TANK_ZC + TANK_B + 0.02), 0.22, 0.08, M["black"], segs=32))
svc("SVC_rinse_tank", "sp_rinse_tank", "Rinse tank (646 L)",
    lambda b: b.box((3.84, 0, 2.30), (0.55, 1.40, 0.84), M["white"]).cyl((3.84, 0, 2.75), 0.1, 0.06, M["black"]))
svc("SVC_solution_pump", "sp_solution_pump", "Solution pump (hydraulically driven centrifugal)",
    lambda b: b.cyl((2.4, 0, FRAME_BOT + 0.08), 0.17, 0.22, M["dsteel"], axis="Y").cyl((2.4, 0.18, FRAME_BOT + 0.08), 0.09, 0.15, M["red"], axis="Y"))

# left-side fill station + plumbing panel
def plumbing(b):
    b.box((2.35, 1.02, 1.72), (1.25, 0.06, 0.40), M["green"])       # panel
    b.tube((1.6, 1.08, 1.60), (3.1, 1.08, 1.60), 0.035, M["steel"])  # manifold
    b.tube((2.4, 0.4, FRAME_BOT + 0.08), (2.4, 1.0, 1.60), 0.04, M["steel"])
part("plumbing_panel", chassis, plumbing)
svc("SVC_suction_strainer", "sp_suction_strainer", "Suction strainer (20 mesh)",
    lambda b: b.cyl((1.75, 1.16, 1.68), 0.08, 0.26, M["white"]).cyl((1.75, 1.16, 1.84), 0.09, 0.05, M["black"]))
svc("SVC_pressure_strainer", "sp_pressure_strainer", "Pressure strainer (50 mesh)",
    lambda b: b.cyl((2.15, 1.16, 1.68), 0.07, 0.24, M["white"]).cyl((2.15, 1.16, 1.83), 0.08, 0.05, M["red"]))
svc("SVC_quick_fill", "sp_quick_fill", "3 in quick-fill coupler (side fill)",
    lambda b: b.cyl((3.0, 1.16, 1.62), 0.055, 0.14, M["black"], axis="Y").cyl((3.0, 1.24, 1.62), 0.07, 0.03, M["yellow"], axis="Y"))
for i, x in enumerate((2.55, 2.75)):
    part(f"valve_handle_{i}", chassis, lambda b, x=x: b.box((x, 1.10, 1.80), (0.04, 0.06, 0.16), M["yellow"]))

# chemical eductor on a parallel-link arm (the hopper stays level)
EDU_PIV = Vector((2.55, 1.06, 1.45))
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
    b.cyl(c + Vector((0, 0, 0.35)), 0.25, 0.03, M["yellow"], segs=32)
part("eductor_hopper", edu_lvl, edu_hopper, smooth=True)
o = part("CTL_eductor_handle", edu_lvl, lambda b: b.box(EDU_PIV + Vector((0.06, 0.50, 0.75)), (0.05, 0.10, 0.03), M["yellow"]))
tag(o, "control", "ex_eductor", "Chemical eductor (lower/raise)")

# ============================================================================= CAB
def cab_shell(b):
    cx = (CAB_X0 + CAB_X1) / 2
    b.box((cx, 0, CAB_FLOOR - 0.10), (CAB_X1 - CAB_X0 + 0.1, 2 * CAB_HW + 0.1, 0.20), M["green"])     # floor pan
    b.box((cx - 0.05, 0, (FRAME_TOP + CAB_FLOOR) / 2 - 0.1), (1.6, 1.2, CAB_FLOOR - FRAME_TOP), M["green"])  # cab mount
    for x in (CAB_X0 + 0.04, CAB_X1 - 0.04):
        for y in (CAB_HW - 0.04, -CAB_HW + 0.04):
            b.box((x, y, (CAB_FLOOR + CAB_ROOF) / 2), (0.08, 0.08, CAB_ROOF - CAB_FLOOR), M["black"])
    b.box((CAB_X0 + 0.02, 0, (CAB_FLOOR + CAB_ROOF) / 2), (0.04, 2 * CAB_HW, CAB_ROOF - CAB_FLOOR), M["black"])  # rear wall
    # roof with overhang + visor + StarFire receiver + beacons + lights
    b.box((cx + 0.08, 0, CAB_ROOF + 0.08), (CAB_X1 - CAB_X0 + 0.35, 2 * CAB_HW + 0.25, 0.16), M["green"])
    b.box((cx + 0.08, 0, CAB_ROOF + 0.17), (CAB_X1 - CAB_X0 + 0.15, 2 * CAB_HW + 0.05, 0.04), M["white"])
    b.sphere((cx - 0.2, 0, CAB_ROOF + 0.26), 0.13, M["yellow"])
    b.box((cx - 0.2, 0, CAB_ROOF + 0.2), (0.3, 0.3, 0.06), M["green"])
    for y in (0.8, -0.8):
        b.box((CAB_X1 + 0.20, y * 0.9, CAB_ROOF + 0.06), (0.04, 0.18, 0.08), M["lens"])
        b.box((CAB_X1 + 0.20, y * 0.45, CAB_ROOF + 0.06), (0.04, 0.18, 0.08), M["lens"])
        b.box((CAB_X0 - 0.12, y, CAB_ROOF + 0.06), (0.04, 0.16, 0.08), M["lens"])
    b.cyl((CAB_X0 + 0.25, 0.85, CAB_ROOF + 0.25), 0.07, 0.14, M["amber"])
    b.cyl((CAB_X0 + 0.25, -0.85, CAB_ROOF + 0.25), 0.07, 0.14, M["amber"])
    # mirrors
    for s in (1, -1):
        b.tube((CAB_X1 - 0.05, s * CAB_HW, CAB_ROOF - 0.3), (CAB_X1 + 0.2, s * (CAB_HW + 0.55), CAB_ROOF - 0.3), 0.02, M["black"])
        b.box((CAB_X1 + 0.2, s * (CAB_HW + 0.55), CAB_ROOF - 0.5), (0.04, 0.22, 0.38), M["black"])
    # front/hood nose under the windscreen, headlights
    b.box((CAB_X1 + 0.12, 0, CAB_FLOOR - 0.25), (0.35, 1.7, 0.5), M["green"])
    for y in (0.55, -0.55):
        b.box((CAB_X1 + 0.30, y, CAB_FLOOR - 0.20), (0.03, 0.30, 0.10), M["lens"])
    b.box((CAB_X1 + 0.31, 0, CAB_FLOOR - 0.33), (0.02, 0.6, 0.12), M["yellow"])  # deer-stripe badge stand-in
part("cab_shell", chassis, cab_shell, bevel=0.012)

def cab_glass(b):
    h = CAB_ROOF - CAB_FLOOR
    zc = (CAB_FLOOR + CAB_ROOF) / 2
    b.box((CAB_X1 - 0.02, 0, zc), (0.02, 2 * CAB_HW - 0.1, h - 0.04), M["glass"])               # windscreen
    b.box(((CAB_X0 + CAB_X1) / 2, -CAB_HW + 0.02, zc), (CAB_X1 - CAB_X0 - 0.1, 0.02, h - 0.04), M["glass"])  # right side
    b.box((CAB_X1 - 0.30, CAB_HW - 0.02, zc), (0.42, 0.02, h - 0.04), M["glass"])               # left fixed pane
part("cab_glass", chassis, cab_glass)

# door: left side, hinged on the rear-left post
DOOR_X0, DOOR_X1 = CAB_X0 + 0.10, CAB_X1 - 0.52
door_piv = empty("PIV_cab_door", (DOOR_X0, CAB_HW, CAB_FLOOR), chassis)
def door(b):
    h = CAB_ROOF - CAB_FLOOR - 0.06
    b.box(((DOOR_X0 + DOOR_X1) / 2, CAB_HW, CAB_FLOOR + h / 2 + 0.02), (DOOR_X1 - DOOR_X0, 0.02, h), M["glass"])
    b.box(((DOOR_X0 + DOOR_X1) / 2, CAB_HW, CAB_FLOOR + 0.05), (DOOR_X1 - DOOR_X0, 0.04, 0.08), M["black"])
    b.box((DOOR_X1 - 0.03, CAB_HW, CAB_FLOOR + h / 2), (0.05, 0.04, h), M["black"])
part("cab_door", door_piv, door)
o = part("CTL_door_handle", door_piv, lambda b: b.box((DOOR_X1 - 0.10, CAB_HW + 0.04, CAB_FLOOR + 1.0), (0.16, 0.03, 0.04), M["dsteel"]))
tag(o, "control", "ex_cab_door", "Cab door handle")

# ---------------------------------------------------------------- interior
SEAT = Vector((4.95, 0.02, CAB_FLOOR))
def interior(b):
    b.box((CAB_X0 + 0.5, 0, CAB_FLOOR + 0.01), (1.8, 1.7, 0.02), M["black"])         # floor mat
    b.box(SEAT + Vector((0, 0, 0.22)), (0.25, 0.25, 0.44), M["dsteel"])            # suspension base
    b.box(SEAT + Vector((0.05, 0, 0.50)), (0.52, 0.52, 0.12), M["seat"])            # cushion
    b.box(SEAT + Vector((-0.20, 0, 0.88)), (0.12, 0.52, 0.68), M["seat"])           # backrest
    b.box(SEAT + Vector((-0.22, 0, 1.28)), (0.10, 0.28, 0.16), M["seat"])           # headrest
    b.box(SEAT + Vector((0.02, 0.32, 0.68)), (0.36, 0.06, 0.05), M["seat"])         # left armrest
    b.box((CAB_X1 - 0.30, 0, CAB_FLOOR + 0.06), (0.30, 0.6, 0.08), M["black"])     # pedals plate
    for y in (0.12, -0.12):
        b.box((CAB_X1 - 0.30, y, CAB_FLOOR + 0.16), (0.06, 0.1, 0.16), M["dsteel"])  # brake / decel pedal
    b.box((CAB_X0 + 0.28, -0.75, CAB_FLOOR + 0.75), (0.35, 0.32, 0.25), M["black"])  # rear right console
    b.box((CAB_X1 - 0.10, 0.70, CAB_ROOF - 0.20), (0.15, 0.40, 0.20), M["black"])     # corner post display
part("cab_interior", chassis, interior, bevel=0.01)

# steering column (tilted) + steering wheel pivot
COL_BASE = Vector((5.62, 0.02, CAB_FLOOR + 0.05))
COL_TOP = Vector((5.42, 0.02, CAB_FLOOR + 0.95))
part("steering_column", chassis, lambda b: b.tube(COL_BASE, COL_TOP, 0.06, M["black"], segs=16).box(COL_BASE + Vector((0, 0, 0.25)), (0.18, 0.28, 0.5), M["black"]))
sw_tilt = empty("PIV_steering_tilt", COL_TOP, chassis)
sw_tilt.rotation_euler = (0, -math.atan2(COL_BASE.x - COL_TOP.x, COL_TOP.z - COL_BASE.z), 0)
sw_piv = bpy.data.objects.new("PIV_steering_wheel", None); COLL.objects.link(sw_piv)
sw_piv.parent = sw_tilt; sw_piv.empty_display_size = 0.1
WORLD["PIV_steering_wheel"] = COL_TOP
bw = Builder("CTL_steering_wheel", sw_piv, local=True)
bw.cyl((0, 0, 0.02), 0.20, 0.035, M["black"], segs=40)
bw.cyl((0, 0, 0.02), 0.17, 0.04, M["black"], segs=40)  # (rim approximated as a solid ring disk; thin)
bw.cyl((0, 0, 0.03), 0.05, 0.05, M["green"], segs=20)
bw.box((0.13, 0, 0.035), (0.02, 0.02, 0.02), M["yellow"])  # spinner / position mark
o = bw.done(smooth=True)
tag(o, "control", "ca_steering_wheel", "Steering wheel")

# ---------------------------------------------------------------- CommandARM
CA_Y = -0.38
CA_Z = CAB_FLOOR + 0.70
def commandarm(b):
    b.box((4.95, CA_Y, CA_Z - 0.20), (0.10, 0.10, 0.40), M["dsteel"])              # mount post
    b.box((5.02, CA_Y, CA_Z), (0.66, 0.20, 0.08), M["black"])                     # armrest body
    b.box((4.90, CA_Y, CA_Z + 0.055), (0.38, 0.17, 0.03), M["seat"])              # padded rest
    b.box((5.22, CA_Y - 0.02, CA_Z + 0.045), (0.28, 0.22, 0.02), M["black"])     # switch deck
    b.tube((5.30, CA_Y - 0.12, CA_Z), (5.48, CA_Y - 0.28, CA_Z + 0.35), 0.022, M["dsteel"])  # display arm
part("commandarm_body", chassis, commandarm, bevel=0.008)

DSP_C = Vector((5.50, CA_Y - 0.30, CA_Z + 0.42))
def display(b):
    rot = Matrix.Rotation(math.radians(-25), 3, "Z") @ Matrix.Rotation(math.radians(-15), 3, "Y")
    b.box(DSP_C, (0.03, 0.28, 0.19), M["black"], rot=rot)
    b.box(DSP_C + rot @ Vector((-0.016, 0, 0)), (0.002, 0.25, 0.165), M["screen"], rot=rot)
o = part("DSP_commandcenter", chassis, display)
tag(o, "control", "ca_display", "CommandCenter touchscreen")
# live screen: a UV-mapped quad facing the operator (the viewer paints a canvas onto it)
_rot = Matrix.Rotation(math.radians(-25), 3, "Z") @ Matrix.Rotation(math.radians(-15), 3, "Y")
_basis = Matrix(((0, 0, -1), (-1, 0, 0), (0, 1, 0)))   # columns: grid X -> -Y, grid Y -> +Z, normal -> -X
_scr = Builder("DSP_screen", o, origin=DSP_C)
_scr.bm.loops.layers.uv.new("UVMap")
_m = Matrix.Translation(_rot @ Vector((-0.018, 0, 0))) @ (_rot @ _basis).to_4x4() @ Matrix.Diagonal((0.125, 0.0825, 1, 1))
_r = bmesh.ops.create_grid(_scr.bm, x_segments=1, y_segments=1, size=1.0, matrix=_m, calc_uvs=True)
_scr._tag(_r["verts"], mat("Display_Screen", srgb("0B1F12"), 0.0, 0.3, emit=srgb("1D4D2A")))
tag(_scr.done(), "control", "ca_display", "CommandCenter touchscreen")

def switch(name, ref, label, pos, m, size=(0.022, 0.016, 0.012), parent=chassis):
    o = part(name, parent, lambda b: b.box(pos, size, m), bevel=0.002)
    tag(o, "control", ref, label)
    return o

DECK_Z = CA_Z + 0.062
# rows of switches on the deck: (mesh, control id, label, dx, dy, material)
deck = [
    ("SW_fold_L_out", "ca_fold_L_out", "Left main fold OUT",   5.13,  0.045, "btn_k"),
    ("SW_fold_L_in",  "ca_fold_L_in",  "Left main fold IN",    5.16,  0.045, "btn_k"),
    ("SW_fold_R_out", "ca_fold_R_out", "Right main fold OUT",  5.13, -0.045, "btn_k"),
    ("SW_fold_R_in",  "ca_fold_R_in",  "Right main fold IN",   5.16, -0.045, "btn_k"),
    ("SW_outer_L_out","ca_outer_L_out","Left outer fold OUT",  5.20,  0.045, "btn_k"),
    ("SW_outer_L_in", "ca_outer_L_in", "Left outer fold IN",   5.23,  0.045, "btn_k"),
    ("SW_outer_R_out","ca_outer_R_out","Right outer fold OUT", 5.20, -0.045, "btn_k"),
    ("SW_outer_R_in", "ca_outer_R_in", "Right outer fold IN",  5.23, -0.045, "btn_k"),
    ("SW_tread_wide", "ca_tread_wider","Tread adjust WIDER",   5.29,  0.040, "btn_y"),
    ("SW_tread_narrow","ca_tread_narrower","Tread adjust NARROWER", 5.29, 0.010, "btn_y"),
    ("SW_ladder",     "ca_ladder",     "Ladder raise/lower",   5.29, -0.030, "btn_y"),
    ("BTN_park",      "ca_park",       "Park",                 5.33, -0.075, "btn_r"),
    ("BTN_transport", "ca_transport",  "Field / Road mode",    5.33,  0.040, "btn_g"),
]
for name, ref, label, x, dy, mm in deck:
    switch(name, ref, label, (x, CA_Y - 0.02 + dy, DECK_Z), M[mm])
o = part("CTL_throttle", chassis, lambda b: b.cyl((5.08, CA_Y - 0.10, DECK_Z + 0.01), 0.022, 0.025, M["btn_k"], segs=20), smooth=True)
tag(o, "control", "ca_throttle", "Engine throttle")

# hydro handle: lever pivots about the lateral axis at the front of the arm
HH_PIV = Vector((5.36, CA_Y, CA_Z + 0.04))
hh_piv = empty("PIV_hydro_handle", HH_PIV, chassis, size=0.08)
def hh(b):
    b.box(HH_PIV + Vector((0, 0, 0.07)), (0.03, 0.03, 0.14), M["black"])
    b.box(HH_PIV + Vector((0.01, 0, 0.17)), (0.07, 0.055, 0.09), M["black"])       # grip
    b.box(HH_PIV + Vector((0.01, 0.0285, 0.17)), (0.06, 0.002, 0.07), M["dsteel"])  # thumb panel (left face)
o = part("CTL_hydro_handle", hh_piv, hh, bevel=0.006)
tag(o, "control", "hh_lever", "Hydro handle")
G = HH_PIV + Vector((0.01, 0.031, 0.17))      # thumb-face centre
hh_buttons = [
    ("BTN_master_spray", "hh_master_spray", "Master spray ON/OFF", ( 0.022,  0.025), "btn_g"),
    ("BTN_resume",       "hh_resume",       "Boom height resume",  (-0.010,  0.025), "btn_y"),
    ("BTN_boom_raise",   "hh_boom_raise",   "Boom raise",          ( 0.008,  0.006), "btn_k"),
    ("BTN_boom_lower",   "hh_boom_lower",   "Boom lower",          ( 0.008, -0.010), "btn_k"),
    ("BTN_tilt_L_up",    "hh_tilt_L_up",    "Left wing tilt up",   (-0.016,  0.006), "btn_k"),
    ("BTN_tilt_L_down",  "hh_tilt_L_down",  "Left wing tilt down", (-0.016, -0.010), "btn_k"),
    ("BTN_tilt_R_up",    "hh_tilt_R_up",    "Right wing tilt up",  ( 0.030,  0.006), "btn_k"),
    ("BTN_tilt_R_down",  "hh_tilt_R_down",  "Right wing tilt down",( 0.030, -0.010), "btn_k"),
    ("BTN_range_up",     "hh_range_up",     "Speed range up",      ( 0.008, -0.028), "btn_y"),
    ("BTN_range_down",   "hh_range_down",   "Speed range down",    (-0.012, -0.028), "btn_y"),
]
for name, ref, label, (dx, dz), mm in hh_buttons:
    switch(name, ref, label, G + Vector((dx, 0, dz)), M[mm], size=(0.012, 0.006, 0.012), parent=hh_piv)

# ---------------------------------------------------------------- ladder (left front, hydraulic)
LAD_PIV = Vector((5.62, 1.16, CAB_FLOOR - 0.06))
lad_piv = empty("PIV_ladder", LAD_PIV, chassis)
def ladder(b):
    bot = LAD_PIV.z - 0.40
    for dx in (-0.22, 0.22):
        b.tube(LAD_PIV + Vector((dx, 0, 0)), Vector((LAD_PIV.x + dx, LAD_PIV.y + 0.05, 0.40)), 0.025, M["dsteel"])
    n = 6
    for i in range(n):
        z = 0.45 + i * (LAD_PIV.z - 0.5) / (n - 1)
        b.box((LAD_PIV.x, LAD_PIV.y + 0.04, z), (0.42, 0.14, 0.03), M["dsteel"])
part("ladder", lad_piv, ladder)
o = part("SW_ladder_ground", chassis, lambda b: b.box((5.30, 1.10, 1.55), (0.06, 0.03, 0.08), M["btn_y"]))
tag(o, "control", "ex_ladder_switch", "Ground-level ladder switch")

# ============================================================================= BOOM
lift_u = empty("PIV_lift_upper", UPPER_PIV, chassis)
lift_l = empty("PIV_lift_lower", LOWER_PIV, chassis)
for piv, nm in ((lift_u, "lift_arm_upper"), (lift_l, "lift_arm_lower")):
    p0 = WORLD[piv.name]
    def arms(b, p0=p0):
        for y in (0.42, -0.42):
            b.box(p0 + Vector((-ARM_L / 2, y, 0)), (ARM_L, 0.10, 0.14), M["green"])
            b.cyl(p0 + Vector((0, y, 0)), 0.07, 0.14, M["dsteel"], axis="Y")
            b.cyl(p0 + Vector((-ARM_L, y, 0)), 0.07, 0.14, M["dsteel"], axis="Y")
    part(nm, piv, arms)

center = empty("PIV_boom_center", (BOOM_X, 0, UPPER_PIV.z), lift_u)
def center_frame(b):
    zt = UPPER_PIV.z + 0.05
    b.box((BOOM_X - 0.10, 0, (BOOM_CHORD_Z + zt) / 2), (0.16, 1.2, zt - BOOM_CHORD_Z), M["green"])    # mast
    b.box((BOOM_X - 0.25, 0, BOOM_CHORD_Z + 0.06), (0.40, 2 * Y_HINGE, 0.14), M["green"])            # bottom beam
    b.box((BOOM_X - 0.25, 0, BOOM_CHORD_Z + 0.75), (0.30, 2 * Y_HINGE - 0.2, 0.12), M["green"])      # top beam
    for y in (Y_HINGE - 0.08, -Y_HINGE + 0.08, 0.6, -0.6):
        b.box((BOOM_X - 0.25, y, BOOM_CHORD_Z + 0.40), (0.30, 0.10, 0.72), M["green"])
    b.tube((BOOM_X - 0.40, Y_HINGE, BOOM_CHORD_Z), (BOOM_X - 0.40, -Y_HINGE, BOOM_CHORD_Z), 0.022, M["steel"])  # spray pipe
    for k in range(int(Y_HINGE / S["nozzle_spacing_m"] - 0.5) + 1):        # centre-section nozzles
        for sg in (1, -1):
            y = sg * (k + 0.5) * S["nozzle_spacing_m"]
            b.box((BOOM_X - 0.40, y, BOOM_CHORD_Z - 0.05), (0.04, 0.035, 0.07), M["black"])
            b.box((BOOM_X - 0.40, y, BOOM_CHORD_Z - 0.09), (0.02, 0.02, 0.025), M["orange"])
part("boom_center_frame", center, center_frame, bevel=0.01)
o = part("SVC_boom_strainer", center, lambda b: b.cyl((BOOM_X + 0.05, -0.30, BOOM_CHORD_Z + 0.30), 0.06, 0.22, M["white"]).cyl((BOOM_X + 0.05, -0.30, BOOM_CHORD_Z + 0.44), 0.07, 0.04, M["orange"]), smooth=True)
tag(o, "service_point", "sp_boom_strainer", "Boom strainer (80 mesh)")
part("boom_center_lights", center, lambda b: b.box((BOOM_X - 0.48, 0.7, BOOM_CHORD_Z + 0.7), (0.03, 0.16, 0.10), M["red"]).box((BOOM_X - 0.48, -0.7, BOOM_CHORD_Z + 0.7), (0.03, 0.16, 0.10), M["red"]))

NOZ_SP = S["nozzle_spacing_m"]

def truss(b, side, y0, y1, h0, h1, depth0=0.50, depth1=0.32):
    """Triangular boom truss from y0 to y1 (absolute |y|) on one side, with ExactApply nozzle bodies."""
    def P(y, dx, dz):
        return Vector((BOOM_X + dx, side * y, BOOM_CHORD_Z + dz))
    L = y1 - y0
    n = max(2, int(L / 0.9))
    def at(t):
        y = y0 + t * L
        return y, h0 + (h1 - h0) * t, depth0 + (depth1 - depth0) * t
    ya, ha, da = at(0); yb, hb, db = at(1)
    m = M["boom"]
    b.tube(P(ya, -0.0, 0), P(yb, -0.0, 0), 0.032, m)            # front bottom chord
    b.tube(P(ya, -da, 0), P(yb, -db, 0), 0.032, m)              # rear bottom chord
    b.tube(P(ya, -da / 2, ha), P(yb, -db / 2, hb), 0.030, m)    # top chord
    for i in range(n + 1):
        y, h, d = at(i / n)
        b.tube(P(y, 0, 0), P(y, -d / 2, h), 0.018, m)
        b.tube(P(y, -d, 0), P(y, -d / 2, h), 0.018, m)
        b.tube(P(y, 0, 0), P(y, -d, 0), 0.016, m)
        if i < n:
            y2, h2, d2 = at((i + 1) / n)
            b.tube(P(y, 0, 0), P(y2, -d2 / 2, h2), 0.014, m)
            b.tube(P(y, -d, 0), P(y2, -d2 / 2, h2), 0.014, m)
    # spray pipe + nozzles behind the rear chord
    b.tube(P(ya, -da - 0.06, -0.02), P(yb, -db - 0.06, -0.02), 0.016, M["steel"])
    k0 = math.ceil(y0 / NOZ_SP - 0.5)
    k = k0
    while (k + 0.5) * NOZ_SP < y1:
        y = (k + 0.5) * NOZ_SP
        t = (y - y0) / L
        d = depth0 + (depth1 - depth0) * t
        b.box(P(y, -d - 0.06, -0.06), (0.04, 0.035, 0.07), M["black"])
        b.box(P(y, -d - 0.06, -0.10), (0.02, 0.02, 0.025), M["orange"])
        k += 1
    return b

for side, sfx in ((1, "L"), (-1, "R")):
    hinge = Vector((BOOM_X, side * Y_HINGE, BOOM_CHORD_Z + 0.35))
    tilt = empty(f"PIV_tilt_{sfx}", hinge, center)
    fold = empty(f"PIV_fold_{sfx}", hinge, tilt)
    part(f"boom_inner_{sfx}", fold, lambda b, s=side: truss(b, s, Y_HINGE, Y_OUTER, 0.85, 0.62).box((BOOM_X - 0.25, s * (Y_HINGE + 0.08), BOOM_CHORD_Z + 0.4), (0.5, 0.16, 0.8), M["green"]))
    outer = empty(f"PIV_outer_{sfx}", (BOOM_X - 0.80, side * Y_OUTER, BOOM_CHORD_Z + 0.3), fold)
    part(f"boom_outer_{sfx}", outer, lambda b, s=side: truss(b, s, Y_OUTER, Y_TIP, 0.60, 0.42, 0.32, 0.26).tube((BOOM_X - 0.80, s * Y_OUTER, BOOM_CHORD_Z), (BOOM_X - 0.80, s * Y_OUTER, BOOM_CHORD_Z + 0.6), 0.03, M["dsteel"]))
    tip = empty(f"PIV_tip_{sfx}", (BOOM_X - 0.13, side * Y_TIP, BOOM_CHORD_Z + 0.2), outer)
    def tip_fn(b, s=side):
        truss(b, s, Y_TIP, Y_END, 0.40, 0.22, 0.26, 0.18)
        b.cyl((BOOM_X - 0.13, s * Y_TIP, BOOM_CHORD_Z + 0.25), 0.05, 0.12, M["yellow"])   # breakaway spring can
        b.box((BOOM_X - 0.08, s * (Y_END - 0.05), BOOM_CHORD_Z - 0.20), (0.06, 0.06, 0.5), M["yellow"])  # end guard
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
bpy.ops.mesh.primitive_plane_add(size=200, location=(0, 0, 0))
g = bpy.context.active_object; g.name = "ENV_ground"; g.data.materials.append(M["ground"])
for c in g.users_collection: c.objects.unlink(g)
ENV.objects.link(g)

# ============================================================================= save + export
bpy.context.view_layer.update()
blend_path = os.path.join(OUT, "john_deere_612r.blend")
bpy.ops.wm.save_as_mainfile(filepath=blend_path)

# glTF export: only the machine collection; custom props -> extras (kind / ref_id / label)
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

# ============================================================================= optional preview renders
if DO_RENDER:
    bpy.ops.wm.open_mainfile(filepath=blend_path)
    scene = bpy.context.scene
    root = bpy.data.objects["SPRAYER_612R"]
    scene.render.engine = "CYCLES"
    scene.cycles.samples = 24
    scene.cycles.use_denoising = True
    scene.render.resolution_x, scene.render.resolution_y = 1280, 720
    world = bpy.data.worlds.new("Sky"); scene.world = world; world.use_nodes = True
    world.node_tree.nodes["Background"].inputs[0].default_value = (0.62, 0.74, 0.88, 1)
    world.node_tree.nodes["Background"].inputs[1].default_value = 0.9
    sun = bpy.data.objects.new("Sun", bpy.data.lights.new("Sun", "SUN"))
    sun.data.energy = 4.0; sun.rotation_euler = (math.radians(50), 0, math.radians(35))
    scene.collection.objects.link(sun)
    cam = bpy.data.objects.new("Cam", bpy.data.cameras.new("Cam")); scene.collection.objects.link(cam)
    scene.camera = cam

    def look(pos, tgt, lens=35):
        cam.location = pos
        cam.rotation_euler = (Vector(tgt) - Vector(pos)).to_track_quat("-Z", "Y").to_euler()
        cam.data.lens = lens

    def pose(**kw):
        for k, v in kw.items():
            root[k] = v
        root.update_tag(); bpy.context.view_layer.update()

    docs = os.path.join(ROOT, "docs")
    os.makedirs(docs, exist_ok=True)
    shots = [
        ("render_field.png",     dict(), ((-16, 26, 11), (-2, 3, 1.0), 30)),
        ("render_transport.png", dict(boom_height=2.2, outer_L=178, outer_R=178, fold_L=90, fold_R=90, ladder=80), ((13, 13, 7), (1.5, 0, 2.0), 35)),
        ("render_service.png",   dict(hood=65, ladder=0, cab_door=75, eductor=85, tread=4.06, steer=25, boom_height=1.6, tilt_L=12), ((7, 9, 4.5), (1.5, 0.5, 1.8), 30)),
    ]
    for fn, p, (pos, tgt, lens) in shots:
        for j in SPEC["joints"]:
            root[j["id"]] = float(j["default"])
        pose(**p)
        look(pos, tgt, lens)
        scene.render.filepath = os.path.join(docs, fn)
        bpy.ops.render.render(write_still=True)
        print("RENDERED", fn)
