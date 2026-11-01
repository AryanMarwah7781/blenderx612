"""
Renders the 612R the way Blender's 3D viewport shows it (Workbench engine, solid and
wireframe shading, grid floor, axis lines), plus a Blender-style quad view.

    python3 blender/render_viewport.py            # writes docs/viewport_*.png
    (Headless Linux needs libEGL; for software GL use EGL_PLATFORM=surfaceless LIBGL_ALWAYS_SOFTWARE=1)
"""
import math
import os
import sys

import bpy
from mathutils import Vector

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DOCS = os.path.join(ROOT, "docs")
TMP = os.path.join(DOCS, "_vp_tmp")
os.makedirs(TMP, exist_ok=True)

bpy.ops.wm.open_mainfile(filepath=os.path.join(ROOT, "build", "john_deere_612r.blend"))
scene = bpy.context.scene
root = bpy.data.objects["SPRAYER_612R"]
for o in list(bpy.data.collections["Environment"].objects):
    bpy.data.objects.remove(o)

# --- Blender viewport look -------------------------------------------------------------
scene.render.engine = "BLENDER_WORKBENCH"
sh = scene.display.shading
sh.light = "STUDIO"
sh.color_type = "MATERIAL"
sh.show_cavity = True
sh.cavity_type = "BOTH"
sh.show_object_outline = True
sh.object_outline_color = (0.0, 0.0, 0.0)
sh.show_specular_highlight = True
sh.background_type = "VIEWPORT"
sh.background_color = (0.0395, 0.0395, 0.0395)     # Blender default theme (#393939, linear)
scene.display.render_aa = "16"
scene.render.film_transparent = False
scene.view_settings.view_transform = "Standard"

# grid floor + red X / green Y axes, like the viewport overlay
def flat_mat(name, rgb):
    m = bpy.data.materials.new(name)
    m.diffuse_color = (*rgb, 1)
    return m

import bmesh
from mathutils import Matrix
bm = bmesh.new()
for i in range(-40, 41):          # 1 m grid lines (Workbench does not draw overlays, so they are thin strips)
    w = 0.03 if i % 10 == 0 else 0.015
    bmesh.ops.create_cube(bm, size=1, matrix=Matrix.Translation((i, 0, 0)) @ Matrix.Diagonal((w, 80, 0.002, 1)))
    bmesh.ops.create_cube(bm, size=1, matrix=Matrix.Translation((0, i, 0)) @ Matrix.Diagonal((80, w, 0.002, 1)))
me = bpy.data.meshes.new("Grid"); bm.to_mesh(me); bm.free()
grid = bpy.data.objects.new("Grid", me); scene.collection.objects.link(grid)
me.materials.append(flat_mat("GridLine", (0.10, 0.10, 0.10)))
for axis, rgb, rot in (("X", (0.85, 0.2, 0.25), (0, math.pi / 2, 0)), ("Y", (0.45, 0.7, 0.12), (math.pi / 2, 0, 0))):
    bpy.ops.mesh.primitive_cylinder_add(radius=0.025, depth=80, rotation=rot, location=(0, 0, 0.002))
    ax = bpy.context.active_object
    ax.data.materials.append(flat_mat("Axis" + axis, rgb))

cam = bpy.data.objects.new("ViewportCam", bpy.data.cameras.new("ViewportCam"))
scene.collection.objects.link(cam)
scene.camera = cam

def pose(**kw):
    defaults = {"tread": 3.05, "boom_height": 0.76}
    for k, v in {**{k: 0.0 for k in ("steer", "suspension", "tilt_L", "tilt_R", "fold_L", "fold_R", "outer_L", "outer_R",
                                    "tip_L", "tip_R", "ladder", "cab_door", "hood", "eductor", "hydro_handle", "steering_wheel")},
                 **defaults, **kw}.items():
        root[k] = float(v)
    root.update_tag()
    bpy.context.view_layer.update()

def shot(path, pos, tgt, ortho=None, lens=35, w=1280, h=800, shading="SOLID", xray=False):
    scene.render.resolution_x, scene.render.resolution_y = w, h
    cam.location = pos
    cam.rotation_euler = (Vector(tgt) - Vector(pos)).to_track_quat("-Z", "Y").to_euler()
    cam.data.type = "ORTHO" if ortho else "PERSP"
    if ortho:
        cam.data.ortho_scale = ortho
    cam.data.lens = lens
    cam.data.clip_end = 500
    sh.type = shading
    sh.show_xray = xray
    sh.show_object_outline = shading == "SOLID"
    scene.render.filepath = path
    bpy.ops.render.render(write_still=True)
    print("RENDERED", os.path.basename(path))

# quad view: field pose
pose()
shot(f"{TMP}/q_persp.png", (16, 21, 10), (0, 2.5, 1.2), lens=28, w=960, h=600)
shot(f"{TMP}/q_front.png", (1.5, -60, 2.0), (1.5, 0, 2.0), ortho=12.5, w=960, h=600)   # Blender "Front" = looking along +Y
shot(f"{TMP}/q_right.png", (60, 0, 2.0), (0, 0, 2.0), ortho=40, w=960, h=600)          # "Right" = looking along -X
shot(f"{TMP}/q_top.png", (0.0, 0, 60), (0.0, 0, 0), ortho=40, w=960, h=600)
# singles
shot(f"{TMP}/solid_hero.png", (13.5, 11.5, 6.5), (2.0, 0.5, 1.9), lens=32)
pose(boom_height=2.2, fold_L=90, fold_R=90, outer_L=178, outer_R=178, ladder=80)
shot(f"{TMP}/solid_transport.png", (12.0, -11.0, 6.0), (1.8, 0, 2.0), lens=32)
# Workbench cannot render "Wireframe" shading, so build the wire look from the real mesh edges
mach = [o for o in bpy.data.collections["612R"].objects if o.type == "MESH"]
for o in mach:
    m = o.modifiers.new("Wire", "WIREFRAME"); m.thickness = 0.008; m.use_replace = True; m.use_even_offset = True
mats = {m for o in mach for m in o.data.materials if m}
saved = {m: tuple(m.diffuse_color) for m in mats}
for m in mats:
    m.diffuse_color = (0.85, 0.85, 0.85, 1)
shot(f"{TMP}/wire_transport.png", (12.0, -11.0, 6.0), (1.8, 0, 2.0), lens=32)
for o in mach:
    o.modifiers.remove(o.modifiers["Wire"])
for m, c in saved.items():
    m.diffuse_color = c
pose(hood=65, cab_door=75, eductor=85, tread=4.06, steer=25, boom_height=1.6, tilt_L=12, ladder=0)
shot(f"{TMP}/xray_service.png", (8.0, 9.5, 4.8), (1.8, 0.4, 1.8), lens=30, xray=True)

# --- compose with Blender-like viewport labels ------------------------------------------
from PIL import Image, ImageDraw, ImageFont
FONT = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 17)
SMALL = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 14)

def label(img, title, sub="(1) Collection | SPRAYER_612R"):
    d = ImageDraw.Draw(img)
    d.text((14, 10), title, fill=(230, 230, 230), font=FONT)
    d.text((14, 32), sub, fill=(200, 200, 200), font=SMALL)
    return img

quad = Image.new("RGB", (1924, 1204), (22, 22, 22))
for (fn, title), (x, y) in zip((("q_persp", "User Perspective"), ("q_top", "Top Orthographic"),
                               ("q_front", "Front Orthographic"), ("q_right", "Right Orthographic")),
                              ((0, 0), (964, 0), (0, 604), (964, 604))):
    quad.paste(label(Image.open(f"{TMP}/{fn}.png").convert("RGB"), title), (x, y))
quad.save(os.path.join(DOCS, "viewport_quad.png"))
for fn, title in (("solid_hero", "User Perspective  ·  Solid"), ("solid_transport", "User Perspective  ·  Solid  ·  transport pose"),
                  ("wire_transport", "User Perspective  ·  Wireframe"), ("xray_service", "User Perspective  ·  X-Ray  ·  service pose")):
    label(Image.open(f"{TMP}/{fn}.png").convert("RGB"), title).save(os.path.join(DOCS, f"viewport_{fn}.png"))
import shutil
shutil.rmtree(TMP)
print("DONE")
