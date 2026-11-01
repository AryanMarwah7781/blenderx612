"""
Builds a RAG-ready index from spec/612r_spec.json.

Output: build/rag_index.jsonl, one JSON object per line:
  {"id", "type": control|service_point|procedure|joint|spec, "text": <embeddable text>,
   "viewer": {<action payload that the viewer accepts via postMessage>}, "verified": bool}

Embed "text" next to your operator's-manual chunks. When the retriever returns a manual
chunk, also retrieve the nearest viewer entries (or let the LLM call the show_on_sprayer tool)
and send the "viewer" payload to the embedded viewer iframe:
    iframe.contentWindow.postMessage({type: "612r", ...viewer}, "*")

Usage: python3 tools/export_rag_index.py
"""
import json
import os

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
spec = json.load(open(os.path.join(ROOT, "spec", "612r_spec.json")))
out = []

for c in spec["controls"]:
    j = f" It operates the {c['joint'].replace('_', ' ')} function." if c.get("joint") else ""
    out.append({
        "id": c["id"], "type": "control",
        "text": f"612R {c['location'].replace('_', ' ')} control: {c['label']}. Kind: {c['kind']}.{j}",
        "viewer": {"cmd": "press" if c["kind"] in ("momentary", "toggle", "toggle_joint", "step") else "highlight",
                   "control": c["id"], "meshes": [c["mesh"]],
                   "camera": "left_side" if c["location"] == "exterior" else "commandarm"},
        "verified": c.get("verified", False),
    })

for s in spec["service_points"]:
    out.append({
        "id": s["id"], "type": "service_point",
        "text": f"612R service point for the {s['audience']}: {s['label']}." + (" Open the engine hood for access." if s.get("requires") else ""),
        "viewer": {"cmd": "highlight", "meshes": [s["mesh"]]},
        "verified": s["src"] != "ESTIMATE",
    })

for p in spec["procedures"]:
    steps = " ".join(f"{i + 1}. {st['text']}" for i, st in enumerate(p["steps"]))
    out.append({
        "id": p["id"], "type": "procedure",
        "text": f"612R procedure ({p['audience']}): {p['title']}. {steps}",
        "viewer": {"cmd": "procedure", "id": p["id"], "step": 0},
        "verified": False,
    })

for j in spec["joints"]:
    out.append({
        "id": "joint_" + j["id"], "type": "joint",
        "text": f"612R moving part: {j['label']}. Range {j['min']} to {j['max']} {j['unit']}. {j.get('note', '')}".strip(),
        "viewer": {"cmd": "set", "joints": {j["id"]: j["max"]}},
        "verified": j["src"] == "OFFICIAL",
    })

for k, v in spec["specs"].items():
    out.append({
        "id": "spec_" + k, "type": "spec",
        "text": f"612R specification {k.replace('_', ' ')}: {v['value']}" + (f" ({v['note']})" if v.get("note") else ""),
        "viewer": None, "verified": v["src"] in ("OFFICIAL", "DERIVED"),
    })

os.makedirs(os.path.join(ROOT, "build"), exist_ok=True)
path = os.path.join(ROOT, "build", "rag_index.jsonl")
with open(path, "w") as f:
    for row in out:
        f.write(json.dumps(row) + "\n")
print(f"wrote {len(out)} entries -> {os.path.relpath(path, ROOT)}")
