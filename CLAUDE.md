# CLAUDE.md: 612R 3D model ↔ manual RAG integration

This repo has an interactive 3D model of the **John Deere 612R self-propelled sprayer** (Blender-generated, shown in a three.js viewer). This branch, `integration-with-rag`, connects it to Aryan's **612R manual RAG**, which runs on this DGX Spark. When the RAG answers a question about a physical control, part or procedure, whether operator use or repair/maintenance, the viewer shows it: it highlights the part, operates the control, or plays the procedure step by step.

You are probably running on the Spark, cloned next to the RAG. Earlier work was done from a cloud session that couldn't reach the tailnet. This session can.

## The two systems

### 1. This repo (3D model + viewer)
| Path | Purpose |
|---|---|
| `spec/612r_spec.json` | **Single source of truth.** Specs (tagged OFFICIAL / DERIVED / ESTIMATE), joints, controls, service points, interlocks, cameras, procedures. Edit this first. |
| `blender/build_612r.py` | Builds the model from the spec → `build/john_deere_612r.blend` + `.glb`. `pip install bpy`, then `python3 blender/build_612r.py --out build` (about 5 s). Add `--render` for Cycles previews in `docs/` (slow; `SAMPLES=` and `ONLY=` env vars). |
| `blender/render_viewport.py` | Blender-viewport-style renders (Workbench). Headless needs `EGL_PLATFORM=surfaceless LIBGL_ALWAYS_SOFTWARE=1`. |
| `viewer/` | three.js viewer (vendored three r160). `python3 -m http.server 8000` from the repo root, then open `/viewer/`. |
| `tools/export_rag_index.py` | Writes `build/rag_index.jsonl`: one entry per control, service point, procedure, joint and spec, each with its viewer payload. |
| `rag/show_on_sprayer.tool.json` | Tool definition so an LLM can drive the viewer. |

The viewer API, already implemented in `viewer/app.js`:
- `postMessage({type:"612r", cmd:"procedure"|"press"|"highlight"|"set"|"camera", ...})`
- `window.sprayer.{runProcedure, press, highlight, setJoint, camera, getState}`
- URL params `?procedure=&step=&control=&highlight=&set=&camera=`

IDs are stable. Rename them only together with the spec and the Blender script.
- Controls: `hh_*` (hydro handle), `ca_*` (CommandARM), `ex_*` (exterior)
- Service points: `sp_*`; meshes are `SVC_*`
- Procedures: `op_*` (operator), `mx_*` (maintenance)
- Joints: `boom_height`, `fold_L`, `tread`, `hood`, …

### 2. The RAG (not in this repo)
- Code: `~/jd_rag_612R` (or `/home/dbw_ltts/jd_rag_612R`). Started by `start.sh`; settings and access key in `.sprayer.env`.
- FastAPI on `:8080`. Pages: `/chat`, `/docs`, `/health`, `/wiki-review`, `/board`. OpenAI-compatible API at `/v1`, model `sprayer-manual`.
- Qdrant vector store (container `sprayer-qdrant`, data in `data/qdrant`), about 1,806 chunks. The LLM is Qwen via Ollama.
- Tailscale address `100.115.38.24:8080`.

## Rules
- **Don't break the RAG.** Treat `~/jd_rag_612R` as read-only unless Aryan agrees to a change. Build the integration as a separate service or in this repo. The RAG may be shared and running (another vLLM shares the GPU).
- **Never print or commit the access key** from `.sprayer.env`. Read it from the environment at runtime.
- **Manual-derived facts must cite their source** (manual ID + page) in the spec. When a fact is confirmed from the manual, set `"verified": true` and `"src": "OM <id> p.<n>"`. Don't invent button positions or procedures. If the manual doesn't say, keep the ESTIMATE tag.
- Copyrighted manual text: store short excerpts and page references in the repo, not whole manual sections.
- Commit to `integration-with-rag`. Run `python3 blender/build_612r.py --out build` and `python3 tools/export_rag_index.py` after spec changes, and commit the regenerated `build/` files.

## Plan (do in order; check items off here as you go)

1. **Probe the RAG** (read-only)
   - [ ] `/health`, `/openapi.json`: list endpoints and their schemas.
   - [ ] Run "How do I fold the boom for transport?" through `/v1/chat/completions` and through the native endpoint. Do responses include source chunks with page numbers, section titles or chunk IDs? If not, can they be exposed without changing answer behaviour?
   - [ ] Qdrant collection(s) and payload fields.
   - [ ] Which manuals are indexed (OM / TM numbers), where the PDFs are, page counts.
2. **Extract the manual's structure**
   - [ ] Table of contents with page numbers → `rag/manual_toc.json`.
   - [ ] The list of every physical control and component the manual names, with page refs → `rag/manual_entities.json`.
3. **Correct the model from the manual**
   - [ ] CommandARM / hydro handle: real button set, positions and functions. Update the `controls` in the spec and the deck/button layout in `build_612r.py` (CommandARM section). These are currently placeholders (`verified: false`).
   - [ ] Service-point locations (filters, strainers, reservoir, batteries, fuel/DEF) and the real interlocks.
   - [ ] Add parts the manual's maintenance/repair procedures need (belts, fuse panel, access covers, grease points, …) as new `SVC_*` meshes and `sp_*` entries.
4. **Procedures from the manual**
   - [ ] Convert operator and maintenance procedures into `procedures` entries. Each step gets `text` (short paraphrase), `highlight`, `set` and `camera`, plus `src` (page) and any `warning` (safety notice).
5. **Linking table** (the core of the integration)
   - [ ] For every chunk in Qdrant, compute the related viewer IDs → `rag/chunk_links.jsonl` as `{chunk_id, page, ids:[...], confidence}`. Use an LLM pass over the chunks with the ID catalog from `build/rag_index.jsonl` and keep only confident links. Spot-check 30 by hand.
   - [ ] Decide with Aryan whether links are also written into the Qdrant payload. Default: keep them in a sidecar file, so the RAG stays untouched.
6. **Middleware + UI**
   - [ ] New service in this repo, `integration/`, for example FastAPI on `:8090`. It takes a question → calls the RAG → collects the retrieved chunks → looks up `chunk_links` → optionally lets the LLM choose with the `show_on_sprayer` tool → returns `{answer, sources, viewer_actions}`.
   - [ ] Chat page: answer on the left, `viewer/` in an iframe on the right, `viewer_actions` sent by postMessage. Sources link to the manual page.
7. **Evaluation**
   - [ ] `eval/questions.jsonl` with 30–50 real questions (operator and mechanic), each with the expected viewer IDs. Script reports hit rate. The target is that the right part lights up for at least 90%.

## Status / handoff notes
- 2026-11-02: branch created from `claude/john-deere-612r-sprayer` (model reworked to match dealer photos of a production 612R: front engine, cab over the front axle, green folded boom). The cloud session had queued the step-1 probe in the old "Context window walkthrough" session. Do step 1 here instead and record results below.

- 2026-11-02: CommandARM rebuilt from dealer photos (shape, colours, key layout). Key→function mapping still unverified. Fix it from the manual in plan step 3.

### Probe results
_(fill in)_
