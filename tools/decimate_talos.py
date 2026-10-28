"""Decimate the Nereus Talos meshes into one web GLB, part by part (ROS FLU frame kept).

Each CAD part is welded and simplified on its own with a share of the face budget
(at least `floor` faces), so small parts keep their shape instead of collapsing.
Parts are then merged per colour. Normals are left to the viewer (creased there).
usage: decimate_talos.py <merged.glb> <out.glb> <face budget>
"""
import sys
import numpy as np
import trimesh
import fast_simplification

src, out, budget = sys.argv[1], sys.argv[2], int(sys.argv[3])
floor = 120
scene = trimesh.load(src, force='scene')
parts = []
for name, geom in scene.geometry.items():
    for node in scene.graph.geometry_nodes.get(name, []):
        T, _ = scene.graph.get(node)
        g = geom.copy()
        g.apply_transform(T)
        try:
            c = getattr(g.visual.material, 'baseColorFactor', None)
            if c is None:
                c = getattr(g.visual.material, 'main_color', None)
            color = tuple(int(x) for x in np.asarray(c).ravel()[:3]) if c is not None else (150, 150, 150)
        except Exception:
            color = (150, 150, 150)
        m = trimesh.Trimesh(g.vertices, g.faces, process=False)
        m.merge_vertices(merge_tex=True, merge_norm=True, digits_vertex=5)
        parts.append((m, color))
total = sum(len(m.faces) for m, _ in parts)
groups = {}
for m, color in parts:
    keep = max(floor, int(budget * len(m.faces) / total))
    if len(m.faces) > keep * 1.05:
        v, f = fast_simplification.simplify(m.vertices.astype(np.float32), m.faces.astype(np.int32), target_reduction=1 - keep / len(m.faces), agg=5)
        m = trimesh.Trimesh(v, f, process=False)
    groups.setdefault(color, []).append(m)
out_scene = trimesh.Scene()
kept = 0
for color, ms in groups.items():
    m = trimesh.util.concatenate(ms)
    m.remove_unreferenced_vertices()
    kept += len(m.faces)
    m.visual = trimesh.visual.TextureVisuals(material=trimesh.visual.material.PBRMaterial(baseColorFactor=list(color) + [255], metallicFactor=0.2, roughnessFactor=0.55))
    out_scene.add_geometry(m, geom_name=f'part_{len(out_scene.geometry)}')
print('parts', len(parts), 'faces in', total, 'out', kept)
out_scene.export(out)
