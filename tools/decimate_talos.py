"""Weld + quadric-decimate the Nereus Talos meshes into one small GLB per material group (ROS FLU frame kept)."""
import sys, numpy as np, trimesh, fast_simplification
src, out, target = sys.argv[1], sys.argv[2], int(sys.argv[3])
scene = trimesh.load(src, force='scene')
parts = []
for name, geom in scene.geometry.items():
    for node in scene.graph.geometry_nodes.get(name, []):
        T, _ = scene.graph.get(node)
        g = geom.copy(); g.apply_transform(T)
        color = None
        try:
            mat = g.visual.material
            c = getattr(mat, 'baseColorFactor', None)
            if c is None: c = getattr(mat, 'main_color', None)
            color = tuple(int(x) for x in np.asarray(c).ravel()[:3]) if c is not None else None
        except Exception: pass
        parts.append((g, color))
total = sum(len(g.faces) for g, _ in parts)
print('parts', len(parts), 'faces', total)
groups = {}
for g, c in parts:
    groups.setdefault(c, []).append(g)
out_scene = trimesh.Scene()
for c, gs in groups.items():
    m = trimesh.util.concatenate([trimesh.Trimesh(g.vertices, g.faces, process=False) for g in gs])
    m.merge_vertices(merge_tex=True, merge_norm=True, digits_vertex=5)
    keep = max(200, int(target * len(m.faces) / total))
    if len(m.faces) > keep:
        v, f = fast_simplification.simplify(m.vertices.astype(np.float32), m.faces.astype(np.int32), target_reduction=1 - keep / len(m.faces))
        m = trimesh.Trimesh(v, f)
    rgba = list(c or (150, 150, 150)) + [255]
    m.visual = trimesh.visual.TextureVisuals(material=trimesh.visual.material.PBRMaterial(baseColorFactor=rgba, metallicFactor=0.1, roughnessFactor=0.7))
    out_scene.add_geometry(m, geom_name=f'part_{len(out_scene.geometry)}')
    print(c, len(m.faces))
out_scene.export(out)
