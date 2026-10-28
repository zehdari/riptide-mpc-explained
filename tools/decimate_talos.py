"""Decimate the Nereus Talos meshes into one web GLB (ROS FLU frame kept).

Every CAD part is split into its connected pieces. Small pieces (and every piece of a
transparent part, like the acrylic end caps) are kept as they are: simplifying their
few large flat triangles cracks them. Larger pieces are welded and simplified toward
the face budget. Pieces are merged back per material (colour + alpha).
usage: decimate_talos.py <merged.glb> <out.glb> <face budget>
"""
import sys
import numpy as np
import trimesh
import fast_simplification

src, out, budget = sys.argv[1], sys.argv[2], int(sys.argv[3])
KEEP = 600  # pieces at or below this many faces are kept whole
scene = trimesh.load(src, force='scene')
pieces = []  # (mesh, rgba, simplify?)
for name, geom in scene.geometry.items():
    for node in scene.graph.geometry_nodes.get(name, []):
        T, _ = scene.graph.get(node)
        g = geom.copy()
        g.apply_transform(T)
        mat = g.visual.material
        c = getattr(mat, 'baseColorFactor', None)
        rgba = [int(x) for x in np.asarray(c).ravel()[:4]] if c is not None else [150, 150, 150, 255]
        clear = getattr(mat, 'alphaMode', None) == 'BLEND' or rgba[3] < 250
        m = trimesh.Trimesh(g.vertices, g.faces, process=False)
        m.merge_vertices(merge_tex=True, merge_norm=True, digits_vertex=5)
        for p in m.split(only_watertight=False):
            pieces.append((p, tuple(rgba), not clear and len(p.faces) > KEEP))
fixed = sum(len(p.faces) for p, _, s in pieces if not s)
big = sum(len(p.faces) for p, _, s in pieces if s)
ratio = max(0.05, min(1.0, (budget - fixed) / max(big, 1)))
groups = {}
for p, rgba, simplify in pieces:
    if simplify:
        keep = max(KEEP, int(len(p.faces) * ratio))
        if len(p.faces) > keep * 1.05:
            v, f = fast_simplification.simplify(p.vertices.astype(np.float32), p.faces.astype(np.int32), target_reduction=1 - keep / len(p.faces), agg=5)
            p = trimesh.Trimesh(v, f, process=False)
    groups.setdefault(rgba, []).append(p)
out_scene = trimesh.Scene()
total = 0
for rgba, ms in groups.items():
    m = trimesh.util.concatenate(ms)
    m.remove_unreferenced_vertices()
    total += len(m.faces)
    mat = trimesh.visual.material.PBRMaterial(baseColorFactor=list(rgba), metallicFactor=0.2, roughnessFactor=0.55,
                                              alphaMode='BLEND' if rgba[3] < 250 else 'OPAQUE', doubleSided=rgba[3] < 250)
    m.visual = trimesh.visual.TextureVisuals(material=mat)
    out_scene.add_geometry(m, geom_name=f'part_{len(out_scene.geometry)}')
print('pieces', len(pieces), 'kept whole', fixed, 'simplified', big, 'ratio', round(ratio, 3), 'out faces', total)
out_scene.export(out)
