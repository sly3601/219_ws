from pathlib import Path
import trimesh

src_dir = Path("meshes")
dst_dir = Path("meshes_obj")
dst_dir.mkdir(exist_ok=True)

names = [
    "trunk",
    "hip",
    "thigh",
    "thigh_mirror",
    "calf",
]

for name in names:
    src = src_dir / f"{name}.dae"
    dst = dst_dir / f"{name}.obj"

    print(f"Converting {src} -> {dst}")

    obj = trimesh.load(src, force="scene")

    if isinstance(obj, trimesh.Scene):
        meshes = []
        for geom in obj.geometry.values():
            if isinstance(geom, trimesh.Trimesh):
                meshes.append(geom)
        if not meshes:
            raise RuntimeError(f"No mesh geometry found in {src}")
        mesh = trimesh.util.concatenate(meshes)
    elif isinstance(obj, trimesh.Trimesh):
        mesh = obj
    else:
        raise RuntimeError(f"Unsupported object type from {src}: {type(obj)}")

    mesh.export(dst)

print("Done.")