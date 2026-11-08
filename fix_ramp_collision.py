#!/usr/bin/env python3
"""Tambahkan normal per-vertex ke file COLLADA (.dae) yang belum punya.

Pemakaian (dari root workspace):
    python3 fix_ramp_collision.py [src/sim_bringup/worlds/dummy_arena.sdf]

Skrip ini membuat salinan mesh khusus collision (*_collision.dae) yang diberi
normal, lalu mengarahkan HANYA blok <collision> ke salinan itu. Mesh visual
asli tidak disentuh.

Pemakaian add_normals lama:
    python3 add_normals.py ramp.dae [ramp_guardrail.dae ...]

File asli disimpan sebagai <nama>.dae.orig (sekali saja, tidak ditimpa).
Gazebo (gz-physics / dartsim) menolak submesh yang jumlah normalnya tidak
sama dengan jumlah vertex-nya, lalu crash. Skrip ini menghitung normal dari
geometri yang sudah ada, tanpa mengubah bentuk, ukuran, atau posisi mesh.
"""
import math
import shutil
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


def process(path: Path) -> bool:
    tree = ET.parse(path)
    root = tree.getroot()
    ns = root.tag[1:].split("}")[0] if root.tag.startswith("{") else ""
    if ns:
        ET.register_namespace("", ns)

    def q(tag):
        return f"{{{ns}}}{tag}" if ns else tag

    changed = False
    for mesh in root.iter(q("mesh")):
        sources = {s.get("id"): s for s in mesh.findall(q("source"))}
        vertices = {v.get("id"): v for v in mesh.findall(q("vertices"))}

        for prim in list(mesh):
            kind = prim.tag.split("}")[-1]
            if kind not in ("triangles", "polylist"):
                continue
            inputs = prim.findall(q("input"))
            if any(i.get("semantic") == "NORMAL" for i in inputs):
                print(f"  {path.name}: sudah punya NORMAL, dilewati")
                continue
            vin = next((i for i in inputs if i.get("semantic") == "VERTEX"), None)
            if vin is None:
                print(f"  {path.name}: tidak ada input VERTEX, dilewati")
                continue

            vert = vertices[vin.get("source").lstrip("#")]
            pos_in = next(i for i in vert.findall(q("input")) if i.get("semantic") == "POSITION")
            pos_id = pos_in.get("source").lstrip("#")
            pos_src = sources[pos_id]
            flat = [float(v) for v in pos_src.find(q("float_array")).text.split()]
            acc = pos_src.find(q("technique_common")).find(q("accessor"))
            stride_pos = int(acc.get("stride", "3"))
            pts = [tuple(flat[i:i + 3]) for i in range(0, len(flat), stride_pos)]

            voff = int(vin.get("offset", "0"))
            stride = max(int(i.get("offset", "0")) for i in inputs) + 1
            p = [int(v) for v in prim.find(q("p")).text.split()]
            idx = [p[k] for k in range(voff, len(p), stride)]

            if kind == "triangles":
                faces = [idx[i:i + 3] for i in range(0, len(idx), 3)]
            else:
                faces, pos = [], 0
                for n in (int(v) for v in prim.find(q("vcount")).text.split()):
                    poly = idx[pos:pos + n]
                    pos += n
                    faces += [[poly[0], poly[k], poly[k + 1]] for k in range(1, n - 1)]

            acc_n = [[0.0, 0.0, 0.0] for _ in pts]
            for a, b, c in faces:
                u = [pts[b][i] - pts[a][i] for i in range(3)]
                v = [pts[c][i] - pts[a][i] for i in range(3)]
                n = [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]]
                for vi in (a, b, c):
                    for i in range(3):
                        acc_n[vi][i] += n[i]
            normals = []
            for n in acc_n:
                m = math.sqrt(sum(x * x for x in n))
                normals.append(tuple(x / m for x in n) if m > 1e-12 else (0.0, 0.0, 1.0))

            nid = f"{pos_id}-normals-auto"
            src = ET.Element(q("source"), id=nid)
            arr = ET.SubElement(src, q("float_array"), id=f"{nid}-array", count=str(3 * len(normals)))
            arr.text = " ".join(f"{x:.6f}" for n in normals for x in n)
            tc = ET.SubElement(src, q("technique_common"))
            a2 = ET.SubElement(tc, q("accessor"), source=f"#{nid}-array", count=str(len(normals)), stride="3")
            for name in "XYZ":
                ET.SubElement(a2, q("param"), name=name, type="float")

            mesh.insert(list(mesh).index(pos_src) + 1, src)
            prim.insert(list(prim).index(vin) + 1,
                        ET.Element(q("input"), semantic="NORMAL", source=f"#{nid}", offset=str(voff)))
            print(f"  {path.name}: {len(pts)} vertex, {len(normals)} normal ditambahkan ({kind})")
            changed = True

    if changed:
        backup = path.with_suffix(path.suffix + ".orig")
        if not backup.exists():
            shutil.copy2(path, backup)
        tree.write(path, encoding="utf-8", xml_declaration=True)
    return changed


if __name__ == "__main__":
    import re
    world = Path(sys.argv[1] if len(sys.argv) > 1 else "src/sim_bringup/worlds/dummy_arena.sdf")
    meshes = world.parent / "meshes"
    text = world.read_text()
    stems = set()

    def fix_collision(m):
        blk = m.group(0)

        def swap(mm):
            stems.add(mm.group(1))
            return f"meshes/{mm.group(1)}_collision.dae"

        return re.sub(r"meshes/([A-Za-z0-9_\-]+?)(?:_collision)?\.(?:dae|stl)", swap, blk)

    new_text = re.sub(r"<collision\b.*?</collision>", fix_collision, text, flags=re.S)
    if not stems:
        sys.exit("Tidak ada <collision> yang memakai mesh. Tidak ada yang diubah.")

    for stem in sorted(stems):
        orig = meshes / f"{stem}.dae"
        dest = meshes / f"{stem}_collision.dae"
        if not orig.exists():
            sys.exit(f"File {orig} tidak ditemukan.")
        shutil.copy2(orig, dest)
        print(dest.name)
        process(dest)

    bak = world.with_suffix(".sdf.before_collision_fix")
    if not bak.exists():
        shutil.copy2(world, bak)
    world.write_text(new_text)
    print(f"{world} diperbarui; collision memakai: " + ", ".join(f"{s}_collision.dae" for s in sorted(stems)))
    print(f"Cadangan SDF: {bak}")
