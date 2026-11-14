# KclSharp

Wii KCL collision reading and writing, and *Skyward Sword HD*'s `.kcl2` triangle files.

```csharp
KclFile kcl = KclFile.FromBinary(room["kcl/room.kcl"]!.Data);
foreach ((int prism, KclTriangle t) in kcl.Triangles())
    Console.WriteLine($"{t.A} {t.B} {t.C} attribute 0x{kcl.Prisms[prism].Attribute:X4}");

Kcl2File hd = Kcl2File.FromBinary(room["kcl/room.kcl2"]!.Data);   // same triangles, little-endian, one per prism
```

A KCL is a header, positions, normals, prisms (height, position, face and three edge normals, attribute) and an
octree. The octree is kept as read; after moving geometry call `RebuildOctree()`, or build a whole KCL from triangles:

```csharp
KclFile kcl = KclFile.FromTriangles(mesh.Select(t => (new KclTriangle(t.A, t.B, t.C), attribute)));
IReadOnlyList<int>? nearby = kcl.Query(position);   // the prisms the game would test, as it walks the octree
```

The octree is a grid of root cubes, each split into eight until it holds `MaxTrianglesPerCube` triangles or reaches
`MinCubeWidth`; a cube lists every triangle that touches it (an exact triangle/box test). Leaves point 2 bytes before
a zero-terminated list of 1-based prism indices, and identical lists are shared.

`FromTriangles` skips zero-area triangles and merges equal normals, falling back to merging normals within 1e-5 to
1e-3 when a mesh would need more than 65535. On every KCL in the dump: `Query` finds 1,026,087 of 1,026,102 prisms
in Nintendo's own octrees at their centroid; rebuilt octrees find every prism at its centroid and near each corner;
triangles taken through `FromTriangles` come back to within 0.1% (all but 3); and its face and edge normals point the
same way as Nintendo's for all but 3 of 1,026,098 prisms. Rebuilt octrees are about 2.5 times the size of Nintendo's. None has been tried in the game.

A KCL2 is the HD port's pre-computed triangle for each prism of the KCL beside it, in prism order: three
little-endian vec3s, no header. `Kcl2File.FromKcl` builds one from a KCL.

All 267 KCL and their KCL2 in the dump rebuild to the same bytes. Of 1,026,107 prisms, 1,025,855 give the same
triangle as the KCL2 (to 0.1%); 247 differ slightly where the HD version moved a vertex, and 5 are degenerate.

## Build

```bash
dotnet build KclSharp.slnx -c Release
```

## Licence

AGPL-3.0-or-later. See [license.md](license.md).
