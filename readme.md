# KclSharp

Wii KCL collision reading and writing, and *Skyward Sword HD*'s `.kcl2` triangle files.

```csharp
KclFile kcl = KclFile.FromBinary(room["kcl/room.kcl"]!.Data);
foreach ((int prism, KclTriangle t) in kcl.Triangles())
    Console.WriteLine($"{t.A} {t.B} {t.C} attribute 0x{kcl.Prisms[prism].Attribute:X4}");

Kcl2File hd = Kcl2File.FromBinary(room["kcl/room.kcl2"]!.Data);   // same triangles, little-endian, one per prism
```

A KCL is a header, positions, normals, prisms (height, position, face and three edge normals, attribute) and an
octree. The octree is kept as bytes and not rebuilt, so moving geometry leaves it stale; changing attributes is safe.

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
