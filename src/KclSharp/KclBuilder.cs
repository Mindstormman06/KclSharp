using System.Buffers.Binary;
using System.Numerics;

namespace KclSharp;

public sealed record KclBuildOptions
{
    /// <summary>A cube with more triangles than this is split, until <see cref="MinCubeWidth"/>.</summary>
    public int MaxTrianglesPerCube { get; init; } = 8;

    /// <summary>The smallest cube width, a power of two.</summary>
    public int MinCubeWidth { get; init; } = 4;

    /// <summary>The most root cubes along the longest axis (a power of two).</summary>
    public int MaxRootCubesPerAxis { get; init; } = 8;

    /// <summary>How far outside a cube a triangle may be and still be listed in it.</summary>
    public float Margin { get; init; } = 0.5f;
}

public sealed partial class KclFile
{
    /// <summary>Builds a KCL from triangles, with prisms, positions, normals and a fresh octree.</summary>
    public static KclFile FromTriangles(IEnumerable<(KclTriangle Triangle, ushort Attribute)> triangles, KclBuildOptions? options = null)
    {
        List<(KclTriangle, ushort)> list = [.. triangles];
        // Merge equal normals; if that leaves too many, merge ones within a small step of each other.
        foreach (float step in (float[])[0, 1e-5f, 1e-4f, 1e-3f])
        {
            try { return Build(list, step, options); }
            catch (KclLimitException) when (step < 1e-3f) { }
        }
        throw new InvalidOperationException("Unreachable.");
    }

    private sealed class KclLimitException() : InvalidOperationException("A KCL holds at most 65535 positions and 65535 normals.");

    private static KclFile Build(List<(KclTriangle Triangle, ushort Attribute)> triangles, float normalStep, KclBuildOptions? options)
    {
        KclFile kcl = new();
        Dictionary<Vector3, ushort> positions = [], normals = [];
        Vector3 Key(Vector3 v) => normalStep == 0 ? v : new Vector3(MathF.Round(v.X / normalStep), MathF.Round(v.Y / normalStep), MathF.Round(v.Z / normalStep));
        ushort Index(Dictionary<Vector3, ushort> map, List<Vector3> list, Vector3 v, Vector3 key)
        {
            if (map.TryGetValue(key, out ushort i)) return i;
            if (list.Count >= 0xFFFF) throw new KclLimitException();
            i = (ushort)list.Count;
            list.Add(v);
            map[key] = i;
            return i;
        }
        ushort Position(Vector3 v) => Index(positions, kcl.Positions, v, v);
        ushort Normal(Vector3 v) => Index(normals, kcl.Normals, v, Key(v));

        foreach ((KclTriangle t, ushort attribute) in triangles)
        {
            Vector3 cross = Vector3.Cross(t.B - t.A, t.C - t.A);
            if (cross.LengthSquared() == 0) continue;
            Vector3 face = Vector3.Normalize(cross);
            Vector3 edgeA = Vector3.Normalize(Vector3.Cross(t.A - t.C, face));
            Vector3 edgeB = Vector3.Normalize(Vector3.Cross(t.B - t.A, face));
            Vector3 edgeC = Vector3.Normalize(Vector3.Cross(t.C - t.B, face));
            kcl.Prisms.Add(new KclPrism
            {
                Height = Vector3.Dot(t.B - t.A, edgeC),
                Position = Position(t.A),
                FaceNormal = Normal(face),
                EdgeNormalA = Normal(edgeA),
                EdgeNormalB = Normal(edgeB),
                EdgeNormalC = Normal(edgeC),
                Attribute = attribute,
            });
        }
        while (kcl.Normals.Count < 2) kcl.Normals.Add(Vector3.UnitY);
        kcl.RebuildOctree(options);
        return kcl;
    }

    /// <summary>Replaces the octree (and the minimum, masks and shifts) with one built from the current prisms.</summary>
    public void RebuildOctree(KclBuildOptions? options = null)
    {
        options ??= new KclBuildOptions();
        List<(int Prism, KclTriangle Triangle)> tris = [.. Triangles()];

        Vector3 lo = new(float.MaxValue), hi = new(float.MinValue);
        foreach ((_, KclTriangle t) in tris)
            foreach (Vector3 v in (Vector3[])[t.A, t.B, t.C]) { lo = Vector3.Min(lo, v); hi = Vector3.Max(hi, v); }
        if (tris.Count == 0) { lo = Vector3.Zero; hi = Vector3.One; }
        lo = new Vector3(MathF.Floor(lo.X - options.Margin), MathF.Floor(lo.Y - options.Margin), MathF.Floor(lo.Z - options.Margin));
        hi += new Vector3(options.Margin);

        uint rangeX = Pow2((uint)MathF.Ceiling(hi.X - lo.X) + 1), rangeY = Pow2((uint)MathF.Ceiling(hi.Y - lo.Y) + 1), rangeZ = Pow2((uint)MathF.Ceiling(hi.Z - lo.Z) + 1);
        uint root = Math.Max(Math.Max(rangeX, Math.Max(rangeY, rangeZ)) / Pow2((uint)options.MaxRootCubesPerAxis), Pow2((uint)options.MinCubeWidth));
        rangeX = Math.Max(rangeX, root);
        rangeY = Math.Max(rangeY, root);
        rangeZ = Math.Max(rangeZ, root);

        int nx = (int)(rangeX / root), ny = (int)(rangeY / root), nz = (int)(rangeZ / root);
        Minimum = lo;
        MaskX = ~(rangeX - 1);
        MaskY = ~(rangeY - 1);
        MaskZ = ~(rangeZ - 1);
        Shift = (uint)BitOperations.Log2(root);
        ShiftY = (uint)BitOperations.Log2((uint)nx);
        ShiftZ = (uint)BitOperations.Log2((uint)(nx * ny));

        Octree = OctreeWriter.Write(tris, lo, (int)root, nx, ny, nz, options);
    }

    private static uint Pow2(uint value) => value <= 1 ? 1 : BitOperations.RoundUpToPowerOf2(value);

    /// <summary>The prisms (0-based) the game would test for a point, following the octree as the game does.</summary>
    public IReadOnlyList<int>? Query(Vector3 point)
    {
        int x = (int)(point.X - Minimum.X), y = (int)(point.Y - Minimum.Y), z = (int)(point.Z - Minimum.Z);
        if (point.X < Minimum.X || point.Y < Minimum.Y || point.Z < Minimum.Z) return null;
        if (((uint)x & MaskX) != 0 || ((uint)y & MaskY) != 0 || ((uint)z & MaskZ) != 0) return null;

        ReadOnlySpan<byte> o = Octree;
        int shift = (int)Shift, baseAt = 0;
        int index = (z >> shift) << (int)ShiftZ | (y >> shift) << (int)ShiftY | x >> shift;
        uint node = BinaryPrimitives.ReadUInt32BigEndian(o[(index * 4)..]);
        while ((node & 0x80000000) == 0)
        {
            baseAt += (int)node;
            shift--;
            index = (z >> shift & 1) << 2 | (y >> shift & 1) << 1 | (x >> shift & 1);
            node = BinaryPrimitives.ReadUInt32BigEndian(o[(baseAt + index * 4)..]);
        }

        List<int> prisms = [];
        for (int at = baseAt + (int)(node & 0x7FFFFFFF) + 2; ; at += 2)
        {
            ushort v = BinaryPrimitives.ReadUInt16BigEndian(o[at..]);
            if (v == 0) break;
            prisms.Add(v - 1);
        }
        return prisms;
    }
}

internal static class OctreeWriter
{
    private sealed class Node
    {
        public Node[]? Children;
        public int[] Prisms = [];
    }

    public static byte[] Write(List<(int Prism, KclTriangle Triangle)> tris, Vector3 lo, int root, int nx, int ny, int nz, KclBuildOptions options)
    {
        Node[] roots = new Node[nx * ny * nz];
        int[] all = [.. Enumerable.Range(0, tris.Count)];
        for (int z = 0; z < nz; z++)
            for (int y = 0; y < ny; y++)
                for (int x = 0; x < nx; x++)
                {
                    Vector3 origin = lo + new Vector3(x, y, z) * root;
                    roots[x | y * nx | z * nx * ny] = Build(tris, all, origin, root, options);
                }

        // Lay out the node arrays: roots first, then each branch's 8 children, breadth first.
        List<Node[]> arrays = [roots];
        for (int a = 0; a < arrays.Count; a++)
            foreach (Node n in arrays[a])
                if (n.Children is not null) arrays.Add(n.Children);

        int at = 0;
        Dictionary<Node[], int> arrayStart = new(ReferenceEqualityComparer.Instance);
        foreach (Node[] array in arrays)
        {
            arrayStart[array] = at;
            at += array.Length * 4;
        }

        // Shared lists: a leading 0, then each list and its 0 terminator. A leaf points 2 bytes before its list.
        int listsStart = at;
        using MemoryStream lists = new();
        WriteU16(lists, 0);
        Dictionary<string, int> listAt = new() { [""] = listsStart - 2 };
        foreach (Node[] array in arrays)
            foreach (Node n in array)
            {
                if (n.Children is not null) continue;
                string key = string.Join(',', n.Prisms);
                if (listAt.ContainsKey(key)) continue;
                listAt[key] = listsStart + (int)lists.Length - 2;
                foreach (int p in n.Prisms) WriteU16(lists, (ushort)(tris[p].Prism + 1));
                WriteU16(lists, 0);
            }
        while (lists.Length % 4 != 0) lists.WriteByte(0);

        byte[] output = new byte[listsStart + lists.Length];
        foreach (Node[] array in arrays)
        {
            int start = arrayStart[array];
            for (int i = 0; i < array.Length; i++)
            {
                Node n = array[i];
                uint value = n.Children is not null
                    ? (uint)(arrayStart[n.Children] - start)
                    : 0x80000000u | (uint)(listAt[string.Join(',', n.Prisms)] - start);
                BinaryPrimitives.WriteUInt32BigEndian(output.AsSpan(start + i * 4), value);
            }
        }
        lists.ToArray().CopyTo(output, listsStart);
        return output;
    }

    private static Node Build(List<(int Prism, KclTriangle Triangle)> tris, int[] candidates, Vector3 origin, float width, KclBuildOptions options)
    {
        Vector3 center = origin + new Vector3(width / 2), half = new(width / 2 + options.Margin);
        int[] inside = [.. candidates.Where(i => TriangleBox.Overlaps(tris[i].Triangle, center, half))];

        if (inside.Length <= options.MaxTrianglesPerCube || width / 2 < options.MinCubeWidth)
            return new Node { Prisms = [.. inside.OrderBy(i => tris[i].Prism)] };

        float h = width / 2;
        Node[] children = new Node[8];
        for (int k = 0; k < 8; k++)
            children[k] = Build(tris, inside, origin + new Vector3(k & 1, k >> 1 & 1, k >> 2 & 1) * h, h, options);
        return new Node { Children = children };
    }

    private static void WriteU16(Stream s, ushort v) { Span<byte> b = stackalloc byte[2]; BinaryPrimitives.WriteUInt16BigEndian(b, v); s.Write(b); }
}

/// <summary>Triangle against axis-aligned box, by the separating axis theorem (Akenine-Möller).</summary>
internal static class TriangleBox
{
    public static bool Overlaps(KclTriangle t, Vector3 center, Vector3 half)
    {
        Vector3 v0 = t.A - center, v1 = t.B - center, v2 = t.C - center;
        Vector3 e0 = v1 - v0, e1 = v2 - v1, e2 = v0 - v2;

        foreach (Vector3 edge in (Vector3[])[e0, e1, e2])
            foreach (Vector3 axis in (Vector3[])[Vector3.UnitX, Vector3.UnitY, Vector3.UnitZ])
            {
                Vector3 a = Vector3.Cross(axis, edge);
                if (a.LengthSquared() < 1e-12f) continue;
                float p0 = Vector3.Dot(v0, a), p1 = Vector3.Dot(v1, a), p2 = Vector3.Dot(v2, a);
                float r = half.X * MathF.Abs(a.X) + half.Y * MathF.Abs(a.Y) + half.Z * MathF.Abs(a.Z);
                if (MathF.Min(p0, MathF.Min(p1, p2)) > r || MathF.Max(p0, MathF.Max(p1, p2)) < -r) return false;
            }

        Vector3 min = Vector3.Min(v0, Vector3.Min(v1, v2)), max = Vector3.Max(v0, Vector3.Max(v1, v2));
        if (min.X > half.X || max.X < -half.X || min.Y > half.Y || max.Y < -half.Y || min.Z > half.Z || max.Z < -half.Z) return false;

        Vector3 normal = Vector3.Cross(e0, e1);
        float d = Vector3.Dot(normal, v0);
        float radius = half.X * MathF.Abs(normal.X) + half.Y * MathF.Abs(normal.Y) + half.Z * MathF.Abs(normal.Z);
        return MathF.Abs(d) <= radius;
    }
}
