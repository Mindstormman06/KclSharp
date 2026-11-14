using System.Buffers.Binary;
using System.Numerics;

namespace KclSharp;

public readonly record struct KclTriangle(Vector3 A, Vector3 B, Vector3 C)
{
    public Vector3 Normal => Vector3.Normalize(Vector3.Cross(B - A, C - A));
}

public struct KclPrism
{
    public float Height;
    public ushort Position;
    public ushort FaceNormal;
    public ushort EdgeNormalA;
    public ushort EdgeNormalB;
    public ushort EdgeNormalC;
    public ushort Attribute;

    public override readonly string ToString() => $"height {Height}, attribute 0x{Attribute:X4}";
}

/// <summary>A Wii KCL collision file: triangles stored as prisms over shared positions and normals, with an octree.</summary>
public sealed partial class KclFile
{
    private const int HeaderSize = 0x38;
    private const int PrismSize = 0x10;

    public List<Vector3> Positions { get; } = [];
    public List<Vector3> Normals { get; } = [];
    public List<KclPrism> Prisms { get; } = [];
    public float Thickness { get; set; } = 1;
    public Vector3 Minimum { get; set; }
    public uint MaskX { get; set; }
    public uint MaskY { get; set; }
    public uint MaskZ { get; set; }
    public uint Shift { get; set; }
    public uint ShiftY { get; set; }
    public uint ShiftZ { get; set; }

    /// <summary>The octree, kept as bytes. It is not rebuilt, so it goes stale if triangles move.</summary>
    public byte[] Octree { get; set; } = [];

    public KclTriangle? Triangle(int prism) => Triangle(Prisms[prism]);

    public KclTriangle? Triangle(KclPrism p)
    {
        Vector3 position = Positions[p.Position], face = Normals[p.FaceNormal];
        Vector3 crossA = Vector3.Cross(Normals[p.EdgeNormalA], face), crossB = Vector3.Cross(Normals[p.EdgeNormalB], face);
        Vector3 c = Normals[p.EdgeNormalC];
        float dotA = Vector3.Dot(crossA, c), dotB = Vector3.Dot(crossB, c);
        if (dotA == 0 || dotB == 0) return null;
        return new KclTriangle(position, position + crossB * (p.Height / dotB), position + crossA * (p.Height / dotA));
    }

    public IEnumerable<(int Prism, KclTriangle Triangle)> Triangles()
    {
        for (int i = 0; i < Prisms.Count; i++)
            if (Triangle(Prisms[i]) is { } t) yield return (i, t);
    }

    public static KclFile FromFile(string path) => FromBinary(File.ReadAllBytes(path));

    public static KclFile FromBinary(ReadOnlySpan<byte> data)
    {
        if (data.Length < HeaderSize) throw new InvalidDataException("KCL is too short.");
        int positions = (int)U32(data, 0), normals = (int)U32(data, 4), prisms = (int)U32(data, 8), octree = (int)U32(data, 12);
        if (positions != HeaderSize || normals < positions || prisms + PrismSize < normals || octree < prisms + PrismSize || octree > data.Length)
            throw new InvalidDataException("KCL section offsets are invalid.");

        KclFile kcl = new()
        {
            Thickness = F32(data, 0x10), Minimum = Vec3(data, 0x14),
            MaskX = U32(data, 0x20), MaskY = U32(data, 0x24), MaskZ = U32(data, 0x28),
            Shift = U32(data, 0x2C), ShiftY = U32(data, 0x30), ShiftZ = U32(data, 0x34),
            Octree = data[octree..].ToArray(),
        };
        for (int at = positions; at + 12 <= normals; at += 12) kcl.Positions.Add(Vec3(data, at));
        for (int at = normals; at + 12 <= prisms + PrismSize; at += 12) kcl.Normals.Add(Vec3(data, at));
        for (int at = prisms + PrismSize; at + PrismSize <= octree; at += PrismSize)
            kcl.Prisms.Add(new KclPrism
            {
                Height = F32(data, at), Position = U16(data, at + 4), FaceNormal = U16(data, at + 6),
                EdgeNormalA = U16(data, at + 8), EdgeNormalB = U16(data, at + 10), EdgeNormalC = U16(data, at + 12), Attribute = U16(data, at + 14),
            });
        return kcl;
    }

    public void WriteTo(string path) => File.WriteAllBytes(path, Write());

    public byte[] Write()
    {
        int positions = HeaderSize;
        int normals = positions + Positions.Count * 12;
        int prisms = normals + Normals.Count * 12 - PrismSize;
        int octree = prisms + PrismSize + Prisms.Count * PrismSize;
        if (prisms < normals) throw new InvalidOperationException("KCL needs at least two normals before the prisms (the first prism slot overlaps them).");

        byte[] o = new byte[octree + Octree.Length];
        Span<byte> s = o;
        BinaryPrimitives.WriteUInt32BigEndian(s, (uint)positions);
        BinaryPrimitives.WriteUInt32BigEndian(s[4..], (uint)normals);
        BinaryPrimitives.WriteUInt32BigEndian(s[8..], (uint)prisms);
        BinaryPrimitives.WriteUInt32BigEndian(s[12..], (uint)octree);
        BinaryPrimitives.WriteSingleBigEndian(s[0x10..], Thickness);
        WriteVec3(s, 0x14, Minimum);
        BinaryPrimitives.WriteUInt32BigEndian(s[0x20..], MaskX);
        BinaryPrimitives.WriteUInt32BigEndian(s[0x24..], MaskY);
        BinaryPrimitives.WriteUInt32BigEndian(s[0x28..], MaskZ);
        BinaryPrimitives.WriteUInt32BigEndian(s[0x2C..], Shift);
        BinaryPrimitives.WriteUInt32BigEndian(s[0x30..], ShiftY);
        BinaryPrimitives.WriteUInt32BigEndian(s[0x34..], ShiftZ);

        for (int i = 0; i < Positions.Count; i++) WriteVec3(s, positions + i * 12, Positions[i]);
        for (int i = 0; i < Normals.Count; i++) WriteVec3(s, normals + i * 12, Normals[i]);
        for (int i = 0; i < Prisms.Count; i++)
        {
            KclPrism p = Prisms[i];
            int at = prisms + PrismSize * (i + 1);
            BinaryPrimitives.WriteSingleBigEndian(s[at..], p.Height);
            BinaryPrimitives.WriteUInt16BigEndian(s[(at + 4)..], p.Position);
            BinaryPrimitives.WriteUInt16BigEndian(s[(at + 6)..], p.FaceNormal);
            BinaryPrimitives.WriteUInt16BigEndian(s[(at + 8)..], p.EdgeNormalA);
            BinaryPrimitives.WriteUInt16BigEndian(s[(at + 10)..], p.EdgeNormalB);
            BinaryPrimitives.WriteUInt16BigEndian(s[(at + 12)..], p.EdgeNormalC);
            BinaryPrimitives.WriteUInt16BigEndian(s[(at + 14)..], p.Attribute);
        }
        Octree.CopyTo(s[octree..]);
        return o;
    }

    private static uint U32(ReadOnlySpan<byte> d, int at) => BinaryPrimitives.ReadUInt32BigEndian(d[at..]);
    private static ushort U16(ReadOnlySpan<byte> d, int at) => BinaryPrimitives.ReadUInt16BigEndian(d[at..]);
    private static float F32(ReadOnlySpan<byte> d, int at) => BinaryPrimitives.ReadSingleBigEndian(d[at..]);
    private static Vector3 Vec3(ReadOnlySpan<byte> d, int at) => new(F32(d, at), F32(d, at + 4), F32(d, at + 8));

    private static void WriteVec3(Span<byte> s, int at, Vector3 v)
    {
        BinaryPrimitives.WriteSingleBigEndian(s[at..], v.X);
        BinaryPrimitives.WriteSingleBigEndian(s[(at + 4)..], v.Y);
        BinaryPrimitives.WriteSingleBigEndian(s[(at + 8)..], v.Z);
    }
}

/// <summary>Skyward Sword HD's <c>.kcl2</c>: one little-endian triangle per prism of the matching KCL, in prism order.</summary>
public sealed class Kcl2File
{
    public List<KclTriangle> Triangles { get; } = [];

    public static Kcl2File FromFile(string path) => FromBinary(File.ReadAllBytes(path));

    public static Kcl2File FromBinary(ReadOnlySpan<byte> data)
    {
        if (data.Length % 36 != 0) throw new InvalidDataException("KCL2 length is not a whole number of triangles.");
        Kcl2File file = new();
        for (int at = 0; at < data.Length; at += 36)
            file.Triangles.Add(new KclTriangle(Vec3(data, at), Vec3(data, at + 12), Vec3(data, at + 24)));
        return file;
    }

    /// <summary>The KCL2 the game would pair with a KCL: each prism's triangle (degenerate prisms give their position three times).</summary>
    public static Kcl2File FromKcl(KclFile kcl)
    {
        Kcl2File file = new();
        foreach (KclPrism p in kcl.Prisms)
            file.Triangles.Add(kcl.Triangle(p) ?? new KclTriangle(kcl.Positions[p.Position], kcl.Positions[p.Position], kcl.Positions[p.Position]));
        return file;
    }

    public void WriteTo(string path) => File.WriteAllBytes(path, Write());

    public byte[] Write()
    {
        byte[] o = new byte[Triangles.Count * 36];
        for (int i = 0; i < Triangles.Count; i++)
        {
            KclTriangle t = Triangles[i];
            int at = i * 36;
            foreach (Vector3 v in (Vector3[])[t.A, t.B, t.C])
            {
                BinaryPrimitives.WriteSingleLittleEndian(o.AsSpan(at), v.X);
                BinaryPrimitives.WriteSingleLittleEndian(o.AsSpan(at + 4), v.Y);
                BinaryPrimitives.WriteSingleLittleEndian(o.AsSpan(at + 8), v.Z);
                at += 12;
            }
        }
        return o;
    }

    private static Vector3 Vec3(ReadOnlySpan<byte> d, int at) => new(
        BinaryPrimitives.ReadSingleLittleEndian(d[at..]), BinaryPrimitives.ReadSingleLittleEndian(d[(at + 4)..]), BinaryPrimitives.ReadSingleLittleEndian(d[(at + 8)..]));
}
