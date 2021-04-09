using System;
using VRageMath;
using System.Text;

public class MemoryStream : IDisposable {
    byte[] buffer = new byte[512];
    int index = 0;
    public MemoryStream() {

    }
    public MemoryStream(string str) {
        FromString(str);
    }
    public void Reset() {
        index = 0;
    }

    public void Clear() {
        buffer = new byte[512];
        Reset();
    }

    public void ReadBool(out bool v) {
        v = BitConverter.ToBoolean(buffer, index);
        index += sizeof(bool);
    }

    public void ReadFloat(out float v) {
        v = BitConverter.ToSingle(buffer, index);
        index += sizeof(float);
    }

    public void ReadDouble(out double v) {
        v = BitConverter.ToDouble(buffer, index);
        index += sizeof(double);
    }

    public void ReadString(out string str) {
        int length;
        ReadInt(out length);
        str = Encoding.ASCII.GetString(buffer, index, length);
        index += length;
    }

    public void ReadInt(out int v) {
        v = BitConverter.ToInt32(buffer, index);
        index += sizeof(int);
    }

    public void ReadLong(out long v) {
        v = BitConverter.ToInt64(buffer, index);
        index += sizeof(long);
    }

    public void Read(out Vector3D v) {
        v = Vector3D.Zero;
        ReadDouble(out v.X); ReadDouble(out v.Y); ReadDouble(out v.Z);
    }

    public void Read(out Vector3 v) {
        v = Vector3.Zero;
        ReadFloat(out v.X); ReadFloat(out v.Y); ReadFloat(out v.Z);
    }

    public void Read(out MatrixD m) {
        m = MatrixD.Identity;
        ReadDouble(out m.M11); ReadDouble(out m.M12); ReadDouble(out m.M13); ReadDouble(out m.M14);
        ReadDouble(out m.M21); ReadDouble(out m.M22); ReadDouble(out m.M23); ReadDouble(out m.M24);
        ReadDouble(out m.M31); ReadDouble(out m.M32); ReadDouble(out m.M33); ReadDouble(out m.M34);
        ReadDouble(out m.M41); ReadDouble(out m.M42); ReadDouble(out m.M43); ReadDouble(out m.M44);
    }

    public void Write(Byte[] data) {
        if (buffer.Length < (index + data.Length))
            Array.Resize(ref buffer, buffer.Length * 2);

        Array.Copy(data, 0, buffer, index, data.Length);
        index += data.Length;
    }

    public void Write(bool v) {
        Write(BitConverter.GetBytes(v));
    }

    public void Write(float v) {
        Write(BitConverter.GetBytes(v));
    }

    public void Write(double v) {
        Write(BitConverter.GetBytes(v));
    }

    public void Write(string str) {
        byte[] bytes = Encoding.ASCII.GetBytes(str);
        Write(bytes.Length);
        Write(bytes);
    }

    public void Write(int v) {
        Write(BitConverter.GetBytes(v));
    }

    public void Write(long v) {
        Write(BitConverter.GetBytes(v));
    }

    public void Write(Vector3D v) {
        Write(BitConverter.GetBytes(v.X));
        Write(BitConverter.GetBytes(v.Y));
        Write(BitConverter.GetBytes(v.Z));
    }

    public void Write(Vector3 v) {
        Write(BitConverter.GetBytes(v.X));
        Write(BitConverter.GetBytes(v.Y));
        Write(BitConverter.GetBytes(v.Z));
    }

    public void Write(MatrixD m) {
        Write(BitConverter.GetBytes(m.M11)); Write(BitConverter.GetBytes(m.M12)); Write(BitConverter.GetBytes(m.M13)); Write(BitConverter.GetBytes(m.M14));
        Write(BitConverter.GetBytes(m.M21)); Write(BitConverter.GetBytes(m.M22)); Write(BitConverter.GetBytes(m.M23)); Write(BitConverter.GetBytes(m.M24));
        Write(BitConverter.GetBytes(m.M31)); Write(BitConverter.GetBytes(m.M32)); Write(BitConverter.GetBytes(m.M33)); Write(BitConverter.GetBytes(m.M34));
        Write(BitConverter.GetBytes(m.M41)); Write(BitConverter.GetBytes(m.M42)); Write(BitConverter.GetBytes(m.M43)); Write(BitConverter.GetBytes(m.M44));
    }

    public void FromString(string str) {
        buffer = System.Convert.FromBase64String(str);
    }

    public override string ToString() {
        return System.Convert.ToBase64String(buffer, 0, index);
    }

    public void Dispose() {

    }
}
