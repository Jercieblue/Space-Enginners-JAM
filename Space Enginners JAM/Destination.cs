using System;
using VRageMath;

namespace IngameScript {
    partial class Program {
        public partial class Spaceship {
            public class Destination : ISerializable {
                public long id;
                public string name;
                public MatrixD matrix;
                public float size;

                public static Destination FromGPS(string gps_text) {
                    string[] tokens = gps_text.Split(':');
                    Destination destination = new Destination();
                    destination.id = 0;
                    destination.name = tokens[1];
                    destination.matrix = MatrixD.Identity;
                    destination.matrix.Translation = new Vector3D(double.Parse(tokens[2]), double.Parse(tokens[3]), double.Parse(tokens[4]));
                    destination.size = 2.5f;
                    return destination;
                }

                public void Serialize(MemoryStream ms) {
                    ms.Write(id);
                    ms.Write(name);
                    ms.Write(matrix);
                    ms.Write(size);
                }

                public void Deserialize(MemoryStream ms) {
                    ms.ReadLong(out id);
                    ms.ReadString(out name);
                    ms.Read(out matrix);
                    ms.ReadFloat(out size);
                }
    
                public override string ToString() {
                    return name;
                }
            }
        }
    }
}