using System;
using System.Collections.Generic;
using VRageMath;

namespace IngameScript {
    partial class Program {

        public partial class Spaceship {
            public class Flightplan :ISerializable {

                public class DockInfo {
                    public MatrixD matrix = MatrixD.Identity;
                }

                public class Waypoint : ISerializable {
                    public string name;
                    public Vector3D position;
                    public float speed;
                    public SpaceshipFlags flags = 0;

                    public void Serialize(MemoryStream ms) {
                        ms.Write(name);
                        ms.Write(position);
                        ms.Write(speed);
                        ms.Write((int)flags);
                    }

                    public void Deserialize(MemoryStream ms) {
                        ms.ReadString(out name);
                        ms.Read(out position);
                        ms.ReadFloat(out speed);
                        int t_flags; ms.ReadInt(out t_flags); flags = (SpaceshipFlags)t_flags;
                    }
                }

                public DockInfo departure = new DockInfo(), arrival = new DockInfo();
                public List<Waypoint> waypoints = new List<Waypoint>();

                public bool IsValid() {
                    return waypoints.Count > 1;
                }

                public void Serialize(MemoryStream ms) {
                    ms.Write(departure.matrix);
                    ms.Write(arrival.matrix);
                    ms.Write(waypoints.Count);
                    for (int i = 0; i < waypoints.Count; i++)
                        waypoints[i].Serialize(ms);
                }

                public void Deserialize(MemoryStream ms) {
                    ms.Read(out departure.matrix);
                    ms.Read(out arrival.matrix);
                    int waypoints_counter;
                    ms.ReadInt(out waypoints_counter);
                    for (int i = 0;i < waypoints_counter; i++) {
                        Waypoint waypoint = new Waypoint();
                        waypoint.Deserialize(ms);
                        waypoints.Add(waypoint);
                    }
                }
            }

        }
    }
}