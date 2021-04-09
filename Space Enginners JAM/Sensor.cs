using Sandbox.ModAPI.Ingame;
using System;
using VRageMath;

namespace IngameScript {
    partial class Program {

        public partial class Spaceship {
            public class Sensor : ISpaceshipComponent {
                static Vector2[] angles = {new Vector2(0, 0), new Vector2(1, 1), new Vector2(-1, -1), new Vector2(1, -1), new Vector2(-1, 1)};
                Vector3D[] detected = new Vector3D[angles.Length];

                IMyCameraBlock camera;
                public float range = 0.0f;
                public float value = 1.0f;
                public Vector3D direction = Vector3D.Zero;
                private int next_angle = 0;

                public Sensor(IMyCameraBlock camera) {
                    this.camera = camera;
                }

                private void CastRay() {
                    if (camera.AvailableScanRange > range) {
                        MyDetectedEntityInfo info = camera.Raycast(range, angles[next_angle].X * Settings.SCAN_ANGLE, angles[next_angle].Y * Settings.SCAN_ANGLE);

                        if (!info.IsEmpty()) {
                            detected[next_angle] = info.HitPosition.Value;
                        } else {
                            detected[next_angle] = Vector3D.Zero;
                        }
                        next_angle++;
                        if (next_angle >= angles.Length) next_angle = 0;
                    }
                }

                void UpdateValues() {
                    value = 1.0f;

                    foreach (Vector3D detected_position in detected) {
                        Vector3D offset = detected_position - camera.WorldMatrix.Translation;
                        float distance = Math.Min(1.0f, (float)offset.Length() / range);
                        if (distance < value) {
                            value = distance;
                            direction = Vector3D.Normalize(offset);
                        }
                    }
                }

                public void OnUpdateFrame() {
                    CastRay();
                    UpdateValues();
                }

                public void DebugDraw(ref string str) {
                    str += string.Format("{0:0}%\n", value * 100.0f);
                }

                public void Enable() {
                    camera.EnableRaycast = true;
                }

                public void Disable() {
                    camera.EnableRaycast = false;
                }
            }
        }
    }
}