using Sandbox.ModAPI.Ingame;
using VRageMath;

namespace IngameScript {
    partial class Program {
       
        public partial class Spaceship {
            public class Autopilot : ISpaceshipComponent {
                Spaceship spaceship;
                public IMyRemoteControl control;
                public IMyGyro gyro;
                public float azimuth, elevation;

                public static float[] required_thrust = new float[6];

                public Autopilot(Spaceship spaceship, IMyRemoteControl control, IMyGyro gyro) {
                    this.spaceship = spaceship;
                    this.control = control;
                    this.gyro = gyro;
                }

                public void CalculateThrust(Vector3D target_velocity, IMyRemoteControl control) {
                    float mass = control.CalculateShipMass().TotalMass;
                    for (int i = 0; i < 6; i++) {
                        Vector3D thruster_vector = Settings.ManagedThrusterDirections[i];
                        required_thrust[i] = 0.0f;
                        if (Vector3D.Dot(target_velocity, thruster_vector) > 0.0f) {
                            Vector3D projected_vector = Vector3D.ProjectOnVector(ref target_velocity, ref thruster_vector);
                            required_thrust[i] = (float)(projected_vector.Length() * mass);
                            if (required_thrust[i] < Settings.EPSILON)
                                required_thrust[i] = 0.0f;
                        }
                    }
                }
                        
                void ApplyGyro(Vector3 v) {
                    gyro.Pitch = v.X;
                    gyro.Yaw = v.Y;
                    gyro.Roll = v.Z;
                }
                            
                public void Allign(Vector3D ws_fwd, Vector3D ws_up, IMyGyro gyro, IMyTerminalBlock block) {
                    Quaternion quat = Quaternion.Conjugate(Quaternion.CreateFromForwardUp(block.WorldMatrix.Forward, block.WorldMatrix.Up));
                    Vector3D up = Vector3D.Normalize(quat * ws_up);
                    Vector3D right = Vector3D.Normalize(Vector3D.Cross(up, quat * ws_fwd));
                    Vector3D front = Vector3D.Normalize(Vector3D.Cross(right, up));
                    Vector3.GetAzimuthAndElevation(front, out azimuth, out elevation);
                    Vector3 final = Vector3.Transform(new Vector3(elevation, azimuth, (float)Vector3D.Dot(-Vector3D.UnitX, -up)),
                        block.WorldMatrix.GetOrientation() * Matrix.Transpose(gyro.WorldMatrix.GetOrientation()));
                    ApplyGyro(-final);
                }

                public void LookAt(Vector3D target, IMyGyro gyro, IMyRemoteControl control) {
                    Vector3D offet = (target - control.GetPosition());
                    if (offet.Length() < 100.0f) {
                        ApplyGyro(Vector3.Zero);
                        return;
                    }

                    Vector3 direction = Vector3D.Normalize(offet);
                    if (spaceship.gravity.LengthSquared() > 1e-1f) {
                        Allign(direction, spaceship.gravity, gyro, control);
                    } else {
                        Quaternion quat = Quaternion.CreateFromForwardUp(control.WorldMatrix.Forward, control.WorldMatrix.Up);
                        quat.Conjugate();
                        direction = quat * direction;
                        Vector3.GetAzimuthAndElevation(direction, out azimuth, out elevation);
                        Vector3 final = Vector3.Transform(new Vector3(elevation, azimuth, 0.0f),
                            control.WorldMatrix.GetOrientation() * Matrix.Transpose(gyro.WorldMatrix.GetOrientation()));

                        ApplyGyro(-final);
                    }
                }

                public void OnUpdateFrame() {
                    if (spaceship.fd.flightplan != null) {
                        if ((spaceship.flags & SpaceshipFlags.LookAt) == SpaceshipFlags.LookAt)
                            LookAt(spaceship.fd.flightplan.waypoints[spaceship.fd.active_waypoint].position, gyro, control);

                        if ((spaceship.flags & SpaceshipFlags.Alln) == SpaceshipFlags.Alln)
                            Allign(-spaceship.fd.flightplan.arrival.matrix.Forward, spaceship.fd.flightplan.arrival.matrix.Right, gyro, spaceship.connector);
                    }

                    CalculateThrust(Vector3D.Transform(spaceship.desired - spaceship.velocity - spaceship.gravity, Matrix.Transpose(control.WorldMatrix.GetOrientation())), control);
                }

                public void Enable() {
                    gyro.GyroOverride = true;
                            
                }

                public void Disable() {
                    gyro.GyroOverride = false;
                    ApplyGyro(Vector3.Zero);
                }
            }
        }
    }
}