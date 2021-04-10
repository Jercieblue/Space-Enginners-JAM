using Sandbox.ModAPI.Ingame;
using System.Collections.Generic;
using VRageMath;

namespace IngameScript {
    partial class Program {

        public partial class Spaceship {

            public class ManagedThruster {
                public IMyThrust thruster;
                public Vector3D direction;
                public ManagedThrusterDirectionType direction_type;
                public float power;
            }
            public class ThrusterManager : ISpaceshipComponent {
                string ThrustersType;
                public List<ManagedThruster> thrusters = new List<ManagedThruster>();

                Spaceship spaceship;
                public float[] max_thrust = new float[6];

                private ManagedThrusterDirectionType GetThrusterDirectionType(Vector3D direction) {
                    if (direction.X > 0.5)
                        return ManagedThrusterDirectionType.Right;
                    else if (direction.X < -0.5)
                        return ManagedThrusterDirectionType.Left;
                    else if (direction.Y > 0.5)
                        return ManagedThrusterDirectionType.Up;
                    else if (direction.Y < -0.5)
                        return ManagedThrusterDirectionType.Down;
                    else if (direction.Z > 0.5)
                        return ManagedThrusterDirectionType.Backward;
                    else if (direction.Z < -0.5)
                        return ManagedThrusterDirectionType.Forward;
                    else
                        return ManagedThrusterDirectionType.Invalid;
                }

                public ThrusterManager(string ThrustersType, Spaceship spaceship, IMyRemoteControl control, IMyGridTerminalSystem GridTerminalSystem) {
                    this.ThrustersType = ThrustersType;
                    this.spaceship = spaceship;
                    ClearThrusters();
                    SetupThrusters(ThrustersType, control, GridTerminalSystem);
                }

                public void ClearThrusters() {
                    thrusters.Clear();
                }

                public void SetupThrusters(string ThrustersType, IMyRemoteControl control, IMyGridTerminalSystem system) {
                    int[] thrusters_per_direction = new int[6];

                    foreach (IMyThrust thruster in FindBlocksOfType<IMyThrust>(system, spaceship)) {
                        if (thruster.CubeGrid == spaceship.cpu.CubeGrid && thruster.BlockDefinition.SubtypeName.Contains(ThrustersType)) {
                            ManagedThruster managed = new ManagedThruster();

                            managed.direction = Vector3D.Transform(thruster.WorldMatrix.Backward, Matrix.Transpose(control.WorldMatrix.GetOrientation()));
                            managed.thruster = thruster;
                            managed.direction_type = GetThrusterDirectionType(managed.direction);

                            thrusters_per_direction[(int)managed.direction_type]++;
                            thrusters.Add(managed);
                        }
                    }

                    thrusters.Sort((left, right) => left.direction_type.CompareTo(right.direction_type));

                    foreach (ManagedThruster thruster in thrusters)
                        thruster.power = 1.0f / (float)thrusters_per_direction[(int)thruster.direction_type];
                }

                public void ApplyThrusters(float[] required_thrust) {
                    for (int i = 0; i < 6; i++)
                        max_thrust[i] = 0.0f;

                    foreach (ManagedThruster thruster in thrusters) {
                        max_thrust[(int)thruster.direction_type] += thruster.thruster.MaxEffectiveThrust;
                        float thrust_effectivness = thruster.thruster.MaxEffectiveThrust / thruster.thruster.MaxThrust;
                        if (thrust_effectivness > Settings.MinThrustEffectivness) {
                            thruster.thruster.ThrustOverride = required_thrust[(int)thruster.direction_type] * thruster.power / thrust_effectivness;
                        } else {
                            thruster.thruster.ThrustOverride = 0.0f;
                        }

                        if (Settings.AutoStartThrusters) {
                            if (thruster.thruster.ThrustOverride >= Settings.EPSILON && !thruster.thruster.Enabled)
                                thruster.thruster.Enabled = true;
                            else if (thruster.thruster.ThrustOverride < Settings.EPSILON && thruster.thruster.Enabled)
                                thruster.thruster.Enabled = false;
                        }
                    }

                    foreach (ManagedThruster thruster in thrusters) {
                        required_thrust[(int)thruster.direction_type] -= thruster.thruster.ThrustOverride;
                    }
                }

                public string DebugString() {
                    string result = ThrustersType + " Thrusters\n";

                    foreach (ManagedThruster thruster in thrusters) {
                        result += string.Format("{0}: {1:0}%", thruster.direction_type.ToString()[0], thruster.thruster.ThrustOverridePercentage * 100.0f, thruster.thruster.MaxEffectiveThrust / thruster.thruster.MaxThrust);
                    }
                    result += "\n";
                    return result;
                }

                public void Enable() {
                    if (Settings.AutoStartThrusters) {
                        foreach (ManagedThruster thruster in thrusters)
                            thruster.thruster.Enabled = false;
                    }

                }

                public void Disable() {
                    foreach (ManagedThruster thruster in thrusters)
                        thruster.thruster.ThrustOverride = 0.0f;

                    if (Settings.AutoStartThrusters) {
                        foreach (ManagedThruster thruster in thrusters)
                            thruster.thruster.Enabled = true;
                    }
                }
            }

            public class ThrustersManager : ISpaceshipComponent {
                public static List<ThrusterManager> managers = new List<ThrusterManager>();
                public static float[] max_thrust = new float[6];

                public ThrustersManager(Spaceship spaceship, IMyRemoteControl control, IMyGridTerminalSystem system) {
                    managers.Add(new ThrusterManager("LargeAtmosphericThrust", spaceship, control, system));
                    managers.Add(new ThrusterManager("SmallAtmosphericThrust", spaceship, control, system));
                    managers.Add(new ThrusterManager("LargeThrust", spaceship, control, system));
                    managers.Add(new ThrusterManager("SmallThrust", spaceship, control, system));
                    managers.Add(new ThrusterManager("LargeHydrogenThrust", spaceship, control, system));
                    managers.Add(new ThrusterManager("SmallHydrogenThrust", spaceship, control, system));
                }

                public void OnUpdateFrame() {
                    for (int i = 0; i < 6; i++)
                        max_thrust[i] = 0.0f;

                    foreach (ThrusterManager manager in managers) {
                        manager.ApplyThrusters(Autopilot.required_thrust);
                        for (int i = 0; i < 6; i++)
                            max_thrust[i] += manager.max_thrust[i];
                    }
                }

                public void Enable() {
                    foreach (ThrusterManager manager in managers)
                        manager.Enable();
                }

                public void Disable() {
                    foreach (ThrusterManager manager in managers)
                        manager.Disable();
                }
            }

        }
    }
}