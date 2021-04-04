using Sandbox.Game.EntityComponents;
using Sandbox.ModAPI.Ingame;
using Sandbox.ModAPI.Interfaces;
using SpaceEngineers.Game.ModAPI.Ingame;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Collections.Immutable;
using System.Linq;
using System.Text;
using VRage;
using VRage.Collections;
using VRage.Game;
using VRage.Game.Components;
using VRage.Game.GUI.TextPanel;
using VRage.Game.ModAPI.Ingame;
using VRage.Game.ModAPI.Ingame.Utilities;
using VRage.Game.ObjectBuilders.Definitions;
using VRageMath;

namespace IngameScript {

    partial class Program : MyGridProgram {
        public static string TAG = "JAM";
        public static float MAX_SPEED = 95.0f;
        public static float ARRIVAL_DIST = 2000.0f;
        public static string channel = "JAM_CH_1";

        public enum ManagedThrusterDirectionType { Forward, Backward, Up, Down, Left, Right, Invalid }
        public static Vector3D[] ManagedThrusterDirections = { -Vector3D.UnitZ, Vector3D.UnitZ, Vector3D.UnitY, -Vector3D.UnitY, -Vector3D.UnitX, Vector3D.UnitX };

        public class Spaceship {
            public interface ISpaceshipComponent {
                void Enable();
                void Disable();

            }
            public enum SpaceshipFlags : UInt64 {
                AP = 1 << 0,
                FD = 1 << 1,
                CAS = 1 << 2,
                TM = 1 << 3,
                LookAt = 1 << 4,
                LCK = 1 << 5,
                Alln = 1 << 6,
                Idle = AP | FD | TM,
                Undock = AP | FD | TM,
                Enroute = AP | FD | CAS | TM | LookAt,
                Approach = AP | FD | TM | Alln,
                Dock = LCK
            }
            public class Settings {
                public bool AutoStartThrusters = true;
                public float CASDistanceExtension = 1.0f;
                public float MinRayDist = 0.0f;
                public float MinThrustEffectivness = 0.30f; // When the effectivness of the thruster drops below gets deactivated
                public float ApproachHeight = 100.0f;
            }
            public class ManagedThruster {
                public IMyThrust thruster;
                public Vector3D direction;
                public ManagedThrusterDirectionType direction_type;
                public float power;
            }
            public class ThrusterManager : ISpaceshipComponent {
                string ThrustersType;
                List<IMyThrust> temp_thrusters = new List<IMyThrust>();
                public List<ManagedThruster> thrusters = new List<ManagedThruster>();
                public bool IsAtmospheric;
                Spaceship spaceship;

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

                public ThrusterManager(string ThrustersType, Spaceship spaceship, IMyRemoteControl control, IMyGridTerminalSystem GridTerminalSystem, bool is_atmospheric = false) {
                    this.ThrustersType = ThrustersType;
                    this.spaceship = spaceship;
                    this.IsAtmospheric = is_atmospheric;
                    ClearThrusters();
                    SetupThrusters(ThrustersType, control, GridTerminalSystem);
                }

                public void ClearThrusters() {
                    thrusters.Clear();
                }

                public void SetupThrusters(string ThrustersType, IMyRemoteControl control, IMyGridTerminalSystem GridTerminalSystem) {
                    GridTerminalSystem.GetBlocksOfType<IMyThrust>(temp_thrusters);
                    int[] thrusters_per_direction = new int[6];

                    foreach (IMyThrust thruster in temp_thrusters) {
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
                    foreach (ManagedThruster thruster in thrusters) {
                        float thrust_effectivness = thruster.thruster.MaxEffectiveThrust / thruster.thruster.MaxThrust;
                        if (thrust_effectivness > settings.MinThrustEffectivness) {
                            thruster.thruster.ThrustOverride = required_thrust[(int)thruster.direction_type] * thruster.power / thrust_effectivness;
                        } else {
                            thruster.thruster.ThrustOverride = 0.0f;
                        }

                        if (IsAtmospheric)
                            spaceship.AtmosphericThrustersOnline = thrust_effectivness > settings.MinThrustEffectivness;

                        if (settings.AutoStartThrusters) {
                            if (thruster.thruster.ThrustOverride >= 1e-1f && !thruster.thruster.Enabled)
                                thruster.thruster.Enabled = true;
                            else if (thruster.thruster.ThrustOverride < 1e-1f && thruster.thruster.Enabled)
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
                    if (settings.AutoStartThrusters) {
                        foreach (ManagedThruster thruster in thrusters)
                            thruster.thruster.Enabled = false;
                    }
                }

                public void Disable() {
                    foreach (ManagedThruster thruster in thrusters)
                        thruster.thruster.ThrustOverride = 0.0f;

                    if (settings.AutoStartThrusters) {
                        foreach (ManagedThruster thruster in thrusters)
                            thruster.thruster.Enabled = true;
                    }
                }
            }
            public class Autopilot : ISpaceshipComponent {
                Spaceship spaceship;
                public IMyRemoteControl control;
                public IMyGyro gyro;
                public float azimuth, elevation;

                public float[] required_thrust = new float[6];

                public Autopilot(Spaceship spaceship, IMyRemoteControl control, IMyGyro gyro) {
                    this.spaceship = spaceship;
                    this.control = control;
                    this.gyro = gyro;
                }

                public float CalculateMaxEffectiveThrust(ManagedThrusterDirectionType type, List<ThrusterManager> managers) {
                    float result = 0.0f;

                    foreach (ThrusterManager manager in managers)
                        foreach (ManagedThruster thruster in manager.thrusters)
                            if (thruster.direction_type == type)
                                result += (thruster.thruster.MaxEffectiveThrust / thruster.thruster.MaxThrust) > settings.MinThrustEffectivness ? thruster.thruster.MaxEffectiveThrust : 0.0f;

                    return result;
                }

                public float CalculateEffectiveBreakingDistance(float mass, float max_speed, List<ThrusterManager> managers) {
                    float max_thrust = CalculateMaxEffectiveThrust(ManagedThrusterDirectionType.Backward, managers);
                    float speed = max_speed;
                    float meters_travelled = 0.0f;
                    while (speed > 0.0f) {
                        meters_travelled += speed;
                        speed -= max_thrust / mass;
                    }
                    meters_travelled += speed;
                    return meters_travelled;
                }

                public void CalculateThrust(Vector3D target_velocity, IMyRemoteControl control) {
                    float mass = control.CalculateShipMass().TotalMass;
                    for (int i = 0; i < 6; i++) {
                        Vector3D thruster_vector = ManagedThrusterDirections[i];
                        required_thrust[i] = 0.0f;
                        if (Vector3D.Dot(target_velocity, thruster_vector) > 0.0f) {
                            Vector3D projected_vector = Vector3D.ProjectOnVector(ref target_velocity, ref thruster_vector);
                            required_thrust[i] = (float)(projected_vector.Length() * mass);
                            if (required_thrust[i] < 1e-4f)
                                required_thrust[i] = 0.0f;
                        }
                    }
                }

                public void Allign(Vector3D ws_fwd, Vector3D ws_up, IMyGyro gyro, IMyShipConnector connector) {
                    Quaternion quat = Quaternion.CreateFromForwardUp(connector.WorldMatrix.Forward, connector.WorldMatrix.Up);
                    quat.Conjugate();
                    Vector3D up = Vector3D.Normalize(quat * ws_up);
                    Vector3D right = Vector3D.Normalize(Vector3D.Cross(up, quat * ws_fwd));
                    Vector3D front = Vector3D.Normalize(Vector3D.Cross(right, up));
                    float up_right = (float)Vector3D.Dot(-Vector3D.UnitX, -up);
                    Vector3.GetAzimuthAndElevation(front, out azimuth, out elevation);
                    Vector3 final = Vector3.Transform(new Vector3(elevation, azimuth, up_right),
                        connector.WorldMatrix.GetOrientation() * Matrix.Transpose(gyro.WorldMatrix.GetOrientation()));

                    gyro.Pitch = -(float)final.X;
                    gyro.Yaw = -(float)final.Y;
                    gyro.Roll = -(float)final.Z;
                }

                public void LookAt(Vector3D target, IMyGyro gyro, IMyRemoteControl control) {
                    Vector3D offet = (target - control.GetPosition());
                    if (offet.Length() < 10.0f) {
                        gyro.Pitch = 0.0f;
                        gyro.Yaw = 0.0f;
                        gyro.Roll = 0.0f;
                        return;
                    }

                    Vector3 direction = Vector3D.Normalize(offet);
                    Quaternion quat = Quaternion.CreateFromForwardUp(control.WorldMatrix.Forward, control.WorldMatrix.Up);
                    quat.Conjugate();
                    direction = quat * direction;

                    if (spaceship.AtmosphericThrustersOnline && spaceship.gravity.LengthSquared() > 1e-1f) {
                        Vector3D up = Vector3D.Normalize(quat * spaceship.gravity);
                        Vector3D right = Vector3D.Normalize(Vector3D.Cross(up, direction));
                        Vector3D front = Vector3D.Normalize(Vector3D.Cross(right, up));
                        float up_right = (float)Vector3D.Dot(-Vector3D.UnitX, -up);
                        Vector3.GetAzimuthAndElevation(front, out azimuth, out elevation);
                        Vector3 final = Vector3.Transform(new Vector3(elevation, azimuth, up_right),
                            control.WorldMatrix.GetOrientation() * Matrix.Transpose(gyro.WorldMatrix.GetOrientation()));

                        gyro.Pitch = -(float)final.X;
                        gyro.Yaw = -(float)final.Y;
                        gyro.Roll = -(float)final.Z;
                    } else {
                        Vector3.GetAzimuthAndElevation(direction, out azimuth, out elevation);
                        Vector3 final = Vector3.Transform(new Vector3(elevation, azimuth, 0.0f),
                            control.WorldMatrix.GetOrientation() * Matrix.Transpose(gyro.WorldMatrix.GetOrientation()));

                        gyro.Pitch = -(float)final.X;
                        gyro.Yaw = -(float)final.Y;
                        gyro.Roll = -(float)final.Z;
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
                    gyro.Pitch = 0.0f;
                    gyro.Yaw = 0.0f;
                    gyro.Roll = 0.0f;
                }
            }

            public class Flightplan {

                public class DockInfo {
                    public MatrixD matrix = MatrixD.Identity;
                }

                public class Waypoint {
                    public string name;
                    public Vector3D position;
                    public float speed;
                    public SpaceshipFlags flags = 0;

                    public static Waypoint FromGPS(Spaceship spaceship, string gps_text, float speed, SpaceshipFlags flags = SpaceshipFlags.Enroute) {
                        string[] tokens = gps_text.Split(':');
                        Waypoint wpt = new Waypoint();
                        wpt.name = tokens[1];
                        wpt.position = new Vector3D(double.Parse(tokens[2]), double.Parse(tokens[3]), double.Parse(tokens[4]));
                        wpt.speed = speed;
                        wpt.flags = flags;
                        return wpt;
                    }
                }

                public DockInfo departure = new DockInfo(), arrival = new DockInfo();
                public List<Waypoint> waypoints = new List<Waypoint>();

                public bool IsValid() {
                    return waypoints.Count() > 1;
                }
            }

            public class FlightDirector : ISpaceshipComponent {

                Spaceship spaceship;
                public Flightplan flightplan;
                public int prev, next;
                public double p = 0.0;
                public double target_speed = 0.0;
                public int active_waypoint = 0;

                public FlightDirector(Spaceship spaceship) {
                    this.spaceship = spaceship;
                }
                public void SetFlightPlan(Flightplan fp) {
                    flightplan = null;
                    prev = 0;
                    next = 0;
                    active_waypoint = 0;
                    if (fp != null && fp.IsValid()) {
                        flightplan = fp;
                    }
                }

                Vector3D v_epsilon = new Vector3D(1e-6f);

                public Vector3D Arrive(Vector3D position, Vector3D velocity, Vector3D target, float max_distance) {
                    Vector3D future_position = position + Vector3D.Normalize(velocity + v_epsilon) * spaceship.effective_breaking_distance;

                    double dist = Vector3D.Distance(target, future_position);
                    if (dist < 1e-2f)
                        return Vector3D.Zero;
                    else if (dist > max_distance)
                        return Vector3D.Normalize(target - future_position) * Program.MAX_SPEED;
                    else
                        return Vector3D.Normalize(target - future_position) * (dist / max_distance) * Program.MAX_SPEED;
                }

                public Vector3D Enroute(Flightplan.Waypoint prev, Flightplan.Waypoint next, Vector3D position, Vector3D velocity) {
                    //Vector3D future_position = position + Vector3D.Normalize(velocity + v_epsilon) * spaceship.effective_breaking_distance;
                    Vector3D future_position = position;
                    Vector3D to_spaceship = future_position - prev.position;
                    Vector3D to_nextwpt = next.position - prev.position;
                    Vector3D position_on_route = Vector3D.ProjectOnVector(ref to_spaceship, ref to_nextwpt);
                    Vector3D result = next.position - future_position;
                    //result += Vector3D.Normalize(position_on_route - future_position);

                    double distance_between_wpts = Vector3D.Distance(prev.position, next.position);
                    double distance_from_prev = Vector3D.Distance(prev.position, position);
                    double distance_to_next = Vector3D.Distance(next.position, position);

                    p = Math.Max(0.0, Math.Min(1.0, distance_from_prev / distance_between_wpts));
                    if (distance_between_wpts > Program.ARRIVAL_DIST) {
                        if (distance_to_next > Program.ARRIVAL_DIST)
                            target_speed = Program.MAX_SPEED;
                        else
                            target_speed = Spaceship.Remap(distance_to_next, Program.ARRIVAL_DIST, 0.0, Program.MAX_SPEED, next.speed);
                    } else {
                        target_speed = (prev.speed * (1.0f - p) + next.speed * p);
                    }
                    Spaceship.Vector3DLimit(ref result, (float)target_speed);
                    return result;
                }

                public void OnUpdateFrame() {
                    if ((prev != next) && (flightplan != null)) {

                        if (Vector3D.Distance(flightplan.waypoints[next].position, spaceship.GetPosition()) < 0.1f)
                            NextWaypoint();

                        if ((spaceship.flags & SpaceshipFlags.Approach) == SpaceshipFlags.Approach)
                            spaceship.desired += Enroute(flightplan.waypoints[prev], flightplan.waypoints[next], spaceship.GetPosition(), spaceship.velocity);
                        else
                            spaceship.desired += Enroute(flightplan.waypoints[prev], flightplan.waypoints[next], spaceship.GetPosition(), spaceship.velocity);


                    }
                }

                void NextWaypoint() {
                    prev = next;
                    p = 0.0f;
                    spaceship.flags = flightplan.waypoints[prev].flags;
                    active_waypoint++;
                    if (active_waypoint < flightplan.waypoints.Count()) {
                        next = active_waypoint;
                    }
                }

                public void StartFlight() {
                    NextWaypoint();
                }

                public void Enable() {

                }

                public void Disable() {

                }

            }

            public class CollisionAvoidanceSystem : ISpaceshipComponent {
                Spaceship spaceship;
                List<Sensor> sensors = new List<Sensor>();

                public CollisionAvoidanceSystem(Spaceship spaceship) {
                    this.spaceship = spaceship;

                    foreach (IMyCameraBlock camera in Program.FindBlocksOfType<IMyCameraBlock>(spaceship.system, spaceship)) {
                        sensors.Add(new Sensor(camera));
                    }
                }

                public void OnUpdateFrame() {
                    foreach (Sensor sensor in sensors) {
                        sensor.range = Math.Max(settings.MinRayDist, 500.0f * ((float)spaceship.velocity.Length() / 100.0f));
                        sensor.OnUpdateFrame();
                        spaceship.desired += -sensor.direction * (1.0f - sensor.value) * Program.MAX_SPEED;
                    }
                }

                public string DebugDraw() {
                    string result = "";
                    foreach (Sensor sensor in sensors)
                        sensor.DebugDraw(ref result);
                    return result;
                }

                public void Enable() {
                    foreach (Sensor sensor in sensors) {
                        sensor.Enable();
                    }
                }

                public void Disable() {
                    foreach (Sensor sensor in sensors) {
                        sensor.Disable();
                    }
                }
            }

            public class Sensor : ISpaceshipComponent {
                static Vector2[] angles = {
                new Vector2(0, 0), new Vector2(30, 30), new Vector2(-30, -30), new Vector2(30, -30), new Vector2(-30, 30)
            };

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
                        MyDetectedEntityInfo info = camera.Raycast(range, angles[next_angle].X, angles[next_angle].Y);

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

            public class Destination {
                public long id;
                public string name;
                public MatrixD matrix;
                public float size;

                public override string ToString() {
                    return name;
                }
            }

            public class TaskManager {
                public enum TaskType {
                    Navigation, Load, Unload, Charge, Repeat, Total
                }

                public enum TaskManagerState {
                    Navigation, Travelling, Action, NextTask
                }
                public class Task {
                    public TaskType type = TaskType.Navigation;
                    public Destination destination;
                }

                public List<Task> tasks = new List<Task>();
                public int active_task = 0;
                public TaskManagerState state = TaskManagerState.Navigation;
                public TaskManagerState next_state = TaskManagerState.Navigation;

                public void OnUpdateFrame() {
                    state = next_state;
                    switch (state) {
                        case TaskManagerState.Navigation:
                            if (tasks[active_task].destination != null) {
                                spaceship.fd.SetFlightPlan(spaceship.GenerateFlightPlan(tasks[active_task].destination));
                                spaceship.fd.StartFlight();
                                next_state = TaskManagerState.Travelling;
                            } else {
                                next_state = TaskManagerState.Action;
                            }
                            break;
                        case TaskManagerState.Travelling:
                            if (spaceship.fd.flightplan == null) {
                                next_state = TaskManagerState.Action;
                            }
                            break;
                        case TaskManagerState.Action:
                            switch (tasks[active_task].type) {
                                case TaskType.Navigation:
                                    break;
                                case TaskType.Load: {
                                        List<IMyCargoContainer> containers = FindBlocksOfType<IMyCargoContainer>(spaceship.system, spaceship, true);
                                        bool all_are_full = true;
                                        foreach (IMyCargoContainer container in containers)
                                            if (!container.GetInventory().IsFull)
                                                all_are_full = false;
                                        if (all_are_full) {
                                            next_state = TaskManagerState.NextTask;
                                        }
                                    }
                                    break;
                                case TaskType.Unload: {
                                        List<IMyCargoContainer> containers = FindBlocksOfType<IMyCargoContainer>(spaceship.system, spaceship, true);
                                        bool all_are_empty = true;
                                        foreach (IMyCargoContainer container in containers)
                                            if (container.GetInventory().CurrentVolume > 0)
                                                all_are_empty = false;
                                        if (all_are_empty) {
                                            next_state = TaskManagerState.NextTask;
                                        }
                                    }
                                    break;
                                case TaskType.Charge:
                                    break;
                                case TaskType.Repeat:
                                    active_task = -1;
                                    next_state = TaskManagerState.NextTask;
                                    break;
                                case TaskType.Total:
                                    break;
                            }
                            break;
                        case TaskManagerState.NextTask:
                            active_task++;
                            if (active_task < tasks.Count) {
                                next_state = TaskManagerState.Navigation;
                            }
                            break;

                    }
                }
            }

            #region Menus

            public enum MenuCommand {
                Prev, Next, Select, Return, Insert, Delete
            }

            public interface IMenu {

                string Draw();
                bool Interact(MenuCommand command, ref Stack<IMenu> stack, params string[] args);
            }

            public class Menu : IMenu {
                public string name = "";

                public Menu(string name) {
                    this.name = name;
                }

                public virtual string Draw() {
                    return string.Format("[{0}]\n", ToString());
                }

                public virtual bool Interact(MenuCommand command, ref Stack<IMenu> stack, params string[] args) {
                    stack.Pop();
                    return true;
                }

                public override string ToString() {
                    return name;
                }
            }

            public class MenuList : Menu {
                public List<Menu> sub;
                public int index = 0;

                public MenuList(string name, params Menu[] sub_menu) : base(name) {
                    sub = sub_menu.ToList();
                }

                public override string Draw() {
                    string result = base.Draw();
                    for (int i = 0; i < sub.Count; i++)
                        result += string.Format("{0} {1}\n", i == index ? ">" : " ", sub[i].ToString());
                    return result;
                }

                public override bool Interact(MenuCommand command, ref Stack<IMenu> stack, params string[] args) {
                    switch (command) {
                        case MenuCommand.Prev:
                            if (index > 0) index--;
                            break;
                        case MenuCommand.Next:
                            if (index < sub.Count - 1) index++;
                            break;
                        case MenuCommand.Select:
                            if (sub.Count > index)
                                stack.Push(sub[index]);
                            else
                                stack.Pop();
                            break;
                        case MenuCommand.Return:
                            stack.Pop();
                            break;
                    }
                    return true;
                }
            }

            public class MenuStack {
                public Stack<IMenu> stack = new Stack<IMenu>();
                private IMenu root;

                public MenuStack(IMenu root) {
                    this.root = root;
                }

                public string Draw(Spaceship spaceship) {
                    if (stack.Count == 0) stack.Push(root);
                    return stack.First().Draw();
                }

                public void Interact(MenuCommand command, params string[] args) {
                    stack.First().Interact(command, ref stack, args);
                }

            }

            public class MenuFMC : Menu {

                public MenuFMC() : base("FMC") {

                }

                public override string Draw() {
                    string result = base.Draw();
                    if (spaceship.fd.flightplan == null)
                        return result;

                    result += string.Format("{0:0.00}%, Speed: {1}\n", spaceship.fd.p * 100, spaceship.fd.target_speed);
                    for (int i = 0; i < spaceship.fd.flightplan.waypoints.Count(); i++)
                        result += string.Format("{0}{1} ({2})\n", i == spaceship.fd.prev ? "Γ" : i == spaceship.fd.next ? "L" : " ", spaceship.fd.flightplan.waypoints[i].name, spaceship.fd.flightplan.waypoints[i].speed);

                    return result;
                }
            }

            public class MenuDestinationPicker : MenuList {
                TaskManager.Task task;

                public MenuDestinationPicker(TaskManager.Task task) : base("Destinations") {
                    this.task = task;
                }

                public override string Draw() {
                    sub.Clear();
                    foreach (Destination destination in spaceship.destinations.Values) {
                        sub.Add(new Menu(destination.name));
                    }
                    return base.Draw();
                }
                public override bool Interact(MenuCommand command, ref Stack<IMenu> stack, params string[] args) {
                    if (command == MenuCommand.Select) {
                        task.destination = spaceship.destinations.Values.ElementAt(index);
                        stack.Pop();
                        return true;
                    } else {
                        return base.Interact(command, ref stack, args);
                    }
                }

            }

            public class MenuTasks : MenuList {

                public class MenuTask : MenuList {
                    TaskManager.Task task;

                    void UpdateMenu() {
                        sub.Clear();
                        sub.Add(new Menu("Task: " + task.type.ToString()));
                        sub.Add(new Menu("Destination: " + (task.destination != null ? task.destination.ToString() : "No Destination")));
                    }

                    public MenuTask(TaskManager.Task task) : base(task.ToString()) {
                        this.task = task;

                    }
                    public override string Draw() {
                        UpdateMenu();
                        return base.Draw();
                    }

                    public override bool Interact(MenuCommand command, ref Stack<IMenu> stack, params string[] args) {
                        if (command == MenuCommand.Select) {
                            if (index == 0) {
                                task.type = (TaskManager.TaskType)((int)++task.type % (int)TaskManager.TaskType.Total);
                            } else if (index == 1) {
                                stack.Push(new MenuDestinationPicker(task));
                            }
                            return true;
                        } else {
                            return base.Interact(command, ref stack, args);
                        }
                    }

                    public override string ToString() {
                        return string.Format("({0}){1}", task.type, task.destination != null ? task.destination.ToString() : "No Destination");
                    }
                }

                public MenuTasks() : base("Tasks") {

                }

                public override string Draw() {
                    sub.Clear();
                    foreach (TaskManager.Task task in spaceship.tasks.tasks) {
                        sub.Add(new MenuTask(task));
                    }
                    return base.Draw();
                }

                public override bool Interact(MenuCommand command, ref Stack<IMenu> stack, params string[] args) {
                    if (command == MenuCommand.Insert) {
                        spaceship.tasks.tasks.Insert(index, new TaskManager.Task());
                        return true;
                    } else if (command == MenuCommand.Delete) {
                        if (sub.Count > index) spaceship.tasks.tasks.RemoveAt(index);
                        return true;
                    } else {
                        return base.Interact(command, ref stack, args);
                    }
                }

            }


            #endregion

            public IMyGridTerminalSystem system;
            public IMyIntergridCommunicationSystem igc;
            public IMyProgrammableBlock cpu;
            public IMyRemoteControl control;
            public IMyShipConnector connector;
            public IMyCockpit cockpit;
            public IMyTextSurface screen, debug;
            public IMyRadioAntenna radio;
            public IMyBroadcastListener listener;

            public static Settings settings = new Settings();
            public Dictionary<long, Destination> destinations = new Dictionary<long, Destination>();

            public List<ThrusterManager> managers = new List<ThrusterManager>();
            public Autopilot autopilot;
            public FlightDirector fd;
            public CollisionAvoidanceSystem cas;
            public TaskManager tasks = new TaskManager();
            public MenuStack menu = new MenuStack(
                    new MenuList("Main Menu",
                        new MenuFMC(), new MenuTasks())
            );

            public Vector3D velocity = Vector3D.Zero;
            public Vector3D gravity = Vector3D.Zero;
            public bool AtmosphericThrustersOnline = false;
            public float effective_breaking_distance = 0.0f;
            public Vector3D desired = Vector3D.Zero;
            private bool is_enabled = false;
            public SpaceshipFlags flags = SpaceshipFlags.Dock;

            public Spaceship(IMyGridTerminalSystem system, IMyIntergridCommunicationSystem igc) {
                this.system = system;
                this.igc = igc;

                cpu = FindRunningPB(system);

                control = FindBlockOfType<IMyRemoteControl>(system, this);
                connector = FindBlockOfType<IMyShipConnector>(system, this);
                cockpit = FindBlockOfType<IMyCockpit>(system, this);

                screen = cockpit.GetSurface(0);
                debug = cockpit.GetSurface(1);
                debug.ContentType = ContentType.TEXT_AND_IMAGE;

                screen.ContentType = ContentType.SCRIPT;

                MySpriteDrawFrame frame = screen.DrawFrame();

                frame.Add(new MySprite(SpriteType.TEXTURE, "SquareSimple", new Vector2(0, 0), new Vector2(512, 512), Color.Black));
                frame.Add(MySprite.CreateText("Test", "DEBUG", Color.White));
                frame.Dispose();

                listener = igc.RegisterBroadcastListener(channel);
                listener.SetMessageCallback(channel);

                managers.Add(new ThrusterManager("LargeAtmosphericThrust", this, control, system, true));
                managers.Add(new ThrusterManager("SmallAtmosphericThrust", this, control, system, true));
                managers.Add(new ThrusterManager("LargeThrust", this, control, system));
                managers.Add(new ThrusterManager("SmallThrust", this, control, system));
                managers.Add(new ThrusterManager("LargeHydrogenThrust", this, control, system));
                managers.Add(new ThrusterManager("SmallHydrogenThrust", this, control, system));

                autopilot = new Autopilot(this, control, FindBlockOfType<IMyGyro>(system, this));
                cas = new CollisionAvoidanceSystem(this);
                fd = new FlightDirector(this);

                if (connector.Status == MyShipConnectorStatus.Connected)
                    flags = SpaceshipFlags.Dock;
                else
                    flags = SpaceshipFlags.Idle;
            }

            public Vector3 GetPosition() {
                return connector.WorldMatrix.Translation + connector.WorldMatrix.Forward * connector.CubeGrid.GridSize * 0.5f + connector.WorldMatrix.Forward * 0.5f;
            }

            void CheckConnector() {
                if ((flags & SpaceshipFlags.Dock) == SpaceshipFlags.Dock) {
                    if (connector.Status == MyShipConnectorStatus.Connectable) {
                        connector.Connect();
                    }
                } else if ((flags & SpaceshipFlags.Undock) == SpaceshipFlags.Undock) {
                    if (connector.Status == MyShipConnectorStatus.Connected) {
                        connector.Disconnect();
                    }
                }
            }

            public static void Vector3DLimit(ref Vector3D vector, float magnitude) {
                if (vector.Length() > magnitude) {
                    vector.Normalize();
                    vector *= magnitude;
                }
            }

            public static double Remap(double value, double low1, double high1, double low2, double high2) {
                return low2 + (value - low1) * (high2 - low2) / (high1 - low1);
            }

            public void OnFlightUpdateFrame() {
                if (is_enabled) {
                    desired = Vector3D.Zero;
                    velocity = control.GetShipVelocities().LinearVelocity;
                    gravity = control.GetNaturalGravity();
                    effective_breaking_distance = autopilot.CalculateEffectiveBreakingDistance(control.CalculateShipMass().TotalMass, (float)velocity.Length(), managers);

                    tasks.OnUpdateFrame();

                    if ((flags & SpaceshipFlags.CAS) == SpaceshipFlags.CAS) cas.OnUpdateFrame();
                    if ((flags & SpaceshipFlags.FD) == SpaceshipFlags.FD) fd.OnUpdateFrame();

                    Vector3DLimit(ref desired, Program.MAX_SPEED);

                    if ((flags & SpaceshipFlags.AP) == SpaceshipFlags.AP) autopilot.OnUpdateFrame();

                    if ((flags & SpaceshipFlags.TM) == SpaceshipFlags.TM) {
                        foreach (ThrusterManager manager in managers)
                            manager.ApplyThrusters(autopilot.required_thrust);
                    } else {
                        foreach (ThrusterManager manager in managers)
                            manager.Disable();
                    }

                    CheckConnector();
                    if (fd.prev == fd.next) {
                        fd.SetFlightPlan(null);
                    }
                }
            }

            public void Enable() {
                is_enabled = true;
                cas.Enable();
                fd.Enable();
                autopilot.Enable();
                foreach (ThrusterManager manager in managers)
                    manager.Enable();
                control.DampenersOverride = false;
                tasks.active_task = 0;
                tasks.state = TaskManager.TaskManagerState.Navigation;
            }

            public void Disable() {
                cas.Disable();
                fd.Disable();
                autopilot.Disable();
                foreach (ThrusterManager manager in managers)
                    manager.Disable();
                is_enabled = false;
                control.DampenersOverride = true;
            }

            public void HandleMenu(string argument) {
                MenuCommand command;
                if (Enum.TryParse(argument, out command)) {
                    menu.Interact(command);
                }
            }

            Flightplan GenerateFlightPlan(Destination destination) {
                var fp = new Flightplan();

                if (connector.Status == MyShipConnectorStatus.Connected) {
                    fp.departure.matrix = connector.OtherConnector.WorldMatrix;
                    fp.waypoints.Add(new Flightplan.Waypoint { name = "UDCK", position = connector.WorldMatrix.Translation, speed = 5, flags = SpaceshipFlags.Undock });
                    fp.waypoints.Add(new Flightplan.Waypoint { name = "DEPT", position = connector.WorldMatrix.Translation + connector.OtherConnector.WorldMatrix.Forward * settings.ApproachHeight, speed = 10, flags = SpaceshipFlags.Enroute });
                } else {
                    fp.waypoints.Add(new Flightplan.Waypoint { name = "WPT", position = connector.WorldMatrix.Translation, speed = MAX_SPEED, flags = SpaceshipFlags.Enroute });
                }

                fp.arrival.matrix = destination.matrix;

                Vector3D world_position = fp.arrival.matrix.Translation + fp.arrival.matrix.Forward * destination.size * 0.5;
                fp.waypoints.Add(new Flightplan.Waypoint { name = "ARVL", position = world_position + fp.arrival.matrix.Forward * settings.ApproachHeight, speed = 10, flags = SpaceshipFlags.Approach });
                fp.waypoints.Add(new Flightplan.Waypoint { name = "DCK", position = world_position, speed = 1.0f, flags = SpaceshipFlags.Dock });
                return fp;
            }

            public void OnUpdateFrame(string argument, UpdateType updateSource) {
                try {
                    if ((updateSource & UpdateType.IGC) == UpdateType.IGC) {
                        while (listener.HasPendingMessage) {
                            MyIGCMessage message = listener.AcceptMessage();
                            MyTuple<long, string, MatrixD, float> packet = message.As<MyTuple<long, string, MatrixD, float>>();
                            if (destinations.ContainsKey(packet.Item1)) {
                                destinations[packet.Item1].id = packet.Item1;
                                destinations[packet.Item1].name = packet.Item2;
                                destinations[packet.Item1].matrix = packet.Item3;
                                destinations[packet.Item1].size = packet.Item4;
                            } else {
                                destinations.Add(packet.Item1, new Destination { id = packet.Item1, name = packet.Item2, matrix = packet.Item3, size = packet.Item4 });
                            }
                        }
                    }

                    if ((updateSource & UpdateType.Trigger) == UpdateType.Trigger) {
                        HandleMenu(argument);
                    }

                    if ((updateSource & UpdateType.Trigger) == UpdateType.Trigger || (updateSource & UpdateType.Terminal) == UpdateType.Terminal) {
                        if (argument == "Enable")
                            Enable();
                        else if (argument == "Disable")
                            Disable();
                        else if (argument == "Start") {

                        }
                    }

                    if ((updateSource & UpdateType.Update10) == UpdateType.Update10) {
                        OnFlightUpdateFrame();
                    }

                    List<string> online_systems = new List<string>();
                    if ((flags & SpaceshipFlags.AP) == SpaceshipFlags.AP) online_systems.Add("AP");
                    if ((flags & SpaceshipFlags.FD) == SpaceshipFlags.FD) online_systems.Add("FD");
                    if ((flags & SpaceshipFlags.TM) == SpaceshipFlags.TM) online_systems.Add("TM");
                    if ((flags & SpaceshipFlags.CAS) == SpaceshipFlags.CAS) online_systems.Add("CAS");
                    if ((flags & SpaceshipFlags.LookAt) == SpaceshipFlags.LookAt) online_systems.Add("LAT");
                    if ((flags & SpaceshipFlags.LCK) == SpaceshipFlags.LCK) online_systems.Add("LCK");
                    if ((flags & SpaceshipFlags.Alln) == SpaceshipFlags.Alln) online_systems.Add("Alln");

                    debug.WriteText(menu.Draw(this));
                } catch (Exception ex) {
                    debug.WriteText(ex.ToString());
                }

            }

        }

        public static Spaceship spaceship;

        public static T FindBlockOfType<T>(IMyGridTerminalSystem system, Spaceship spaceship, bool use_tag = false) where T : class {
            List<T> temp = new List<T>();
            system.GetBlocksOfType<T>(temp);
            foreach (T block in temp) {
                if ((spaceship.cpu.CubeGrid == ((IMyTerminalBlock)block).CubeGrid) && ((!use_tag) || (use_tag && ((IMyTerminalBlock)block).CustomName.Contains(Program.TAG))))
                    return (T)block;
            }
            return default(T);
        }

        public static List<T> FindBlocksOfType<T>(IMyGridTerminalSystem system, Spaceship spaceship, bool use_tag = false) where T : class {
            List<T> temp = new List<T>();
            List<T> result = new List<T>();
            system.GetBlocksOfType<T>(temp);
            foreach (T block in temp) {
                if ((spaceship.cpu.CubeGrid == ((IMyTerminalBlock)block).CubeGrid) && ((!use_tag) || (use_tag && ((IMyTerminalBlock)block).CustomName.Contains(Program.TAG))))
                    result.Add(block);
            }
            return result;
        }

        public static IMyProgrammableBlock FindRunningPB(IMyGridTerminalSystem system) {
            List<IMyProgrammableBlock> blocks = new List<IMyProgrammableBlock>();
            system.GetBlocksOfType<IMyProgrammableBlock>(blocks);
            foreach (IMyProgrammableBlock block in blocks)
                if (block.IsRunning)
                    return block;
            return null;
        }

        public Program() {
            Runtime.UpdateFrequency = UpdateFrequency.Update10;
            spaceship = new Spaceship(GridTerminalSystem, IGC);
        }

        public void Save() {

        }

        public void Main(string argument, UpdateType updateSource) {
            spaceship.OnUpdateFrame(argument, updateSource);
            Echo(spaceship.menu.stack.Count.ToString());
        }
    }
}