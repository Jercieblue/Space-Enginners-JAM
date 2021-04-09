using Sandbox.ModAPI.Ingame;
using System;
using System.Collections.Generic;
using System.Linq;
using VRage;
using VRage.Game.GUI.TextPanel;
using VRageMath;

/*
        static string CHANNEL = "JAM_CH_1";
        static string TAG = "JAM";

        public class Station {
            IMyGridTerminalSystem system;
            IMyIntergridCommunicationSystem igc;
            IMyProgrammableBlock cpu;
            IMyTextSurface screen2;

            public Station(IMyGridTerminalSystem system, IMyIntergridCommunicationSystem igc) {
                this.system = system;
                this.igc = igc;
                List<IMyProgrammableBlock> blocks = new List<IMyProgrammableBlock>();
                system.GetBlocksOfType<IMyProgrammableBlock>(blocks);
                foreach (IMyProgrammableBlock block in blocks)
                    if (block.IsRunning)
                        cpu = block;

                screen2 = cpu.GetSurface(0);
                screen2.ContentType = ContentType.TEXT_AND_IMAGE;
            }

            void TransmitConnector(IMyShipConnector connector) {
                var packet = new MyTuple<long, string, MatrixD, float>(connector.EntityId, connector.CustomName, connector.WorldMatrix, connector.CubeGrid.GridSize);
                screen2.WriteText(connector.CubeGrid.GridSize.ToString(), true);
                igc.SendBroadcastMessage(CHANNEL, packet);
            }

            void TransmitDocks() {
                List<IMyShipConnector> connectors = new List<IMyShipConnector>();
                system.GetBlocksOfType<IMyShipConnector>(connectors);
                foreach (IMyShipConnector connector in connectors) {
                    if (connector.CustomName.Contains(TAG))
                        TransmitConnector(connector);
                }
            }

            public void OnUpdateFrame(string argument, UpdateType updateSource) {
                screen2.WriteText("");
                if (updateSource == UpdateType.Update100) {
                    TransmitDocks();
                }
            }
        }

        Station station;

        public Program() {
            Runtime.UpdateFrequency = UpdateFrequency.Update100;
            station = new Station(GridTerminalSystem, IGC);
        }

        public void Save() {

        }

        public void Main(string argument, UpdateType updateSource) {
            station.OnUpdateFrame(argument, updateSource);
        }
 */


namespace IngameScript {
    partial class Program : MyGridProgram {
        public class Settings {
            public const string TAG = "JAM";
            public const bool AutoStartThrusters = true;
            public const float CASDistanceExtension = 1.0f;
            public const float MinRayDist = 0.0f;
            public const float MinThrustEffectivness = 0.30f; // When the effectivness of the thruster drops below gets deactivated
            public const float ApproachHeight = 100.0f;
            public const float APPROACH_SPEED = 5.0f;
            public const float DEPARTURE_SPEED = 10.0f;
            public const float DOCKING_SPEED = 1.0f;
            public const float MAX_SPEED = 95.0f;
            public const float ARRIVAL_DIST = 1000.0f;
            public const float MIN_CHECK_DIST = 1e-1f;
            public const float MAX_SCAN_DIST = 500.0f;  // The max scan range for the radar to avoid obsticles
            public const float SCAN_ANGLE = 15.0f;
            public const string COMM_CHANNEL = "JAM_CH_1";
            public const float EPSILON = 1e-3f;
            public const float MIN_LOOKAT_DIST = 10.0f;
            public const int STORAGE_VER = 112;
            public static Vector3D v_epsilon = new Vector3D(EPSILON);
            public static Vector3D[] ManagedThrusterDirections = { -Vector3D.UnitZ, Vector3D.UnitZ, Vector3D.UnitY, -Vector3D.UnitY, -Vector3D.UnitX, Vector3D.UnitX };
        }
        public interface ISerializable {
            void Serialize(MemoryStream ms);
            void Deserialize(MemoryStream ms);
        }

        public partial class Spaceship : ISerializable {
            public enum ManagedThrusterDirectionType { Forward, Backward, Up, Down, Left, Right, Invalid }
            public enum SpaceshipFlags : long {
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
            
            public interface ISpaceshipComponent {
                void Enable();
                void Disable();
            }

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

            public IMyGridTerminalSystem system;
            public IMyIntergridCommunicationSystem igc;
            public IMyProgrammableBlock cpu;
            public IMyRemoteControl control;
            public IMyShipConnector connector;
            public IMyTextSurface debug;
            public IMyBroadcastListener listener;

            public Dictionary<long, Destination> destinations = new Dictionary<long, Destination>();

            public static ThrustersManager thrusters;
            public Autopilot autopilot;
            public FlightDirector fd;
            public CollisionAvoidanceSystem cas;
            public TaskManager tasks = new TaskManager();
            public MenuStack menu = new MenuStack(
                    new MenuList("Main Menu",
                        new MenuFMC(), new MenuTasks(), new MenuSystem(), new MenuThrusters())
            );

            public Vector3D velocity = Vector3D.Zero;
            public Vector3D gravity = Vector3D.Zero;
         
            public static float effective_breaking_distance = 0.0f;
            public static MyShipMass ship_mass;
            public Vector3D desired = Vector3D.Zero;
            private bool is_enabled = false;
            public SpaceshipFlags flags = SpaceshipFlags.Dock;



            public Spaceship(IMyGridTerminalSystem system, IMyIntergridCommunicationSystem igc) {
                this.system = system;
                this.igc = igc;

                cpu = FindRunningPB(system);

                control = FindBlockOfType<IMyRemoteControl>(system, this);
                connector = FindBlockOfType<IMyShipConnector>(system, this);
                debug = FindBlockOfType<IMyTextPanel>(system, this);
                debug.Font = "Monospace";
                debug.ContentType = ContentType.TEXT_AND_IMAGE;
                    
            /*
                screen = cockpit.GetSurface(0);
                screen.ContentType = ContentType.SCRIPT;
                MySpriteDrawFrame frame = screen.DrawFrame();
                

                frame.Add(new MySprite(SpriteType.TEXTURE, "SquareSimple", new Vector2(0, 0), new Vector2(512, 512), Color.Black));
                frame.Add(MySprite.CreateText("Ftiaxe Me!!!", "DEBUG", Color.White));
                frame.Dispose();
            */

                listener = igc.RegisterBroadcastListener(Settings.COMM_CHANNEL);
                listener = igc.RegisterBroadcastListener(Settings.COMM_CHANNEL);
                listener.SetMessageCallback(Settings.COMM_CHANNEL);
                thrusters = new ThrustersManager(this, control, system);

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
                    ship_mass = control.CalculateShipMass();
                    effective_breaking_distance = Settings.ARRIVAL_DIST * (ship_mass.TotalMass / ship_mass.BaseMass);

                    tasks.OnUpdateFrame();

                    if ((flags & SpaceshipFlags.CAS) == SpaceshipFlags.CAS) cas.OnUpdateFrame();
                    if ((flags & SpaceshipFlags.FD) == SpaceshipFlags.FD) fd.OnUpdateFrame();

                    Vector3DLimit(ref desired, Settings.MAX_SPEED);

                    if ((flags & SpaceshipFlags.AP) == SpaceshipFlags.AP) autopilot.OnUpdateFrame();

                    if ((flags & SpaceshipFlags.TM) == SpaceshipFlags.TM) {
                        thrusters.OnUpdateFrame();
                    } else {
                        thrusters.Disable();    
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
                thrusters.Enable();
                    
                control.DampenersOverride = false;

            }
            public void Disable() {
                cas.Disable();
                fd.Disable();
                autopilot.Disable();
                thrusters.Disable();
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
                    fp.waypoints.Add(new Flightplan.Waypoint { name = "UDCK", position = connector.WorldMatrix.Translation, speed = Settings.DEPARTURE_SPEED, flags = SpaceshipFlags.Undock });
                    fp.waypoints.Add(new Flightplan.Waypoint { name = "DEPT", position = connector.WorldMatrix.Translation + connector.OtherConnector.WorldMatrix.Forward * Settings.ApproachHeight, speed = 10, flags = SpaceshipFlags.Enroute });
                } else {
                    fp.waypoints.Add(new Flightplan.Waypoint { name = "WPT", position = connector.WorldMatrix.Translation, speed = Settings.MAX_SPEED, flags = SpaceshipFlags.Enroute });
                }

                fp.arrival.matrix = destination.matrix;

                Vector3D world_position = fp.arrival.matrix.Translation + fp.arrival.matrix.Forward * destination.size * 0.5;
                fp.waypoints.Add(new Flightplan.Waypoint { name = "ARVL", position = world_position + fp.arrival.matrix.Forward * Settings.ApproachHeight, speed = Settings.APPROACH_SPEED, flags = SpaceshipFlags.Approach });
                fp.waypoints.Add(new Flightplan.Waypoint { name = "DCK", position = world_position, speed = Settings.DOCKING_SPEED, flags = SpaceshipFlags.Dock });
                return fp;
            }
            public void Go(string location) {
                foreach(Destination destination in destinations.Values)
                    if (destination.name.ToLower() == location.ToLower()) {
                        tasks.tasks.Clear();
                        TaskManager.Task custom_task = new TaskManager.Task();
                        custom_task.destination = destination.id;
                        custom_task.type = TaskManager.TaskType.Navigation;
                        tasks.tasks.Add(custom_task);
                        tasks.active_task = 0;
                        tasks.state = TaskManager.TaskManagerState.Navigation;
                        tasks.next_state = TaskManager.TaskManagerState.Navigation;
                        Enable();
                        return;
                    }

                    Destination custom = Destination.FromGPS(location);
                    custom.id = -981723;
                    if (destinations.ContainsKey(custom.id))
                        destinations[custom.id] = custom;
                    else
                        destinations.Add(custom.id, custom);

                    tasks.tasks.Clear();
                    TaskManager.Task task = new TaskManager.Task();
                    task.destination = custom.id;
                    task.type = TaskManager.TaskType.Navigation;
                    tasks.tasks.Add(task);
                    tasks.active_task = 0;
                    tasks.state = TaskManager.TaskManagerState.Navigation;
                    tasks.next_state = TaskManager.TaskManagerState.Navigation;
                    Enable();
            }

            public void OnUpdateFrame(string[] arguments, UpdateType updateSource) {
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
                    HandleMenu(arguments[0]);
                }

                if ((updateSource & UpdateType.Trigger) == UpdateType.Trigger || (updateSource & UpdateType.Terminal) == UpdateType.Terminal) {
                    if (arguments[0] == "Enable")
                        Enable();
                    else if (arguments[0] == "Disable")
                        Disable();
                    else if (arguments[0] == "Go" && arguments.Length == 2) {
                        Go(arguments[1]);
                    }
                }

                if ((updateSource & UpdateType.Update10) == UpdateType.Update10) {
                    OnFlightUpdateFrame();
                }

                debug.WriteText(menu.Draw(this));
                    
            }

            public void Serialize(MemoryStream ms) {
                debug.WriteText("Writing\n");
                    
                ms.Write(destinations.Count);
                debug.WriteText(destinations.Count.ToString() + "\n");
                foreach (Destination destination in destinations.Values) {
                    debug.WriteText(string.Format("DestinationID {0}\n", destination.id), true);
                    destination.Serialize(ms);
                }   
                    
                tasks.Serialize(ms);
                fd.Serialize(ms);
                ms.Write((long)flags);
                ms.Write(is_enabled);
            }

            public void Deserialize(MemoryStream ms) {
                debug.WriteText("Reading\n");
                int destinations_total;
                ms.ReadInt(out destinations_total);
                debug.WriteText(destinations_total.ToString() +"\n");
                for(int i = 0; i < destinations_total; i++) {
                    var destination = new Destination();
                    destination.Deserialize(ms);
                    debug.WriteText(string.Format("DestinationID {0}\n",destination.id), true);
                    destinations.Add(destination.id, destination);
                }
                tasks.Deserialize(ms);
                fd.Deserialize(ms);
                long temp_flags; ms.ReadLong(out temp_flags); flags = (SpaceshipFlags)temp_flags;
                bool temp_is_enabled; ms.ReadBool(out temp_is_enabled);if (temp_is_enabled) Enable();
            }
        }

        public static Spaceship spaceship;

        #region Diafores Xazomares
            public static T FindBlockOfType<T>(IMyGridTerminalSystem system, Spaceship spaceship, bool use_tag = false) where T : class {
                List<T> temp = new List<T>();
                system.GetBlocksOfType<T>(temp);
                foreach (T block in temp) {
                    if ((spaceship.cpu.CubeGrid == ((IMyTerminalBlock)block).CubeGrid) && ((!use_tag) || (use_tag && ((IMyTerminalBlock)block).CustomName.Contains(Settings.TAG))))
                        return (T)block;
                }
                return default(T);
            }
            public static List<T> FindBlocksOfType<T>(IMyGridTerminalSystem system, Spaceship spaceship, bool use_tag = false) where T : class {
                List<T> temp = new List<T>();
                List<T> result = new List<T>();
                system.GetBlocksOfType<T>(temp);
                foreach (T block in temp) {
                    if ((spaceship.cpu.CubeGrid == ((IMyTerminalBlock)block).CubeGrid) && ((!use_tag) || (use_tag && ((IMyTerminalBlock)block).CustomName.Contains(Settings.TAG))))
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
        #endregion

        public Program() {
            try {
                Runtime.UpdateFrequency = UpdateFrequency.Update10;
                spaceship = new Spaceship(GridTerminalSystem, IGC);
                if (Storage.Length > 0) {
                    using (var ms = new MemoryStream(Storage)) {
                        int storage_ver;
                        ms.ReadInt(out storage_ver);
                        if (storage_ver == Settings.STORAGE_VER) {
                            spaceship.Deserialize(ms);
                        }
                    }
                }
            } catch (Exception ex) {
                spaceship.debug.WriteText(ex.ToString(), true);
                throw;
            }
        }

        public void Console(string str) {
            Echo(str);
        }

        public void Save() {
            try {
                using (var ms = new MemoryStream()) {
                    ms.Write(Settings.STORAGE_VER);
                    spaceship.Serialize(ms);
                    Storage = ms.ToString();
                }
            } catch (Exception ex) {
                spaceship.debug.WriteText(ex.ToString(), true);
                throw;
            }
        }

        public void Main(string argument, UpdateType updateSource) {
            try {
                spaceship.OnUpdateFrame(argument.Split(' ').ToArray(), updateSource);
            } catch (Exception ex) {
                spaceship.debug.WriteText(ex.ToString(), true);
                throw;
            }
            
        }
    }
}