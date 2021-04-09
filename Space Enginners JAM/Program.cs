using Sandbox.ModAPI.Ingame;
using System;
using System.Collections.Generic;
using System.Linq;
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