using Sandbox.ModAPI.Ingame;
using System;
using System.Collections.Generic;

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
    partial class Program {

        #region Serializer

        #endregion

        public partial class Spaceship {
            public class CollisionAvoidanceSystem : ISpaceshipComponent {
                Spaceship spaceship;
                List<Sensor> sensors = new List<Sensor>();

                public CollisionAvoidanceSystem(Spaceship spaceship) {
                    this.spaceship = spaceship;

                    foreach (IMyCameraBlock camera in Program.FindBlocksOfType<IMyCameraBlock>(spaceship.system, spaceship, true)) {
                        sensors.Add(new Sensor(camera));
                    }
                }

                public void OnUpdateFrame() {
                    foreach (Sensor sensor in sensors) {
                        sensor.range = Math.Max(Settings.MinRayDist, Settings.MAX_SCAN_DIST * ((float)spaceship.velocity.Length() / 100.0f));
                        sensor.OnUpdateFrame();
                        spaceship.desired += -sensor.direction * (1.0f - sensor.value) * Settings.MAX_SPEED;
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
        }
    }
}