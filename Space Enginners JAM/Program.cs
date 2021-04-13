using Sandbox.ModAPI.Ingame;
using System;
using System.Collections.Generic;
using System.Linq;

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

        public static Spaceship spaceship;

        #region Diafores Xazomares
            
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
                spaceship.OnUpdateFrame(argument, updateSource);
            } catch (Exception ex) {
                spaceship.debug.WriteText(ex.ToString(), true);
                throw;
            }
            
        }
    }
}