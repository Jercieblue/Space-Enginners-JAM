using Sandbox.ModAPI.Ingame;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;


namespace IngameScript {
    partial class Program {
        
        public static T FindBlockOfType<T>(IMyGridTerminalSystem system, Spaceship spaceship, string tag = "") where T : class {
            List<T> temp = new List<T>();
            system.GetBlocksOfType<T>(temp);
            foreach (T block in temp) {
                if ((spaceship.cpu.CubeGrid == ((IMyTerminalBlock)block).CubeGrid) && ((tag.Length == 0) || ((IMyTerminalBlock)block).CustomName.Contains(tag)))
                    return (T)block;
            }
            return default(T);
        }
        public static List<T> FindBlocksOfType<T>(IMyGridTerminalSystem system, Spaceship spaceship, string tag = "") where T : class {
            List<T> temp = new List<T>();
            List<T> result = new List<T>();
            system.GetBlocksOfType<T>(temp);
            foreach (T block in temp) {
                if ((spaceship.cpu.CubeGrid == ((IMyTerminalBlock)block).CubeGrid) && ((tag.Length == 0) || ((IMyTerminalBlock)block).CustomName.Contains(tag)))
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
    }
    
}
