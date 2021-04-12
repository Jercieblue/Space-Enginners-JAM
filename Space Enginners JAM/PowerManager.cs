using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Sandbox.ModAPI.Ingame;

namespace IngameScript {
    partial class Program {
        public partial class Spaceship {
            public class PowerManager : ISpaceshipComponent {
                
                public float baterries = 0.0f;

                public void OnUpdateFrame() {
                    baterries = 0.0f;
                    int counter = 0;
                    foreach (IMyBatteryBlock block in FindBlocksOfType<IMyBatteryBlock>(spaceship.system, spaceship)) {
                        baterries += block.CurrentStoredPower / block.MaxStoredPower;
                        counter++;
                    }

                    if (counter > 0)
                        baterries /= counter;
                }

                public void Disable() {
                    
                }

                public void Enable() {
                    
                }
            }
        }
    }
    
}
