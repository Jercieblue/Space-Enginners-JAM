using Sandbox.ModAPI.Ingame;
using System.Collections.Generic;

namespace IngameScript {
    partial class Program {

        public partial class Spaceship {
            public class TaskManager : ISerializable {
                public enum TaskType {
                    Navigation, Load, Unload, Charge, Repeat, Total
                }
                public static string[] TaskStringTypes = { "Navigation", "Load", "Unload", "Charge", "Repeat", "Total" };
                public static string[] TaskManagerStringStates2 = { "Navigation", "Travelling", "Action", "NextTask" };
                public enum TaskManagerState {
                    Navigation, Travelling, Action, NextTask
                }
                public class Task : ISerializable {
                    public int type = 0;
                    public long destination = 0;

                    public void Serialize(MemoryStream ms) {
                        ms.Write(type);
                        ms.Write(destination);
                    }

                    public void Deserialize(MemoryStream ms) {
                        int t; ms.ReadInt(out t); type = t;
                        ms.ReadLong(out destination);
                    }

                public override string ToString() {
                    return string.Format("[{0}]{1}",
                        TaskManager.TaskStringTypes[type], destination != 0 ? spaceship.destinations[destination].ToString() : "No Destination");
                    }
                }
    
                public List<Task> tasks = new List<Task>();
                public int active_task = 0;
                public TaskManagerState state = TaskManagerState.Navigation;
                public TaskManagerState next_state = TaskManagerState.Navigation;

                public void OnUpdateFrame() {
                    state = next_state;
                    switch (state) {
                        case TaskManagerState.Navigation:
                            if (tasks[active_task].destination != 0) {
                                spaceship.fd.SetFlightPlan(spaceship.GenerateFlightPlan(spaceship.destinations[tasks[active_task].destination]));
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
                            switch ((TaskType)tasks[active_task].type) {
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

                public void Serialize(MemoryStream ms) {
                    ms.Write(tasks.Count);
                    foreach (Task task in tasks) {
                        task.Serialize(ms);
                    }
                    ms.Write(active_task);
                    ms.Write((int)state);
                    ms.Write((int)next_state);
                }

                public void Deserialize(MemoryStream ms) {
                    int tasks_count;
                    ms.ReadInt(out tasks_count);
                    for (int i = 0; i < tasks_count; i++) {
                        var task = new Task();
                        task.Deserialize(ms);
                        tasks.Add(task);
                    }
                    ms.ReadInt(out active_task);
                    spaceship.debug.WriteText(active_task.ToString()+"\n");
                            

                    int temp_state; ms.ReadInt(out temp_state); state = (TaskManagerState)temp_state;
                    int temp_next_state; ms.ReadInt(out temp_next_state); next_state = (TaskManagerState)temp_next_state;
                }
            }
        }
    }
}