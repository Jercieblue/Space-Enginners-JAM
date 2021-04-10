using System;
using System.Collections.Generic;
using System.Linq;

namespace IngameScript {
    partial class Program {
        public partial class Spaceship {

            public enum MenuCommand {
                Prev, Next, Select, Return, Insert, Delete, Invalid
            }
            public static Dictionary<string, MenuCommand> StringCommandsToMenuLookUp = new Dictionary<string, MenuCommand> {
                {"Prev", MenuCommand.Prev }, {"Next",MenuCommand.Next }, {"Select", MenuCommand.Select}, {"Return", MenuCommand.Return}, 
                {"Insert", MenuCommand.Insert}, {"Delete", MenuCommand.Delete}
            };

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

                public string DrawTextProgressBar(float v, int size) {
                    string result = "";
                    int iv = (int)Math.Round(v * size);
                    for (int i = 0; i < size; i++)
                        if (i < iv)
                            result += "▓";
                        else
                            result += "░";

                    return result;
                }

                public override string Draw() {
                    string result = base.Draw();

                    List<string> online_systems = new List<string>();
                    if ((spaceship.flags & SpaceshipFlags.AP) == SpaceshipFlags.AP) online_systems.Add("AP");
                    if ((spaceship.flags & SpaceshipFlags.FD) == SpaceshipFlags.FD) online_systems.Add("FD");
                    if ((spaceship.flags & SpaceshipFlags.TM) == SpaceshipFlags.TM) online_systems.Add("TM");
                    if ((spaceship.flags & SpaceshipFlags.CAS) == SpaceshipFlags.CAS) online_systems.Add("CAS");
                    if ((spaceship.flags & SpaceshipFlags.LookAt) == SpaceshipFlags.LookAt) online_systems.Add("LAT");
                    if ((spaceship.flags & SpaceshipFlags.LCK) == SpaceshipFlags.LCK) online_systems.Add("LCK");
                    if ((spaceship.flags & SpaceshipFlags.Alln) == SpaceshipFlags.Alln) online_systems.Add("ALN");
                    result += string.Join(" ", online_systems) + "\n";
                    result += string.Format("Task:{0}\nState: {1}\n", spaceship.tasks.active_task > -1 && spaceship.tasks.active_task < spaceship.tasks.tasks.Count ? spaceship.tasks.tasks[spaceship.tasks.active_task].ToString() : "", TaskManager.TaskManagerStringStates2[(int)spaceship.tasks.state]);
                    if (spaceship.fd.flightplan != null) {
                        for (int i = 0; i < spaceship.fd.flightplan.waypoints.Count(); i++)
                            result += string.Format("{0}{1} ({2})\n", i == spaceship.fd.prev ? "╔" : i == spaceship.fd.next ? "╚" : " ", spaceship.fd.flightplan.waypoints[i].name, spaceship.fd.flightplan.waypoints[i].speed);
                    }
                    result += string.Format("{0:0.00}%, Speed: {1}\n", spaceship.fd.p * 100, spaceship.fd.target_speed);
                    result += DrawTextProgressBar((float)spaceship.fd.p, 20) + "\n";
                    result += string.Format("Breaking: {0:0.00}\n", effective_breaking_distance);
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
                        task.destination = spaceship.destinations.Values.ElementAt(index).id;
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
                        sub.Add(new Menu("Task: " + TaskManager.TaskStringTypes[task.type]));
                        sub.Add(new Menu("Destination: " + (task.destination != 0 ? spaceship.destinations[task.destination].ToString() : "No Destination")));
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
                                task.type = (task.type + 1) % TaskManager.TaskStringTypes.Length;
                            } else if (index == 1) {
                                stack.Push(new MenuDestinationPicker(task));
                            }
                            return true;
                        } else {
                            return base.Interact(command, ref stack, args);
                        }
                    }

                    public override string ToString() {
                        return string.Format("({0}){1}", TaskManager.TaskStringTypes[task.type], task.destination != 0 ? spaceship.destinations[task.destination].ToString() : "No Destination");
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

            public class MenuSystem : MenuList {
                public MenuSystem() : base("System") {
                    sub.Add(new Menu("Start Flightplan"));
                }

                public override bool Interact(MenuCommand command, ref Stack<IMenu> stack, params string[] args) {
                    if (command == MenuCommand.Select) {
                        if (index == 0) {
                            spaceship.tasks.active_task = 0;
                            spaceship.tasks.state = TaskManager.TaskManagerState.Navigation;
                            spaceship.tasks.next_state = TaskManager.TaskManagerState.Navigation;
                        }
                        return true;
                    } else {
                        return base.Interact(command, ref stack, args);
                    }
                }
            }

            public class MenuThrusters : MenuList {
                public MenuThrusters() : base("Thrusters") {

                }

                public override string Draw() {
                    sub.Clear();
                    for (int i = 0; i < 6; i++)
                        sub.Add(new Menu(string.Format("{0}: {1}", Enum.GetNames(typeof(ManagedThrusterDirectionType))[i], ThrustersManager.max_thrust[i])));
                    return base.Draw();
                }

            }

        }
    }
}