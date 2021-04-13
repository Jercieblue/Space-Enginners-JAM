using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace IngameScript {
    partial class Program {
        partial class Spaceship {

            public class Command {
                public string name = "";
                public Action<string> action;

                public Command(string name, Action<string> action) {
                    this.name = name;
                    this.action = action;
                }

                public bool Validate(string args) {
                    return args.ToLower().Contains(name.ToLower());
                }

                public bool Execute(string args) {
                    action(args.Replace(name, "").Trim());
                    return true;
                }
            }

            public class Commands {
                readonly List<Command> commands = new List<Command>();

                public Commands() {
                    commands.Add(new Command("Start", s => { spaceship.Enable(); }));
                    commands.Add(new Command("Stop", s => { spaceship.Disable(); }));
                    commands.Add(new Command("Go", s => { spaceship.Go(s); }));
                    commands.Add(new Command("Reset Flightplan", s => {
                        spaceship.tasks.active_task = 0;
                        spaceship.tasks.state = TaskManager.TaskManagerState.Navigation;
                        spaceship.tasks.next_state = spaceship.tasks.state;
                    }));
                    commands.Add(new Command("Next Task", s => {
                        spaceship.tasks.active_task = (spaceship.tasks.active_task + 1) % spaceship.tasks.tasks.Count;
                        spaceship.tasks.state = TaskManager.TaskManagerState.Navigation;
                        spaceship.tasks.next_state = spaceship.tasks.state;
                    }));
                }

                public bool Execute(string input) {
                    foreach(Command cmd in commands) {
                        if (cmd.Validate(input)) {
                            cmd.Execute(input);
                            return true;
                        } 
                    }
                    return false;
                }
            }
        }
    }
}
