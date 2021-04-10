using System;
using System.Linq;
using VRageMath;

namespace IngameScript {
    partial class Program {
        public partial class Spaceship {
            public class FlightDirector : ISpaceshipComponent, ISerializable {

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

                public Vector3D Enroute(Flightplan.Waypoint prev, Flightplan.Waypoint next, Vector3D position, Vector3D velocity) {
                    Vector3D to_spaceship = position - prev.position;
                    Vector3D to_nextwpt = next.position - prev.position;
                    //Vector3D position_on_route = Vector3D.ProjectOnVector(ref to_spaceship, ref to_nextwpt);
                    Vector3D result = next.position - position;
                    //result += position_on_route - position;
                    
                    double distance_between_wpts = Vector3D.Distance(prev.position, next.position);
                    double distance_from_prev = Vector3D.Distance(prev.position, position);
                    double distance_to_next = Vector3D.Distance(next.position, position);
                    float arrival_dist = effective_breaking_distance;
                    p = Math.Max(0.0, Math.Min(1.0, distance_from_prev / distance_between_wpts));
                    if (distance_between_wpts > arrival_dist) {
                        if (distance_to_next > arrival_dist)
                            target_speed = Settings.MAX_SPEED;
                        else
                            target_speed = Remap(distance_to_next, arrival_dist, 0.0, Settings.MAX_SPEED, next.speed);
                    } else {
                        target_speed = Lerp(prev.speed, next.speed, p);
                    }
                    Vector3DLimit(ref result, (float)target_speed);
                    return result.Length() > Settings.EPSILON ? result : Vector3D.Zero;
                }

                public void OnUpdateFrame() {
                    if ((prev != next) && (flightplan != null)) {

                        if (Vector3D.Distance(flightplan.waypoints[next].position, spaceship.GetPosition()) < Settings.MIN_CHECK_DIST)
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

                public void Serialize(MemoryStream ms) {
                    if (flightplan != null) {
                        ms.Write(true);
                        flightplan.Serialize(ms);
                    } else {
                        ms.Write(false);
                    }
                    ms.Write(prev);
                    ms.Write(next);
                    ms.Write(active_waypoint);
                }

                public void Deserialize(MemoryStream ms) {
                    bool has_flightplan; ms.ReadBool(out has_flightplan);
                    if (has_flightplan) {
                        flightplan = new Flightplan();
                        flightplan.Deserialize(ms);
                    }
                    ms.ReadInt(out prev);
                    ms.ReadInt(out next);
                    ms.ReadInt(out active_waypoint);
                }
            }
        }
    }
}