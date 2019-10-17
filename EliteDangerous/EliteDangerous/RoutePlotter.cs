/*
 * Copyright © 2019 EDDiscovery development team
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not use this
 * file except in compliance with the License. You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software distributed under
 * the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF
 * ANY KIND, either express or implied. See the License for the specific language
 * governing permissions and limitations under the License.
 * 
 * EDDiscovery is not affiliated with Frontier Developments plc.
 */

using EMK.LightGeometry;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using BaseUtils;
using EliteDangerousCore.DB;
using EMK.Cartography;

namespace EliteDangerousCore
{
    public class RoutePlotter
    {
        public float MaxRange;
        public Point3D Coordsfrom;
        public Point3D Coordsto;
        public string FromSystem;
        public string ToSystem;
        public Metric RouteMethod;
        public bool UseFsdBoost;
        private AStar<ISystem> aStarAlgortihm;
        private bool stopPlotter = false;

        public bool StopPlotter
        {
            get => stopPlotter;
            set
            {
                stopPlotter = value;
                if (aStarAlgortihm != null)
                    aStarAlgortihm.CancelPathSearch = value;
            }
        }

        private static readonly string[] MetricNames = {
            "Nearest to Waypoint",
            "Minimum Deviation from Path",
            "Nearest to Waypoint with dev<=100ly",
            "Nearest to Waypoint with dev<=250ly",
            "Nearest to Waypoint with dev<=500ly",
            "A* (VERY slow) average. Finds best path!",
            "A* (VERY slow) boosts have high penalty",
            "A* (VERY slow) boosts have little penalty",
            "A* (EXTREMELY slow) economic mode",
        };

        /// <summary>
        /// Returns the string representation of a Metric enum value.
        /// </summary>
        /// <exception cref="ArgumentOutOfRangeException">The int representation of the variable metric has to be smaller than the length of the static MetricNames array.</exception>
        /// <param name="metric">The metric enum value.</param>
        public static string GetMetricName(Metric metric)
        {
            Debug.Assert(Enum.GetNames(typeof(Metric)).Length == MetricNames.Length);
            if ((int) metric >= MetricNames.Length)
                throw new ArgumentOutOfRangeException(nameof(metric));

            return MetricNames[(int) metric];
        }

        public enum Metric
        {
            IterativeNearestWaypoint,
            IterativeMinDevFromPath,
            IterativeMaximumDev100Ly,
            IterativeMaximumDev250Ly,
            IterativeMaximumDev500Ly,
            IterativeWaypointDevHalf,
            AstarStandard,         // A* standard with medium weight for boosts
            AstarAvoidBoosts,      // A* with high penalty for boosts
            AstarCheapBoosts,      // A* with very little penality for boosts
            AstarEconomic,         // A* with penalty for longer jumps
        }
        private const int MaxRouteMethodIterative = (int) Metric.IterativeWaypointDevHalf;    // The maximal index of metric_options which still is using the iterative route algorithm

        public class ReturnInfo
        {
            public string name;             // always
            public double dist;             // can be Nan
            public Point3D pos;             // 3dpos can be null
            public double waypointdist;     // can be Nan
            public double deviation;        // can be Nan
            public ISystem system;          // only if its a real system

            public ReturnInfo(string s, double d = double.NaN, Point3D p = null, double way = double.NaN, double dev = double.NaN, ISystem sys = null)
            {
                name = s;dist = d;pos = p;waypointdist = way;deviation = dev; system = sys;
            }
        }

        public List<ISystem> RouteHandler(Action<ReturnInfo> info)
        {
            if ((int)RouteMethod <= MaxRouteMethodIterative)
                return RouteIterative(info);
            if (RouteMethod == Metric.AstarStandard ||
                RouteMethod == Metric.AstarEconomic ||
                RouteMethod == Metric.AstarAvoidBoosts ||
                RouteMethod == Metric.AstarCheapBoosts)
                return RouteAstar(info);

            throw new NotSupportedException();
        }

        public List<ISystem> RouteIterative(Action<ReturnInfo> info)
        {
            double traveldistance = Point3D.DistanceBetween(Coordsfrom, Coordsto);      // its based on a percentage of the traveldistance
            List<ISystem> routeSystems = new List<ISystem>();
            System.Diagnostics.Debug.WriteLine("From " + FromSystem + " to  " + ToSystem + ", using metric " + GetMetricName(RouteMethod));

            routeSystems.Add(new SystemClass(FromSystem, Coordsfrom.X, Coordsfrom.Y, Coordsfrom.Z));

            info(new ReturnInfo(FromSystem, double.NaN, Coordsfrom,double.NaN,double.NaN,routeSystems[0]));

            Point3D curpos = Coordsfrom;
            int jump = 1;
            double actualdistance = 0;

            float maxfromwanted = (MaxRange<100) ? (MaxRange-1) : (100+MaxRange * 1 / 5);       // if <100, then just make sure we jump off by 1 yr, else its a 100+1/5
            maxfromwanted = Math.Min(maxfromwanted, MaxRange - 1);

            do
            {
                double distancetogo = Point3D.DistanceBetween(Coordsto, curpos);      // to go

                if (distancetogo <= MaxRange)                                         // within distance, we can go directly
                    break;

                Point3D travelvector = new Point3D(Coordsto.X - curpos.X, Coordsto.Y - curpos.Y, Coordsto.Z - curpos.Z); // vector to destination
                Point3D travelvectorperly = new Point3D(travelvector.X / distancetogo, travelvector.Y / distancetogo, travelvector.Z / distancetogo); // per ly travel vector

                Point3D expectedNextPosition = GetNextPosition(curpos, travelvectorperly, MaxRange);    // where we would like to be..
                ISystem bestsystem = GetBestJumpSystem(curpos, travelvectorperly, maxfromwanted, MaxRange);    // see if we can find a system near  our target

                // if we haven't found a system in range, let's try boosting
                int boostStrength = 0;
                while (UseFsdBoost && bestsystem == null && boostStrength < 4)
                {
                    boostStrength = 1 << boostStrength;
                    float maxRangeWithBoost = MaxRange * (1.0f + BoostPercentage(boostStrength));
                    ISystem bestSystemWithBoost = GetBestJumpSystem(curpos, travelvectorperly, maxfromwanted, maxRangeWithBoost);

                    if (bestSystemWithBoost != null)
                        bestsystem = bestSystemWithBoost;
                }

                Point3D nextpos = expectedNextPosition;    // where we really are going to be
                string sysname = "WAYPOINT";
                double deltafromwaypoint = 0;
                double deviation = 0;

                if (bestsystem != null)
                {
                    nextpos = new Point3D(bestsystem.X, bestsystem.Y, bestsystem.Z);
                    deltafromwaypoint = Point3D.DistanceBetween(nextpos, expectedNextPosition);     // how much in error
                    deviation = Point3D.DistanceBetween(curpos.InterceptPoint(expectedNextPosition, nextpos), nextpos);
                    sysname = bestsystem.Name;
                    if (boostStrength > 0)
                        sysname += " (+" + BoostPercentage(boostStrength) * 100 + "% Boost)";
                    routeSystems.Add(bestsystem);
                }

                info(new ReturnInfo(sysname, Point3D.DistanceBetween(curpos, nextpos), nextpos, deltafromwaypoint, deviation , bestsystem));

                actualdistance += Point3D.DistanceBetween(curpos, nextpos);
                curpos = nextpos;
                jump++;

            } while ( !StopPlotter);

            routeSystems.Add(new SystemClass(ToSystem, Coordsto.X, Coordsto.Y, Coordsto.Z));

            actualdistance += Point3D.DistanceBetween(curpos, Coordsto);

            info(new ReturnInfo(ToSystem, Point3D.DistanceBetween(curpos, Coordsto), Coordsto, double.NaN, double.NaN, routeSystems.Last()));

            info(new ReturnInfo("Straight Line Distance", traveldistance));
            info(new ReturnInfo("Travelled Distance", actualdistance));

            return routeSystems;
        }

        private ISystem GetBestJumpSystem(Point3D currentPosition, Point3D travelVectorPerLy, float maxDistanceFromWanted, float maxRange)
        {
            Point3D nextPosition = GetNextPosition(currentPosition, travelVectorPerLy, maxRange);
            ISystem bestSystem = DB.SystemCache.GetSystemNearestTo(currentPosition, nextPosition, maxRange, maxDistanceFromWanted, RouteMethod, 1000);  // at least get 1/4 way there, otherwise waypoint.  Best 1000 from waypoint checked
            return bestSystem;
        }

        private static Point3D GetNextPosition(Point3D currentPosition, Point3D travelVectorPerLy, float maxRange)
        {
            return new Point3D(currentPosition.X + maxRange * travelVectorPerLy.X,
                currentPosition.Y + maxRange * travelVectorPerLy.Y,
                currentPosition.Z + maxRange * travelVectorPerLy.Z); // where we would like to be..
        }

        private static float BoostPercentage(int boostStrength)
        {
            return boostStrength / 4.0f;
        }

        private static float BoostPercentage(float jumpedDistance, float maxiumJumpRange)
        {
            float exactboost = jumpedDistance / maxiumJumpRange;
            if (exactboost <= 1f)
                return 0;
            if (exactboost <= 1.25f)
                return .25f;
            if (exactboost <= 1.5f)
                return .50f;
            if (exactboost <= 2.0f)
                return 1;

            throw new ArgumentOutOfRangeException();
        }
        
        // AStar method, for large jumps with large max ranges, or over long distances, it gets bogged down.

        public List<ISystem> RouteAstar(Action<ReturnInfo> info)
        {
            List<ISystem> routeSystems = new List<ISystem>();
            Graph<ISystem> currentGraph = new Graph<ISystem>();
            var coordsfrom = Coordsfrom;
            var coordsto = Coordsto;
            var maxrange = MaxRange;

            // Set maxrange to boosts. A* weights depends on the metric being used to avoid/prefer FSD boosts
            if(UseFsdBoost)
                maxrange *= (1.0f + BoostPercentage(3));

            double xwindow = Math.Abs(coordsfrom.X - coordsto.X) + maxrange / 2f;       // we need a window of co-ords
            double ywindow = Math.Abs(coordsfrom.Y - coordsto.Y) + maxrange / 2f;       // to pick up systems. consider from 0,0,0 to 0,1000,1000
            double zwindow = Math.Abs(coordsfrom.Z - coordsto.Z) + maxrange / 2f;       // x has no window.
            double traveldistance = Point3D.DistanceBetween(coordsfrom, coordsto);      // its based on a percentage of the traveldistance
            double wanderpercentage = 0.1;
            double wanderwindow = traveldistance * wanderpercentage;                    // this is the minimum window size

            if (wanderwindow > 100)                                                     // limit, otherwise we just get too many
                wanderwindow = 100;

            xwindow = (xwindow < wanderwindow) ? (wanderwindow / 2) : 10;               // if less than the wander window, open it up, else open it up a little so it can
            ywindow = (ywindow < wanderwindow) ? (wanderwindow / 2) : 10;               // find start/end points without any rounding errors..
            zwindow = (zwindow < wanderwindow) ? (wanderwindow / 2) : 10;

            Point3D minpos = new EMK.LightGeometry.Point3D(
                    Math.Min(coordsfrom.X, coordsto.X) - xwindow,
                    Math.Min(coordsfrom.Y, coordsto.Y) - ywindow,
                    Math.Min(coordsfrom.Z, coordsto.Z) - zwindow );
            Point3D maxpos = new EMK.LightGeometry.Point3D(
                    Math.Max(coordsfrom.X, coordsto.X) + xwindow,
                    Math.Max(coordsfrom.Y, coordsto.Y)+ ywindow,
                    Math.Max(coordsfrom.Z, coordsto.Z) + zwindow );

            System.Diagnostics.Debug.WriteLine("Bounding Box " + minpos.X.ToString("0.0") + "," + minpos.Y.ToString("0.0") + "," + minpos.Z.ToString("0.0") + " to " + maxpos.X.ToString("0.0") + "," + maxpos.Y.ToString("0.0") + "," + maxpos.Z.ToString("0.0") + " window " + wanderwindow.ToString("0.0") + Environment.NewLine);

            {
                info(new ReturnInfo("A*: Collecting stars..."));
                var sw = new Stopwatch();
                sw.Start();
                AddStarNodes(currentGraph, minpos, maxpos);
                sw.Stop();
                System.Diagnostics.Debug.WriteLine("Added stars within bounds in " + sw.Elapsed.TotalSeconds.ToString("0.000s") + Environment.NewLine);
                if (StopPlotter)
                {
                    info(new ReturnInfo("A*: Canceled."));
                    return routeSystems;
                }
            }

            {
                info(new ReturnInfo("A*: Calculating distances between "+ currentGraph.Nodes.Count.ToString("N0") +" stars."));
                var sw = new Stopwatch();
                sw.Start();
                CalculateArcs(currentGraph, maxrange);
                sw.Stop();
                System.Diagnostics.Debug.WriteLine("Calculated arcs in " + sw.Elapsed.TotalSeconds.ToString("0.000s") + Environment.NewLine);
                if (StopPlotter)
                {
                    info(new ReturnInfo("A*: Canceled."));
                    return routeSystems;
                }
            }

            aStarAlgortihm = new AStar<ISystem>(currentGraph);
            Node<ISystem> start = currentGraph.GetNodes.FirstOrDefault(x => x.System.Name == FromSystem);
            Node<ISystem> stop = currentGraph.GetNodes.FirstOrDefault(x => x.System.Name == ToSystem);

            if ( start == null || stop == null )
            {
                info(new ReturnInfo("A*: Code failed - Please report failure on the EDDiscovery Discord or EDDiscovery forum thread"));
                return routeSystems;
            }

            bool res;
            {
                info(new ReturnInfo("A*: Checking "+ currentGraph.Arcs.Count.ToString("N0") +" distances for best possible path."));
                var sw = new Stopwatch();
                sw.Start();
                res = aStarAlgortihm.SearchPath(start, stop);
                sw.Stop();
                System.Diagnostics.Debug.WriteLine("A* routing finished after " + sw.Elapsed.TotalSeconds.ToString("0.000s") + Environment.NewLine);
                if (StopPlotter)
                {
                    info(new ReturnInfo("A*: Canceled."));
                    return routeSystems;
                }
            }

            if (res)
            {
                System.Diagnostics.Debug.WriteLine(Environment.NewLine + string.Format("{0,-30}Depart Co-Ords:{1:0.00},{2:0.00},{3:0.00}" + Environment.NewLine, start.System.Name, start.X, start.Y, start.Z));

                double totalDistance = 0;
                //TODO: remove variable
                int[] boosts = {0, 0, 0};

                foreach (Arc<ISystem> A in aStarAlgortihm.PathByArcs)
                {
                    // have to do it manually in case using the START, WAYPOINT or END points
                    double dist = Point3D.DistanceBetween(A.StartNode.Position, A.EndNode.Position);
                    double weight = A.EndNode.ArcComingFrom(A.StartNode).Weight;

                    var boostPostfix = "";
                    if (dist > MaxRange)
                    {
                        var boostPercentage = (int)(BoostPercentage((float) dist, MaxRange) * 100);
                        boostPostfix = " (+" + boostPercentage + "% Boost)";
                        if(boostPercentage <= 25)
                            boosts[0]++;
                        else if (boostPercentage <= 50)
                            boosts[1]++;
                        else
                            boosts[2]++;
                    }

                    info(new ReturnInfo(A.EndNode.System.Name + boostPostfix, dist, A.EndNode.Position, Point3D.DistanceBetween(A.EndNode.Position, Coordsto), weight-1, A.EndNode.System as ISystem));
                    totalDistance += dist;

                    System.Diagnostics.Debug.WriteLine(A.ToString());
                    routeSystems.Add(A.EndNode.System as ISystem);
                }

                System.Diagnostics.Debug.WriteLine(string.Format(Environment.NewLine + "Straight Line Distance {0,8:0.00}ly vs Travelled Distance {1,8:0.00}ly" + Environment.NewLine, traveldistance.ToString("0.00"), totalDistance.ToString("0.00")));
                if (boosts.Max() > 0)
                    info(new ReturnInfo("Boosts used (Basic, Standard, Premium)", double.NaN, new Point3D(boosts[0], boosts[1], boosts[2])));
                info(new ReturnInfo("Straight Line Distance", traveldistance));
                info(new ReturnInfo("Travelled Distance", totalDistance));
            }
            else
                info(new ReturnInfo(
                    "A*: No Solution found - jump distance " + MaxRange + "ly is too small or not enough star data between systems: " + currentGraph.Nodes.Count.ToString("N0")));

            return routeSystems;
        }

        private void AddStarNodes(Graph<ISystem> graph, Point3D minpos, Point3D maxpos=null)
        {
            List<ISystem> systems = GetSystemsBetween(minpos, maxpos);
            foreach (ISystem system in systems.TakeWhile(system => !StopPlotter))
            {
                graph.AddNodeWithNoChk(new Node<ISystem>(system.X, system.Y, system.Z, system));
                //System.Diagnostics.Debug.WriteLine("A*: Adding StarNode " + system.Name );
            }
            System.Diagnostics.Debug.WriteLine("Number of stars within bounds: " + graph.Nodes.Count);
        }

        public bool IsFromOrToNearInBubble()
        {
            // Check if From is near Sol
            if (Point3D.DistanceBetween(new Point3D(0, 0, 0), Coordsfrom) < 200)
                return true;

            // Check if To is near Sol
            if (Point3D.DistanceBetween(new Point3D(0, 0, 0), Coordsto) < 200)
                return true;

            return false;
        }

        private List<ISystem> GetSystemsBetween(Point3D point1, Point3D point2)
        {
            const int maxConsideredSystem = int.MaxValue - 1; // -1 is required for SQL code not to fail
            var distlist = new BaseUtils.SortedListDoubleDuplicate<ISystem>();

            double radius = Point3D.DistanceBetween(point1, point2)/2.0;
            var center = new Point3D((point1.X + point2.X)/2.0, (point1.Y + point2.Y)/2.0, (point1.Z + point2.Z) / 2.0);
            System.Diagnostics.Debug.WriteLine("GetSystemsBetween() - radius={0}, point1={1}, point2={2}, center={3}", radius, point1, point2, center);
            SystemCache.GetSystemListBySqDistancesFrom(distlist, center.X, center.Y, center.Z, maxConsideredSystem,0, radius, true);

            var systemlist = new List<ISystem>();
            foreach (string systemname in distlist.TakeWhile(systemname => !StopPlotter).Select(o => o.Value.Name))
            {
                systemlist.Add(new SystemClass(SystemCache.FindSystem(systemname)));
            }
            return systemlist;
        }
        private void CalculateArcs(Graph<ISystem> G, float jumpRange )
        {
            float distanceSquared;
            float jumpRangeSquared = jumpRange * jumpRange;
            const float minRangeSquared = 1; // 1 ly.
            Node<ISystem> N1, N2;
            float dx, dy, dz;

            int numberOfArcs = 0;


            for (int ii = 0; !StopPlotter && ii < G.Count; ii++)
            {
                N1 = G.GetNode(ii);

                for (int jj = ii; !StopPlotter && jj < G.Count; jj++)
                {
                    float weight = 1;
                    N2 = G.GetNode(jj);

                    dx = (float)(N1.X - N2.X);
                    dy = (float)(N1.Y - N2.Y);
                    dz = (float)(N1.Z - N2.Z);
                    distanceSquared = dx * dx + dy * dy + dz * dz;

                    if (minRangeSquared >= distanceSquared || distanceSquared > jumpRangeSquared)
                        continue;

                    // Economic: the shorther the jumps, the better. Weigh shorter jumps less than longer jumps (it is always independent of FSD boosts)
                    if (RouteMethod == Metric.AstarEconomic)
                    {
                        float maxRangeSquared = MaxRange * MaxRange;
                        weight = distanceSquared*distanceSquared / maxRangeSquared*maxRangeSquared;
                    }
                    // Standard, AvoidBoosts, MaximumBoosts: Weight strength depends on the mode chosen
                    else if (UseFsdBoost)
                    {
                        float maxRangeSquared = MaxRange * MaxRange;
                        if (distanceSquared > maxRangeSquared)
                        {
                            float boostPercentage = BoostPercentage((float) Math.Sqrt(distanceSquared), MaxRange);
                            if (RouteMethod == Metric.AstarStandard)
                            {
                                // the stronger the boost, the higher the weight (EoD: linear increase looked reasonable)
                                weight += boostPercentage;
                            }
                            else if (RouteMethod == Metric.AstarAvoidBoosts)
                            {
                                // exponential penalty for each boost type. Only do a 100% boost if it is really necessary
                                weight += (1 + boostPercentage) * (1 + boostPercentage);
                            }
                            else if (RouteMethod == Metric.AstarCheapBoosts)
                            {
                                // slightly increase weight on boosts
                                weight += .25f * boostPercentage;
                            }
                        }
                    }

                    G.AddArcWithNoChk(N1, N2, weight);  // add N1->N2, its in the right direction
                    G.AddArcWithNoChk(N2, N1, weight);

                    numberOfArcs++;
                }
            }

            System.Diagnostics.Debug.WriteLine("Created " + numberOfArcs + " arcs.");
        }
    }

}
