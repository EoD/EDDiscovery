﻿
using Newtonsoft.Json.Linq;
using System.Linq;

namespace EDDiscovery.EliteDangerous.JournalEvents
{
    //When written: when scooping fuel from a star
    //Parameters:
    //•	Scooped: tons fuel scooped
    //•	Total: total fuel level after scooping
    public class JournalFuelScoop : JournalEntry
    {
        public JournalFuelScoop(JObject evt, EDJournalReader reader) : base(evt, JournalTypeEnum.FuelScoop, reader)
        {
            Scooped = Tools.GetDouble("Scooped");
            Total = Tools.GetDouble("Total");
        }
        public double Scooped { get; set; }
        public double Total { get; set; }
    }
}
