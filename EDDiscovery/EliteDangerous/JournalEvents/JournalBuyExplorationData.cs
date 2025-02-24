﻿/*
 * Copyright © 2016 EDDiscovery development team
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
using Newtonsoft.Json.Linq;
using System.Linq;

namespace EDDiscovery.EliteDangerous.JournalEvents
{
    //When Written: when buying system data via the galaxy map
    //Parameters:
    //•	System
    //•	Cost
    [JournalEntryType(JournalTypeEnum.BuyExplorationData)]
    public class JournalBuyExplorationData : JournalEntry, ILedgerJournalEntry
    {
        public JournalBuyExplorationData(JObject evt ) : base(evt, JournalTypeEnum.BuyExplorationData)
        {
            System = evt["System"].Str();
            Cost = evt["Cost"].Long();

        }
        public string System { get; set; }
        public long Cost { get; set; }

        public override System.Drawing.Bitmap Icon { get { return EDDiscovery.Properties.Resources.buyexplorationdata; } }

        public void Ledger(Ledger mcl, DB.SQLiteConnectionUser conn)
        {
            mcl.AddEvent(Id, EventTimeUTC, EventTypeID, System, -Cost);
        }

        public override void FillInformation(out string summary, out string info, out string detailed) //V
        {
            summary = EventTypeStr.SplitCapsWord();
            info = BaseUtils.FieldBuilder.Build("System:", System, "Cost:; credits", Cost);
            detailed = "";
        }
    }
}
