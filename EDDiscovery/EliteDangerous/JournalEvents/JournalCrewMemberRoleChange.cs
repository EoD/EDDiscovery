﻿
/*
 * Copyright © 2017 EDDiscovery development team
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
    //    When written: When another player leaves your ship's crew
    //Parameters:
    //•	Crew: player's commander name
    // Role:
    // { "timestamp":"2017-04-11T18:11:06Z", "event":"CrewMemberRoleChange", "Crew":"[cmdr name]", "Role":"Idle" }
    [JournalEntryType(JournalTypeEnum.CrewMemberRoleChange)]
    public class JournalCrewMemberRoleChange : JournalEntry
    {
        public JournalCrewMemberRoleChange(JObject evt) : base(evt, JournalTypeEnum.CrewMemberRoleChange)
        {
            Crew = evt["Crew"].Str();
            Role = evt["Role"].Str();
        }
        public string Crew { get; set; }
        public string Role { get; set; }

        public override System.Drawing.Bitmap Icon { get { return EDDiscovery.Properties.Resources.crewmemberjoins; } }

        public override void FillInformation(out string summary, out string info, out string detailed) //V
        {
            summary = EventTypeStr.SplitCapsWord();
            //info = Crew;
            info = BaseUtils.FieldBuilder.Build("Crew", Crew, "Role", Role);
            detailed = "";
        }
    }
}
