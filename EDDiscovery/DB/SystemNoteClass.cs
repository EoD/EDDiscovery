﻿/*
 * Copyright © 2015 - 2016 EDDiscovery development team
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
using EDDiscovery.DB;
using System;
using System.Collections.Generic;
using System.Data;
using System.Data.Common;
using System.Linq;
using System.Text;

namespace EDDiscovery.DB
{
    public class SystemNoteClass
    {
        public long id;
        public long Journalid;              //Journalid = 0, Name set, system marker
        public string SystemName;           //Journalid <>0, Name set or clear, journal marker
        public DateTime Time;
        public string Note { get; private set; }
        public long EdsmId;
        public bool Dirty;                  // changed but uncommitted

        public static List<SystemNoteClass> globalSystemNotes = new List<SystemNoteClass>();        // global cache, kept updated

        public SystemNoteClass()
        {
        }

        public SystemNoteClass(DataRow dr)
        {
            id = (long)dr["id"];
            Journalid = (long)dr["journalid"];
            SystemName = (string)dr["Name"];
            Time = (DateTime)dr["Time"];
            Note = (string)dr["Note"];
            EdsmId = (long)dr["EdsmId"];
        }


        private bool Add()
        {
            using (SQLiteConnectionUser cn = new SQLiteConnectionUser())
            {
                bool ret = Add(cn);
                return ret;
            }
        }

        private bool Add(SQLiteConnectionUser cn)
        {
            using (DbCommand cmd = cn.CreateCommand("Insert into SystemNote (Name, Time, Note, journalid, edsmid) values (@name, @time, @note, @journalid, @edsmid)"))
            {
                cmd.AddParameterWithValue("@name", SystemName);
                cmd.AddParameterWithValue("@time", Time);
                cmd.AddParameterWithValue("@note", Note);
                cmd.AddParameterWithValue("@journalid", Journalid);
                cmd.AddParameterWithValue("@edsmid", EdsmId);

                SQLiteDBClass.SQLNonQueryText(cn, cmd);

                using (DbCommand cmd2 = cn.CreateCommand("Select Max(id) as id from SystemNote"))
                {
                    id = (long)SQLiteDBClass.SQLScalar(cn, cmd2);
                }

                globalSystemNotes.Add(this);

                Dirty = false;

                return true;
            }
        }

        private bool Update()
        {
            using (SQLiteConnectionUser cn = new SQLiteConnectionUser())
            {
                return Update(cn);
            }
        }

        private bool Update(SQLiteConnectionUser cn)
        {
            using (DbCommand cmd = cn.CreateCommand("Update SystemNote set Name=@Name, Time=@Time, Note=@Note, Journalid=@journalid, EdsmId=@EdsmId  where ID=@id"))
            {
                cmd.AddParameterWithValue("@ID", id);
                cmd.AddParameterWithValue("@Name", SystemName);
                cmd.AddParameterWithValue("@Note", Note);
                cmd.AddParameterWithValue("@Time", Time);
                cmd.AddParameterWithValue("@journalid", Journalid);
                cmd.AddParameterWithValue("@EdsmId", EdsmId);

                SQLiteDBClass.SQLNonQueryText(cn, cmd);

                Dirty = false;
            }

            return true;
        }

        public bool Delete()
        {
            using (SQLiteConnectionUser cn = new SQLiteConnectionUser())
            {
                return Delete(cn);
            }
        }

        private bool Delete(SQLiteConnectionUser cn)
        {
            using (DbCommand cmd = cn.CreateCommand("DELETE FROM SystemNote WHERE id = @id"))
            {
                cmd.AddParameterWithValue("@id", id);
                SQLiteDBClass.SQLNonQueryText(cn, cmd);

                globalSystemNotes.RemoveAll(x => x.id == id);     // remove from list any containing id.
                return true;
            }
        }

        public SystemNoteClass UpdateNote(string s, bool commit , DateTime time , long edsmid )
        {
            Note = s;
            Time = time;
            EdsmId = edsmid;

            Dirty = true;

            if (commit)
            {
                if (s.Length == 0)        // empty ones delete the note
                {
                    System.Diagnostics.Debug.WriteLine("Delete note " + Journalid + " " + SystemName + " " + Note);
                    Delete();           // delete and remove notes..
                    return null;
                }
                else
                {
                    System.Diagnostics.Debug.WriteLine("Update note " + Journalid + " " + SystemName + " " + Note);
                    Update();
                }
            }
            else
            {
                System.Diagnostics.Debug.WriteLine("Editmem note " + Journalid + " " + SystemName + " " + Note);
            }

            return this;
        }

        public static void ClearEDSMID()
        {
            using (SQLiteConnectionUser cn = new SQLiteConnectionUser(utc: true))
            {
                using (DbCommand cmd = cn.CreateCommand("UPDATE SystemNote SET EdsmId=0"))
                {
                    SQLiteDBClass.SQLNonQueryText(cn, cmd);
                }
            }
        }

        public static bool GetAllSystemNotes()
        {
            try
            {
                using (SQLiteConnectionUser cn = new SQLiteConnectionUser(mode: EDDbAccessMode.Reader))
                {
                    using (DbCommand cmd = cn.CreateCommand("select * from SystemNote"))
                    {
                        DataSet ds = SQLiteDBClass.SQLQueryText(cn, cmd);
                        if (ds.Tables.Count == 0 || ds.Tables[0].Rows.Count == 0)
                        {
                            return false;
                        }

                        globalSystemNotes.Clear();

                        foreach (DataRow dr in ds.Tables[0].Rows)
                        {
                            SystemNoteClass sys = new SystemNoteClass(dr);
                            globalSystemNotes.Add(sys);
                            //System.Diagnostics.Debug.WriteLine("Global note " + sys.Journalid + " " + sys.SystemName + " " + sys.Note);
                        }

                        return true;

                    }
                }
            }
            catch
            {
                return false;
            }
        }


        public static void CommitDirtyNotes()
        {
            foreach( SystemNoteClass sys in globalSystemNotes )
            {
                if (sys.Dirty)
                {
                    System.Diagnostics.Debug.WriteLine("Commit dirty note " + sys.Journalid + " " + sys.SystemName + " " + sys.Note);
                    sys.Update();       // clears the dirty flag
                }
            }
        }

        public static SystemNoteClass GetNoteOnSystem(string name, long edsmid = -1)      // case insensitive.. null if not there
        {
            return globalSystemNotes.FindLast(x => x.SystemName.Equals(name, StringComparison.InvariantCultureIgnoreCase) && (edsmid <= 0 || x.EdsmId <= 0 || x.EdsmId == edsmid));
        }

        public static SystemNoteClass GetNoteOnJournalEntry(long jid)
        {
            if (jid > 0)
                return globalSystemNotes.FindLast(x => x.Journalid == jid);
            else
                return null;
        }

        public static SystemNoteClass GetSystemNote(long journalid, bool fsd, EliteDangerous.ISystem sys)
        {
            SystemNoteClass systemnote = SystemNoteClass.GetNoteOnJournalEntry(journalid);

            if (systemnote == null && fsd)      // this is for older system name notes
            {
                systemnote = SystemNoteClass.GetNoteOnSystem(sys.name, sys.id_edsm);

                if (systemnote != null)      // if found..
                {
                    if (sys.id_edsm > 0 && systemnote.EdsmId <= 0)    // if we have a system id, but snc not set, update it for next time.
                    {
                        systemnote.EdsmId = sys.id_edsm;
                        systemnote.Dirty = true;
                    }
                }
            }

            if (systemnote != null)
            {
//                System.Diagnostics.Debug.WriteLine("HE " + Journalid + " Found note " + +snc.Journalid + " " + snc.SystemName + " " + snc.Note);
            }

            return systemnote;
        }

        public static SystemNoteClass MakeSystemNote(string text, DateTime time, string sysname, long journalid, long edsmid )
        {
            SystemNoteClass sys = new SystemNoteClass();
            sys.Note = text;
            sys.Time = time;
            sys.SystemName = sysname;
            sys.Journalid = journalid;                          // any new ones gets a journal id, making the Get always lock it to a journal entry
            sys.EdsmId = edsmid;
            sys.Add();
            System.Diagnostics.Debug.WriteLine("made note " + sys.Journalid + " " + sys.SystemName + " " + sys.Note);
            return sys;
        }
    }
}
