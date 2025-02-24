﻿/*
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
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace BaseUtils
{
    public static class BrowserInfo
    {
        public static string GetDefault()
        {
            const string userChoice = @"Software\Microsoft\Windows\Shell\Associations\UrlAssociations\http\UserChoice";
            using (Microsoft.Win32.RegistryKey userChoiceKey = Microsoft.Win32.Registry.CurrentUser.OpenSubKey(userChoice))
            {
                if (userChoiceKey != null)
                {
                    object progIdValue = userChoiceKey.GetValue("Progid");
                    if (progIdValue != null)
                        return progIdValue.ToString();
                }
            }

            return null;
        }

        public static string GetPath(string defbrowser)
        {
            const string exeSuffix = ".exe";
            string path = defbrowser + @"\shell\open\command";

            using (Microsoft.Win32.RegistryKey pathKey = Microsoft.Win32.Registry.ClassesRoot.OpenSubKey(path))
            {
                if (pathKey == null)
                {
                    return null;
                }

                // Trim parameters.
                try
                {
                    path = pathKey.GetValue(null).ToString().ToLower().Replace("\"", "");
                    if (!path.EndsWith(exeSuffix))
                    {
                        path = path.Substring(0, path.LastIndexOf(exeSuffix, StringComparison.Ordinal) + exeSuffix.Length);
                        return path;
                    }
                }
                catch
                {
                    // Assume the registry value is set incorrectly, or some funky browser is used which currently is unknown.
                }
            }

            return null;
        }
    }
}
