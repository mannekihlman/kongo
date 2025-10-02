namespace MovePakFiles {

    //----------------------------------------------
    int GetMaxRunNr() {
        int maxrunnr = 0;
        DIR *d = opendir(".");
        if (d != 0) {
            struct dirent *dir;
            while (dir = readdir(d)) {

                if ((dir->d_type == DT_DIR) && (dir->d_name[0] == 'r')) {
                    int nr;
                    sscanf(&dir->d_name[1], "%x", &nr);
                    if (maxrunnr < nr) maxrunnr = nr;
                }
            }
            closedir(d);
        }
        return (maxrunnr);
    }

    //----------------------------------------------
    void CreateDirAndCheck(char *dirname) {
        (void) mkdir(dirname, 0777);
        DIR *d = opendir(dirname);
        if (d != 0) {
            closedir(d);
        } else {
            if (verboseFileName == 1) {
                VerboseDeleteOldest();
            } else {
                DeleteOldest();
            }
            (void) mkdir(dirname, 0777);
        }
    }

    //----------------------------------------------
    void MovePakFiles() {
        char dirname[32];
        int nrOfFiles = 0;
        syslog(LOG, "Making MovePakFiles");
        if (verboseFileName == 1) {
            // we need to find the max date and time to save as a folder name
            int maxDate = 0;
            int maxTime = 0;

            // we are going to keep track of the count of upload.pak and regular .pak files

            DIR *d = opendir(".");
            if (d != NULL) {
                struct dirent *dir;
                while (dir = readdir(d)) {
                    if (strstr(dir->d_name, ".pak") && dir->d_type != DT_DIR && strstr(dir->d_name, "upload") == NULL) {

                        int t_date, t_time;

                        // get the max date and time from the file and save it off if it is the largest found so far
                        sscanf(&dir->d_name[strlen(dir->d_name) - 17], "%u_%u", &t_date, &t_time);
                        if (t_date >= maxDate) {
                            maxDate = t_date;
                            if (t_time > maxTime) maxTime = t_time;
                        }
                    }
                }
                // no proper pak file with time and date were found, set time and date to right now
                if (maxDate == 0 && maxTime == 0) {
                    // std::string t_string = hhmmssSS_time;
                    // sprintf(dirname, "%06d_%s", yymmdd_date, t_string.substr(0, 6).c_str());
                    sprintf(dirname, "%06d_%s", yymmdd_date, hhmmss_time);
                } else {
                    sprintf(dirname, "%06d_%06d", maxDate, maxTime);
                }
            }
        } else {
            sprintf(dirname, "r%03x", GetMaxRunNr() + 1);
        }

        if (FileSize(uploadname) > 0) {
            CreateDirAndCheck(dirname);
            nrOfFiles++;
            sprintf(txt, "cp %s %s/ufff.pak", uploadname, dirname);
            system(txt);
            syslog(LOG, "Moved %s to %s", uploadname, dirname);
        }
        remove(uploadname);
        DIR *d = opendir(".");
        if (d != NULL) {
            struct dirent *dir;
            while (dir = readdir(d)) {
                // if(dir->d_name[0]=='u' && strstr(dir->d_name,".pak") && dir->d_type!=DT_DIR)  //Depricate
                if (strstr(dir->d_name, ".pak") && dir->d_type != DT_DIR) {
                    if (nrOfFiles == 0) {
                        CreateDirAndCheck(dirname);
                    }
                    sprintf(txt, "cp %s %s/%s", dir->d_name, dirname, dir->d_name);
                    system(txt);
                }
            }
            closedir(d);
            msleep(128);
            system("rm -f *.pak");
        }
        syslog(LOG, "MovePakFiles moved %d files. DONE", nrOfFiles);
    }

} // end of namespace


