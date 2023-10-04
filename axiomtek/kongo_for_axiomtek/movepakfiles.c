namespace MovePakFiles
{

  //----------------------------------------------
  int GetMaxRunNr()
  {
    int maxrunnr=0;
    DIR *d=opendir(".");
    if(d!=0)
      {
	struct dirent *dir;
	while(dir=readdir(d))
	  {

	    if( (dir->d_type==DT_DIR) && (dir->d_name[0]=='r'))
	      {
		int nr;
		sscanf(&dir->d_name[1],"%x",&nr);
		if(maxrunnr<nr) maxrunnr=nr;
	      }
	  }
	closedir(d);
      }
    return(maxrunnr);
  }

  //----------------------------------------------
  void CreateDirAndCheck(char *dirname)
  {
    (void) mkdir(dirname,0777);
    DIR *d=opendir(dirname);
    if(d!=0)
      {
	closedir(d);
      }
    else
      {
	DeleteOldest();
	(void) mkdir(dirname,0777);
      }   
  }

  //----------------------------------------------
  void MovePakFiles()
  {
    char dirname[32];
    int nrOfFiles = 0;
    syslog(LOG,"Making MovePakFiles");
    sprintf(dirname,"r%03x",GetMaxRunNr() + 1);
    if(FileSize(uploadname)>0)
      {
	CreateDirAndCheck(dirname);
	nrOfFiles++;      
	sprintf(txt,"cp %s %s/ufff.pak",uploadname,dirname);
	system(txt);
	syslog(LOG,"Moved %s to %s",uploadname,dirname);

      }
    remove(uploadname);
    DIR *d=opendir(".");
    if(d!=NULL)
      {    
	struct dirent *dir;
	while(dir=readdir(d))
	  {
	    if(dir->d_name[0]=='u' && strstr(dir->d_name,".pak") && dir->d_type!=DT_DIR)
	      {
		if(nrOfFiles==0)
		  {
		    CreateDirAndCheck(dirname);
		  }
		sprintf(txt,"cp %s %s/%s",dir->d_name,dirname,dir->d_name);
		system(txt);
	      }
	  }
	closedir(d);
	msleep(128);
	system("rm -f u*.pak");
      }
    syslog(LOG,"MovePakFiles moved %d files. DONE",nrOfFiles);
  }

} // end of namespace


