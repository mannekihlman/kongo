#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <dirent.h>
#include <fcntl.h>
#include <syslog.h>

int LOG=(LOG_USER|LOG_DEBUG);

int debugflag=1;
int DelOldestCnt=0;
int uploadcnt=0;
int realtime=0;

char uploadname[]="upload.pak";
char cfgtxtname[]="../cfg.txt";
char statusname[]="/var/status.dat";

int ReadSettingFile(char *filename)
{
  char *pt;
  FILE *fil;
  char nl[2]={ 0x0a, 0 };
  char lf[2]={ 0x0d, 0 };
  long i;
  short flag;
  char temp[30],txt[256];
  
  fil = fopen(filename, "r");
  if(fil<(FILE *)1)
    {
      return(-1);
    }
  while(fgets(txt,sizeof(txt)-1,fil) )
    {
      if(strlen(txt)>4 && txt[0]!='%')
        {                 
          pt=txt;
          if(pt=strstr(txt,nl)) pt[0]=0;
          pt=txt;
          if(pt=strstr(txt,lf)) pt[0]=0;

          if(pt=strstr(txt,"REALTIME="))
            {
              pt=strstr(txt,"=");
              sscanf(&pt[1],"%d",&realtime);
            }

        }
    }
  fclose(fil);

  return(0);
}


long FileSize(char *name)
{
  int f;
  long a;
  f=open(name,O_RDONLY);
  if(f==-1) return(-1);
  a=lseek(f,0,SEEK_END);
  close(f);
  return(a);      
}

#define Execute system
#define Delete remove

void msleep(unsigned short t)
{
   if(t<1) t=1;
   usleep(t*1000);
}

void Reboot()
{
   syslog(LOG,"Rebooting");
   system("reboot");
}

int DeleteOldest()
{
    struct dirent *dir;
    DIR *d;

    int minr,minu,nr;
    char oldestname[20];
    char cmd[20];

    msleep(128);
    syslog(LOG,"DeleteOldest");
    DelOldestCnt++;
    minr=minu=0x7fff;

    d=opendir(".");
    if(d!=NULL)
      {
	while( dir=readdir(d))
	  {
	    sscanf(&dir->d_name[1],"%x",&nr);
	    
	    if( (dir->d_name[0]=='r') && dir->d_type==DT_DIR)
	      {
		if(minr>nr) minr=nr;
	      }
	    if( (dir->d_type!=DT_DIR) && ((dir->d_name[0]=='u') && (strstr(dir->d_name,".pak"))))
	      {
		if(minu>nr) minu=nr;
	      }
	  }
	closedir(d);
      }
   
    if((minr==0x7fff) && (minu==0x7fff))
      {
        if(DelOldestCnt>200) Reboot();
        return(0);
      }
    if(minr!=0x7fff)
       { 
       sprintf(oldestname,"r%03x",minr);
       sprintf(cmd,"rm -f %s/*",oldestname);
       Execute(cmd);
       rmdir(oldestname);
       return(1);       
       }
    sprintf(oldestname,"u%03x.pak",minu);       
    syslog(LOG,"Deleting %s to free up space\n",oldestname);
    Delete(oldestname);
    return(1);
}

int GetMaxRunNr(int *maxunr)
{
    int nr,maxrunnr;
    struct dirent *dir;
    DIR *d;

    maxrunnr=0;

    d=opendir(".");
    if(d!=NULL)
      {
	while(dir=readdir(d))
	  {

	    if( (dir->d_type==DT_DIR) && (dir->d_name[0]=='r'))
	      {
		sscanf(&dir->d_name[1],"%x",&nr);
		//		syslog(LOG,"Found dir %d\n",nr);
		if(maxrunnr<nr) maxrunnr=nr;
	      }

          if( (dir->d_name[0]=='u') && (dir->d_name[1]!='p')
	      && (strstr(dir->d_name,"pak")) )
            {
	      sscanf(&dir->d_name[1],"%x",&nr);
	      if(*maxunr<nr) *maxunr=nr;
            }
	  }
	closedir(d);
      }
    return(maxrunnr);
}

void CreateDirAndCheck(char *dirname)
{
  int ok;
    struct dirent *dir;
    DIR *d;

     ok=0;
     mkdir(dirname);
     d=opendir(dirname);
     if(d!=NULL)
      {
	closedir(d);
	ok=1;
      }
    
    if(!ok)
      {
        ok=DeleteOldest();
	mkdir(dirname);
      }
    
}

void MovePakFiles()
{
    struct dirent *dir;
    DIR *d;

    char dirname[32],txt[64];
    int nr,maxunr;    
    uploadcnt=1;
    maxunr=-1;

    syslog(LOG,"Making MovePakFiles");

    nr=GetMaxRunNr(&maxunr);    
    
    nr++;
    sprintf(dirname,"r%03x",nr);
    nr=maxunr;
    
    if(FileSize(uploadname)>0)
       {
         if(maxunr==-1) maxunr=0;
         CreateDirAndCheck(dirname);

         sprintf(txt,"cp %s %s/u%03x.pak",uploadname,dirname,0);
         Execute(txt);
       }
    Delete(uploadname);
    
    if(nr==-1) {  return; }
    syslog(LOG,"Nr of uxxx.pak files: %d\n",nr);
    CreateDirAndCheck(dirname);

    d=opendir(".");
    if(d!=NULL)
      {    
	while(dir=readdir(d))
	  {
	    if(dir->d_name[0]=='u' && strstr(dir->d_name,".pak") && dir->d_type!=DT_DIR)
	      {
		sprintf(txt,"cp %s %s/%s",dir->d_name,dirname,dir->d_name);
		Execute(txt);
		msleep(128);
	      }
	  }
	closedir(d);
	Execute("rm -f u*.pak");
      }

}


int main(int argc,char *argv[])
{
  realtime=0;
  if(debugflag>0) syslog(LOG,"Compiled " __DATE__  " " __TIME__ "\n");      
   
  ReadSettingFile(cfgtxtname);
  if(realtime!=2)
    MovePakFiles();

  syslog(LOG,"MovePakFiles DONE");

  exit(0);
}

