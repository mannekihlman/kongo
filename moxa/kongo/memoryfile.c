
char *bufVec[4096];
long sizVec[4096];
short bufcnt=0;

long MemoryFileSize()
{
  int i;
  long siz=0;
  for(i=0;i<bufcnt;i++)
    {
      siz+=sizVec[i];     
    }
  return(siz);
}

char *dosallocate(long siz)
{
  
  char *buffert=malloc(siz);
  return(buffert);
}

void dosfree(char *ptr)
{
  free(ptr);
}

long mwrite(char *mem1,long len1,char *mem2,long len2)
{
  char *buffert;

  if(bufcnt>4095) return(0);

  buffert=dosallocate(len1+len2);
  if(buffert==0) return(0);
  
  bufVec[bufcnt]=buffert;
  sizVec[bufcnt]=len1+len2;
  bufcnt++;
  memcpy(buffert,mem1,len1);  
  memcpy(buffert+len1,mem2,len2);  
  if(debugflag>0) syslog(LOG,"MemoryFileSize %ld\n",MemoryFileSize());
  return(len1+len2);
}

void SaveMemoryFile(char *name)
{
  FILE *f;
  char *buffert;
  long siz;
  short i;
  do {        
    f=fopen(name,"r+b");
    if(f<(FILE *)1) f=fopen(name,"w+b");
    if(f<(FILE *)1)
      {
        f=fopen(name,"r");
        if(f<(FILE *)1)
          {
            if(!DeleteOldest()) { goto failure; }
          }
        else
          {
            fclose(f);
            syslog(LOG,"File is occupied. Waiting 4 seconds");
            msleep(4000);
          }
      }
  } while(f<(FILE *)1);

  fseek(f,0,SEEK_END);
  for(i=0;i<bufcnt;i++)
    {
      buffert=bufVec[i];
      siz=sizVec[i];
      while(!fwrite(buffert,siz,1,f))
        {
          if(!DeleteOldest()) { goto failure; }
        }
    }

 failure:
  for(i=0;i<bufcnt;i++)
    {
      dosfree(bufVec[i]);
    }
  bufcnt=0;
  fclose(f);
}

