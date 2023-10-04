
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
  
  char* buffert=(char *)malloc(siz);
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
  long siz;
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
  } while(f==NULL);

  fseek(f,0,SEEK_END);
  for(int i=0;i<bufcnt;i++)
    {
      char *buffert=bufVec[i];
      siz=sizVec[i];
      for(int s=0;s<siz;)
      {
          int a=fwrite(&buffert[s],1,(siz-s),f);
          if (a<(siz-s))
          {
            
              if(!DeleteOldest()) { goto failure; }
              if(a>0)
              {
                s+=a;
              }
          }
          else
          {
            s+=a;
          }
      }
    }

 failure:
  for(int i=0;i<bufcnt;i++)
    {
      dosfree(bufVec[i]);
    }
  bufcnt=0;
  fclose(f);
}

