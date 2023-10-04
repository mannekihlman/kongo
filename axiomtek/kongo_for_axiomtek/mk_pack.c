#include "mk_compress.c"

long UnPack(unsigned char *inpek,long kvar,long *ut )
{
  long *utpek;
  short len,curr;
  short j,jj;
  long a;
  unsigned short lentofile=0;
  long bit=0;
  
  utpek=ut;
  lentofile=0;  
  while(kvar>0)
    {
      len=0;
      for(j=0;j<7;j++)
        {
          len+=len;
          len|=inpek[(bit>>3)]>>(7-(bit&0x7))&1;
          bit++;
        }      
      curr=0;
      for(j=0;j<5;j++)
        {
          curr+=curr;
          curr|=inpek[(bit>>3)]>>(7-(bit&0x7))&1;
          bit++;
        }
      if(curr)
        {
          for(jj=0;jj<len;jj++)
            {
              a=inpek[(bit>>3)]>>(7-(bit&0x7))&1;
              if(a) a=-1;
              bit++;
              for(j=1;j<curr;j++)
                {
                  a+=a;
                  a|=inpek[(bit>>3)]>>(7-(bit&0x7))&1;
                  bit++;
                }
              *utpek++=a;
            }
        }
      else for(jj=0;jj<len;jj++) *utpek++=0;      
      kvar-=len;      
      lentofile+=len;
    }
  for(jj=1;jj<lentofile;jj++)
    {
      ut[jj]+=ut[jj-1];
    }
  return(lentofile);
}
