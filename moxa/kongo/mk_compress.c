
#define hdr_version 5

struct MKZYhdr
{
  char ident[4];                  // 0 "MKZY"
  unsigned short hdrsize;         // 4 this is the size in bytes of the header
  unsigned short hdrversion;      // 6 version of the header
  unsigned short size;            // 8 the number of bytes with compressed data
  unsigned short checksum;        // 10 checksum for the uncompressed data
  char name[12];                  // 12 the name of this specific measurement
  char instrumentname[16];        // 24 the name of the instrument
  unsigned short startc;          // 40 the startchannel for the first data-point
  unsigned short pixels;          // 42 number of pixels saved in the data-field
  short viewangle;                // 44 the viewing angle of the instrument
  unsigned short scans;           // 46 total number of scans added
  short exptime;                  // 48 exposure time, negative if set automatic
  unsigned char channel;          // 50 channel of the spectrometer, typically 0
  unsigned char flag;             // 51 for further use, currently contains the
                                  // status of the solenoid(s) in bit 0 and 1
  unsigned long date;             // 52 date
  unsigned long starttime;        // 56 time when the scanning was started
  unsigned long stoptime;         // 60 time when the scanning was finished
  double lat;                     // 64 GPS latitude in degrees
  double lon;                     // 72 GPS longitude in degrees
  short altitude;                 // 80 new in version 2
  char measureidx;                // 82 new in version 2, nr between 0 and measurecnt-1
  char measurecnt;                 // 83 new in version 2
                                   // number of MEAS= lines in cfg.txt
  short viewangle2;                // 84 new in version 3, direction of 2nd motor
  short compassdir;                // 86 new in version 3, given in cfg.txt or read from compass
  short tiltX;                     // 88 new in version 3, given in cfg.txt
  short tiltY;                     // 90 new in version 3, given in cfg.txt
  float temperature;               // 92 new in version 3, given in cfg.txt or read from spectrometer with "?t" command
  char coneangle;                  // 96 new in version 4, given in cfg.txt
  char extra;                      // 97
  // extra inserted here to fix problem with alignment
  //   UNUSED can be used for new purposes

  unsigned short ADC[8];           // 98 new in version 5
}; // Total length 98+16=114

struct MKZYhdr MKZY;


#define headsiz 12

void SetBit(unsigned char *pek,long bit)
{
  unsigned short ut=0x80;
  pek[bit>>3]|=(ut>>(bit&7));
}

void ClearBit(unsigned char *pek,long bit)
{
  unsigned char ut=0x80;
  pek[bit>>3]&=~(ut>>(bit&7));
}

void WriteBits(short a,short curr,long *inpek,unsigned char *utpek,long bitnr)
{
  short jj,j;
  long utwrd;
  long kk;
  unsigned short utlng=0x80;
  
  if(debugflag>7) printf("WriteBits %d %d %ld\n",a,curr,bitnr);

  utwrd=(unsigned long)( (a<<5) | (curr & 0x1f) );  
  kk=1L<<(headsiz-1);
  for(j=0;j<headsiz;j++)
    {
      if( utwrd & kk ) utpek[(bitnr>>3)]|=(utlng>>(bitnr&7));
      bitnr++;
      kk=kk>>1;
    }
  if(curr)
    {
      for(jj=0;jj<a;jj++)             /* spara undan alla */
	{
	  kk=(1L<<(curr-1));
	  utwrd=*inpek++;
	  for(j=0;j<curr;j++)
	    {
	      if( utwrd & kk ) utpek[(bitnr>>3)]|=(utlng>>(bitnr&7));
	      bitnr++;
	      kk=kk>>1;
        }
	}
    }
  if(debugflag>7) printf("WriteBits done\n");

}

short BitsPrec(long i)
{
short j=1;
if(!i) return(0);
if(i<0)
        {
        if(i == -1) return(1);
        while(i != -1 )
                {
                j++;
                i=(i>>1);
                }
        }
else
while( i )
        {
        j++;
        i=(i>>1);
        }
return(j);
}

long bitnr;
long *strt;

void PackSeg(unsigned char *utpek, long *kvar )
{
  short len[33];
  long j;
  long *incpy;
  short curr,i,a;
  
  if(debugflag>7) printf("PackSeg kvar=%ld\n",*kvar);

  for(j=0;j<33;j++) len[j]=0;
  incpy=strt;

  i=BitsPrec(*incpy++);
  curr=i; 
  a=0;
  do {
    a++;
    for(j=0;j<curr;j++)
      {
        if(i>j) len[j]=0;
        else {
          len[j]++;
          if( len[j]*(curr-j)>headsiz*2)
            {
              a-=len[j];
              goto Fixat;
            }
        }
      } 
    i=BitsPrec(*incpy++);    
    if(i>curr)
      {
        /* i har blivit för stort. Vi ska då titta bakåt så att
           vi tjänar in plats bakåt också på att öka bitantalet */      
        if( a*(i-curr)>headsiz ) goto Fixat;
        
        /* gå till fixat om det inte lönar sig att öka
           bitantalet på den föregående gruppen */
               
        while(curr!=i)          /* det lönade sig att byta */
          {
            len[curr]=a;
            curr++;                     /* öka bitantalet */
          }
      }
  } while( a<*kvar && a<127 );
 Fixat:
    
  WriteBits(a,curr,strt,utpek,bitnr);  
  *kvar -=a;
  strt += a;                  /* öka strt */
  bitnr += a*curr+headsiz;

}

unsigned short mk_compress(long *in,unsigned char *ut,unsigned short size)
{
  long kvar;
  unsigned short outsize;

  strt=in;
  kvar=size;
  bitnr=0;
  do {
    PackSeg(ut,&kvar);
  } while( kvar>0);
  outsize=(bitnr+7)>>3;
  return(outsize);
}
