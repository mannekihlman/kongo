
#include <sys/socket.h>
#include <resolv.h>

#define PORT_FTP 21
int ftptimeout=10000;

int IsDataAvail(int fil,int wait)
{
  fd_set rfds;
  struct timeval tv;
  int s;
  FD_ZERO(&rfds);
  FD_SET(fil,&rfds);
  tv.tv_sec=wait;
  tv.tv_usec=0;
  return(select(fil+1,&rfds,NULL,NULL,&tv));
}

int ftpSendCommand(int sd,char *txt)
{
  int cmd,status,siz;
  send(sd,txt,strlen(txt),MSG_DONTWAIT);
  siz=0;
  if(IsDataAvail(sd,ftptimeout))
    {
      siz=recv(sd,txt,1024,O_NONBLOCK);
    }
  if(!siz) return(0);
  cmd=atoi(txt);
  return(cmd);
}

int ftpReceiveReply(int sd,char *txt)
{
  int cmd,status,siz;
  siz=0;
  if(IsDataAvail(sd,ftptimeout))
    {
      siz=recv(sd,txt,1024,O_NONBLOCK);
    }
  if(!siz) return(0);
  cmd=atoi(txt);    
  return(cmd);
}

int ftpSendDataToSocket(int sd, const char* const txt,int siz)
{
  unsigned int i,timeout;
  int result,error;
  for( i = 0, timeout = ftptimeout; i < siz && timeout > 0; )
    {
      if( ( result = send( sd, txt+i, siz-i,MSG_DONTWAIT) ) > 0 )
        {
        i += result;
	timeout = ftptimeout;
        }
      else
        {
        msleep( 1000 );
        timeout -= ( timeout >= 1000 ) ? 1000 : timeout;
        }
      }      
    if( timeout == 0 )
    {
      return(0);
    }
  return(1);
}

int ftpSendFileToSocket(int sd,const char* const filename)
{
  int len;
  FILE *fil;

  if(debugflag>2) printf("Sending %s\n",filename);

  fil = fopen(filename,"r+b");
  if( fil< (FILE *) 1 ) return(0);
  do {
    len=fread(sbuf,1,16384,fil);
    if(debugflag>2) printf("%d bytes\n",len);
    if(!ftpSendDataToSocket(sd,(char *)sbuf,len)) { fclose(fil); return(0); }
  } while(len==16384);
  fclose(fil);
  return(1);
}

int ftpconnect()
{
  struct sockaddr_in socketAddress;
  int sd,status,result,timout;
  int dataSocket = 0;
  unsigned int listenPort;    
  char *pt;
  unsigned int tmp[6];
  char remoteIp[16];
  struct hostent *hname;

  sd = socket(PF_INET,SOCK_STREAM, 0);
  if(sd==-1) return 0;
  
  fcntl(sd,F_SETFL,O_NONBLOCK);
  //  SetBlockingMode(sd,0);

  socketAddress.sin_family = AF_INET;
  socketAddress.sin_addr.s_addr = serverip;
  socketAddress.sin_port = htons( PORT_FTP );
  for(timout=ftptimeout;connect(sd,(struct sockaddr *)&socketAddress,sizeof(socketAddress))!=0 && timout>0; timout-=1000)
      msleep( 1000 );
  if(timout<=0)      
     {
      if(debugflag) puts("Timeout when connecting FTP");
      close( sd); return 0; }

    if(debugflag>1) puts("FTP Connected");
  
    if(ftpReceiveReply(sd,txt)!=220)
       { close(sd); return 0; }
    if(debugflag>1) printf("Got 220");

    sprintf(txt,"USER %s\r\n",username);
    if( ftpSendCommand(sd,txt)!=331)
       { close(sd); return 0; }
    if(debugflag>1) printf(",331");

    sprintf(txt,"PASS %s\r\n",password);
    if( ftpSendCommand(sd,txt)!=230)
       { close(sd); return 0; }
    if(debugflag>1) printf(",230");

    strcpy(txt,"TYPE I\r\n");
    if( ftpSendCommand(sd,txt)!=200)
       { close(sd); return 0; }
    if(debugflag>1) printf(",200");

    strcpy(txt,"PASV\r\n");
    if( ftpSendCommand(sd,txt)!=227)
       { close(sd); return 0; }
    if( strlen( txt ) < 3  ) return 0;
    if(debugflag>1) printf(",227\n");

  pt=strstr(txt,"(");
  if(pt==0) return 0;
  if( sscanf( pt+1, "%u,%u,%u,%u,%u,%u",
      &tmp[0], &tmp[1], &tmp[2], &tmp[3], &tmp[4], &tmp[5] ) != 6 ||
      tmp[0] > 255 || tmp[1] > 255 || tmp[2] > 255 ||
      tmp[3] > 255 || tmp[4] > 255 || tmp[5] > 255 ) 
    return 0;

  sprintf( remoteIp, "%u.%u.%u.%u", tmp[0], tmp[1], tmp[2], tmp[3] );
  listenPort = ( tmp[4] << 8 ) + tmp[5];

  if(debugflag>1) printf("Passive mode %s %d\n",remoteIp,listenPort);

  if( ( dataSocket = socket(PF_INET, SOCK_STREAM,0) ) == -1 )
    { if(debugflag>1) puts("Unable to open socket"); return 0; }
  // Prepare information for connection
  socketAddress.sin_family = AF_INET;
  socketAddress.sin_port = htons( listenPort );

  hname=gethostbyname(remoteIp);

  memcpy( &socketAddress.sin_addr, hname->h_addr, hname->h_length );       

  // Connect to server
  if( connect( dataSocket, (struct sockaddr *)&socketAddress,
	       sizeof(socketAddress) ) != 0 )
  {
    close( dataSocket );
    close( sd );    
    return 0;
  }

  GetCPUTime();
  sprintf(txt,"STOR upload_%s_%06ld_%08ld.pak\r\n",instrumentname,gpsdate,gpstime);
  if(debugflag>1) puts(txt);
  if( ftpSendCommand(sd,txt)!=150)
       { close(sd); printf("Did not get 150: %s",txt); return 0; }
  if(debugflag>1) puts("Got 150");

  if(ftpSendFileToSocket(dataSocket,"upload.pak")==0)
    {
      close(dataSocket);
      close(sd);      
      return(0);
    }
  close(dataSocket);
  
  // Receive and analyse second reply
  while( ( result = ftpReceiveReply( sd, txt ) ) == 0 );
  switch( result )
  {
    case 226: case 250:  // OK
      close(sd);
      return 1;

    case 110: case 425: case 426: case 451: case 551: case 552:
    case 0:
    default:  // Error
      close(sd);
      return 0;                           
  }     
}
