

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

main()
{
  float test1=0.0;
  FILE *f;
  f=fopen("tst","r+b");  
  if(f>(FILE *)1)
    {
      fread(&test1,sizeof(test1),1,f);
      fclose(f);
      printf("%lf\n",test1);
    }

  float test2=1.03;
  f=fopen("tst2","r+b");  
  if(f<(FILE *)1) f=fopen("tst2","w+b");
  if(f>(FILE *)1)
    {
      fwrite(&test2,sizeof(test2),1,f);
      fclose(f);
    }

}
