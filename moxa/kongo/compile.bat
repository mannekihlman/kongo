#arm-linux-gcc -o kongo_sun kongo_for_solartracker.c -lm
#arm-linux-strip kongo_sun

arm-linux-gcc -o kongo kongo1.c -lm
arm-linux-strip kongo 

