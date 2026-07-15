# First run the following to setup compiler for Axiomtek IFB122:
# source /opt/poky/2.4.4/environment-setup-armv7vehf-neon-poky-linux-gnueabi

# For Linux5.10:
# . /opt/fsl-imx-xwayland/1.0.3/environment-setup-cortexa7t2hf-neon-poky-linux-gnueabi

all: kongo

# Include files
INCLUDE = .

kongo:
	$(CC) -o kongo kongo1.cxx -lusb-1.0 -lavs -lrsb10x -lm -L.

clean:
	rm -f kongo

