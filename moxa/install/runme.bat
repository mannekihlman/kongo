cp bin/* /home/

umount /dev/mmc1

mount /dev/mmc1 /mnt
mkdir -p /mnt/flash

cp cfg.txt /mnt/flash/

cp interfaces /etc/network
cp rc.local /etc/rc.d
cp rcS /etc/init.d/

ln -s -f /home/tx /home/a:\tx
cp conf/* /etc
chmod 777 /mnt/flash/cfg.txt
rm /etc/motd

mkdir -p /mnt/flash/novac
chmod 777 /mnt/flash/novac
