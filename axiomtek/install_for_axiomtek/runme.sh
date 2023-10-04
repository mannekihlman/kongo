echo "Installing files for NOVAC"

cp etc/* /etc
cp home/* /home
cp bin/* /usr/bin
cp lib/* /lib
cp xinetd.d/* /etc/xinetd.d
cp interfaces /etc/network
cp cfg.txt /home/root

sync
echo "Installation done"

