echo "Installing files for NOVAC"
cp etc/* /etc
cp home/* /home
cp bin/* /usr/bin
cp sys_rdy.sh /etc/init.d
sync
echo "Installation done"
