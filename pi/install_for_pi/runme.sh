cp kongo ~
cp kongo.sh ~
cp cfg.txt ~

sudo cp showlog /usr/bin
sudo cp nolog /usr/bin
sudo cp libavs.so /usr/lib/libavs.so.0

# enable any user to use Avantes USB devices
sudo cp 99-com.rules /etc/udev/rules.d/99-com.rules

# enable sudo users to use date to set time&date without password
sudo cp 010_date /etc/sudoers.d

# enable ssh login
sudo systemctl enable ssh

sudo cp kongo.service /etc/systemd/system/kongo.service
sudo systemctl enable kongo

# assure flash to storage
sudo sync

