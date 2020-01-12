grep 'secs\|ranges' laser.txt > laser_filt.txt
grep 'secs\|x\:\|y\:\|z\:\|w\:' odom.txt > odom_filt.txt
sed -i -e s/x\:// odom_filt.txt
sed -i -e s/y\:// odom_filt.txt
sed -i -e s/z\:// odom_filt.txt
sed -i -e s/w\:// odom_filt.txt
rm odom_filt.txt-e
