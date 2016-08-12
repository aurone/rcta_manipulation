
if [ `id -u` -eq 0 ]
then
        echo "running as super user"
	source /opt/ros/fuerte/setup.bash
	rmmod esd_usb2 ; modprobe esd_usb2
	roscd hdt_arm_driver/bin
	./motion_download_interface_HDT 
else

        echo "This script needs to be run as superuser"
        exit 1
fi


