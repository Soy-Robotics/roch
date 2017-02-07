Udev rules
==========

These udev rules are provided as a reference for some sensors typically installed on the Roch. If you are using an
SawYer-provided image to setup your Roch PC, they should be installed automatically.

To install them manually, execute:

sudo cp $(rospack find roch_bringup)/udev/* /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo service udev restart && sudo udevadm trigger
