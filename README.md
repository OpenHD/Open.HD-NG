[![Actions Status](https://github.com/OpenHD/Open.HD-NG/workflows/build-debs/badge.svg)  [ ![Download](https://api.bintray.com/packages/webbbn/openhd_test/Open.HD-NG/images/download.svg) ](https://bintray.com/webbbn/openhd_test/Open.HD-NG/_latestVersion)

# Open.HD-NG
Refactor and update of OpenHD

## Installation

### Update the submodules

~~~
git submodule update --init
~~~

### Install dependencies on a Raspberry Pi

~~~
sudo apt install cython libv4l-dev python3-numpy python3-serial python3-lxml libasio-dev
~~~

### Install Raspberry Pi specific dependencies

~~~
sudo apt install python3-picamera
~~~

Install wifibroadcast_bridge from here: https://github.com/webbbn/wifibroadcast_bridge

### Configure the software

This defaults to installing in the system directories. Add -DCMAKE_INSTALL_PREFIX=<dir> to install in a different directory.

~~~
mkdir build
cd build
cmake ..
~~~

### Install the software

Add `sudo` if you're installing in the system directories and not already running as root.

~~~
make install
~~~

### Install into system directories

~~~
cmake -DCMAKE_INSTALL_PREFIX=/ ..
sudo make install
~~~

### Activate system services (after intalling in root)

~~~
sudo ../scripts/postinst
~~~

### Create a debian package

~~~
cpack
~~~

This can be installed with:

~~~
sudo dpkg -i *.deb
~~~

## Configuration

Prior to installation various configuration values can be changed in the conf/openhd file. This file gets intalled into /etc/default/openhd during installation.

## Starting

The installation includes a standard systemd service file. The following commands will start the service and enable it to automatically start on boot.

~~~
sudo systemctl enable openhd
sudo systemctl start openhd
~~~
