sudo apt-get update && sudo apt-get -y upgrade
sudo apt-get install curl 
sudo apt-get install build-essential 
sudo apt-get install vim 
sudo apt-get install nano
sudo apt-get -y install libglu1-mesa-dev freeglut3-dev mesa-common-dev xterm vnc4server
set -x \
	&& sudo mkdir -p /usr/src/python \
	&& sudo curl -SL "https://www.python.org/ftp/python/3.4.2/Python-3.4.2.tar.xz" \
		| sudo tar -xJC /usr/src/python --strip-components=1 \
	&& cd /usr/src/python \
	&& sudo ./configure --enable-shared \
	&& sudo make -j$(nproc) \
	&& sudo make install \
	&& sudo ldconfig \
	&& sudo find /usr/local \
		\( -type d -a -name test -o -name tests \) \
		-o \( -type f -a -name '*.pyc' -o -name '*.pyo' \) \
		-exec rm -rf '{}' + \
	&& sudo rm -rf /usr/src/python

cd /usr/local/bin \
	&& sudo ln -s easy_install-3.4 easy_install \
	&& sudo ln -s idle3 idle \
	&& sudo ln -s pip3 pip \
	&& sudo ln -s pydoc3 pydoc \
	&& sudo ln -s python3 python \
	&& sudo ln -s python-config3 python-config
	
sudo apt-get -y install apt-utils wget git libfreetype6 libxi-dev
sudo apt-get -y install pkg-config 
sudo apt-get install cmake

sudo mkdir /opt/blender
sudo wget -q http://mirror.cs.umn.edu/blender.org/release/Blender2.73/blender-2.73-linux-glibc211-x86_64.tar.bz2 -O /opt/blender/blender-2.73.tar.bz2
cd /opt/blender
sudo tar jxf blender-2.73.tar.bz2


cd /usr/src
sudo git clone https://github.com/morse-simulator/morse -b master
cd /usr/src/morse

sudo mkdir build
cd /usr/src/morse/build
export MORSE_BLENDER=/opt/blender/blender-2.73-linux-glibc211-x86_64/blender
sudo cmake ..
sudo make install
morse --noaudio check
