gtsam库安装
git clone https://github.com/borglab/gtsam.git
cd gtsam
git checkout tags/4.2.0-ros
mkdir -p build
cd build
cmake -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF ..
make -j$(nproc)
sudo make install
sudo ln -s /usr/local/lib/libmetis-gtsam.so /usr/lib/libmetis-gtsam.so