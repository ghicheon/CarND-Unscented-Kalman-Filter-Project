mkdir Build
cd Build
cmake -DCMAKE_BUILD_TYPE=Debug ..
make
./UnscentedKF > ../output.log
cd .. ; python nis.py
