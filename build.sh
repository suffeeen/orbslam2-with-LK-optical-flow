echo "tar zxvf ORBvoc.tar.gz ..."
cd Vocabulary
tar zxvf ORBvoc.tar.gz
echo "Configuring and building Thirdparty/DBoW2 ..."
cd ..

cd Thirdparty/DBoW2
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j12

cd ../../g2o

echo "Configuring and building Thirdparty/g2o ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j12

cd ../../../

echo "Uncompress vocabulary ..."

cd Vocabulary
tar -xf ORBvoc.txt.tar.gz
cd ..

echo "Configuring and building ORB_SLAM2 ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j12
