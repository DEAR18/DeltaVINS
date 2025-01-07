# DeltaVINS

# How to Build

```
# make build folder
mkdir build
cd build

# make sure to install conan 1.x
pip install conan==1.66.0

# install dependency
conan install ..
# if you get error for missing pangolin,try following in project home folder
cd 3rdParty/pangolin
conan create .

# run in build folder
conan build ..


```
