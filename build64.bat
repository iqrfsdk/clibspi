set project=clibspi

rem //expected build dir structure
set buildexp=build\\Visual_Studio_14_2015\\x64

set currentdir=%cd%
set builddir=.\\%buildexp%

mkdir %builddir%

rem //launch cmake to generate build environment
pushd %builddir%
cmake -G "Visual Studio 14 2015 Win64" %currentdir%
popd

rem //build from generated build environment
cmake --build %builddir%
