#!/bin/bash
# Script for building clibspi on Linux machine

set -e
project=clibspi

#expected build dir structure
buildexp=build/Unix_Makefiles

currentdir=$PWD
builddir=./${buildexp}

mkdir -p ${builddir}

#debug
if [ ! -z $1 ]
then
# user selected
        if [ $1 == "Debug" ]
        then
                debug=-DCMAKE_BUILD_TYPE=$1
        else
                debug=-DCMAKE_BUILD_TYPE="Debug"
        fi
else
# release by default
        debug=""
fi
echo ${debug}

#launch cmake to generate build environment
pushd ${builddir}
cmake -G "Unix Makefiles" ${currentdir} ${debug}
popd

#build from generated build environment
cmake --build ${builddir}
