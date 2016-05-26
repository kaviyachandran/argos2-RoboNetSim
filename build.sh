#!/bin/bash

function build_common() {
    # create build directory
    mkdir -p $ARGOS_BUILD_DIR/build/common/simulator; cd $ARGOS_BUILD_DIR/build/common/simulator || { echo "Error: directory $ARGOS_BUILD_DIR/build/common/simulator does not exist, and the attemp to create it failed: permissions trouble?" 1>&2; exit 1; }
    # configure package
    echo "cmake $ARGOS_SOURCE_DIR/common -DCMAKE_INSTALL_PREFIX=/usr -G 'Eclipse CDT4 - Unix Makefiles' -DCMAKE_BUILD_TYPE=${BUILDTYPE} || { echo 'cmake failed' 1>&2; exit 1; }" | sh
    
    # compile
    make || { echo "make failed" 1>&2; exit 1; }
}

function build_simulator() {
    # create build directory
    mkdir -p $ARGOS_BUILD_DIR/build/simulator; cd $ARGOS_BUILD_DIR/build/simulator || { echo "Error: directory $ARGOS_BUILD_DIR/build/simulator does not exist, and the attemp to create it failed: permissions trouble?" 1>&2; exit 1; }
    # configure package
    echo "cmake $ARGOS_SOURCE_DIR/simulator -DCMAKE_INSTALL_PREFIX=/usr -G 'Eclipse CDT4 - Unix Makefiles' -DCMAKE_BUILD_TYPE=${BUILDTYPE} || { echo 'cmake failed' 1>&2; exit 1; }" | sh
    
    # compile
    make || { echo "make failed" 1>&2; exit 1; }
}

BUILDTYPE="Release"
while [ $# -gt 0 ]; do
    OPTION="$1"
    shift

    # clean the build dirs if requested
    if [ "${OPTION}" == "clean" ]; then
        rm -rf build
    fi

    # silent build if requested
    if [ "${OPTION}" == "quiet" ]; then
        QUIET="quiet"
    fi

    # build type
    if [ "${OPTION}" == "debug" ]; then
        export BUILDTYPE="Debug"
    fi
    if [ "${OPTION}" == "release" ]; then
        export BUILDTYPE="Release"
    fi
    if [ "${OPTION}" == "relwithdebinfo" ]; then
        export BUILDTYPE="RelWithDebInfo"
    fi
done

echo $ARGOS_BUILD_DIR
if [ -z "$ARGOS_BUILD_DIR" ]; then
  ARGOS_BUILD_DIR=./
fi

echo "Building ARGOS in $ARGOS_BUILD_DIR"
ARGOS_SOURCE_DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
# create the base build directory
mkdir -p $ARGOS_BUILD_DIR/build

# build the common package
echo "+++++   [ BUILDING THE COMMON PACKAGE FOR THE SIMULATOR ]   +++++"
if [ x${QUIET} == xquiet ]; then
    ( build_common > /dev/null || { echo "An error occurred during compilation."; exit 1; } )
else
    ( build_common || { echo "An error occurred during compilation."; exit 1; } )
fi
echo
echo

# build the simulator package
echo "+++++   [ BUILDING THE SIMULATOR PACKAGE FOR THE SIMULATOR ]   +++++"
if [ x${QUIET} == xquiet ]; then
    ( build_simulator > /dev/null || { echo "An error occurred during compilation."; exit 1; } )
else
    ( build_simulator || { echo "An error occurred during compilation."; exit 1; } )
fi
echo
echo
echo "Compilation successfull."
echo
echo "All the stuff is in the $ARGOS_BUILD_DIR/build/common/simulator and $ARGOS_BUILD_DIR/build/simulator/ directories."

exit 0
