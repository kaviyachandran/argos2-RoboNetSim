#!/bin/bash

PACKAGE_DIR=/tmp/argos_package
PACKAGE_VERSION=1cp

function Usage() {
    echo "Usage: $0 [source|plain|slack|debian|macosx]"
    exit 1
}

function SourcePackage() {
    echo -n "Creating source package... "
    tar cjf ${CURDIR}/argos2-source-$(date +%Y%m%d)-$(uname -m).tar.bz2 .
}

function PlainPackage() {
    echo -n "Creating plain binary package... "
    tar cjf ${CURDIR}/argos2-$(date +%Y%m%d)-$(uname -m).tar.bz2 .
}

function SlackPackage() {
    echo -n "Creating slackware binary package... "
    sudo mkdir -p ${PACKAGE_DIR}/install
    sudo sh -c "cat <<'EOF' > ${PACKAGE_DIR}/install/slack-desc
      |-----handy-ruler------------------------------------------------------|
argos2: ARGoS (multi-physics multi-robot simulator)
argos2:
argos2: ARGoS is a highly scalable multi-robot simulator. Among its
argos2: distinctive features, there are:
argos2: - modularity: robots, devices, physics engines, visualizations and
argos2:               controllers are plug-ins;
argos2: - tunable accuracy;
argos2: - the possibility to run multiple physics engines at the
argos2:   same time.
argos2:
argos2: Packager: Carlo Pinciroli <cpinciro@ulb.ac.be>
EOF
"
    /sbin/makepkg -l y -c n ${CURDIR}/argos2-$(date +%Y%m%d)-$(uname -m)-${PACKAGE_VERSION}.txz &> /dev/null
}

function DebianPackage() {
    echo -n "Creating debian binary package... "
    local PACKAGE_ARCH=$(dpkg --print-architecture)
    sudo mkdir -p debian debian/DEBIAN
    sudo mv usr debian
    sudo sh -c "cat <<'EOF' > debian/DEBIAN/control
Package: argos2
Version: $(date +%Y%m%d)-${PACKAGE_VERSION}
Section: contrib/science
Priority: optional
Architecture: ${PACKAGE_ARCH}
Depends: libgsl0-dev, freeglut3-dev, libqt4-opengl-dev, libxi-dev, libxmu-dev
Recommends: povray (>= 3.6)
Maintainer: Carlo Pinciroli <cpinciro@ulb.ac.be>
Homepage: http://iridia.ulb.ac.be/argos
Description: ARGoS (multi-physics multi-robot simulator)
 ARGoS is a highly scalable multi-robot simulator. Among its distinctive
 features, there are: (i) modularity (robots, devices, physics engines,
 visualizations and controllers are plug-ins), (ii) tunable accuracy, and (iii)
 the possibility to run multiple physics engines at the same time.
EOF
"
    find debian -name '*.so' -print0 | xargs -0 sudo chmod 644
    sudo dpkg-deb --build debian &> /dev/null
    sudo mv debian.deb ${CURDIR}/argos2-$(date +%Y%m%d)-${PACKAGE_ARCH}-${PACKAGE_VERSION}.deb &> /dev/null
}

function MacOSXPackage() {
    echo -n "Adding uninstall script... "
    sudo sh -c "cat <<'EOF' > usr/share/argos2/uninstall_argos.sh
#!/bin/bash

echo '====================='
echo '= ARGoS Uninstaller ='
echo '====================='
echo
echo 'To complete this operation, you will need administrative privileges.'
echo
read -p 'Press CTRL-C to exit, or any other key to proceed...'
echo
echo -n 'Uninstalling ARGoS... ' 
sudo rm -rf /usr/bin/argos
sudo rm -rf /usr/bin/launch_argos
sudo rm -rf /usr/include/argos2
sudo rm -rf /usr/lib/argos2
sudo rm -rf /usr/share/doc/argos2
sudo rm -rf /usr/share/man/man1/argos.1.gz
sudo rm -rf /usr/share/man/man1/launch_argos.1.gz
sudo rm -rf /usr/share/argos2
echo 'done.'
cd ..

EOF
"
    sudo chmod +x usr/share/argos2/uninstall_argos.sh
    echo "OK"
    echo -n "Creating MacOSX binary package... "
    PACKAGE_BASENAME=${CURDIR}/argos2-$(date +%Y%m%d)-macosx-${PACKAGE_VERSION}
    /Developer/usr/bin/packagemaker --root . --out ${PACKAGE_BASENAME}.pkg --id be.ac.ulb.iridia.argos2 --version $(date +%Y%m%d) --target 10.5 --no-relocate --root-volume-only
    echo "OK"
    echo -n "Creating MacOSX disk image... "
    PACKAGE_SIZE=$(du -k ${PACKAGE_BASENAME}.pkg | cut -f1)
    PACKAGE_SIZE=$((PACKAGE_SIZE+1024))
    sudo hdiutil create -size ${PACKAGE_SIZE}k -volname ARGoS2 -fs HFS+ argos-temp-dmg.dmg
    TEMP_DMG=$(sudo hdiutil attach -readwrite -noverify -noautoopen argos-temp-dmg.dmg | grep ARGoS2 | cut -f1)
    sleep 5
    sudo mv -v ${PACKAGE_BASENAME}.pkg /Volumes/ARGoS2
    sudo sync
    sudo hdiutil detach ${TEMP_DMG}
    sudo hdiutil convert argos-temp-dmg.dmg -format UDBZ -o ${PACKAGE_BASENAME}.dmg
}

function AddFindARGoSCMake() {
    sudo mkdir -p ${PACKAGE_DIR}/usr/share/argos2/
sudo sh -c "cat <<'EOF' > ${PACKAGE_DIR}/usr/share/argos2/FindARGoS.cmake
# Find ARGoS
#
# This module defines these variables:
#
#  ARGOS_FOUND       - True if the ARGoS libraries was found
#  ARGOS_INCLUDE_DIR - The include directory of the ARGoS libraries
#  ARGOS_LINK_DIR    - The link directory of the ARGoS libraries
#
# AUTHOR: Carlo Pinciroli <cpinciro@ulb.ac.be>

#
# Check compiler version
#
EXECUTE_PROCESS(COMMAND \${CMAKE_C_COMPILER} -dumpversion OUTPUT_VARIABLE GCC_VERSION)
IF(GCC_VERSION VERSION_GREATER 4.2 OR GCC_VERSION VERSION_EQUAL 4.2)
  MESSAGE(STATUS \"GCC/G++ version >= 4.2\")
ELSE(GCC_VERSION VERSION_GREATER 4.2 OR GCC_VERSION VERSION_EQUAL 4.2)
  MESSAGE(FATAL_ERROR \"To use ARGoS you need to have at least GCC/G++ version 4.2!\")
ENDIF(GCC_VERSION VERSION_GREATER 4.2 OR GCC_VERSION VERSION_EQUAL 4.2)

#
# Find the header file
#
FIND_PATH(ARGOS_INCLUDE_DIR
  NAMES
  argos2/simulator/simulator.h
  PATHS
  /usr/include
  /usr/local/include
  DOC \"ARGoS header location\"
)

#
# Find the library directory
#
IF(APPLE)
  FIND_PATH(ARGOS_LINK_DIR
    NAMES
    libargos2_simulator.dylib
    PATHS
    /usr/lib/argos2
    /usr/local/lib/argos2
    DOC \"ARGoS library location\"
  )
ELSE(APPLE)
  IF(UNIX)
    FIND_PATH(ARGOS_LINK_DIR
      NAMES
      libargos2_simulator.so
      PATHS
      /usr/lib/argos2
      /usr/local/lib/argos2
      DOC \"ARGoS library location\"
    )
  ENDIF(UNIX)
ENDIF(APPLE)

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(ARGoS DEFAULT_MSG ARGOS_LINK_DIR ARGOS_INCLUDE_DIR)

IF(ARGOS_INCLUDE_DIR AND ARGOS_LINK_DIR)
  SET(ARGOS_INCLUDE_DIRS ${ARGOS_INCLUDE_DIR})
  SET(ARGOS_LINK_DIRS	 ${ARGOS_LINK_DIR})
ENDIF(ARGOS_INCLUDE_DIR AND ARGOS_LINK_DIR)

MARK_AS_ADVANCED(ARGOS_INCLUDE_DIR)
MARK_AS_ADVANCED(ARGOS_LINK_DIR)

# Needed to use TinyXML++ bindings
ADD_DEFINITIONS(-DTIXML_USE_TICPP)
EOF
"
}

function AddQTOpenGLStuff() {
    sudo mkdir -p ${PACKAGE_DIR}/usr/share/argos2/simulator/visualizations/qt-opengl
    sudo cp -r simulator/visualizations/qt-opengl/textures \
        simulator/visualizations/qt-opengl/resources \
        simulator/visualizations/qt-opengl/icons \
        ${PACKAGE_DIR}/usr/share/argos2/simulator/visualizations/qt-opengl
}

function AddPOVRayStuff() {
    sudo mkdir -p ${PACKAGE_DIR}/usr/share/argos2/simulator/visualizations/povray
    sudo cp -r simulator/visualizations/povray/ini \
        simulator/visualizations/povray/include \
        simulator/visualizations/povray/scripts \
        simulator/visualizations/povray/textures \
        simulator/visualizations/povray/render_all_frames_on_cluster.sh \
        simulator/visualizations/povray/render_all_frames_on_pc.sh \
        simulator/visualizations/povray/render_single_frame_on_pc.sh \
        ${PACKAGE_DIR}/usr/share/argos2/simulator/visualizations/povray
}

function AddLicenses() {
    sudo mkdir -p ${PACKAGE_DIR}/usr/share/doc/argos2/licenses
    sudo cp LICENSE.txt ${PACKAGE_DIR}/usr/share/doc/argos2/licenses/ARGoS.txt
    sudo cp simulator/libs/FreeImage/license-fi.txt ${PACKAGE_DIR}/usr/share/doc/argos2/licenses/FreeImage.txt
    sudo cp simulator/physics_engines/dynamics2d/chipmunk-physics/LICENSE.txt ${PACKAGE_DIR}/usr/share/doc/argos2/licenses/Chipmunk.txt
}

function AddCopyright() {
    sudo sh -c "cat <<EOF > ${PACKAGE_DIR}/usr/share/doc/argos2/copyright
ARGoS (multi-physics multi-robot simulator)
Copyright 2006-2011 Carlo Pinciroli <cpinciro@ulb.ac.be>

For licensing information, refer to the files in the
/usr/share/doc/argos2/licenses directory.
EOF
"
}

function AddCodingGuidelines() {
    ( cd documentation/coding_guidelines; pdflatex coding_guidelines.tex &> /dev/null; pdflatex coding_guidelines.tex &> /dev/null )
    sudo mkdir -p ${PACKAGE_DIR}/usr/share/doc/argos2/coding_guidelines
    sudo cp documentation/coding_guidelines/coding_guidelines.tex documentation/coding_guidelines/coding_guidelines.pdf ${PACKAGE_DIR}/usr/share/doc/argos2/coding_guidelines
}

function AddUserManual() {
    ( cd documentation/manual; pdflatex argos_user_manual.tex &> /dev/null; pdflatex argos_user_manual.tex &> /dev/null )
    sudo mkdir -p ${PACKAGE_DIR}/usr/share/doc/argos2/manual
    sudo cp documentation/manual/argos.sty documentation/manual/*.tex documentation/manual/argos_user_manual.pdf ${PACKAGE_DIR}/usr/share/doc/argos2/manual
}

function AddManPage() {
    sudo mkdir -p ${PACKAGE_DIR}/usr/share/man/man1
    sudo cp documentation/argos.1 ${PACKAGE_DIR}/usr/share/man/man1
    sudo gzip -9 ${PACKAGE_DIR}/usr/share/man/man1/argos.1
    ( cd ${PACKAGE_DIR}/usr/share/man/man1; sudo ln -s argos.1.gz launch_argos.1.gz )
}

function MakeInstall() {
    local BUILD_TYPE=$(cmake -L . | grep CMAKE_BUILD_TYPE | cut -d= -f2)
    if [ "x${BUILD_TYPE}" != "xRelease" ]; then
        echo "In build directory: $PWD: the code is not compiled in Release."
        exit 1
    fi
    sudo make DESTDIR=${PACKAGE_DIR} install &> /dev/null
}

function AddBuildScript() {
cat <<'EOF' > $1
#!/bin/bash

function build_common() {
    # create build directory
    mkdir -p build/common/simulator; cd build/common/simulator || { echo "Error: directory build/common/simulator does not exist, and the attemp to create it failed: permissions trouble?" 1>&2; exit 1; }
    # configure package
    echo "cmake ../../../common -DCMAKE_INSTALL_PREFIX=/usr -G 'Eclipse CDT4 - Unix Makefiles' -DCMAKE_BUILD_TYPE=${BUILDTYPE} || { echo 'cmake failed' 1>&2; exit 1; }" | sh
    
    # compile
    make || { echo "make failed" 1>&2; exit 1; }
}

function build_simulator() {
    # create build directory
    mkdir -p build/simulator; cd build/simulator || { echo "Error: directory build/simulator does not exist, and the attemp to create it failed: permissions trouble?" 1>&2; exit 1; }
    # configure package
    echo "cmake ../../simulator -DCMAKE_INSTALL_PREFIX=/usr -G 'Eclipse CDT4 - Unix Makefiles' -DCMAKE_BUILD_TYPE=${BUILDTYPE} || { echo 'cmake failed' 1>&2; exit 1; }" | sh
    
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

# create the base build directory
mkdir -p build

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
echo "All the stuff is in the build/common/simulator and build/simulator/ directories."

exit 0
EOF
}

function PopulateSource() {
    echo -n "Creating tmp package directory... "
    sudo rm -rf ${PACKAGE_DIR}
    mkdir -p ${PACKAGE_DIR}
    echo "OK"
    echo -n "Populating tmp package directory... "
    mkdir -p ${PACKAGE_DIR}/argos2
    # populate the base directory
    cp -a LICENSE.txt ${PACKAGE_DIR}/argos2/    
    ( cd ${PACKAGE_DIR}/argos2; ln -s . argos2 )
    AddBuildScript ${PACKAGE_DIR}/argos2/build.sh
    chmod +x ${PACKAGE_DIR}/argos2/build.sh
    cp -a make_distribution.sh ${PACKAGE_DIR}/argos2/    
    chmod +x ${PACKAGE_DIR}/argos2/make_distribution.sh
    # populate the common/ directory
    mkdir -p ${PACKAGE_DIR}/argos2/common
    cp -a common/cmake ${PACKAGE_DIR}/argos2/common
    cp -a common/CMakeLists.txt ${PACKAGE_DIR}/argos2/common
    cp -a common/control_interface ${PACKAGE_DIR}/argos2/common
    cp -a common/utility ${PACKAGE_DIR}/argos2/common
    # populate the simulator/ directory
    mkdir -p ${PACKAGE_DIR}/argos2/simulator
    cp -a simulator/actuators ${PACKAGE_DIR}/argos2/simulator
    cp -a simulator/cmake ${PACKAGE_DIR}/argos2/simulator
    cp -a simulator/CMakeLists.txt ${PACKAGE_DIR}/argos2/simulator
    cp -a simulator/dynamic_linking ${PACKAGE_DIR}/argos2/simulator
    cp -a simulator/factories ${PACKAGE_DIR}/argos2/simulator
    cp -a simulator/libs ${PACKAGE_DIR}/argos2/simulator
    cp -a simulator/*.h ${PACKAGE_DIR}/argos2/simulator
    cp -a simulator/*.cpp ${PACKAGE_DIR}/argos2/simulator
    cp -a simulator/physics_engines ${PACKAGE_DIR}/argos2/simulator
    cp -a simulator/sensors ${PACKAGE_DIR}/argos2/simulator
    cp -a simulator/simulator.cpp ${PACKAGE_DIR}/argos2/simulator
    cp -a simulator/simulator.h ${PACKAGE_DIR}/argos2/simulator
    cp -a simulator/space ${PACKAGE_DIR}/argos2/simulator
    cp -a simulator/visualizations ${PACKAGE_DIR}/argos2/simulator
    # populate the documentation/ directory
    mkdir -p ${PACKAGE_DIR}/argos2/documentation
    cp -a documentation/argos.1 ${PACKAGE_DIR}/argos2/documentation
    cp -a documentation/coding_guidelines ${PACKAGE_DIR}/argos2/documentation
    cp -a documentation/manual ${PACKAGE_DIR}/argos2/documentation
    echo "OK"
}

function PopulateBinary() {
    echo -n "Creating tmp package directory... "
    sudo rm -rf ${PACKAGE_DIR}
    sudo mkdir -p ${PACKAGE_DIR}
    echo "OK"
    # go through directories and install stuff
    echo -n "Populating tmp package directory... "

    for DIR in build/common/simulator build/simulator
    do
        ( cd $DIR; MakeInstall ) || exit 1
    done
    echo "OK"

    # strip symbols
    if [ "x${MODE}" != "xmacosx" ]; then
	echo -n "Stripping symbols... "
	find ${PACKAGE_DIR} -name '*.so' -print0 | xargs -0 sudo strip -s
	sudo strip -s ${PACKAGE_DIR}/usr/bin/argos
	echo "OK"
    fi
    
    # add launch wrapper
    echo -n "Creating launch wrapper... "
    if [ "x${MODE}" = "xmacosx" ]; then
	export LDLIBPATH=DYLD_LIBRARY_PATH
    else
	export LDLIBPATH=LD_LIBRARY_PATH
    fi
    sudo sh -c "cat <<'EOF' > ${PACKAGE_DIR}/usr/bin/launch_argos
#!/bin/bash

# Set the resource directory
export ARGOSINSTALLDIR=/usr/share/argos2

# Set the library path
if [ -n \"\${$LDLIBPATH}\" ]
then
    $LDLIBPATH=\${$LDLIBPATH}:/usr/lib/argos2
else
    $LDLIBPATH=/usr/lib/argos2
fi
export $LDLIBPATH

# Execute argos with bash internal command exec, which replaces
# the script with the argos executable. In this way, there is
# only one PID and the return value is handled correctly.
exec argos \"\$@\"
EOF
"
    sudo chmod +x ${PACKAGE_DIR}/usr/bin/launch_argos
    echo "OK"

    # add shared files
    echo -n "Adding shared files... "
    AddFindARGoSCMake
    AddQTOpenGLStuff
    AddPOVRayStuff
    echo "OK"

    # add documentation
    echo -n "Adding documentation... "
    AddLicenses
    AddCopyright
    AddCodingGuidelines
    AddUserManual
    AddManPage
    echo "OK"
}

# check mode
if [ $# -eq 1 ]; then
    MODE=$1
    if [ "x${MODE}" != "xsource" -a "x${MODE}" != "xplain" -a "x${MODE}" != "xslack" -a "x${MODE}" != "xdebian" -a "x${MODE}" != "xmacosx" ]; then
        Usage
    fi
else
    Usage
fi

# create package
if [ "x${MODE}" = "xsource" ]; then
    PopulateSource
    ( export CURDIR=$PWD; cd ${PACKAGE_DIR}; SourcePackage )
else if [ "x${MODE}" = "xplain" ]; then
    PopulateBinary
    ( export CURDIR=$PWD; cd ${PACKAGE_DIR}; PlainPackage )
else if [ "x${MODE}" = "xslack" ]; then
    PopulateBinary
    ( export CURDIR=$PWD; cd ${PACKAGE_DIR}; SlackPackage )
else if [ "x${MODE}" = "xdebian" ]; then
    PopulateBinary
    ( export CURDIR=$PWD; cd ${PACKAGE_DIR}; DebianPackage )
else if [ "x${MODE}" = "xmacosx" ]; then
    PopulateBinary
    ( export CURDIR=$PWD; cd ${PACKAGE_DIR}; MacOSXPackage )
fi
fi
fi
fi
fi
echo "OK"

# cleanup
echo -n "Cleaning up... "
sudo rm -rf ${PACKAGE_DIR}
echo "OK"
