#!/bin/bash

# find the hdt directory, required for linking apt and rosdep to its pre-packaged debians
export HDT_DIR=`rospack find rcta`
if [ -z "$HDT_DIR" ]; then
    echo "rospack failed to find the 'hdt' package"
    exit 2
fi

# generate the listing for HDT pre-packaged debians
if [ ! -d "/etc/apt/" ]; then
    echo "/etc/apt/ does not exist. I'm not sure what that means"
fi
export HDT_PACKAGE_LIST_DIR="$HDT_DIR/extern"
echo "Creating HDT repository listing in /etc/apt/sources.list.d/hdt.list"
sudo sh -c "HDT_PACKAGE_LIST_DIR=$HDT_PACKAGE_LIST_DIR \
        echo 'deb [ trusted=yes ] file:$HDT_PACKAGE_LIST_DIR ./' > '/etc/apt/sources.list.d/hdt.list'"
if [ ! -e "/etc/apt/sources.list.d/hdt.list" ]; then
    echo "Failed to generate package listing for HDT pre-packaged debians"
    exit 3
fi

# update package listings
echo "Updating APT repositories listings..."
sudo apt-get update > /dev/null
if [ $? -ne 0 ]; then
    echo "apt-get update failed to update some sources. Attempting to proceed anyway..."
#    exit 4
fi
echo "Finished updating APT repository listings"

# generate the listing for HDT rosdep rules
echo "Creating HDT rosdep rules in /etc/ros/rosdep/sources.list.d/hdt.list"
if [ ! -d "/etc/ros/rosdep/sources.list.d/" ]; then
    # TODO: generate hdt.list that points to rosdep.yaml in the hdt package
    echo "/etc/ros/rosdep/sources.list.d/ does not exist. Attempting to initialize rosdep"
    sudo rosdep init
#    exit 5
fi
sudo sh -c "HDT_DIR=$HDT_DIR \
        echo 'yaml file://$HDT_DIR/rosdep.yaml' > '/etc/ros/rosdep/sources.list.d/hdt.list'"

# update rosdep
echo "Updating rosdep..."
rosdep update

echo "Installing missing rosdep dependencies for source packages"
echo " -> hdt"
rosdep install --ignore-src hdt
echo " -> sbpl_manipulation"
rosdep install --ignore-src sbpl_manipulation
echo " -> pviz"
rosdep install --ignore-src pviz
echo " -> leatherman"
rosdep install --ignore-src leatherman
echo " -> pr2_vfh_database"
rosdep install --ignore-src pr2_vfh_database

