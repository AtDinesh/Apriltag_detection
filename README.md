# Apriltag_detection

This is a trial to implement apriltag detection based on https://april.eecs.umich.edu/software/apriltag.html
This code is not intended to improve the detection but only to use the apriltag linrary as an external dependency instead of including all the source files for building stages.

## Requirements

In order to use apriltag as an external library it is first required to install the apriltag library from the archive files downloadable here: https://april.eecs.umich.edu/software/apriltag.html

To install this library, you need to extract the files of the archive and then move to the folder the extraction just created.
It should then be enough to run 

    make

There is no CMakeLists file associated to the original apriltag library. Running the aforementioned command should be enough to install the library. You may have some permission problems due to the installation of the library in /usr/lib.
It is possible to change the installation directory by specifying the PREFIX variable in the first line of the Makefile.

Please be aware that if the installation directory is changed, some environment variables need to be updated as well so that the library can be found for building other programs.
For example, if the PREFIX is set to /local/users/<login>/install/, then you will need to change both CMAKE_PREFIX_PATH and PKG_CONFIG_PATH: 

    export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:/local/users/<login>/install/lib/
    export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:/local/users/<login>/install/lib/pkgconfig