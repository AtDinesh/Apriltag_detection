# Apriltag_detection

This is a trial to implement apriltag detection based on https://april.eecs.umich.edu/software/apriltag.html
This code is not intended to improve the detection but only to use the apriltag linrary as an external dependency instead of including all the source files for building stages.

## Requirements

In order to use apriltag as an external library it is first required to install the apriltag library from the archive files downloadable here: https://april.eecs.umich.edu/software/apriltag.html

To install this library, you need to extract the files of the archive and then move to the folder the extraction just created.
It should then be enough to run 

    make install

There is no CMakeLists file associated to the original apriltag library. Running the aforementioned command should be enough to install the library. You may have some permission problems due to the installation of the library and creation of the file `/usr/local/lib/libapriltag.so`.
There are 2 possibilities to solve this issue described hereafter. Please read both and use the option that you find the more suitable to your case. 

**OPTION 1**: It is possible to change the installation directory by specifying the `PREFIX` variable in the first line of the Makefile.

Please be aware that if the installation directory is changed, some environment variables need to be updated as well so that the library can be found for building other programs.
For example, if the `PREFIX` is set to `/local/users/<login>/install/`, then you will need to change both **CMAKE_PREFIX_PATH** and **PKG_CONFIG_PATH**: 

    export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:/local/users/<login>/install/lib/
    export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:/local/users/<login>/install/lib/pkgconfig

You may also need to add to your `.bashrc` file: 

    export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/local/users/<login>/install/lib
    export PATH=${PATH}:/local/users/<login>/install/bin
    export CPATH=${CPATH}:/local/users/<login>/install/include

If `make` returns an error at the end of installation due to `ldconfig`, then you can just run

    sudo ldconfig

**OPTION 2**: You can edit the Makefile so that the `install.sh` script and the `ldconfig` command are run with sudo rights.

Finally, some other dependencies are required:
* OpenCV (version >= 2.3)
* Threads library