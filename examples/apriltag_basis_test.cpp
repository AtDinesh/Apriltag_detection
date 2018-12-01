#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include <ctype.h>
#include <unistd.h>
#include <math.h>

#include "apriltag.h"
#include "tag36h11.h"
#include "tag36h10.h"
#include "tag25h9.h"
#include "tag25h7.h"
#include "common/zarray.h"

int main(int argc, char *argv[])
{
    apriltag_family_t *tf = NULL;
    tf = tag36h11_create();

    //apriltag_detector_t *td = apriltag_detector_create();

    return 0;
}