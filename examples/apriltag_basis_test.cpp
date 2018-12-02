#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include <ctype.h>
#include <unistd.h>
#include <math.h>

#include "opencv2/opencv.hpp"
#include "apriltag.h"
#include "tag36h11.h"
//#include "tag36h10.h"
//#include "tag25h9.h"
//#include "tag25h7.h"

#include "common/image_u8.h"
#include "common/image_u8x4.h"
#include "common/pjpeg.h"
#include "common/zarray.h"

bool str_ends_with (std::string const &fullString, std::string const &ending) {
    if (fullString.length() >= ending.length()) {
        return (0 == fullString.compare (fullString.length() - ending.length(), ending.length(), ending));
    } else {
        return false;
    }
}

int main(int argc, char *argv[])
{
    apriltag_family_t *tf = NULL;
    tf = tag36h11_create();

    apriltag_detector_t *td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);

    cv::Mat frame, gray;

    int quiet = 0;
    int maxiters = 5;
    const int hamm_hist_max = 10;

    for (int iter = 0; iter < maxiters; iter++) {

        int total_quads = 0;
        int total_hamm_hist[hamm_hist_max];
        memset(total_hamm_hist, 0, sizeof(total_hamm_hist));
        double total_time = 0;

        if (maxiters > 1)
            printf("iter %d / %d\n", iter + 1, maxiters);

        for (int input = 1; input <= argc; input++) {

            int hamm_hist[hamm_hist_max];
            memset(hamm_hist, 0, sizeof(hamm_hist));

            char *path = argv[input];
            //zarray_get(inputs, input, &path);
            if (!quiet)
                printf("loading %s\n", path);
            else
                printf("%20s ", path);
        }
    }

    return 0;
}