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

            image_u8_t *im = NULL;
            if (str_ends_with(path, "pnm") || str_ends_with(path, "PNM") ||
                str_ends_with(path, "pgm") || str_ends_with(path, "PGM"))
                im = image_u8_create_from_pnm(path);
            else if (str_ends_with(path, "jpg") || str_ends_with(path, "JPG")) {
                int err = 0;
                pjpeg_t *pjpeg = pjpeg_create_from_file(path, 0, &err);
                if (pjpeg == NULL) {
                    printf("pjpeg error %d\n", err);
                    continue;
                }

                if (1) {
                    im = pjpeg_to_u8_baseline(pjpeg);
                } else {
                    printf("illumination invariant\n");

                    image_u8x3_t *imc =  pjpeg_to_u8x3_baseline(pjpeg);

                    im = image_u8_create(imc->width, imc->height);

                    for (int y = 0; y < imc->height; y++) {
                        for (int x = 0; x < imc->width; x++) {
                            double r = imc->buf[y*imc->stride + 3*x + 0] / 255.0;
                            double g = imc->buf[y*imc->stride + 3*x + 1] / 255.0;
                            double b = imc->buf[y*imc->stride + 3*x + 2] / 255.0;

                            double alpha = 0.42;
                            double v = 0.5 + log(g) - alpha*log(b) - (1-alpha)*log(r);
                            int iv = v * 255;
                            if (iv < 0)
                                iv = 0;
                            if (iv > 255)
                                iv = 255;

                            im->buf[y*im->stride + x] = iv;
                        }
                    }
                    image_u8x3_destroy(imc);
                    if (td->debug)
                        image_u8_write_pnm(im, "debug_invariant.pnm");
                }

                pjpeg_destroy(pjpeg);
            }

            if (im == NULL) {
                printf("couldn't load %s\n", path);
                continue;
            }
        }
    }

    return 0;
}