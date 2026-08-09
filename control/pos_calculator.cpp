#include "pos_calculator.hpp"

double pixel_offset_to_ground_m(double pixel_offset, double altitude_m, double focal_length_px) {
    return pixel_offset / focal_length_px * altitude_m;
}
