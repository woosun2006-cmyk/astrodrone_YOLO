#pragma once

#include <string>
#include <vector>

// One decoded, NMS-survived, frame-coordinate detection box.
struct Detection {
    float x1, y1, x2, y2;  // pixel coords in the ORIGINAL (pre-letterbox) frame
    float conf;
    int cls;
};

// xyxy box in letterboxed (network-input) space, used internally between
// decode and scale-back.
struct RawBox {
    float x1, y1, x2, y2;
    float conf;
    int cls;
};
