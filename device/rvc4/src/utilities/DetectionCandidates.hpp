#pragma once

#include <optional>
#include <string>

struct DetectionCandidate {
    float xmin, ymin, xmax, ymax, score;
    int label, headIndex, rowIndex, columnIndex;
    std::optional<std::string> labelName;
};
