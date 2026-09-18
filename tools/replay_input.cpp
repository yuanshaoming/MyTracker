#include "replay_input.h"

#include <charconv>
#include <cmath>
#include <fstream>
#include <limits>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry.h"

namespace tracking::tool {
namespace {

constexpr const char* kFramesHeader = "frame_id,timestamp_ms";
constexpr const char* kDetectionsHeader = "frame_id,x1,y1,x2,y2,confidence,class_id";

class ParseError : public std::runtime_error {
public:
    ParseError(const std::string& path, std::size_t line, const std::string& message)
        : std::runtime_error(path + ":" + std::to_string(line) + ": " + message) {}
};

std::vector<std::string> splitCsvLine(const std::string& line) {
    std::vector<std::string> fields;
    std::size_t start = 0;
    while (true) {
        const std::size_t comma = line.find(',', start);
        if (comma == std::string::npos) {
            fields.push_back(line.substr(start));
            return fields;
        }
        fields.push_back(line.substr(start, comma - start));
        start = comma + 1;
    }
}

void removeTrailingCarriageReturn(std::string& line) {
    if (!line.empty() && line.back() == '\r') {
        line.pop_back();
    }
}

std::int64_t parseInt64(
    const std::string& text,
    const std::string& path,
    std::size_t line,
    const char* fieldName) {
    std::int64_t value = 0;
    const auto parsed = std::from_chars(text.data(), text.data() + text.size(), value);
    if (parsed.ec != std::errc{} || parsed.ptr != text.data() + text.size()) {
        throw ParseError(path, line, std::string("invalid integer for ") + fieldName);
    }
    return value;
}

float parseFloat(
    const std::string& text,
    const std::string& path,
    std::size_t line,
    const char* fieldName) {
    std::istringstream stream(text);
    stream.imbue(std::locale::classic());
    float value = 0.0F;
    char extra = '\0';
    if (!(stream >> value) || (stream >> extra) || !std::isfinite(value)) {
        throw ParseError(path, line, std::string("invalid finite number for ") + fieldName);
    }
    return value;
}

int parseClassId(const std::string& text, const std::string& path, std::size_t line) {
    const std::int64_t value = parseInt64(text, path, line, "class_id");
    if (value < std::numeric_limits<int>::min() || value > std::numeric_limits<int>::max()) {
        throw ParseError(path, line, "class_id is outside the int range");
    }
    return static_cast<int>(value);
}

std::vector<ReplayFrame> parseFrames(const std::string& path) {
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error(path + ": cannot open input file");
    }

    std::string line;
    if (!std::getline(input, line)) {
        throw ParseError(path, 1, "missing header");
    }
    removeTrailingCarriageReturn(line);
    if (line != kFramesHeader) {
        throw ParseError(path, 1, "expected header frame_id,timestamp_ms");
    }

    std::vector<ReplayFrame> frames;
    std::size_t lineNumber = 1;
    while (std::getline(input, line)) {
        ++lineNumber;
        removeTrailingCarriageReturn(line);
        const auto fields = splitCsvLine(line);
        if (fields.size() != 2) {
            throw ParseError(path, lineNumber, "expected 2 columns");
        }

        const std::int64_t frameId = parseInt64(fields[0], path, lineNumber, "frame_id");
        const std::int64_t timestampMs = parseInt64(fields[1], path, lineNumber, "timestamp_ms");
        if (frameId < 0) {
            throw ParseError(path, lineNumber, "frame_id must be non-negative");
        }
        if (timestampMs < 0) {
            throw ParseError(path, lineNumber, "timestamp_ms must be non-negative");
        }
        if (!frames.empty() && frameId <= frames.back().id) {
            throw ParseError(path, lineNumber, "frame_id must be strictly increasing");
        }
        if (!frames.empty() && timestampMs <= frames.back().timestampMs) {
            throw ParseError(path, lineNumber, "timestamp_ms must be strictly increasing");
        }
        frames.push_back(ReplayFrame{frameId, timestampMs});
    }
    return frames;
}

std::map<std::int64_t, std::vector<Detection>> parseDetections(
    const std::string& path,
    const std::vector<ReplayFrame>& frames) {
    std::map<std::int64_t, bool> knownFrames;
    for (const ReplayFrame& frame : frames) {
        knownFrames.emplace(frame.id, true);
    }

    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error(path + ": cannot open input file");
    }

    std::string line;
    if (!std::getline(input, line)) {
        throw ParseError(path, 1, "missing header");
    }
    removeTrailingCarriageReturn(line);
    if (line != kDetectionsHeader) {
        throw ParseError(path, 1, "expected detection CSV header");
    }

    std::map<std::int64_t, std::vector<Detection>> detectionsByFrame;
    std::int64_t previousFrameId = -1;
    std::size_t lineNumber = 1;
    while (std::getline(input, line)) {
        ++lineNumber;
        removeTrailingCarriageReturn(line);
        const auto fields = splitCsvLine(line);
        if (fields.size() != 7) {
            throw ParseError(path, lineNumber, "expected 7 columns");
        }

        const std::int64_t frameId = parseInt64(fields[0], path, lineNumber, "frame_id");
        if (frameId < 0) {
            throw ParseError(path, lineNumber, "frame_id must be non-negative");
        }
        if (frameId < previousFrameId) {
            throw ParseError(path, lineNumber, "frame_id must be non-decreasing");
        }
        previousFrameId = frameId;
        if (knownFrames.find(frameId) == knownFrames.end()) {
            throw ParseError(path, lineNumber, "frame_id does not exist in frames.csv");
        }

        const Detection detection{
            BBox{
                parseFloat(fields[1], path, lineNumber, "x1"),
                parseFloat(fields[2], path, lineNumber, "y1"),
                parseFloat(fields[3], path, lineNumber, "x2"),
                parseFloat(fields[4], path, lineNumber, "y2"),
            },
            parseFloat(fields[5], path, lineNumber, "confidence"),
            parseClassId(fields[6], path, lineNumber),
        };
        if (!geometry::isValid(detection.bbox)) {
            throw ParseError(path, lineNumber, "bbox must be finite with x2>x1 and y2>y1");
        }
        if (detection.confidence < 0.0F || detection.confidence > 1.0F) {
            throw ParseError(path, lineNumber, "confidence must be in [0,1]");
        }
        detectionsByFrame[frameId].push_back(detection);
    }
    return detectionsByFrame;
}

}  // namespace

ReplayInputs parseReplayInputs(const std::string& framesPath, const std::string& detectionsPath) {
    ReplayInputs inputs;
    inputs.frames = parseFrames(framesPath);
    inputs.detectionsByFrame = parseDetections(detectionsPath, inputs.frames);
    return inputs;
}

}  // namespace tracking::tool
