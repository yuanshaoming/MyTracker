#include <charconv>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <locale>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry.h"
#include "tracking/sort_tracker.h"

namespace {

constexpr const char* kFramesHeader = "frame_id,timestamp_ms";
constexpr const char* kDetectionsHeader = "frame_id,x1,y1,x2,y2,confidence,class_id";
constexpr const char* kTracksHeader =
    "frame_id,timestamp_ms,track_id,x1,y1,x2,y2,confidence,state,velocity_x,velocity_y,age,lost_frames";

class ParseError : public std::runtime_error {
public:
    ParseError(const std::string& path, std::size_t line, const std::string& message)
        : std::runtime_error(path + ":" + std::to_string(line) + ": " + message) {}
};

struct Frame {
    std::int64_t id = 0;
    std::int64_t timestampMs = 0;
};

struct Inputs {
    std::vector<Frame> frames;
    std::map<std::int64_t, std::vector<tracking::Detection>> detectionsByFrame;
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

std::vector<Frame> parseFrames(const std::string& path) {
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

    std::vector<Frame> frames;
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
        frames.push_back(Frame{frameId, timestampMs});
    }
    return frames;
}

std::map<std::int64_t, std::vector<tracking::Detection>> parseDetections(
    const std::string& path,
    const std::vector<Frame>& frames) {
    std::map<std::int64_t, bool> knownFrames;
    for (const Frame& frame : frames) {
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

    std::map<std::int64_t, std::vector<tracking::Detection>> detectionsByFrame;
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

        const tracking::Detection detection{
            tracking::BBox{
                parseFloat(fields[1], path, lineNumber, "x1"),
                parseFloat(fields[2], path, lineNumber, "y1"),
                parseFloat(fields[3], path, lineNumber, "x2"),
                parseFloat(fields[4], path, lineNumber, "y2"),
            },
            parseFloat(fields[5], path, lineNumber, "confidence"),
            parseClassId(fields[6], path, lineNumber),
        };
        if (!tracking::geometry::isValid(detection.bbox)) {
            throw ParseError(path, lineNumber, "bbox must be finite with x2>x1 and y2>y1");
        }
        if (detection.confidence < 0.0F || detection.confidence > 1.0F) {
            throw ParseError(path, lineNumber, "confidence must be in [0,1]");
        }
        detectionsByFrame[frameId].push_back(detection);
    }
    return detectionsByFrame;
}

Inputs parseInputs(const std::string& framesPath, const std::string& detectionsPath) {
    Inputs inputs;
    inputs.frames = parseFrames(framesPath);
    inputs.detectionsByFrame = parseDetections(detectionsPath, inputs.frames);
    return inputs;
}

const char* stateName(tracking::TrackState state) {
    switch (state) {
        case tracking::TrackState::Tentative:
            return "Tentative";
        case tracking::TrackState::Confirmed:
            return "Confirmed";
        case tracking::TrackState::Lost:
            return "Lost";
    }
    return "Unknown";
}

void replay(const Inputs& inputs, const std::string& outputPath) {
    std::ofstream output(outputPath, std::ios::trunc);
    if (!output) {
        throw std::runtime_error(outputPath + ": cannot open output file");
    }
    output.imbue(std::locale::classic());
    output << kTracksHeader << '\n';
    output << std::fixed << std::setprecision(6);

    tracking::SortTracker tracker;
    for (const Frame& frame : inputs.frames) {
        const auto detections = inputs.detectionsByFrame.find(frame.id);
        const std::vector<tracking::Detection> emptyDetections;
        const std::vector<tracking::Detection>& frameDetections =
            detections == inputs.detectionsByFrame.end() ? emptyDetections : detections->second;
        const std::vector<tracking::TrackResult> tracks = tracker.update(
            frameDetections,
            frame.timestampMs);
        for (const tracking::TrackResult& track : tracks) {
            output << frame.id << ',' << frame.timestampMs << ',' << track.trackId << ','
                   << track.bbox.x1 << ',' << track.bbox.y1 << ',' << track.bbox.x2 << ','
                   << track.bbox.y2 << ',' << track.confidence << ',' << stateName(track.state) << ','
                   << track.velocityX << ',' << track.velocityY << ',' << track.age << ','
                   << track.lostFrames << '\n';
        }
    }
    if (!output) {
        throw std::runtime_error(outputPath + ": failed while writing output");
    }
}

struct CommandLine {
    std::string framesPath;
    std::string detectionsPath;
    std::string outputPath;
};

CommandLine parseCommandLine(int argc, char* argv[]) {
    if (argc != 7) {
        throw std::invalid_argument(
            "usage: tracker_replay --frames <frames.csv> --detections <detections.csv> --output <tracks.csv>");
    }

    CommandLine commandLine;
    for (int index = 1; index < argc; index += 2) {
        const std::string option = argv[index];
        const std::string value = argv[index + 1];
        if (value.empty()) {
            throw std::invalid_argument("option value must not be empty");
        }
        if (option == "--frames" && commandLine.framesPath.empty()) {
            commandLine.framesPath = value;
        } else if (option == "--detections" && commandLine.detectionsPath.empty()) {
            commandLine.detectionsPath = value;
        } else if (option == "--output" && commandLine.outputPath.empty()) {
            commandLine.outputPath = value;
        } else {
            throw std::invalid_argument("invalid or duplicate command-line option: " + option);
        }
    }
    if (commandLine.framesPath.empty() || commandLine.detectionsPath.empty() || commandLine.outputPath.empty()) {
        throw std::invalid_argument(
            "usage: tracker_replay --frames <frames.csv> --detections <detections.csv> --output <tracks.csv>");
    }
    return commandLine;
}

}  // namespace

int main(int argc, char* argv[]) {
    try {
        std::locale::global(std::locale::classic());
        const CommandLine commandLine = parseCommandLine(argc, argv);
        const Inputs inputs = parseInputs(commandLine.framesPath, commandLine.detectionsPath);
        replay(inputs, commandLine.outputPath);
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
