#include <fstream>
#include <iomanip>
#include <iostream>
#include <locale>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "replay_input.h"
#include "tracking/itracker.h"
#include "tracking/ocsort_tracker.h"
#include "tracking/sort_tracker.h"

namespace {

constexpr const char* kTracksHeader =
    "frame_id,timestamp_ms,track_id,x1,y1,x2,y2,confidence,state,velocity_x,velocity_y,age,lost_frames";

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

std::unique_ptr<tracking::ITracker> makeTracker(const std::string& trackerName) {
    if (trackerName == "sort") {
        return std::make_unique<tracking::SortTracker>();
    }
    if (trackerName == "ocsort") {
        return std::make_unique<tracking::OCSortTracker>();
    }
    throw std::invalid_argument("tracker must be sort or ocsort");
}

void replay(
    const tracking::tool::ReplayInputs& inputs,
    const std::string& outputPath,
    const std::string& trackerName) {
    std::unique_ptr<tracking::ITracker> tracker = makeTracker(trackerName);
    std::ofstream output(outputPath, std::ios::trunc);
    if (!output) {
        throw std::runtime_error(outputPath + ": cannot open output file");
    }
    output.imbue(std::locale::classic());
    output << kTracksHeader << '\n';
    output << std::fixed << std::setprecision(6);

    for (const tracking::tool::ReplayFrame& frame : inputs.frames) {
        const auto detections = inputs.detectionsByFrame.find(frame.id);
        const std::vector<tracking::Detection> emptyDetections;
        const std::vector<tracking::Detection>& frameDetections =
            detections == inputs.detectionsByFrame.end() ? emptyDetections : detections->second;
        const std::vector<tracking::TrackResult> tracks = tracker->update(
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
    std::string trackerName = "sort";
    bool hasTracker = false;
};

CommandLine parseCommandLine(int argc, char* argv[]) {
    if (argc != 7 && argc != 9) {
        throw std::invalid_argument(
            "usage: tracker_replay --frames <frames.csv> --detections <detections.csv> --output <tracks.csv> [--tracker sort|ocsort]");
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
        } else if (option == "--tracker" && !commandLine.hasTracker) {
            commandLine.trackerName = value;
            commandLine.hasTracker = true;
        } else {
            throw std::invalid_argument("invalid or duplicate command-line option: " + option);
        }
    }
    if (commandLine.framesPath.empty() || commandLine.detectionsPath.empty() || commandLine.outputPath.empty()) {
        throw std::invalid_argument(
            "usage: tracker_replay --frames <frames.csv> --detections <detections.csv> --output <tracks.csv> [--tracker sort|ocsort]");
    }
    return commandLine;
}

}  // namespace

int main(int argc, char* argv[]) {
    try {
        std::locale::global(std::locale::classic());
        const CommandLine commandLine = parseCommandLine(argc, argv);
        const tracking::tool::ReplayInputs inputs = tracking::tool::parseReplayInputs(
            commandLine.framesPath,
            commandLine.detectionsPath);
        replay(inputs, commandLine.outputPath, commandLine.trackerName);
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
