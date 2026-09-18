#include <deque>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <locale>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "replay_input.h"
#include "tracking/itracker.h"
#include "tracking/ocsort_tracker.h"
#include "tracking/sort_tracker.h"

namespace {

constexpr int kHistoryLength = 24;
constexpr int kPlaybackDelayMs = 125;
constexpr const char* kWindowName = "tracker_visualizer";

struct CommandLine {
    std::string framesPath;
    std::string detectionsPath;
    std::string imagesPath;
    std::string trackerName = "sort";
    bool hasTracker = false;
    bool autoPlay = false;
};

CommandLine parseCommandLine(int argc, char* argv[]) {
    if (argc < 7 || argc > 10) {
        throw std::invalid_argument(
            "usage: tracker_visualizer --frames <frames.csv> --detections <detections.csv> --images <images-dir> [--tracker sort|ocsort] [--auto-play]");
    }

    CommandLine commandLine;
    for (int index = 1; index < argc; ++index) {
        const std::string option = argv[index];
        if (option == "--auto-play") {
            if (commandLine.autoPlay) {
                throw std::invalid_argument("invalid or duplicate command-line option: " + option);
            }
            commandLine.autoPlay = true;
            continue;
        }
        if (++index == argc) {
            throw std::invalid_argument("option value must not be empty");
        }
        const std::string value = argv[index];
        if (value.empty()) {
            throw std::invalid_argument("option value must not be empty");
        }
        if (option == "--frames" && commandLine.framesPath.empty()) {
            commandLine.framesPath = value;
        } else if (option == "--detections" && commandLine.detectionsPath.empty()) {
            commandLine.detectionsPath = value;
        } else if (option == "--images" && commandLine.imagesPath.empty()) {
            commandLine.imagesPath = value;
        } else if (option == "--tracker" && !commandLine.hasTracker) {
            commandLine.trackerName = value;
            commandLine.hasTracker = true;
        } else {
            throw std::invalid_argument("invalid or duplicate command-line option: " + option);
        }
    }
    if (commandLine.framesPath.empty() || commandLine.detectionsPath.empty() || commandLine.imagesPath.empty()) {
        throw std::invalid_argument(
            "usage: tracker_visualizer --frames <frames.csv> --detections <detections.csv> --images <images-dir> [--tracker sort|ocsort] [--auto-play]");
    }
    return commandLine;
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

std::filesystem::path imagePathForFrame(
    const std::filesystem::path& imagesPath,
    std::int64_t frameId) {
    std::ostringstream filename;
    filename << std::setw(6) << std::setfill('0') << frameId << ".jpg";
    return imagesPath / filename.str();
}

cv::Scalar colorForTrackState(tracking::TrackState state) {
    switch (state) {
        case tracking::TrackState::Tentative:
            return cv::Scalar(0, 165, 255);
        case tracking::TrackState::Confirmed:
            return cv::Scalar(255, 0, 0);
        case tracking::TrackState::Lost:
            return cv::Scalar(0, 255, 255);
    }
    return cv::Scalar(255, 255, 255);
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

cv::Rect rectForBox(const tracking::BBox& box) {
    return cv::Rect(
        cvRound(box.x1),
        cvRound(box.y1),
        cvRound(box.x2 - box.x1),
        cvRound(box.y2 - box.y1));
}

bool isRightArrow(int key) {
    return key == 83 || key == 65363 || key == 63235 || key == 16777236 || key == 2555904;
}

class Visualizer {
public:
    Visualizer(
        tracking::tool::ReplayInputs inputs,
        std::filesystem::path imagesPath,
        const std::string& trackerName)
        : inputs_(std::move(inputs)),
          imagesPath_(std::move(imagesPath)),
          tracker_(makeTracker(trackerName)) {}

    int run(bool autoPlay) {
        if (inputs_.frames.empty()) {
            throw std::runtime_error("frames.csv contains no frames");
        }

        validateImages();
        if (autoPlay) {
            while (true) {
                loadCurrentFrame();
                if (atEnd()) {
                    break;
                }
                ++currentIndex_;
            }
            return 0;
        }

        cv::namedWindow(kWindowName, cv::WINDOW_AUTOSIZE);
        loadCurrentFrame();
        bool paused = true;
        while (true) {
            showCurrentFrame();
            const int key = cv::waitKeyEx(paused || atEnd() ? 0 : kPlaybackDelayMs);
            if (key == 27) {
                return 0;
            }
            if (key == ' ') {
                paused = !paused;
                continue;
            }
            if (key == 'd' || key == 'D') {
                showDetections_ = !showDetections_;
                continue;
            }
            if (key == 't' || key == 'T') {
                showTracks_ = !showTracks_;
                continue;
            }
            if (key == 'h' || key == 'H') {
                showHistory_ = !showHistory_;
                continue;
            }
            if (key == 'r' || key == 'R') {
                reset();
                paused = true;
                continue;
            }
            if (isRightArrow(key)) {
                if (!atEnd()) {
                    advance();
                }
                paused = true;
                continue;
            }
            if (!paused && !atEnd()) {
                advance();
            }
        }
    }

private:
    void validateImages() {
        cv::Size imageSize;
        for (const tracking::tool::ReplayFrame& frame : inputs_.frames) {
            const std::filesystem::path imagePath = imagePathForFrame(imagesPath_, frame.id);
            if (!std::filesystem::is_regular_file(imagePath)) {
                throw std::runtime_error(
                    "frame " + std::to_string(frame.id) + ": missing image " + imagePath.string());
            }
            const cv::Mat image = cv::imread(imagePath.string(), cv::IMREAD_COLOR);
            if (image.empty()) {
                throw std::runtime_error(
                    "frame " + std::to_string(frame.id) + ": cannot decode image " + imagePath.string());
            }
            if (imageSize.empty()) {
                imageSize = image.size();
            } else if (image.size() != imageSize) {
                throw std::runtime_error(
                    "frame " + std::to_string(frame.id) + ": image size differs from the first frame");
            }
        }
        expectedImageSize_ = imageSize;
    }

    void reset() {
        tracker_->reset();
        historyById_.clear();
        currentIndex_ = 0;
        loadCurrentFrame();
    }

    bool atEnd() const {
        return currentIndex_ + 1 >= inputs_.frames.size();
    }

    void loadCurrentFrame() {
        const tracking::tool::ReplayFrame& frame = inputs_.frames[currentIndex_];
        const std::filesystem::path imagePath = imagePathForFrame(imagesPath_, frame.id);
        if (!std::filesystem::is_regular_file(imagePath)) {
            throw std::runtime_error(
                "frame " + std::to_string(frame.id) + ": missing image " + imagePath.string());
        }
        currentImage_ = cv::imread(imagePath.string(), cv::IMREAD_COLOR);
        if (currentImage_.empty()) {
            throw std::runtime_error(
                "frame " + std::to_string(frame.id) + ": cannot decode image " + imagePath.string());
        }
        if (expectedImageSize_.empty()) {
            expectedImageSize_ = currentImage_.size();
        } else if (currentImage_.size() != expectedImageSize_) {
            throw std::runtime_error(
                "frame " + std::to_string(frame.id) + ": image size differs from the first frame");
        }

        const auto detections = inputs_.detectionsByFrame.find(frame.id);
        const std::vector<tracking::Detection> emptyDetections;
        currentDetections_ = detections == inputs_.detectionsByFrame.end()
            ? emptyDetections
            : detections->second;
        currentTracks_ = tracker_->update(currentDetections_, frame.timestampMs);
        for (const tracking::TrackResult& track : currentTracks_) {
            std::deque<cv::Point>& history = historyById_[track.trackId];
            history.emplace_back(
                cvRound((track.bbox.x1 + track.bbox.x2) * 0.5F),
                cvRound((track.bbox.y1 + track.bbox.y2) * 0.5F));
            if (history.size() > kHistoryLength) {
                history.pop_front();
            }
        }
    }

    void advance() {
        if (!atEnd()) {
            ++currentIndex_;
            loadCurrentFrame();
        }
    }

    void showCurrentFrame() const {
        cv::Mat canvas = currentImage_.clone();
        if (showDetections_) {
            for (const tracking::Detection& detection : currentDetections_) {
                cv::rectangle(canvas, rectForBox(detection.bbox), cv::Scalar(0, 255, 0), 2);
            }
        }
        if (showHistory_) {
            for (const auto& [trackId, history] : historyById_) {
                static_cast<void>(trackId);
                for (std::size_t index = 1; index < history.size(); ++index) {
                    cv::line(canvas, history[index - 1], history[index], cv::Scalar(255, 255, 255), 1);
                }
            }
        }
        if (showTracks_) {
            for (const tracking::TrackResult& track : currentTracks_) {
                const cv::Scalar color = colorForTrackState(track.state);
                cv::rectangle(canvas, rectForBox(track.bbox), color, 2);
                const std::string label = "ID " + std::to_string(track.trackId) + " " + stateName(track.state);
                cv::putText(
                    canvas,
                    label,
                    cv::Point(cvRound(track.bbox.x1), cvRound(track.bbox.y1) - 5),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.5,
                    color,
                    1);
            }
        }

        const tracking::tool::ReplayFrame& frame = inputs_.frames[currentIndex_];
        cv::putText(
            canvas,
            "frame " + std::to_string(frame.id) + "  timestamp " + std::to_string(frame.timestampMs) + " ms",
            cv::Point(10, 24),
            cv::FONT_HERSHEY_SIMPLEX,
            0.6,
            cv::Scalar(255, 255, 255),
            2);
        cv::putText(
            canvas,
            "Space: play/pause  Right: step  R: reset  D/T/H: overlays  Esc: exit",
            cv::Point(10, 50),
            cv::FONT_HERSHEY_SIMPLEX,
            0.5,
            cv::Scalar(255, 255, 255),
            1);
        cv::imshow(kWindowName, canvas);
    }

    tracking::tool::ReplayInputs inputs_;
    std::filesystem::path imagesPath_;
    std::unique_ptr<tracking::ITracker> tracker_;
    std::size_t currentIndex_ = 0;
    cv::Size expectedImageSize_;
    cv::Mat currentImage_;
    std::vector<tracking::Detection> currentDetections_;
    std::vector<tracking::TrackResult> currentTracks_;
    std::map<int, std::deque<cv::Point>> historyById_;
    bool showDetections_ = true;
    bool showTracks_ = true;
    bool showHistory_ = true;
};

}  // namespace

int main(int argc, char* argv[]) {
    try {
        std::locale::global(std::locale::classic());
        const CommandLine commandLine = parseCommandLine(argc, argv);
        Visualizer visualizer(
            tracking::tool::parseReplayInputs(commandLine.framesPath, commandLine.detectionsPath),
            commandLine.imagesPath,
            commandLine.trackerName);
        return visualizer.run(commandLine.autoPlay);
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
