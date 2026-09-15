#include "tracking/sort_tracker.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <utility>
#include <vector>

#include "geometry.h"
#include "hungarian.h"
#include "kalman_box_tracker.h"

namespace tracking {
namespace {

TrackerConfig validateConfig(TrackerConfig config) {
    if (!std::isfinite(config.detectionThreshold) || config.detectionThreshold < 0.0F ||
        config.detectionThreshold > 1.0F || !std::isfinite(config.iouThreshold) ||
        config.iouThreshold < 0.0F || config.iouThreshold > 1.0F || config.minHits < 1 ||
        config.maxAgeFrames < 0 || !std::isfinite(config.nominalFps) ||
        config.nominalFps <= 0.0F) {
        throw std::invalid_argument("invalid tracker configuration");
    }
    return config;
}

bool isUsableDetection(const Detection& detection, const TrackerConfig& config) {
    return geometry::isValid(detection.bbox) && std::isfinite(detection.confidence) &&
           detection.confidence >= 0.0F && detection.confidence <= 1.0F &&
           detection.confidence >= config.detectionThreshold;
}

}  // namespace

struct SortTracker::Impl {
    struct Track {
        Track(int trackId, const Detection& detection, int minHits)
            : id(trackId), filter(detection.bbox), confidence(detection.confidence) {
            if (minHits == 1) {
                state = TrackState::Confirmed;
            }
        }

        int id = -1;
        kalman::KalmanBoxTracker filter;
        float confidence = 0.0F;
        TrackState state = TrackState::Tentative;
        int age = 1;
        int hitStreak = 1;
        int lostFrames = 0;
    };

    explicit Impl(TrackerConfig trackerConfig) : config(trackerConfig) {}

    TrackerConfig config;
    std::vector<Track> tracks;
    int nextTrackId = 1;
    bool hasTimestamp = false;
    std::int64_t lastTimestampMs = 0;
};

SortTracker::SortTracker(TrackerConfig config)
    : impl_(std::make_unique<Impl>(validateConfig(config))) {}

SortTracker::~SortTracker() = default;

std::vector<TrackResult> SortTracker::update(
    const std::vector<Detection>& detections,
    std::int64_t timestampMs) {
    if (timestampMs < 0 || (impl_->hasTimestamp && timestampMs <= impl_->lastTimestampMs)) {
        throw std::invalid_argument("timestamps must be non-negative and strictly increasing");
    }

    std::vector<Detection> usableDetections;
    usableDetections.reserve(detections.size());
    for (const Detection& detection : detections) {
        if (isUsableDetection(detection, impl_->config)) {
            usableDetections.push_back(detection);
        }
    }

    for (auto track = impl_->tracks.begin(); track != impl_->tracks.end();) {
        ++track->age;
        if (!track->filter.predict()) {
            track = impl_->tracks.erase(track);
        } else {
            ++track;
        }
    }

    assignment::CostMatrix costs;
    costs.rows = impl_->tracks.size();
    costs.columns = usableDetections.size();
    costs.values.reserve(costs.rows * costs.columns);
    assignment::ValidityMask validEdges;
    validEdges.rows = costs.rows;
    validEdges.columns = costs.columns;
    validEdges.values.reserve(costs.rows * costs.columns);

    for (const Impl::Track& track : impl_->tracks) {
        const auto predictedBox = track.filter.estimate();
        if (!predictedBox) {
            throw std::logic_error("predicted track has no valid box");
        }

        for (const Detection& detection : usableDetections) {
            const float overlap = geometry::iou(*predictedBox, detection.bbox);
            costs.values.push_back(1.0F - overlap);
            validEdges.values.push_back(overlap >= impl_->config.iouThreshold);
        }
    }

    const assignment::AssignmentResult assignment = assignment::solve(costs, validEdges);
    std::vector<bool> matchedTracks(impl_->tracks.size(), false);
    std::vector<bool> matchedDetections(usableDetections.size(), false);
    std::vector<bool> removeTracks(impl_->tracks.size(), false);

    for (const auto& match : assignment.matches) {
        Impl::Track& track = impl_->tracks[match.first];
        const Detection& detection = usableDetections[match.second];
        const TrackState previousState = track.state;
        if (!track.filter.update(detection.bbox)) {
            removeTracks[match.first] = true;
            continue;
        }

        matchedTracks[match.first] = true;
        matchedDetections[match.second] = true;
        track.confidence = detection.confidence;
        ++track.hitStreak;
        track.lostFrames = 0;
        if (previousState == TrackState::Lost || track.hitStreak >= impl_->config.minHits) {
            track.state = TrackState::Confirmed;
        }
    }

    for (std::size_t index = 0; index < impl_->tracks.size(); ++index) {
        Impl::Track& track = impl_->tracks[index];
        if (removeTracks[index] || matchedTracks[index]) {
            continue;
        }

        track.hitStreak = 0;
        if (track.state == TrackState::Tentative) {
            removeTracks[index] = true;
            continue;
        }

        ++track.lostFrames;
        track.state = TrackState::Lost;
        if (track.lostFrames > impl_->config.maxAgeFrames) {
            removeTracks[index] = true;
        }
    }

    impl_->tracks.erase(
        std::remove_if(
            impl_->tracks.begin(),
            impl_->tracks.end(),
            [&removeTracks, index = std::size_t{0}](const Impl::Track&) mutable {
                return removeTracks[index++];
            }),
        impl_->tracks.end());

    for (std::size_t index = 0; index < usableDetections.size(); ++index) {
        if (!matchedDetections[index]) {
            impl_->tracks.emplace_back(
                impl_->nextTrackId++,
                usableDetections[index],
                impl_->config.minHits);
        }
    }

    impl_->hasTimestamp = true;
    impl_->lastTimestampMs = timestampMs;

    std::vector<TrackResult> results;
    results.reserve(impl_->tracks.size());
    for (const Impl::Track& track : impl_->tracks) {
        const auto bbox = track.filter.estimate();
        const auto velocity = track.filter.velocityPerFrame();
        if (!bbox || !velocity) {
            throw std::logic_error("active track has no valid Kalman estimate");
        }

        results.push_back(TrackResult{
            track.id,
            *bbox,
            track.confidence,
            track.state,
            velocity->x * impl_->config.nominalFps,
            velocity->y * impl_->config.nominalFps,
            track.age,
            track.lostFrames,
        });
    }
    std::sort(
        results.begin(),
        results.end(),
        [](const TrackResult& left, const TrackResult& right) {
            return left.trackId < right.trackId;
        });
    return results;
}

void SortTracker::reset() {
    impl_->tracks.clear();
    impl_->nextTrackId = 1;
    impl_->hasTimestamp = false;
    impl_->lastTimestampMs = 0;
}

}  // namespace tracking
