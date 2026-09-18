#include "tracking/ocsort_tracker.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <utility>
#include <vector>

#include "geometry.h"
#include "hungarian.h"
#include "kalman_box_tracker.h"
#include "ocsort_association.h"
#include "ocsort_reupdate.h"

namespace tracking {
namespace {

TrackerConfig validateConfig(TrackerConfig config) {
    if (!std::isfinite(config.detectionThreshold) || config.detectionThreshold < 0.0F ||
        config.detectionThreshold > 1.0F ||
        !std::isfinite(config.continuationDetectionThreshold) ||
        config.continuationDetectionThreshold < 0.0F ||
        config.continuationDetectionThreshold > config.detectionThreshold ||
        !std::isfinite(config.iouThreshold) ||
        config.iouThreshold < 0.0F || config.iouThreshold > 1.0F || config.minHits < 1 ||
        config.maxAgeFrames < 0 || !std::isfinite(config.nominalFps) ||
        config.nominalFps <= 0.0F || config.deltaT < 1 || !std::isfinite(config.inertia) ||
        config.inertia < 0.0F || config.inertia > 1.0F) {
        throw std::invalid_argument("invalid tracker configuration");
    }
    return config;
}

bool isContinuationDetection(const Detection& detection, const TrackerConfig& config) {
    return geometry::isValid(detection.bbox) && std::isfinite(detection.confidence) &&
           detection.confidence >= 0.0F && detection.confidence <= 1.0F &&
           detection.confidence >= config.continuationDetectionThreshold;
}

}  // namespace

struct OCSortTracker::Impl {
    struct Track {
        Track(int trackId, const Detection& detection, int minHits)
            : id(trackId),
              filter(detection.bbox),
              posterior(filter),
              confidence(detection.confidence),
              lastObservation(detection.bbox) {
            history.record(detection, age);
            if (minHits == 1) {
                state = TrackState::Confirmed;
            }
        }

        int id = -1;
        kalman::KalmanBoxTracker filter;
        kalman::KalmanBoxTracker posterior;
        ocsort::ObservationHistory history;
        BBox lastObservation{};
        int lastObservationAge = 1;
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

OCSortTracker::OCSortTracker(TrackerConfig config)
    : impl_(std::make_unique<Impl>(validateConfig(config))) {}

OCSortTracker::~OCSortTracker() = default;

std::vector<TrackResult> OCSortTracker::update(
    const std::vector<Detection>& detections,
    std::int64_t timestampMs) {
    if (timestampMs < 0 || (impl_->hasTimestamp && timestampMs <= impl_->lastTimestampMs)) {
        throw std::invalid_argument("timestamps must be non-negative and strictly increasing");
    }

    std::vector<Detection> continuationDetections;
    continuationDetections.reserve(detections.size());
    for (const Detection& detection : detections) {
        if (isContinuationDetection(detection, impl_->config)) {
            continuationDetections.push_back(detection);
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

    std::vector<ocsort::CandidateTrack> candidates;
    candidates.reserve(impl_->tracks.size());
    for (const Impl::Track& track : impl_->tracks) {
        const auto predictedBox = track.filter.estimate();
        if (!predictedBox) {
            throw std::logic_error("predicted track has no valid box");
        }
        candidates.push_back(ocsort::CandidateTrack{
            *predictedBox,
            &track.history,
            track.age,
            track.state == TrackState::Confirmed ? impl_->config.continuationDetectionThreshold
                                                 : impl_->config.detectionThreshold,
            track.state != TrackState::Lost ||
                track.confidence >= impl_->config.detectionThreshold,
        });
    }

    const assignment::AssignmentResult firstAssociation = ocsort::associate(
        candidates,
        continuationDetections,
        impl_->config.deltaT,
        impl_->config.inertia,
        impl_->config.iouThreshold);
    std::vector<std::pair<std::size_t, std::size_t>> matches = firstAssociation.matches;
    std::vector<bool> firstMatchedTracks(impl_->tracks.size(), false);
    std::vector<bool> firstMatchedDetections(continuationDetections.size(), false);
    for (const auto& match : firstAssociation.matches) {
        firstMatchedTracks[match.first] = true;
        firstMatchedDetections[match.second] = true;
    }

    std::vector<std::size_t> unmatchedTrackIndices;
    std::vector<std::size_t> unmatchedDetectionIndices;
    for (std::size_t index = 0; index < impl_->tracks.size(); ++index) {
        if (!firstMatchedTracks[index]) {
            unmatchedTrackIndices.push_back(index);
        }
    }
    for (std::size_t index = 0; index < continuationDetections.size(); ++index) {
        if (!firstMatchedDetections[index]) {
            unmatchedDetectionIndices.push_back(index);
        }
    }

    assignment::CostMatrix recoveryCosts{
        unmatchedTrackIndices.size(), unmatchedDetectionIndices.size(), {}};
    assignment::ValidityMask recoveryEdges{
        unmatchedTrackIndices.size(), unmatchedDetectionIndices.size(), {}};
    recoveryCosts.values.reserve(unmatchedTrackIndices.size() * unmatchedDetectionIndices.size());
    recoveryEdges.values.reserve(unmatchedTrackIndices.size() * unmatchedDetectionIndices.size());
    for (const std::size_t trackIndex : unmatchedTrackIndices) {
        const BBox& lastObservation = impl_->tracks[trackIndex].lastObservation;
        for (const std::size_t detectionIndex : unmatchedDetectionIndices) {
            const float overlap = geometry::iou(lastObservation, continuationDetections[detectionIndex].bbox);
            recoveryCosts.values.push_back(1.0F - overlap);
            recoveryEdges.values.push_back(
                overlap >= impl_->config.iouThreshold &&
                (continuationDetections[detectionIndex].confidence >=
                     impl_->config.detectionThreshold ||
                 impl_->tracks[trackIndex].state == TrackState::Confirmed) &&
                (impl_->tracks[trackIndex].state != TrackState::Lost ||
                 impl_->tracks[trackIndex].confidence >= impl_->config.detectionThreshold));
        }
    }
    const assignment::AssignmentResult recoveryAssociation = assignment::solve(recoveryCosts, recoveryEdges);
    for (const auto& match : recoveryAssociation.matches) {
        matches.emplace_back(
            unmatchedTrackIndices[match.first],
            unmatchedDetectionIndices[match.second]);
    }

    std::vector<bool> matchedTracks(impl_->tracks.size(), false);
    std::vector<bool> matchedDetections(continuationDetections.size(), false);
    std::vector<bool> removeTracks(impl_->tracks.size(), false);
    for (const auto& match : matches) {
        Impl::Track& track = impl_->tracks[match.first];
        const Detection& detection = continuationDetections[match.second];
        const TrackState previousState = track.state;
        const bool needsReupdate = previousState == TrackState::Lost &&
                                   track.age > track.lastObservationAge + 1;
        const bool updated = needsReupdate
                                 ? ocsort::reupdate(
                                       track.filter,
                                       track.posterior,
                                       track.lastObservation,
                                       track.lastObservationAge,
                                       detection.bbox,
                                       track.age)
                                       .has_value()
                                 : track.filter.update(detection.bbox);
        if (!updated) {
            removeTracks[match.first] = true;
            continue;
        }

        matchedTracks[match.first] = true;
        matchedDetections[match.second] = true;
        track.posterior = track.filter;
        track.history.record(detection, track.age);
        track.lastObservation = detection.bbox;
        track.lastObservationAge = track.age;
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

    for (std::size_t index = 0; index < continuationDetections.size(); ++index) {
        if (!matchedDetections[index] &&
            continuationDetections[index].confidence >= impl_->config.detectionThreshold) {
            impl_->tracks.emplace_back(
                impl_->nextTrackId++,
                continuationDetections[index],
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

void OCSortTracker::reset() {
    impl_->tracks.clear();
    impl_->nextTrackId = 1;
    impl_->hasTimestamp = false;
    impl_->lastTimestampMs = 0;
}

}  // namespace tracking
