#include "domain/DescriptorMatching.h"

#include "domain/GeometryMath.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <optional>

namespace cadstep {
namespace detail {
namespace {

// How "the same size" is judged for a measure pair.
enum class MeasureGate {
  // Faces: the measure is an area, compared relative to the reference measure.
  RelativeToReference,
  // Edges: the measure is a length, compared against the distance tolerance.
  Absolute,
};

double DescriptorScore(double measureErr, double centroidErr, double boundsErr) {
  const double score = 1.0 - 0.4 * measureErr - 0.3 * centroidErr - 0.3 * boundsErr;
  return std::clamp(score, 0.0, 1.0);
}

MatchCollection MatchDescriptors(const std::vector<DescriptorView> &reference,
                                 const std::vector<DescriptorView> &candidate,
                                 const CompareConfig &config,
                                 double characteristicScale,
                                 const TolerancePolicy &tolerances,
                                 MeasureGate gate,
                                 const char *idPrefix,
                                 EntityKind kind) {
  MatchCollection collection;
  collection.attempted = true;
  collection.referenceCount = static_cast<int>(reference.size());
  collection.candidateCount = static_cast<int>(candidate.size());

  std::map<std::string, int> refHistogram;
  for (const auto &item : reference) {
    refHistogram[item.typeName]++;
  }
  std::map<std::string, int> candHistogram;
  for (const auto &item : candidate) {
    candHistogram[item.typeName]++;
  }
  collection.typeHistogramEqual = (refHistogram == candHistogram);

  std::vector<bool> candidateUsed(candidate.size(), false);
  int matchPairIdx = 1;

  for (std::size_t i = 0; i < reference.size(); ++i) {
    const DescriptorView &refItem = reference[i];

    int bestCandidateIdx = -1;
    double bestScore = -1.0;
    int secondBestIdx = -1;
    double secondBestScore = -1.0;
    MatchMetrics bestMetrics;

    for (std::size_t j = 0; j < candidate.size(); ++j) {
      if (candidateUsed[j]) {
        continue;
      }
      const DescriptorView &candItem = candidate[j];
      if (refItem.typeName != candItem.typeName) {
        continue;
      }

      const double measureDiff = std::abs(refItem.measure - candItem.measure);
      const double relMeasureDiff =
          refItem.measure > 0.0 ? (measureDiff / refItem.measure) : 0.0;
      const double centroidDist = PointDistance(refItem.centroidMm, candItem.centroidMm);
      const double boundsDiff =
          ComputeBoundsDifference(refItem.boundsMm, candItem.boundsMm);

      const bool measurePass =
          gate == MeasureGate::RelativeToReference
              ? relMeasureDiff <= config.relativeVolumeTolerance * 100.0
              : measureDiff <= tolerances.distanceMm();
      const bool distPass = centroidDist <= tolerances.distanceMm();
      const bool boundsPass = boundsDiff <= tolerances.distanceMm();

      if (!measurePass || !distPass || !boundsPass) {
        continue;
      }

      const double normMeasureErr =
          gate == MeasureGate::RelativeToReference
              ? std::min(1.0, relMeasureDiff)
              : (characteristicScale > 0.0
                     ? std::min(1.0, measureDiff / characteristicScale)
                     : 0.0);
      const double normCentroidErr =
          characteristicScale > 0.0 ? std::min(1.0, centroidDist / characteristicScale) : 0.0;
      const double normBoundsErr =
          characteristicScale > 0.0 ? std::min(1.0, boundsDiff / characteristicScale) : 0.0;
      const double score = DescriptorScore(normMeasureErr, normCentroidErr, normBoundsErr);

      if (score > bestScore) {
        secondBestScore = bestScore;
        secondBestIdx = bestCandidateIdx;
        bestScore = score;
        bestCandidateIdx = static_cast<int>(j);
        bestMetrics = {measureDiff, relMeasureDiff, centroidDist, boundsDiff};
      } else if (score > secondBestScore) {
        secondBestScore = score;
        secondBestIdx = static_cast<int>(j);
      }
    }

    EntityMatch item;
    item.id = std::string(idPrefix) +
              MakeEntityId(EntitySide::Reference, kind, matchPairIdx++);
    item.referenceId = refItem.id;
    item.geometryType = refItem.typeName;
    item.verificationLevel = VerificationLevel::Descriptor;

    if (bestCandidateIdx >= 0) {
      const bool isAmbiguous =
          (secondBestIdx >= 0) && ((bestScore - secondBestScore) < config.ambiguousMatchMargin);
      if (isAmbiguous) {
        item.status = MatchStatus::Ambiguous;
        item.candidateId = candidate[bestCandidateIdx].id;
        item.score = bestScore;
        item.metrics = bestMetrics;
        item.reasonCodes.push_back("MULTIPLE_SIMILAR_CANDIDATES");
        collection.ambiguousCount++;
      } else {
        item.status = MatchStatus::Matched;
        item.candidateId = candidate[bestCandidateIdx].id;
        item.score = bestScore;
        item.metrics = bestMetrics;
        candidateUsed[bestCandidateIdx] = true;
        collection.matchedCount++;
      }
    } else {
      item.status = MatchStatus::Unmatched;
      item.candidateId = "";
      item.score = std::nullopt;
      item.metrics = std::nullopt;
      item.reasonCodes.push_back("NO_ONE_TO_ONE_CANDIDATE");
      collection.unmatchedReferenceIds.push_back(refItem.id);
    }
    collection.items.push_back(item);
  }

  for (std::size_t j = 0; j < candidate.size(); ++j) {
    if (!candidateUsed[j]) {
      collection.unmatchedCandidateIds.push_back(candidate[j].id);
    }
  }

  collection.allMatched = (collection.matchedCount == collection.referenceCount) &&
                          (collection.referenceCount == collection.candidateCount);
  return collection;
}

} // namespace

MatchCollection MatchFaceDescriptors(const std::vector<DescriptorView> &reference,
                                     const std::vector<DescriptorView> &candidate,
                                     const CompareConfig &config,
                                     double characteristicScale,
                                     const TolerancePolicy &tolerances) {
  return MatchDescriptors(reference, candidate, config, characteristicScale, tolerances,
                          MeasureGate::RelativeToReference, "face-match:",
                          EntityKind::NormalizedFace);
}

MatchCollection MatchEdgeDescriptors(const std::vector<DescriptorView> &reference,
                                     const std::vector<DescriptorView> &candidate,
                                     const CompareConfig &config,
                                     double characteristicScale,
                                     const TolerancePolicy &tolerances) {
  return MatchDescriptors(reference, candidate, config, characteristicScale, tolerances,
                          MeasureGate::Absolute, "edge-match:", EntityKind::NormalizedEdge);
}

} // namespace detail
} // namespace cadstep
