#include "domain/DescriptorMatching.h"

#include "domain/GeometryMath.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <optional>

namespace cadstep {
namespace detail {
namespace {

/**
 * @brief 一对度量之间"尺寸是否相当"的判据方式。
 */
enum class MeasureGate {
  RelativeToReference, ///< 面：度量为面积，相对参考度量做比较
  Absolute,            ///< 边：度量为长度，与距离容差做绝对比较
};

/**
 * @brief 由三项归一化误差合成的匹配得分。
 * @param measureErr 尺寸误差，已归一化到 [0,1]
 * @param centroidErr 质心距离误差，已按特征尺度归一化到 [0,1]
 * @param boundsErr 包围盒差误差，已按特征尺度归一化到 [0,1]
 * @return [0,1] 区间内的得分，三项权重分别为 0.4 / 0.3 / 0.3
 *
 * @note 权重与夹取范围属于可观察行为：得分决定"最优候选"的挑选与
 *       歧义裕度的比较，改动会直接改变匹配结果。
 */
double DescriptorScore(double measureErr, double centroidErr, double boundsErr) {
  const double score = 1.0 - 0.4 * measureErr - 0.3 * centroidErr - 0.3 * boundsErr;
  return std::clamp(score, 0.0, 1.0);
}

/**
 * @brief 贪心一对一匹配的共享实现，面与边两个公开入口都委托到这里。
 * @param reference 参考侧描述符
 * @param candidate 候选侧描述符
 * @param config 提供歧义匹配裕度；面的相对判据还会用到 relativeVolumeTolerance
 * @param characteristicScale 特征尺度，把质心/包围盒/长度误差归一化到 [0,1]
 * @param tolerances 由 Derive() 得到的容差
 * @param gate 尺寸判据方式（面为相对判据，边为绝对判据）
 * @param idPrefix 生成 EntityMatch::id 时的前缀，如 "face-match:" / "edge-match:"
 * @param kind 生成配对 id 时使用的实体种类（面或边）
 * @return 匹配集合
 *
 * 算法要点：
 *  - 先比较两侧的类型直方图，写入 typeHistogramEqual；
 *  - 对每个参考实体，在**未被占用**且**类型相同**的候选里挑三项判据都
 *    通过、得分最高者，同时记录次优得分用于歧义判定；
 *  - 严格小于比较保证并列时取先出现的候选；
 *  - 只有 Matched 才占用候选，Ambiguous 不占用；
 *  - unmatchedReferenceIds 按参考顺序、unmatchedCandidateIds 按候选顺序写入。
 */
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
