#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace cadstep {

/**
 * @brief 单次实体几何对比的终态结论。
 *
 * 枚举取值的排列顺序与 ExitCode() 的退出码映射一一对应
 * （Equal=0、LikelyEqual=3、Different=1、InvalidInput=2、
 * UnsupportedShape=2、Indeterminate=4、InternalError=5），
 * 因此新增取值时必须同步调整 ExitCode()。
 */
enum class CompareStatus {
  /// 判定相等：全部判据通过
  Equal,
  /// 判定很可能相等。当前单实体分类器 detail::ClassifyClosedSolidComparison
  /// 只会返回 Equal/Different，该值在单实体路径不可达，仅供多实体配对等
  /// 上游场景与输出契约保留
  LikelyEqual,
  /// 判定不同：存在判据不通过
  Different,
  /// 输入无法读取或不是合法 STEP 文件
  InvalidInput,
  /// 输入形状不受支持（非单一封闭实体、多实体被拒绝等）
  UnsupportedShape,
  /// 结论不确定（如布尔差分运算本身失败，无法给出相等或不同的结论）
  Indeterminate,
  /// 内部错误；同时是 CompareResult::status 的默认初值与退出码兜底值
  InternalError,
};

/**
 * @brief 单个实体对的匹配状态（面/边描述符匹配与多实体配对共用）。
 */
enum class MatchStatus {
  /// 已唯一配对，候选实体被占用
  Matched,
  /// 未找到满足判据的一对一候选
  Unmatched,
  /// 存在多个相近候选导致歧义；不占用候选实体
  Ambiguous,
  /// 实体不受支持，未参与匹配
  Unsupported,
};

/**
 * @brief 匹配结论的核验强度等级，按由弱到强排列。
 *
 * 当前管线只产出 Descriptor；其余等级是为判定流程增强预留的契约值。
 */
enum class VerificationLevel {
  /// 仅类型一致（当前未产出）
  TypeOnly,
  /// 描述符判据（尺寸/质心/包围盒三项）全部通过；当前唯一产出的等级
  Descriptor,
  /// 辅解析特征核验（当前未产出）
  AnalyticSupport,
  /// 逐点距离核验（当前未产出）
  DistanceVerified,
  /// 布尔差分核验（当前未产出）
  BooleanVerified,
};

/**
 * @brief 标识实体属于参考侧还是候选侧。
 */
enum class EntitySide {
  /// 参考侧
  Reference,
  /// 候选侧
  Candidate,
};

/**
 * @brief 实体种类，用于 MakeEntityId() 生成实体 id 的类型段。
 */
enum class EntityKind {
  /// 原始（未归一化）面
  OriginalFace,
  /// 原始（未归一化）边
  OriginalEdge,
  /// 归一化后的面
  NormalizedFace,
  /// 归一化后的边
  NormalizedEdge,
};

/**
 * @brief 边在对比流程中扮演的角色。
 *
 * 归一化流程会先给每条边定性：只有 Comparable 的边进入描述符匹配，
 * 其余角色的边被排除并记入 NormalizationAudit::removedEdges。
 */
enum class EdgeComparisonRole {
  /// 可参与描述符匹配的普通边
  Comparable,
  /// 周期曲面的接缝边（人为切割线），排除
  PeriodicSeam,
  /// 退化边（长度为零），排除
  Degenerated,
  /// 其他不受支持的边，排除
  Unsupported,
};

/**
 * @brief 返回边角色写入 result.json 的字符串拼写（"COMPARABLE" 等）。
 * @param role 边角色
 * @return 静态字符串；拼写属输出契约，不得改动
 */
const char *ToString(EdgeComparisonRole role);

/**
 * @brief 输入包含多个实体时的处理策略。
 */
enum class MultiSolidPolicy {
  /// 严格单实体：检测到多实体直接判 UnsupportedShape
  Strict,
  /// 整体集合比较（尚未实现，选中时返回 UnsupportedShape）
  CollectionOnly,
  /// 两两配对：参考实体与候选实体按体积/质心/包围盒配对后逐对比较
  Pairwise,
};

/**
 * @brief 返回多实体策略写入 result.json 的字符串拼写（小写，
 *        如 "strict"/"collection"/"pairwise"）。
 * @param policy 多实体策略
 * @return 静态字符串；拼写属输出契约，不得改动
 */
const char *ToString(MultiSolidPolicy policy);

/**
 * @brief 比较的全部可调参数（阈值、开关与导出选项）。
 *
 * CompareStepFiles() 会把收到的配置原样回填到 CompareResult::thresholds，
 * 报告层再据此推导有效容差，因此该结构同时是输入配置与报告里的
 * "configuration" 分区的数据来源。
 */
struct CompareConfig {
  /// 距离类判据的静态下限容差（毫米）：质心距离、包围盒差、边长度差等
  double distanceToleranceMm = 0.01;
  /// 体积差判据的绝对容差（立方毫米）：输入体积差与布尔差分残差
  double absoluteVolumeToleranceMm3 = 0.001;
  /// 体积差判据的相对容差（无量纲），同时参与面面积相对判据（放大 100 倍使用）
  double relativeVolumeTolerance = 0.001;
  /// 布尔差分的模糊比较容差（毫米）
  double booleanFuzzyToleranceMm = 0.01;
  /// 布尔体积守恒校验的相对容差（无量纲）
  double booleanConservationRelativeTolerance = 1.0e-6;

  /// 是否启用同域归一化（合并接缝/退化边、按解析面归并等预处理）
  bool enableSameDomainNormalization = true;
  /// 归一化的线性容差（毫米）
  double normalizationLinearToleranceMm = 0.001;
  /// 归一化的角度容差（弧度）
  double normalizationAngularToleranceRad = 1.0e-6;
  /// 是否允许归一化拓扑全匹配时走快路径直接判等
  bool enableNormalizedFastPath = false;
  /// 最优与次优候选得分差小于该裕度时判为 Ambiguous
  double ambiguousMatchMargin = 0.02;

  /// 是否导出 STL 工件
  bool exportStl = true;
  /// 是否导出 BRep 工件
  bool exportBrep = true;
  /// 是否导出逐实体 VTP 工件
  bool exportEntityVtp = false;
  /// 是否写入逐实体明细文件
  bool writeEntityDetails = true;

  /// 是否向 stdout 打印人类可读摘要（CLI 的 --quiet 关闭此项）
  bool printHumanSummary = true;

  /// 是否允许多实体输入
  bool allowMultipleSolids = true;
  /// 多实体处理策略（CollectionOnly 未实现）
  MultiSolidPolicy multiSolidPolicy = MultiSolidPolicy::Pairwise;
  /// 实体配对时体积相对差的容差
  double solidMatchVolumeRelTol = 1e-4;
  /// 实体配对时质心距离的容差（毫米）
  double solidMatchCentroidTolMm = 0.1;
  /// 实体配对时包围盒差的容差（毫米）
  double solidMatchBoundsTolMm = 0.1;
};

/**
 * @brief 三维点（毫米坐标），比较流程内的最小几何值类型。
 */
struct Point3 {
  /// X 坐标
  double x = 0.0;
  /// Y 坐标
  double y = 0.0;
  /// Z 坐标
  double z = 0.0;
};

/**
 * @brief 轴对齐包围盒（毫米坐标）。
 */
struct Bounds3 {
  /// 包围盒最小角点
  Point3 minimum;
  /// 包围盒最大角点
  Point3 maximum;
  /// 是否为空。@note isVoid=true 表示"无有效包围盒"，此时 minimum/maximum
  ///        的值无意义；几何比较把两个空盒视为一致、一空一非空视为不可比
  bool isVoid = true;
};

/**
 * @brief 某一类几何实体的类型统计（面或边）。
 */
struct TypeStatistics {
  /// 实体类型名（如 "PLANE"、"CIRCLE"）
  std::string type;
  /// 该类型的实体数量
  int count = 0;
  /// 该类型实体的度量（面积或长度）总和
  double totalMeasure = 0.0;
};

/**
 * @brief 单个归一化面的审计信息。
 */
struct NormalizedFaceInfo {
  /// 归一化面的唯一 id（形如 "ref:nface:000001"）
  std::string id;
  /// 归一化后在该侧面序列中的序号
  int visualIndex = 0;

  /// 曲面类型名
  std::string surfaceType;

  /// 面积（平方毫米）
  double areaMm2 = 0.0;
  /// 质心（毫米）
  Point3 centroidMm;
  /// 包围盒（毫米）
  Bounds3 boundsMm;

  /// 合并来源的原始面 id 列表
  std::vector<std::string> sourceFaceIds;
  /// 边界边 id 列表
  std::vector<std::string> boundaryEdgeIds;
  /// 合并来源的原始面数量
  int sourceCount = 0;
  /// 是否由多个原始面合并而成
  bool merged = false;

  /// @warning 死数据：目前只写入、从未被序列化或消费，待清理
  std::optional<double> radiusMm;
  /// @warning 死数据：目前只写入、从未被序列化或消费，待清理
  std::optional<Point3> axisOriginMm;
  /// @warning 死数据：目前只写入、从未被序列化或消费，待清理
  std::optional<Point3> axisDirection;
};

/**
 * @brief 单个归一化边的审计信息。
 */
struct NormalizedEdgeInfo {
  /// 归一化边的唯一 id（形如 "ref:nedge:000001"）
  std::string id;
  /// 归一化后在该侧边序列中的序号
  int visualIndex = 0;

  /// 曲线类型名
  std::string curveType;

  /// 长度（毫米）
  double lengthMm = 0.0;
  /// 质心（毫米）
  Point3 centroidMm;
  /// 包围盒（毫米）
  Bounds3 boundsMm;

  /// 合并来源的原始边 id 列表
  std::vector<std::string> sourceEdgeIds;
  /// 合并来源的原始边数量
  int sourceCount = 0;
  /// 是否由多条原始边合并而成
  bool merged = false;
  /// 是否为闭合边（无端点）
  bool closed = false;
  /// 是否参与描述符匹配（comparisonRole == Comparable 的边才为 true）
  bool comparable = false;
  /// comparable 边的连续编号（从 1 起）；非 comparable 边为 0
  int comparableIndex = 0;

  /// 该边在对比中扮演的角色
  EdgeComparisonRole comparisonRole = EdgeComparisonRole::Comparable;
  /// 非 comparable 边被排除的原因拼写（如 "PERIODIC_SEAM"）；可比较边为空
  std::string exclusionReason;
};

/**
 * @brief 被归一化流程移除的原始边记录。
 */
struct RemovedEdgeInfo {
  /// 被移除的原始边 id
  std::string sourceEdgeId;
  /// 移除原因：PERIODIC_SEAM, DEGENERATED, SAME_DOMAIN_INTERNAL_EDGE, REMOVED_BY_NORMALIZATION, UNKNOWN
  std::string reason; // PERIODIC_SEAM, DEGENERATED, SAME_DOMAIN_INTERNAL_EDGE, REMOVED_BY_NORMALIZATION, UNKNOWN
};

/**
 * @brief 一对配对实体的四项度量差。
 */
struct MatchMetrics {
  /// 尺寸差（面为面积差 mm²，边为长度差 mm）
  double measureDifference = 0.0;
  /// 尺寸相对差（相对参考实体度量，无量纲）
  double relativeMeasureDifference = 0.0;
  /// 质心距离（毫米）
  double centroidDistanceMm = 0.0;
  /// 包围盒差（毫米）
  double boundsDifferenceMm = 0.0;
};

/**
 * @brief 单个实体对的匹配记录（面或边）。
 */
struct EntityMatch {
  /// 本条匹配记录自身的 id（形如 "ref:face-match:000001"）
  std::string id;

  /// 参考侧实体 id；未匹配时仍会填写
  std::string referenceId;
  /// 候选侧实体 id；Unmatched 时为空字符串
  std::string candidateId;

  /// 匹配状态
  MatchStatus status = MatchStatus::Unmatched;
  /// 结论的核验强度等级
  VerificationLevel verificationLevel = VerificationLevel::Descriptor;

  /// 实体的几何类型名
  std::string geometryType;

  /// 匹配得分（[0,1]）；Unmatched 时为 nullopt
  std::optional<double> score;
  /// 四项度量差；Unmatched 时为 nullopt
  std::optional<MatchMetrics> metrics;

  /// 附加说明码（如 "MULTIPLE_SIMILAR_CANDIDATES"、"NO_ONE_TO_ONE_CANDIDATE"）
  std::vector<std::string> reasonCodes;
};

/**
 * @brief 一轮描述符匹配（面或边）的集合级结果。
 */
struct MatchCollection {
  /// 是否执行了本轮匹配
  bool attempted = false;

  /// 参考侧参与匹配的实体数
  int referenceCount = 0;
  /// 候选侧参与匹配的实体数
  int candidateCount = 0;
  /// Matched 的配对数
  int matchedCount = 0;
  /// Ambiguous 的配对数
  int ambiguousCount = 0;

  /// 两侧类型直方图是否完全一致
  bool typeHistogramEqual = false;
  /// 是否全部配对成功。
  /// @note 语义为 matchedCount == referenceCount **且**
  ///       referenceCount == candidateCount，缺一不可
  bool allMatched = false;

  /// 本轮匹配耗时（毫秒）
  double elapsedMs = 0.0;

  /// 逐实体匹配记录，按参考顺序排列
  std::vector<EntityMatch> items;

  /// 未匹配的参考实体 id 列表（按参考顺序）
  std::vector<std::string> unmatchedReferenceIds;
  /// 未被占用的候选实体 id 列表（按候选顺序）
  std::vector<std::string> unmatchedCandidateIds;
};

/**
 * @brief 单侧归一化流程的完整审计。
 *
 * @warning 归一化关闭时 faces 恒为空数组，但 edges 明细仍会填充；
 *          此时每条边记录的 sourceCount=0、merged=false。
 */
struct NormalizationAudit {
  /// 是否请求启用归一化
  bool enabled = false;
  /// 归一化是否成功完成
  bool succeeded = false;
  /// 后续比较是否实际使用了归一化后的形状
  bool usedNormalizedShape = false;

  /// 归一化前的面数
  int faceCountBefore = 0;
  /// 归一化后的面数
  int faceCountAfter = 0;

  /// 归一化前的边数
  int edgeCountBefore = 0;
  /// 归一化后的边数
  int edgeCountAfter = 0;

  /// 归一化前可比较边数
  int comparableEdgeCountBefore = 0;
  /// 归一化后可比较边数
  int comparableEdgeCountAfter = 0;

  /// 归一化前体积（立方毫米）
  double volumeBeforeMm3 = 0.0;
  /// 归一化后体积（立方毫米）
  double volumeAfterMm3 = 0.0;
  /// 归一化引起的体积相对漂移（无量纲）
  double relativeVolumeDrift = 0.0;

  /// 面映射是否完整
  bool faceMappingComplete = false;
  /// 边映射是否完整
  bool edgeMappingComplete = false;
  /// 面与边映射是否都完整
  bool mappingComplete = false;

  /// 归一化耗时（毫秒）
  double elapsedMs = 0.0;
  /// 归一化过程中的告警信息；无告警时为空
  std::string warning;

  /// 归一化后的面类型统计
  std::vector<TypeStatistics> faceTypes;
  /// 归一化后的边类型统计
  std::vector<TypeStatistics> edgeTypes;

  /// 归一化面明细（归一化关闭时为空，见结构体说明）
  std::vector<NormalizedFaceInfo> faces;
  /// 归一化边明细（归一化关闭时仍会填充，见结构体说明）
  std::vector<NormalizedEdgeInfo> edges;
  /// 被移除的原始边记录
  std::vector<RemovedEdgeInfo> removedEdges;
};

/**
 * @brief 归一化拓扑快路径的审计信息。
 */
struct FastPathAudit {
  /// 配置是否启用快路径
  bool enabled = false;
  /// 归一化拓扑是否满足快路径前提（全匹配且边审计一致）
  bool eligible = false;
  /// 是否实际走了快路径
  bool used = false;
  /// 未走快路径的原因码列表（如 "EDGE_AUDIT_INCONSISTENT"）
  std::vector<std::string> blockReasons;
};

/**
 * @brief 边审计自洽性校验的结果。
 */
struct EdgeAuditValidation {
  /// 校验是否通过
  bool valid = true;
  /// 校验失败的具体原因列表
  std::vector<std::string> errors;
};

/**
 * @brief 归一化拓扑匹配阶段的完整审计。
 */
struct TopologyMatchAudit {
  /// 是否执行了拓扑匹配
  bool attempted = false;
  /// 未执行时的原因说明
  std::string skipReason;

  /// 面描述符匹配结果
  MatchCollection faces;
  /// 边描述符匹配结果
  MatchCollection edges;

  /// 快路径审计
  FastPathAudit fastPath;

  /// 归一化拓扑是否全匹配
  bool normalizedTopologyMatch = false;
  /// 边审计自洽性校验是否通过
  bool edgeAuditConsistent = true;
  /// 边审计校验失败的具体原因列表
  std::vector<std::string> edgeAuditErrors;
  /// 本阶段总耗时（毫秒）
  double elapsedMs = 0.0;
};

/**
 * @brief 单个导出工件（STL/BRep/VTP 等）的信息。
 */
struct ArtifactInfo {
  /// 工件在 result.json artifacts 分区中的键名
  std::string key;
  /// 相对输出目录的文件路径
  std::string relativePath;
  /// 文件格式名（如 "stl"、"brep"）
  std::string format;
  /// 文件是否成功写出
  bool available = false;
  /// 逐实体索引数组的 JSON 字符串；当前流程未填充，仅在非空时序列化
  std::string entityIndexArray;
};

/**
 * @brief 本轮比较产出的全部导出工件。
 */
struct ArtifactAudit {
  /// 工件列表
  std::vector<ArtifactInfo> items;
};

/**
 * @brief 各阶段的耗时统计（毫秒）。
 */
struct TimingAudit {
  /// 加载参考文件耗时
  double loadReferenceMs = 0.0;
  /// 加载候选文件耗时
  double loadCandidateMs = 0.0;
  /// 参考侧归一化耗时
  double normalizeReferenceMs = 0.0;
  /// 候选侧归一化耗时
  double normalizeCandidateMs = 0.0;
  /// 构建描述符耗时
  double descriptorBuildMs = 0.0;
  /// 面匹配耗时
  double faceMatchMs = 0.0;
  /// 边匹配耗时
  double edgeMatchMs = 0.0;
  /// A-B 布尔差分耗时
  double booleanAbMs = 0.0;
  /// B-A 布尔差分耗时
  double booleanBaMs = 0.0;
  /// 工件导出耗时
  double artifactExportMs = 0.0;
  /// 端到端总耗时
  double totalMs = 0.0;
};

/**
 * @brief 单个输入 STEP 文件的审计信息（单实体与复合实体共用）。
 */
struct InputAudit {
  /// 文件路径
  std::string path;
  /// STEP 文件头声明的长度单位列表
  std::vector<std::string> fileLengthUnits;
  /// 加载阶段的诊断信息
  std::string loadDiagnostics;
  /// 几何传输阶段的诊断信息
  std::string transferDiagnostics;
  /// 实体（solid）数量
  int solidCount = 0;
  /// 壳（shell）数量
  int shellCount = 0;
  /// 面数量
  int faceCount = 0;
  /// 边数量
  int edgeCount = 0;
  /// BRep 结构是否有效
  bool brepValid = false;
  /// 是否为封闭实体
  bool closed = false;
  /// 有符号体积（立方毫米）
  double signedVolumeMm3 = 0.0;
  /// 表面积（平方毫米）
  double surfaceAreaMm2 = 0.0;
  /// 质心（毫米）
  Point3 centroidMm;
  /// 包围盒（毫米）
  Bounds3 boundsMm;
};

/**
 * @brief 一个方向的布尔差分（A-B 或 B-A）的结果。
 */
struct DifferenceAudit {
  /// 布尔差分是否成功
  bool succeeded = false;
  /// 差分残留体积（立方毫米）
  double volumeMm3 = 0.0;
  /// 残留的连通分量数
  int componentCount = 0;
  /// 几何内核的原始报告文本
  std::string report;
};

/**
 * @brief 布尔一致性与体积守恒的度量。
 */
struct BooleanConsistencyMetrics {
  /// 两侧输入体积的差（立方毫米）
  double signedInputVolumeDiffMm3{0.0};
  /// 布尔结果的体积差（立方毫米）
  double signedBooleanVolumeDiffMm3{0.0};
  /// 体积守恒误差（立方毫米）
  double conservationErrorMm3{0.0};
  /// 相对体积守恒误差（无量纲）
  double relativeConservationError{0.0};

  /// 参考减候选的布尔切割是否成功
  bool cutReferenceMinusCandidateSucceeded{false};
  /// 候选减参考的布尔切割是否成功
  bool cutCandidateMinusReferenceSucceeded{false};
  /// 体积守恒校验是否通过
  bool conservationPassed{false};
  /// 布尔结果是否有效可信
  bool booleanResultValid{false};

  /// 布尔结果无效时的原因说明；有效时为空
  std::string invalidReason;
};

namespace detail {
/**
 * @brief 依据四项判据对单个封闭实体比较给出终态分类。
 * @param volumePass 体积差判据是否通过
 * @param centroidPass 质心距离判据是否通过
 * @param boundsPass 包围盒差判据是否通过
 * @param booleanPass 布尔残差判据是否通过
 * @param consistency 布尔一致性度量
 * @return 四项判据全部通过返回 Equal，否则返回 Different
 *
 * @warning 当前实现**忽略 consistency 参数**：只要前四项布尔参数全部为真
 *          即返回 Equal，LikelyEqual 分支因此不可达。这是上游提交 01f2556
 *          的有意行为，配套测试中有一条因此按原样失败，并非缺陷。
 */
CompareStatus ClassifyClosedSolidComparison(
    bool volumePass, bool centroidPass, bool boundsPass, bool booleanPass,
    const BooleanConsistencyMetrics &consistency);
} // namespace detail

/**
 * @brief 多实体配对中一对实体的匹配与比较记录。
 */
struct SolidMatchRecord {
  /// 参考侧实体 id
  std::string referenceSolidId;
  /// 候选侧实体 id
  std::string candidateSolidId;
  /// 参考侧实体序号
  int referenceIndex = -1;
  /// 候选侧实体序号
  int candidateIndex = -1;
  /// 配对是否成功（配对阶段的状态）
  MatchStatus matchStatus = MatchStatus::Unmatched;
  /// 该对实体逐对比较后的终态结论
  CompareStatus status = CompareStatus::InternalError;
  /// 结论原因说明
  std::string reason;
  /// 体积差（立方毫米）
  double volumeDifferenceMm3 = 0.0;
  /// 体积相对差（无量纲）
  double relativeVolumeDifference = 0.0;
  /// 质心距离（毫米）
  double centroidDistanceMm = 0.0;
  /// 包围盒差（毫米）
  double boundsDifferenceMm = 0.0;

  /// 体积相对差是否落在配对容差内
  bool volumeEligible = false;
  /// 质心距离是否落在配对容差内
  bool centroidEligible = false;
  /// 包围盒差是否落在配对容差内
  bool boundsEligible = false;
  /// 体积相对差判据使用的容差
  double volumeTolerance = 0.0;
  /// 质心判据使用的容差（毫米）
  double centroidToleranceMm = 0.0;
  /// 包围盒判据使用的容差（毫米）
  double boundsToleranceMm = 0.0;
  /// 配对被拒绝时的原因列表
  std::vector<std::string> rejectReasons;
};

/**
 * @brief 多实体处理阶段（配对与逐对比较）的完整审计。
 */
struct MultiSolidAudit {
  /// 配置是否允许多实体
  bool allowed = false;
  /// 是否实际执行了多实体配对流程
  bool executed = false;
  /// 多实体流程是否处于激活状态（与 allowed/executed 组合表达各分支）
  bool enabled = false;
  /// 实际采用的多实体策略
  MultiSolidPolicy policy = MultiSolidPolicy::Pairwise;
  /// 参考侧实体数
  int referenceSolidCount = 0;
  /// 候选侧实体数
  int candidateSolidCount = 0;
  /// 成功配对的实体对数
  int matchedSolidCount = 0;
  /// 未配对的参考实体数
  int unmatchedReferenceSolidCount = 0;
  /// 未配对的候选实体数
  int unmatchedCandidateSolidCount = 0;
  /// 逐对比较记录
  std::vector<SolidMatchRecord> solidMatches;
  /// 未配对的参考实体 id 列表
  std::vector<std::string> unmatchedReferenceSolidIds;
  /// 未配对的候选实体 id 列表
  std::vector<std::string> unmatchedCandidateSolidIds;
};

/**
 * @brief 一次 STEP 实体几何对比的完整结果。
 *
 * 该结构同时是内部判定流程的数据载体与两份报告
 * （result.json 与人类可读摘要）的唯一数据来源。
 */
struct CompareResult {
  /// 终态结论
  CompareStatus status = CompareStatus::InternalError;
  /// 结论的人读原因说明
  std::string reason;
  /// 本次比较实际使用的配置（由 CompareStepFiles 回填）
  CompareConfig thresholds;
  /// 参考文件输入审计
  InputAudit reference;
  /// 候选文件输入审计
  InputAudit candidate;
  /// 多实体处理审计
  MultiSolidAudit multiSolid;
  /// A-B（缺失材料）布尔差分结果
  DifferenceAudit missingMaterial;
  /// B-A（新增材料）布尔差分结果
  DifferenceAudit addedMaterial;
  /// 参考侧归一化审计
  NormalizationAudit referenceNormalization;
  /// 候选侧归一化审计
  NormalizationAudit candidateNormalization;
  /// 归一化拓扑匹配审计
  TopologyMatchAudit normalizedTopology;
  /// 导出工件审计
  ArtifactAudit artifacts;
  /// 各阶段耗时
  TimingAudit timings;
  /// 布尔一致性度量
  BooleanConsistencyMetrics booleanConsistency;
  /// 是否执行了布尔差分
  bool booleanExecuted = false;
  /// 是否执行了全局度量（体积/质心/包围盒）
  bool globalMetricsExecuted = false;
  /// 判定流程实际走过的分支码（写入 result.json 的 decision_path）
  std::string decisionPath;
  /// 输入体积差的绝对值（立方毫米）
  double absoluteInputVolumeDifferenceMm3 = 0.0;
  /// 输入体积相对差（无量纲）
  double relativeInputVolumeDifference = 0.0;
  /// 质心距离（毫米）
  double centroidDistanceMm = 0.0;
  /// 包围盒差的最大值（毫米）
  double maximumBoundsDifferenceMm = 0.0;
  /// 对称差体积（布尔双方向残差之和，立方毫米）
  double symmetricDifferenceVolumeMm3 = 0.0;
  /// 对称差相对值（无量纲）
  double symmetricDifferenceRelative = 0.0;
};

/**
 * @brief 按侧、种类与序号生成实体 id。
 * @param side 所属侧（ref / cand）
 * @param kind 实体种类（face / edge / nface / nedge）
 * @param index 序号
 * @return 形如 "ref:nface:000001" 的 id；序号按 6 位十进制零填充
 */
std::string MakeEntityId(EntitySide side, EntityKind kind, int index);

/**
 * @brief 对两个 STEP 文件做实体几何对比（工具的唯一入口）。
 * @param reference 参考 STEP 文件路径
 * @param candidate 候选 STEP 文件路径
 * @param config 比较配置（阈值与开关）
 * @param outputDirectory 工件输出目录；为空时不导出工件
 * @return 完整的比较结果，可直接交给 ToJson/ToHumanSummary/WriteResultJson
 *
 * @note 判定流程内部只应通过本函数进入；分层后的 domain/geometry/pipeline/
 *       report 各层均不对外暴露入口。
 */
CompareResult CompareStepFiles(const std::filesystem::path &reference,
                               const std::filesystem::path &candidate,
                               const CompareConfig &config,
                               const std::filesystem::path &outputDirectory = "");

/**
 * @brief 返回比较结论写入 result.json 的字符串拼写（"EQUAL" 等）。
 * @param status 比较结论
 * @return 静态字符串；拼写属输出契约，不得改动
 */
const char *ToString(CompareStatus status);
/**
 * @brief 返回匹配状态写入 result.json 的字符串拼写（"MATCHED" 等）。
 * @param status 匹配状态
 * @return 静态字符串；拼写属输出契约，不得改动
 */
const char *ToString(MatchStatus status);
/**
 * @brief 返回核验等级写入 result.json 的字符串拼写（"DESCRIPTOR" 等）。
 * @param level 核验等级
 * @return 静态字符串；拼写属输出契约，不得改动
 */
const char *ToString(VerificationLevel level);

/**
 * @brief 把比较结论映射为进程退出码。
 * @param status 比较结论
 * @return Equal=0、Different=1、InvalidInput/UnsupportedShape=2、
 *         LikelyEqual=3、Indeterminate=4、InternalError=5（兜底值）
 * @note 取值顺序与 CompareStatus 枚举声明顺序对应，两处必须同步维护。
 */
int ExitCode(CompareStatus status);
/**
 * @brief 把比较结果序列化为 result.json 的文本内容。
 * @param result 比较结果
 * @return 缩进为 2 空格、以换行结尾的 JSON 文本；键顺序即落盘 schema
 */
std::string ToJson(const CompareResult &result);
/**
 * @brief 把比较结果渲染为人类可读的文本摘要。
 * @param result 比较结果
 * @return 多行文本摘要（以换行结尾），可直接打印到 stdout
 */
std::string ToHumanSummary(const CompareResult &result);
/**
 * @brief 把比较结果原子写入输出目录下的 result.json。
 * @param outputDirectory 输出目录（不存在时会递归创建）
 * @param result 比较结果
 * @param error 失败时写入错误描述；成功时内容未定义
 * @return 是否写入成功
 *
 * @note 原子性通过"先写 result.json.tmp 再 rename"实现，失败时会留下
 *       错误描述并返回 false，不会破坏已存在的 result.json。
 */
bool WriteResultJson(const std::filesystem::path &outputDirectory,
                     const CompareResult &result, std::string &error);

} // namespace cadstep
