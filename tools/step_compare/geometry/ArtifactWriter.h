#pragma once

#include "StepCompare.h"

#include <TopoDS_Shape.hxx>
#include <TopTools_IndexedMapOfShape.hxx>

#include <filesystem>
#include <vector>

namespace cadstep {
namespace detail {

bool ExportShapeStl(const TopoDS_Shape &shape, const std::filesystem::path &stlPath);
bool ExportShapeBrep(const TopoDS_Shape &shape, const std::filesystem::path &brepPath);

// The VTP writers take the audit records and the match collection so the exported
// CellData carries the same entity indices and match statuses the report shows.
bool ExportFacesVtp(const TopoDS_Shape &solid,
                    const TopTools_IndexedMapOfShape &normalizedFaces,
                    const std::vector<NormalizedFaceInfo> &faceInfos,
                    const MatchCollection &faceMatches,
                    EntitySide side,
                    const std::filesystem::path &outputPath);

bool ExportEdgesVtp(const TopoDS_Shape &solid,
                    const TopTools_IndexedMapOfShape &normalizedEdges,
                    const std::vector<NormalizedEdgeInfo> &edgeInfos,
                    const MatchCollection &edgeMatches,
                    EntitySide side,
                    const std::filesystem::path &outputPath);

void RemoveOldArtifacts(const std::filesystem::path &outputDirectory);


} // namespace detail
} // namespace cadstep
