#pragma once

#include "../../core/UnifiedFeatures.h"

#include <utility>
#include <vector>

namespace CADExchange {
namespace Geometry {

/**
 * @brief CRTP 基类：统一管理几何边容器，派生类负责具体 CAD 接口读取。
 *
 * 用法：
 * 1) 派生类实现 `CollectImpl(...)`
 * 2) 通过基类 `Collect(...)` 触发采集（先清空，再调用派生实现）
 */
template <typename Derived, typename EdgeT = CRefEdge>
class GeometryCollectorBase {
public:
  using EdgeType = EdgeT;

  template <typename... Args>
  auto Collect(Args &&...args)
      -> decltype(std::declval<Derived &>().CollectImpl(
          std::forward<Args>(args)...)) {
    Clear();
    return DerivedSelf().CollectImpl(std::forward<Args>(args)...);
  }

  const std::vector<EdgeType> &GetEdges() const noexcept { return m_edges; }
  std::size_t EdgeCount() const noexcept { return m_edges.size(); }
  void Clear() noexcept { m_edges.clear(); }

protected:
  void AddEdge(const EdgeType &edge) { m_edges.push_back(edge); }
  void AddEdge(EdgeType &&edge) { m_edges.emplace_back(std::move(edge)); }

private:
  Derived &DerivedSelf() noexcept { return static_cast<Derived &>(*this); }

private:
  std::vector<EdgeType> m_edges;
};

} // namespace Geometry
} // namespace CADExchange
