# 基准面跨系统参数对照（Creo / UG）2026-03-18

## 1. 目标
将 Creo 与 UG 的基准面读取/创建参数统一到一套 CADExchange 可落地字段，作为 `UnifiedFeatures` 评估与扩展依据。

## 2. 统一字段对照表

| 统一字段 | 字段含义 | Creo 读取来源 | Creo 创建输入 | UG 读取来源 | UG 创建输入 | 建议优先级 |
|---|---|---|---|---|---|---|
| `construction_method` | 基准面构造方法 | `PRO_E_DTMPLN_CONSTR_TYPE` | 必填（每条约束） | 由创建法/约束类型反推（feature + API） | `fixed/point_dir/relative` 或 Builder 方法 | P0 |
| `references[]` | 引用实体列表（ID/类型/角色） | `PRO_E_DTMPLN_CONSTR_REF` | 必填（按构造法） | object/feature 关系 + 约束对象 | `object_tags[]` / Builder 几何对象 | P0 |
| `constraint_type` | 单条约束类型 | `ProDtmplnConstrType` | 必填 | Builder `ConstraintType` / relative 组合 | 必填（relative/Builder） | P0 |
| `offset_value` | 偏置数值 | `PRO_E_DTMPLN_CONSTR_REF_OFFSET` | Offset 场景必填 | `UF_MODL_ask_datum_plane_parms` 的 `offset`（表达式） | `offset_string` / `SetFaceAndOffset` | P0 |
| `angle_value` | 角度数值 | `PRO_E_DTMPLN_CONSTR_REF_ANGLE` | Angle 场景必填 | `UF_MODL_ask_datum_plane_parms` 的 `angle`（表达式） | `angle_string` / Builder 角度约束 | P0 |
| `driving_expression` | 公式/表达式文本 | 可从元素值与参数系统补取 | 推荐保留 | `offset/angle` 原生为 expression string | `offset_string/angle_string/constraint expr` | P0 |
| `flip/reverse` | 方向翻转 | `PRO_E_DTMPLN_FLIP_DIR` | 可选 | 通过解选择/法向可推断 | `which_plane` / alternate solution | P1 |
| `solution_selector` | 多解消歧参数 | 通过约束组合/方向推断 | 部分场景需要 | `which_plane` + `reference_point` | relative 必备（多解时） | P1 |
| `point_select_mode` | edge 取点方式 | N/A | N/A | N/A | `point_select[]`（edge/mid/end） | P1 |
| `fit_options` | 平面显示范围/拟合 | `PRO_E_DTMPLN_FIT_*` | 可选 | N/A | N/A | P2 |
| `section_index` | 剖面索引 | `PRO_E_DTMPLN_SEC_IND` | Section 场景 | N/A | N/A | P2 |
| `csys_axis_offset` | CSYS 轴向偏置 | `PRO_E_DTMPLN_OFF_CSYS*` | Offset+CSYS 场景 | N/A | N/A | P2 |
| `result_origin` | 创建后几何中心 | 几何查询/推导 | 回填 | `UF_MODL_ask_datum_plane(_parms)` | 回填 | P0 |
| `result_normal` | 创建后法向 | 几何查询/推导 | 回填 | `UF_MODL_ask_datum_plane(_parms)` | 回填 | P0 |

## 3. 各系统创建方式到统一 `construction_method` 的映射

## 3.1 Creo
1. `PRO_DTMPLN_OFFS` -> `offset_plane`
2. `PRO_DTMPLN_ANG` -> `angle_plane`
3. `PRO_DTMPLN_PRL` -> `parallel_plane`
4. `PRO_DTMPLN_NORM` -> `normal_to_ref`
5. `PRO_DTMPLN_THRU` -> `through_ref`
6. `PRO_DTMPLN_TANG` -> `tangent_plane`
7. `PRO_DTMPLN_MIDPLN` -> `mid_plane`
8. `PRO_DTMPLN_BISECTOR1/2` -> `bisector_plane`
9. `PRO_DTMPLN_DEF_X/Y/Z` -> `default_plane`
10. `PRO_DTMPLN_THRU_CSYS_*` -> `csys_plane`

## 3.2 UG（UFUN / NXOpen）
1. `UF_MODL_create_fixed_dplane` -> `fixed_plane`
2. `UF_MODL_create_point_dirr_dplane` -> `point_direction_plane`
3. `UF_MODL_create_relative_dplane` -> `relative_constraint_plane`
4. `DatumPlaneBuilder::SetFaceAndOffset` -> `offset_plane`
5. `DatumPlaneBuilder::SetPointAndDirection` -> `point_direction_plane`
6. `DatumPlaneBuilder::SetThreePoints` -> `three_points_plane`
7. `DatumPlaneBuilder::SetPointOnCurve` -> `point_on_curve_plane`
8. `DatumPlaneBuilder::SetGeometryAndConstraints` -> `dual_geometry_constraint_plane`
9. `DatumPlaneBuilder::SetFixedDatumPlane` -> `fixed_plane`

## 4. 最小可落地字段集（建议先实现）

P0（先打通偏置平面与固定平面闭环）：
1. `construction_method`
2. `references[]`（`id/type/role`）
3. `offset_value` / `angle_value`
4. `driving_expression`
5. `result_origin` + `result_normal`

P1（提高稳定重建）：
1. `flip/reverse`
2. `solution_selector`
3. `point_select_mode`

P2（高级场景）：
1. `fit_options`
2. `section_index`
3. `csys_axis_offset`

## 5. 推荐验证顺序
1. `fixed_plane`（UG）/ `default_plane`（Creo）先验证创建链路。
2. `offset_plane` 跨系统统一验证（你当前目标最匹配）。
3. 加入 `angle_plane` 与 `parallel_plane`。
4. 最后再上 `mid_plane/bisector/tangent/section`。

## 6. 参考文档与 API 地址（本机路径）

## 6.1 Creo
1. `E:/Application/Creo/install_path/CREO7/Creo 7.0.0.0/Common Files/protoolkit/includes/ProDtmPln.h`
2. `E:/Application/Creo/install_path/CREO7/Creo 7.0.0.0/Common Files/protoolkit/includes/ProFeature.h`
3. `E:/Application/Creo/install_path/CREO7/Creo 7.0.0.0/Common Files/protoolkit/protkdoc/api/prodtmpln_h.html`

## 6.2 UG
1. `E:/Application/UG/install_path/NX 11.0/UGOPEN/uf_modl_datum_features.h`
2. `E:/Application/UG/install_path/NX 11.0/UGOPEN/uf_modl.h`
3. `E:/Application/UG/install_path/NX 11.0/UGOPEN/NXOpen/Features_DatumPlaneBuilder.hxx`
4. `E:/Application/UG/install_path/NX 11.0/UGOPEN/ufd_modl_ask_datum_plane_parms.c`

## 6.3 本仓库对应调研文档
1. `E:/MyProject/UgLibs/doc/ug_datum_plane_read_create_research_20260318.md`
2. `E:/MyProject/CreoLibs/doc/creo_datum_plane_read_create_research_20260318.md`
