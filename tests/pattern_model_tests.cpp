#include "CADSerializer.h"
#include "FeatureFormatter.h"
#include "PatternAccessor.h"
#include "PatternBuilder.h"
#include "PatternGeometry.h"
#include "ReferenceBuilder.h"
#include <cmath>
#include <filesystem>
#include <functional>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <sstream>
using namespace CADExchange;
namespace {
void Require(bool ok, const std::string &detail) {
  if (!ok) throw std::runtime_error(detail);
}
std::shared_ptr<CRefBody> Body() {
  return Builder::Ref::Body().BoundsMin(0,0,0).BoundsMax(10,20,30)
      .Volume(6000).VolumeCentroid(5,10,15).SurfaceArea(2200)
      .SurfacePoints({{0,5,5},{10,5,5},{5,20,5},{5,5,30}})
      .SourceFeatureIDs({"diagnostic-only"}).Build();
}
std::shared_ptr<CRefEdge> Direction() {
  auto edge=std::make_shared<CRefEdge>();
  edge->startPoint={0,0,0};edge->endPoint={1,0,0};
  edge->midPoint={0.5,0,0};edge->length=1;edge->curveType=CGeoCurveType::LINE;
  return edge;
}
std::shared_ptr<CLinearPattern> Linear(UnifiedModel &model) {
  CLinearPatternDir dir;
  dir.directionRef=Direction();dir.direction={1,0,0};dir.spacing=12;dir.count=4;
  Builder::LinearPatternBuilder builder(model,"linear");
  builder.SetDir1(dir).SetScope(PatternScope::BODIES).AddSeedObject(Body()).Build();
  return builder.GetFeature();
}
void TestExistingContract() {
  UnifiedModel model(UnitType::MILLIMETER);
  auto pattern=Linear(model);
  Require(model.Validate().isValid,"valid Body linear pattern");
  const auto saved=*pattern;
  const std::vector<std::function<void()>> invalid={
    [&]{pattern->seedObjects.clear();},
    [&]{pattern->dir1.count=0;},
    [&]{pattern->dir1.count=1;},
    [&]{pattern->dir1.spacing=-1;},
    [&]{pattern->dir1.spacing=std::numeric_limits<double>::quiet_NaN();},
    [&]{pattern->dir1.direction={0,0,0};},
    [&]{pattern->dir1.direction={2,0,0};},
    [&]{pattern->dir1.directionRef.reset();},
    [&]{pattern->scope=static_cast<PatternScope>(999);},
    [&]{pattern->dir1.spacingType=static_cast<PatternSpacingType>(999);},
    [&]{pattern->seedObjects={nullptr};},
    [&]{pattern->seedObjects={Direction()};},
    [&]{pattern->seedObjects.push_back(pattern->seedObjects.front());},
    [&]{pattern->geometryPattern=true;},
    [&]{pattern->skippedInstances={{0,0}};},
    [&]{pattern->skippedInstances={{-1,0}};},
    [&]{pattern->skippedInstances={{4,0}};},
    [&]{pattern->skippedInstances={{1,1}};},
    [&]{pattern->skippedInstances={{1,0},{1,0}};},
    [&]{pattern->patternSeedOnly=true;},
  };
  for(size_t i=0;i<invalid.size();++i) {
    *pattern=saved;invalid[i]();
    Require(!model.Validate().isValid,"reject invalid ordinary pattern case "+std::to_string(i));
  }
  *pattern=saved;
  auto dir2=pattern->dir1;dir2.count=3;dir2.direction={0,1,0};
  pattern->dir2=dir2;pattern->patternSeedOnly=true;
  pattern->skippedInstances={{1,1}};
  Require(!model.Validate().isValid,"seed-only has no interior grid instances");
  pattern->skippedInstances={{0,2},{2,0}};
  Require(model.Validate().isValid,"seed-only outer axes are valid");
}
void Near(double value,double expected,const std::string &detail) {
  Require(std::abs(value-expected)<=1e-10*(std::max)(1.0,std::abs(expected)),detail);
}
std::shared_ptr<CCircularPattern> Circular(UnifiedModel &model,const std::shared_ptr<CRefBody> &body) {
  CCircularPatternDir dir;dir.axisRef=Builder::Ref::Axis(StandardID::AXIS_Z);
  dir.direction={0,0,1};dir.origin={1,2,3};
  dir.angle=2*GeoUtils::PI;dir.count=6;dir.spacingType=PatternSpacingType::SPAN_AND_COUNT;
  Builder::CircularPatternBuilder builder(model,"circular");
  builder.SetDir1(dir).SetScope(PatternScope::BODIES).AddSeedObject(body).AddSkippedInstance({4,0}).Build();
  return builder.GetFeature();
}
void XmlReject(const std::filesystem::path &source,const std::filesystem::path &out,
               const std::string &name,const std::function<void(tinyxml2::XMLElement*)> &mutate,
               bool cereal=false) {
  tinyxml2::XMLDocument doc;
  Require(doc.LoadFile(source.string().c_str())==tinyxml2::XML_SUCCESS,"load XML fixture");
  auto *feature=cereal?doc.FirstChildElement("cereal")->FirstChildElement("Feature")->FirstChildElement("ptr_wrapper")->FirstChildElement("data")
                      :doc.FirstChildElement("UnifiedModel")->FirstChildElement("Feature");
  Require(feature!=nullptr,"fixture feature");mutate(feature);
  auto file=out/("bad-"+name+".xml");
  Require(doc.SaveFile(file.string().c_str())==tinyxml2::XML_SUCCESS,"save negative fixture");
  UnifiedModel loaded;std::string error;
  Require(!LoadModel(loaded,file,&error,cereal?SerializationFormat::CEREAL:SerializationFormat::TINYXML),"reject XML "+name);
  Require(!error.empty(),"XML rejection explains "+name);
}
template<class Pattern>
void Compare(const Pattern &before,const Pattern &after) {
  Require(before.scope==after.scope&&before.patternSeedOnly==after.patternSeedOnly&&
          before.geometryPattern==after.geometryPattern,"pattern flags roundtrip");
  Require(before.affectedBodyMode==after.affectedBodyMode&&before.affectedBodies.size()==after.affectedBodies.size(),"affected body roundtrip");
  Require(before.seedObjects.size()==after.seedObjects.size()&&before.skippedInstances==after.skippedInstances,"seed and skip roundtrip");
  Require(before.dir1.count==after.dir1.count&&before.dir1.spacingType==after.dir1.spacingType,"primary counts and spacing mode");
  Require(before.dir2.has_value()==after.dir2.has_value(),"second direction presence");
  Near(after.dir1.direction.x,before.dir1.direction.x,"direction x roundtrip");
  Near(after.dir1.direction.y,before.dir1.direction.y,"direction y roundtrip");
  Near(after.dir1.direction.z,before.dir1.direction.z,"direction z roundtrip");
  if constexpr(std::is_same<Pattern,CLinearPattern>::value) Near(after.dir1.spacing,before.dir1.spacing,"linear spacing roundtrip");
  else {
    Near(after.dir1.angle,before.dir1.angle,"angle roundtrip");
    Near(after.dir1.origin.x,before.dir1.origin.x,"axis origin roundtrip");
  }
  if(before.dir2) {
    Near(after.dir2->spacing,before.dir2->spacing,"second spacing roundtrip");
    Require(after.dir2->count==before.dir2->count&&after.dir2->spacingType==before.dir2->spacingType,"second count and spacing mode");
  }
  for(size_t i=0;i<after.seedObjects.size();++i) {
    Require(after.seedObjects[i]->refType==before.seedObjects[i]->refType,"seed RefType roundtrip");
    if(auto body=std::dynamic_pointer_cast<CRefBody>(after.seedObjects[i])) {
      Near(body->volume,6000,"seed Body volume");Near(body->surfaceArea,2200,"seed Body area");
      Require(body->surfacePoints.size()==4&&body->sourceFeatureIDs==std::vector<std::string>{"diagnostic-only"},"Body witnesses and diagnostic IDs");
    }
  }
}
template<class Pattern>
void JsonRoundtrip(const std::shared_ptr<Pattern> &before,const std::filesystem::path &file) {
  const auto json=Builder::FeatureFormatter::ToJson(before);
  Require(json.find("\"error\"")==std::string::npos,"FeatureFormatter JSON save succeeds");
  {
    std::ofstream output(file);output<<json;
    Require(output.good(),"write actual feature JSON fixture");
  }
  std::ifstream input(file);
  cereal::JSONInputArchive archive(input);
  Pattern after;
  archive(cereal::make_nvp("Feature",after));
  Compare(*before,after);
  Require(before->featureID==after.featureID&&before->featureName==after.featureName,"feature identity JSON roundtrip");
  for(size_t i=0;i<before->affectedBodies.size();++i) {
    Near(after.affectedBodies[i]->volume,before->affectedBodies[i]->volume,"affected Body JSON volume");
    Near(after.affectedBodies[i]->surfaceArea,before->affectedBodies[i]->surfaceArea,"affected Body JSON area");
    Require(after.affectedBodies[i]->surfacePoints==before->affectedBodies[i]->surfacePoints,"affected Body JSON surface witnesses");
  }
}
void Roundtrip(const UnifiedModel &model,const std::filesystem::path &out,const std::string &name) {
  for(size_t i=0;i<model.GetFeatures().size();++i) {
    auto file=out/(name+"-feature-"+std::to_string(i)+".json");
    if(auto linear=std::dynamic_pointer_cast<CLinearPattern>(model.GetFeatures()[i])) JsonRoundtrip(linear,file);
    if(auto circular=std::dynamic_pointer_cast<CCircularPattern>(model.GetFeatures()[i])) JsonRoundtrip(circular,file);
  }
  for(auto format:{SerializationFormat::TINYXML,SerializationFormat::CEREAL}) {
    auto file=out/(name+(format==SerializationFormat::TINYXML?"-tiny.xml":"-cereal.xml"));
    std::string error;UnifiedModel loaded;
    const bool saved=SaveModel(model,file,&error,format);
    Require(saved,"save "+name+": "+error);
    const bool read=LoadModel(loaded,file,&error,format);
    Require(read,"load "+name+": "+error);
    Require(loaded.GetFeatures().size()==model.GetFeatures().size(),"pattern model feature count");
    for(size_t i=0;i<model.GetFeatures().size();++i) {
      if(auto before=std::dynamic_pointer_cast<CLinearPattern>(model.GetFeatures()[i])) {
        auto after=std::dynamic_pointer_cast<CLinearPattern>(loaded.GetFeatures()[i]);Require(after!=nullptr,"linear class");Compare(*before,*after);
      } else if(auto before=std::dynamic_pointer_cast<CCircularPattern>(model.GetFeatures()[i])) {
        auto after=std::dynamic_pointer_cast<CCircularPattern>(loaded.GetFeatures()[i]);Require(after!=nullptr,"circular class");Compare(*before,*after);
      }
    }
  }
}
void TestPlaneFrame(const std::filesystem::path &out) {
  UnifiedModel model(UnitType::MILLIMETER);auto pattern=Linear(model);
  auto plane=std::make_shared<CRefPlane>();plane->targetFeatureID=StandardID::PLANE_XY;
  plane->normal={0,0,1};plane->xDir={1,0,0};plane->yDir={0,1,0};
  pattern->dir1.directionRef=plane;pattern->dir1.direction={0,0,1};
  Require(model.Validate().isValid,"finite plane direction reference");
  const auto saved=*plane;
  for(int i=0;i<6;++i) {
    *plane=saved;
    auto &direction=i<3?plane->xDir:plane->yDir;
    const auto nan=std::numeric_limits<double>::quiet_NaN();
    if(i%3==0)direction.x=nan;
    if(i%3==1)direction.y=nan;
    if(i%3==2)direction.z=nan;
    Require(!model.Validate().isValid,"nonfinite plane frame fails validation "+std::to_string(i));
    std::string error;
    Require(!SaveModel(model,out/"must-not-save-plane.xml",&error,SerializationFormat::TINYXML),"nonfinite plane frame cannot save");
    Require(!error.empty(),"nonfinite plane frame save explains rejection");
  }
  *plane=saved;
  Roundtrip(model,out,"plane-reference");
}
template<class Point>
void SameTriple(const Point &before,const Point &after,const std::string &detail) {
  Near(after.x,before.x,detail+" x");Near(after.y,before.y,detail+" y");Near(after.z,before.z,detail+" z");
}
void CompareReferenceGeometry(const std::shared_ptr<CRefEntityBase> &before,
                              const std::shared_ptr<CRefEntityBase> &after) {
  Require(before&&after&&before->refType==after->refType,"strict reference class");
  if(auto edge=std::dynamic_pointer_cast<CRefEdge>(before)) {
    auto loaded=std::dynamic_pointer_cast<CRefEdge>(after);Require(loaded!=nullptr,"strict Edge class");
    SameTriple(edge->startPoint,loaded->startPoint,"strict edge start");
    SameTriple(edge->endPoint,loaded->endPoint,"strict edge end");
    SameTriple(edge->midPoint,loaded->midPoint,"strict edge middle");
  } else if(auto axis=std::dynamic_pointer_cast<CRefAxis>(before)) {
    auto loaded=std::dynamic_pointer_cast<CRefAxis>(after);Require(loaded!=nullptr,"strict Axis class");
    SameTriple(axis->origin,loaded->origin,"strict axis origin");SameTriple(axis->direction,loaded->direction,"strict axis direction");
  } else if(auto plane=std::dynamic_pointer_cast<CRefPlane>(before)) {
    auto loaded=std::dynamic_pointer_cast<CRefPlane>(after);Require(loaded!=nullptr,"strict Plane class");
    SameTriple(plane->origin,loaded->origin,"strict plane origin");SameTriple(plane->normal,loaded->normal,"strict plane normal");
    SameTriple(plane->xDir,loaded->xDir,"strict plane X");SameTriple(plane->yDir,loaded->yDir,"strict plane Y");
  } else if(auto face=std::dynamic_pointer_cast<CRefFace>(before)) {
    auto loaded=std::dynamic_pointer_cast<CRefFace>(after);Require(loaded!=nullptr,"strict Face class");
    SameTriple(face->centroid,loaded->centroid,"strict face witness");SameTriple(face->normal,loaded->normal,"strict face normal");
    SameTriple(face->uDir,loaded->uDir,"strict face U");SameTriple(face->vDir,loaded->vDir,"strict face V");
  } else throw std::runtime_error("Unexpected geometry test reference");
}
void TestStrictReferenceWhitespace(const std::filesystem::path &out) {
  auto edge=Direction();edge->startPoint={1,2,3};edge->endPoint={17,2,3};edge->midPoint={9,2,3};
  auto axis=std::make_shared<CRefAxis>();axis->targetFeatureID=StandardID::AXIS_X;axis->origin={17,2,3};axis->direction={1,0,0};
  auto plane=std::make_shared<CRefPlane>();plane->targetFeatureID=StandardID::PLANE_YZ;
  plane->origin={17,2,3};plane->normal={1,0,0};plane->xDir={0,1,0};plane->yDir={0,0,1};
  auto face=std::make_shared<CRefFace>();face->centroid={17,2,3};face->normal={1,0,0};face->uDir={0,1,0};face->vDir={0,0,1};
  const std::vector<std::shared_ptr<CRefEntityBase>> refs={edge,axis,plane,face};
  const std::vector<std::vector<const char*>> attrs={
    {"StartPoint","EndPoint","MidPoint"},{"Origin","Direction"},
    {"Origin","Normal","XDir","YDir"},{"Center","Normal","U","V"}
  };
  for(size_t i=0;i<refs.size();++i) {
    UnifiedModel model(UnitType::MILLIMETER);auto pattern=Linear(model);pattern->dir1.directionRef=refs[i];
    const auto source=out/("strict-ref-"+std::to_string(i)+".xml");std::string error;
    Require(SaveModel(model,source,&error,SerializationFormat::TINYXML),"save strict reference fixture");
    for(int whitespace=0;whitespace<3;++whitespace) {
      tinyxml2::XMLDocument doc;Require(doc.LoadFile(source.string().c_str())==tinyxml2::XML_SUCCESS,"load strict reference fixture");
      auto *reference=doc.FirstChildElement("UnifiedModel")->FirstChildElement("Feature")->FirstChildElement("Dir1")->FirstChildElement("DirectionReference");
      for(auto name:attrs[i]) {
        std::string value=reference->Attribute(name);
        if(whitespace!=1)value=" \t"+value;
        if(whitespace!=0)value+=" \t";
        reference->SetAttribute(name,value.c_str());
      }
      const auto file=out/("strict-whitespace-"+std::to_string(i)+"-"+std::to_string(whitespace)+".xml");
      Require(doc.SaveFile(file.string().c_str())==tinyxml2::XML_SUCCESS,"save whitespace fixture");
      UnifiedModel loaded;const bool ok=LoadModel(loaded,file,&error,SerializationFormat::TINYXML);
      Require(ok,"strict whitespace reference load: "+error);
      auto after=std::dynamic_pointer_cast<CLinearPattern>(loaded.GetFeatures().front());
      Require(after!=nullptr,"strict whitespace pattern class");CompareReferenceGeometry(refs[i],after->dir1.directionRef);
    }
    for(auto name:attrs[i]) {
      XmlReject(source,out,"reference-garbage-"+std::to_string(i)+name,[=](auto*f){f->FirstChildElement("Dir1")->FirstChildElement("DirectionReference")->SetAttribute(name," (17,2,3)garbage ");});
      XmlReject(source,out,"reference-malformed-"+std::to_string(i)+name,[=](auto*f){f->FirstChildElement("Dir1")->FirstChildElement("DirectionReference")->SetAttribute(name," (17,2) ");});
    }
  }
}
void TestSerialization(const std::filesystem::path &out) {
  UnifiedModel model(UnitType::MILLIMETER);auto linear=Linear(model);
  auto body=std::dynamic_pointer_cast<CRefBody>(linear->seedObjects[0]);
  auto circular=Circular(model,body);Roundtrip(model,out,"bodies");
  const auto file=out/"bodies-tiny.xml";
  for(auto name:{"Scope","PatternSeedOnly","GeometryPattern"})
    XmlReject(file,out,std::string("missing-")+name,[=](auto*f){f->DeleteAttribute(name);});
  for(auto name:{"Direction","SpacingType","Count","Spacing"})
    XmlReject(file,out,std::string("missing-dir-")+name,[=](auto*f){f->FirstChildElement("Dir1")->DeleteAttribute(name);});
  for(auto name:{"Scope","PatternSeedOnly","GeometryPattern"})
    XmlReject(file,out,std::string("bad-")+name,[=](auto*f){f->SetAttribute(name,"nonsense");});
  for(auto name:{"Count","Spacing","Direction","SpacingType"})
    XmlReject(file,out,std::string("bad-dir-")+name,[=](auto*f){f->FirstChildElement("Dir1")->SetAttribute(name,"2junk");});
  XmlReject(file,out,"direction-trailing",[](auto*f){f->FirstChildElement("Dir1")->SetAttribute("Direction","(1,0,0)junk");});
  XmlReject(file,out,"missing-direction-ref",[](auto*f){auto*d=f->FirstChildElement("Dir1");d->DeleteChild(d->FirstChildElement("DirectionReference"));});
  XmlReject(file,out,"missing-seeds",[](auto*f){f->DeleteChild(f->FirstChildElement("SeedObjects"));});
  XmlReject(file,out,"unknown-seed",[](auto*f){f->FirstChildElement("SeedObjects")->FirstChildElement()->SetAttribute("Type","UnknownThing");});
  XmlReject(file,out,"partial-unknown-seed",[](auto*f){auto*s=f->FirstChildElement("SeedObjects");auto*r=s->GetDocument()->NewElement("ReferenceEntity");r->SetAttribute("Type","UnknownThing");s->InsertEndChild(r);});
  XmlReject(file,out,"wrong-seed-type",[](auto*f){f->SetAttribute("Scope","FEATURES");});
  for(auto name:{"BoundsMin","Volume","VolumeCentroid","SurfaceArea"})
    XmlReject(file,out,std::string("missing-body-")+name,[=](auto*f){f->FirstChildElement("SeedObjects")->FirstChildElement()->DeleteAttribute(name);});
  XmlReject(file,out,"body-collinear",[](auto*f){auto*s=f->FirstChildElement("SeedObjects")->FirstChildElement()->FirstChildElement("SurfacePoints");for(auto*p=s->FirstChildElement();p;p=p->NextSiblingElement())p->SetAttribute("Position","(0,0,0)");});
  XmlReject(file,out,"circular-origin",[](auto*f){f->NextSiblingElement()->FirstChildElement("Dir1")->DeleteAttribute("Origin");});
  XmlReject(file,out,"missing-skip-index",[](auto*f){f->NextSiblingElement()->FirstChildElement("SkippedInstances")->FirstChildElement()->DeleteAttribute("Dir2Index");});
  for(auto name:{"AffectedBodyMode","AffectedBodies","Scope","Dir1","SeedObjects"})
    XmlReject(out/"bodies-cereal.xml",out,std::string("cereal-missing-")+name,[=](auto*f){f->DeleteChild(f->FirstChildElement(name));},true);
  CLinearPatternDir second=linear->dir1;second.count=3;second.direction={0,1,0};second.spacing=20;
  linear->dir2=second;linear->patternSeedOnly=true;linear->skippedInstances={{0,1},{2,0}};
  Roundtrip(model,out,"seed-only");
  // Existing circular D2 remains a radial translation and a length quantity.
  circular->dir2=second;Roundtrip(model,out,"radial");circular->dir2.reset();
  auto featureSeed=std::make_shared<CRefFeature>();featureSeed->targetFeatureID=linear->featureID;
  Builder::CircularPatternBuilder geometry(model,"geometry circular");
  geometry.SetDir1(circular->dir1).AddSeedObject(featureSeed).SetGeometryPattern(true)
      .SetAffectedBodyMode(PatternAffectedBodyMode::SelectedBodies).SetAffectedBodies({body}).Build();
  const auto selected=geometry.GetFeature();
  Require(model.Validate().isValid,"valid geometry pattern scope");
  Accessor::CircularPatternAccessor accessor(selected);
  Require(accessor.GetAffectedBodyMode()==PatternAffectedBodyMode::SelectedBodies&&accessor.GetAffectedBodies().size()==1,"circular scope accessor");
  Roundtrip(model,out,"selected");
  // A second affected Body is only reachable through a linear geometry
  // pattern; its dimensions must still be converted exactly once.
  auto affectedOnly=Body();
  Builder::LinearPatternBuilder linearGeometry(model,"geometry linear");
  linearGeometry.SetDir1(linear->dir1).AddSeedObject(featureSeed).SetGeometryPattern(true)
      .SetAffectedBodyMode(PatternAffectedBodyMode::SelectedBodies).AddAffectedBody(affectedOnly).Build();
  Accessor::LinearPatternAccessor linearAccessor(linearGeometry.GetFeature());
  Require(linearAccessor.GetAffectedBodyMode()==PatternAffectedBodyMode::SelectedBodies&&
          linearAccessor.GetAffectedBodies().size()==1,"linear affected scope accessor");
  Roundtrip(model,out,"selected-both");
  XmlReject(out/"selected-both-tiny.xml",out,"geometry-mode-missing",[](auto*f){
    while(f->NextSiblingElement("Feature")) f=f->NextSiblingElement("Feature");
    f->DeleteAttribute("AffectedBodyMode");
  });
  XmlReject(out/"selected-both-tiny.xml",out,"geometry-mode-unknown",[](auto*f){
    while(f->NextSiblingElement("Feature")) f=f->NextSiblingElement("Feature");
    f->SetAttribute("AffectedBodyMode","unknown");
  });
  const auto options=*selected;
  for(int i=0;i<7;++i) {
    *selected=options;
    if(i==0)selected->affectedBodyMode.reset();
    if(i==1)selected->affectedBodyMode=static_cast<PatternAffectedBodyMode>(777);
    if(i==2)selected->affectedBodies.clear();
    if(i==3)selected->affectedBodyMode=PatternAffectedBodyMode::AllBodies;
    if(i==4)selected->affectedBodyMode=PatternAffectedBodyMode::AutoSelect;
    if(i==5)selected->affectedBodies.push_back(body);
    if(i==6)selected->geometryPattern=false;
    Require(!model.Validate().isValid,"reject affected options "+std::to_string(i));
  }
  *selected=options;
  selected->affectedBodies.clear();
  for(auto mode:{PatternAffectedBodyMode::AllBodies,PatternAffectedBodyMode::AutoSelect}) {
    selected->affectedBodyMode=mode;Roundtrip(model,out,mode==PatternAffectedBodyMode::AllBodies?"all":"auto");
  }
  *selected=options;
  const auto goodID=featureSeed->targetFeatureID;
  featureSeed->targetFeatureID="missing";Require(!model.Validate().isValid,"feature seed must exist earlier");featureSeed->targetFeatureID=selected->featureID;
  Require(!model.Validate().isValid,"feature seed cannot refer to self");featureSeed->targetFeatureID=goodID;
  selected->seedObjects.push_back(std::make_shared<CRefFeature>(*featureSeed));
  Require(!model.Validate().isValid,"distinct reference objects cannot repeat same seed ID");selected->seedObjects.pop_back();
  std::string error;
  Require(ConvertModelUnit(model,UnitType::METER,&error),error);
  Near(body->volume,6e-6,"shared Body volume scales once");Near(body->surfaceArea,0.0022,"shared Body area scales once");
  Near(affectedOnly->volume,6e-6,"affected-only Body volume scales");
  Near(affectedOnly->boundsMax.y,0.02,"affected-only Body length scales");
  Near(body->boundsMax.z,0.03,"shared Body bounds scale once");Near(body->surfacePoints[3].z,0.03,"surface witness units");
  Near(linear->dir1.spacing,0.012,"linear pitch units");Near(linear->dir2->spacing,0.02,"second direction units");
  Near(circular->dir1.origin.y,0.002,"axis origin units");Near(circular->dir1.angle,2*GeoUtils::PI,"angle stays radians");
  Require(ConvertModelUnit(model,UnitType::MILLIMETER,&error),error);
  Near(body->volume,6000,"unit volume roundtrip");Require(model.Validate().isValid,"unit roundtrip remains valid");
  Near(affectedOnly->volume,6000,"affected-only unit volume roundtrip");
}
void TestPitchAndDomain() {
  double pitch=0;std::string error;
  CLinearPatternDir linear;linear.count=5;linear.spacing=12;linear.spacingType=PatternSpacingType::SPAN_AND_COUNT;
  Require(ResolveLinearPatternPitch(linear,pitch,error),error);Near(pitch,3,"linear total span divided by n-1");
  CCircularPatternDir circular;circular.count=4;circular.angle=2*GeoUtils::PI;circular.spacingType=PatternSpacingType::SPAN_AND_COUNT;
  Require(ResolveCircularPatternPitch(circular,pitch,error),error);Near(pitch,GeoUtils::PI/2,"closed circle excludes duplicate end seed");
  circular.angle=GeoUtils::PI;Require(ResolveCircularPatternPitch(circular,pitch,error),error);Near(pitch,GeoUtils::PI/3,"partial circle includes both span ends");
  Require(PatternInstanceCount(4,3,false)==12&&PatternInstanceCount(4,3,true)==6,"domain size includes seed");
  Require(IsPatternIndexInDomain({3,2},4,3,false)&&!IsPatternIndexInDomain({3,2},4,3,true),"seed-only grid excludes interior");
  Require(!IsPatternIndexInDomain({-1,0},4,3,false)&&PatternInstanceCount(0,3,false)==0,"invalid counts/indices rejected");
}
}
int main(int argc,char**argv) {
  try {
    const std::filesystem::path out=argc>1?argv[1]:"../tmp/pattern-model-tests";
    std::filesystem::create_directories(out);
    TestExistingContract();
    TestPlaneFrame(out);
    TestStrictReferenceWhitespace(out);
    TestSerialization(out);
    TestPitchAndDomain();
    std::cout<<"PASS ordinary pattern contract\n";
    return 0;
  } catch(const std::exception&e) {std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}
}
