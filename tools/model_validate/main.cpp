#include "CADSerializer.h"
#include <filesystem>
#include <iostream>

int Validate(const std::filesystem::path &path) {
  CADExchange::UnifiedModel model;
  std::string detail;
  if (!CADExchange::LoadModel(model, path, &detail,
                              CADExchange::SerializationFormat::TINYXML)) {
    std::cerr << detail << '\n';
    return 1;
  }
  tinyxml2::XMLDocument document;
  if (document.LoadFile(path.string().c_str()) != tinyxml2::XML_SUCCESS)
    return 1;
  const auto *root = document.RootElement();
  size_t count = 0;
  for (auto *feature = root ? root->FirstChildElement("Feature") : nullptr;
       feature; feature = feature->NextSiblingElement("Feature"))
    ++count;
  if (count != model.GetFeatures().size()) {
    std::cerr << "unsupported or unparsed feature would be omitted\n";
    return 1;
  }
  std::cout << "VALID features=" << count << '\n';
  return 0;
}
#ifdef _WIN32
int wmain(int argc, wchar_t **argv) {
#else
int main(int argc, char **argv) {
#endif
  if (argc != 2) {
    std::cerr << "usage: cadexchange_validate <tinyxml.xml>\n";
    return 2;
  }
  try {
    return Validate(std::filesystem::path(argv[1]));
  } catch (const std::exception &ex) {
    std::cerr << ex.what() << '\n';
    return 1;
  }
}
