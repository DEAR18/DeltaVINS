#pragma once
#include <fstream>
#include <unordered_map>

namespace DeltaVins {
struct FileOutput {
    static std::ofstream& getFout(std::string fileName);

   private:
    static std::unordered_map<std::string, std::ofstream> m_fileMap;
};
}  // namespace DeltaVins
