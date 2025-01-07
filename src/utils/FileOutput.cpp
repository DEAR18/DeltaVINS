#include "precompile.h"
#include "utils/FileOutput.h"

 std::unordered_map<std::string, std::ofstream> DeltaVins::FileOutput::m_fileMap;


std::ofstream& DeltaVins::FileOutput::getFout(std::string fileName)
{
	if (m_fileMap.count(fileName))
		return m_fileMap[fileName];
	else
	{
		auto& file = m_fileMap[fileName];
		file.open(fileName+".txt");
		return file;
	}
}
