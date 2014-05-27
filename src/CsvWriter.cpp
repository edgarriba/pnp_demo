#include <boost/lexical_cast.hpp>
#include "CsvWriter.h"

CsvWriter::CsvWriter(const std::string &path, const std::string &separator){
  _file.open(path.c_str(), std::ofstream::out);
  _isFirstTerm = true;
  _separator = separator;
}

CsvWriter::~CsvWriter() {
  _file.flush();
  _file.close();
}

void CsvWriter::addTerm(const std::vector<cv::Point3f> &list_points3d)
{
  std::string x, y, z;
  for(unsigned int i = 0; i < list_points3d.size(); ++i)
  {
    x = boost::lexical_cast< std::string >(list_points3d[i].x);
    y = boost::lexical_cast< std::string >(list_points3d[i].y);
    z = boost::lexical_cast< std::string >(list_points3d[i].z);

    _file << x << _separator << y << _separator << z << std::endl;
  }

}
