#ifndef CSVWRITER_H
#define	CSVWRITER_H

#include <fstream>
#include <iostream>

#include <opencv2/core.hpp>

class CsvWriter {
public:
  CsvWriter(const std::string &path, const std::string &separator = " ");
  ~CsvWriter();
  void addTerm(const std::vector<cv::Point3f> &list_points3d);
private:
  std::ofstream _file;
  std::string _separator;
  bool _isFirstTerm;
};

#endif
