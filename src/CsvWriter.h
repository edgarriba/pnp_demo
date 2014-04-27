#ifndef CSVWRITER_H
#define	CSVWRITER_H

#include <fstream>
#include <string>

using namespace std;

class CsvWriter {
public:
    CsvWriter(const string &path, const string &separator = " ");
	~CsvWriter();
    void addTerm(const string &s);
private:
    ofstream _file;
	string _separator;
    bool _isFirstTerm;
};

#endif
