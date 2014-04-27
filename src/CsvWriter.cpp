#include "CsvWriter.h"

CsvWriter::CsvWriter(const string &path, const string &separator){
    _file.open(path.c_str(), ofstream::out);
	_isFirstTerm = true;
	_separator = separator;
}

CsvWriter::~CsvWriter() {
    _file.flush();
    _file.close();
}

void CsvWriter::addTerm(const string &s) {
    _file << (_isFirstTerm ? "" : _separator) << s;
	if (!_isFirstTerm) _file << endl;
	_isFirstTerm = !_isFirstTerm;
    
}
