#pragma once

#include <exception>
#include <fstream>
#include <set>
#include <string>
#include <chrono>

namespace RailOS::IO {
bool set_locale_result_ok = false;
char decimal_point = '.';
void writeFileDiagnostics(const std::string &content);
void writeBool(std::ostream &out_stream, bool value);
void writeFileString(std::ostream &out_stream, const std::string &content);
void writeInteger(std::ostream &out_stream, int value);
void writeDouble(std::ostream &out_stream, double value);
bool loadBool(std::ifstream &in_stream);
int loadInteger(std::ifstream &in_stream);
double loadDouble(std::ifstream &in_stream);
std::string loadFileString(std::ifstream &in_stream);
bool checkFileBool(std::ifstream &in_stream);
bool checkAndReadFileString(std::ifstream &in_stream,
                            const std::string &in_string);
bool checkFileDouble(std::ifstream &in_stream, const std::string &in_string);
bool checkFileInt(std::ifstream &in_stream, int lowest, int highest);
bool checkStringDouble(const std::string &in_string);
bool checkFileString(std::ifstream &in_stream);
bool checkFileStringZeroDelimiter(std::ifstream &in_stream);
bool checkAndCompareFileString(std::ifstream &in_stream,
                               const std::string &in_string);
bool checkAndReadFileString(std::ifstream &in_stream, std::string &out_string);
bool checkAndReadOneLineFromConfigFile(std::ifstream &in_stream,
                                       std::string &out_string);
bool readOneLineFromCouplingFile(std::ifstream &in_stream,
                                 std::string &out_string);
} // namespace RailOS::IO
