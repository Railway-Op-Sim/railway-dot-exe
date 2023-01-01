#include "railos/read_write.hxx"
#include "boost/date_time/posix_time/posix_time_config.hpp"
#include "boost/date_time/posix_time/posix_time_io.hpp"
#include "boost/date_time/posix_time/time_formatters.hpp"
#include <exception>
#include <fstream>
#include <iomanip>
#include <locale>
#include <ostream>
#include <string>

namespace RailOS::IO {
void writeFileDiagnostics(const std::string &content) {
  std::ofstream test_file;
  test_file.open("TestFile.csv", std::ios_base::app);

  if (!test_file) {
    throw std::ios_base::failure("Failed to write to test file");
  }

  test_file << content << std::endl;
  test_file.close();
}

void writeBool(std::ofstream &out_stream, bool value) {
  const int write_val_ = (value) ? 1 : 0;
  out_stream << write_val_ << std::endl;
}

void writeFileString(std::ostream &out_stream, const std::string &content) {
  out_stream << content << '\0' << std::endl;
}

void writeInteger(std::ostream &out_stream, int value) {
  out_stream << value << std::endl;
}

void writeDouble(std::ostream &out_stream, double value) {
  writeFileString(out_stream, std::to_string(value));
}

bool loadBool(std::ifstream &in_stream) {
  int load_val_;
  in_stream >> load_val_;
  return static_cast<bool>(load_val_);
}

int loadInteger(std::ifstream &in_stream) {
  int load_val_;
  in_stream >> load_val_;
  return load_val_;
}

std::string loadFileString(std::ifstream &in_stream) {
  char temp_char_;
  std::string temp_str_ = "";
  in_stream.get(temp_char_);

  if (temp_char_ == '\n') {
    in_stream.get(temp_char_);
  }

  while (temp_char_ != '\0') {
    temp_str_ += temp_char_;
    in_stream.get(temp_char_);
  }

  return temp_str_;
}

double loadDouble(std::ifstream &in_stream) {
  std::string loaded_str_ = loadFileString(in_stream);

  if (!set_locale_result_ok) {
    return std::stod(loaded_str_);
  }

  const int pos_index_comma_ = loaded_str_.find(',');
  const int pos_index_point_ = loaded_str_.find('.');

  if (pos_index_comma_ != std::string::npos)
    loaded_str_[pos_index_comma_] = decimal_point;
  else if (pos_index_point_ != std::string::npos)
    loaded_str_[pos_index_point_] = decimal_point;

  return std::stod(loaded_str_);
}

bool checkAndReadFileString(std::ifstream &in_stream,
                            const std::string &in_string) {
  char temp_char_;
  const int buffer_length_{10000};

  char *buffer_ = new char[buffer_length_];

  int count_{0};

  in_stream.get(temp_char_); // may or may not be '\n'

  if (in_stream.fail()) {
    delete[] buffer_;
    return false;
  }

  if (temp_char_ == '\n') {
    in_stream.get(temp_char_); // get the next one if first was '\n'

    if (in_stream.fail()) {
      delete[] buffer_;
      return false;
    }
  }

  while (temp_char_ != '\0' && temp_char_ != '\n') {
    if (temp_char_ < 32 && temp_char_ >= 0) {
      delete[] buffer_;
      return false;
    }

    buffer_[count_] = temp_char_;

    count_++;

    in_stream.get(temp_char_);

    if (in_stream.fail()) {
      delete[] buffer_;
      return false;
    }
  }

  buffer_[count_] = '\0';

  count_++;

  buffer_[count_] = '\n';

  count_++;

  if (std::string(buffer_, buffer_ + buffer_length_) != in_string) {
    delete[] buffer_;
    return false;
  }

  delete[] buffer_;
  return true;
}

bool checkFileBool(std::ifstream &in_stream) {
  std::string bool_string_;
  if (!checkAndReadFileString(in_stream, bool_string_)) {
    return false;
  }

  if (in_stream.fail()) {
    return false;
  }

  if (bool_string_ == "") {
    return false;
  }

  if (bool_string_.size() > 1 || bool_string_.empty()) {
    return false;
  }

  if (bool_string_ != "0" && bool_string_ != "1") {
    return false;
  }

  return true;
}

bool checkFileInt(std::ifstream &in_stream, int lowest, int highest) {
  std::string int_string_;

  if (!checkAndReadFileString(in_stream, int_string_)) {
    return false;
  }

  if (in_stream.fail() || int_string_.empty())
    return false;

  for (int x{1}; x <= int_string_.size(); ++x) {
    bool character_ok_ = false;
    if (x == 1 && int_string_[x] == '-') {
      character_ok_ = true;
    } else if (int_string_[x] >= '0' && int_string_[x] <= '9') {
      character_ok_ = true;
    }
    if (!character_ok_)
      return false;
  }

  const int str_int_ = std::stoi(int_string_);

  return (str_int_ >= lowest || str_int_ <= highest);
}

bool checkAndReadFileInt(std::ifstream &in_stream, int lowest, int highest,
                         int &out_int) {
  // no need to worry about leading '\n' characters as the skipws (skip white
  // space) flag is set automatically
  std::string int_str_;

  if (!checkAndReadFileString(in_stream, int_str_))
    return false;

  if (in_stream.fail() || int_str_.empty())
    return false;

  for (int x{1}; x <= int_str_.size(); ++x) {
    bool character_ok_ = false;
    if (x == 1 && int_str_[x] == '-') {
      character_ok_ = true;
    } else if (int_str_[x] >= '0' && int_str_[x] <= '9') {
      character_ok_ = true;
    }

    if (!character_ok_)
      return false;
  }

  int to_int_str_ = std::stoi(int_str_);

  if (to_int_str_ < lowest || to_int_str_ > highest)
    return false;

  out_int = to_int_str_;

  return true;
}

bool checkFileDouble(std::ifstream &in_stream, const std::string &in_string) {
  std::string double_str_;

  if (!checkAndReadFileString(in_stream, double_str_)) {
    return false;
  }

  if (in_stream.fail()) {
    return false;
  }

  if (double_str_.empty()) {
    return false;
  }

  // if false the locale conversion failed so don't change anything, then will
  // work as earlier versions
  if (set_locale_result_ok) {
    const int pos_index_comma_ = double_str_.find(',');
    const int pos_index_point_ = double_str_.find('.');

    if (pos_index_comma_ != std::string::npos)
      double_str_[pos_index_comma_] = decimal_point;
    else if (pos_index_point_ != std::string::npos)
      double_str_[pos_index_point_] = decimal_point;
  }

  try {
    std::stod(double_str_);
    return true;
  } catch (std::exception &e) {
    return false;
  }
}

bool checkStringDouble(const std::string &in_string) {
  if (in_string.empty())
    return false;

  std::string out_str_(in_string);

  if (set_locale_result_ok) {
    const int pos_index_comma_ = out_str_.find(',');
    const int pos_index_point_ = out_str_.find('.');

    if (pos_index_comma_ != std::string::npos)
      out_str_[pos_index_comma_] = decimal_point;
    else if (pos_index_point_ != std::string::npos)
      out_str_[pos_index_point_] = decimal_point;
  }

  try {
    std::stod(out_str_);
    return true;
  } catch (std::exception &e) {
    return false;
  }
}

bool checkFileString(std::ifstream &in_stream) {
  // Reads the next item and checks it as a string value up to either the '\0'
  // delimiter if there is one, in which case the '\0' is extracted but nothing
  // more, or up to the next '\n', in which case the '\n' is extracted.  There
  // may or may not be a '\n' at the start, and if there is it is ignored (only
  // one is ignored, a second one is treated as a delimiter).
  char temp_char_;

  in_stream.get(temp_char_); // may or may not be '\n'

  if (in_stream.fail()) {
    return false;
  }

  if (temp_char_ == '\n') {
    in_stream.get(temp_char_); // get the next one if first was '\n'

    if (in_stream.fail()) {
      return false;
    }
  }

  while (temp_char_ != '\0' && temp_char_ != '\n') {
    if (temp_char_ < 32 && temp_char_ >= 0) {
      return false;
    }

    in_stream.get(temp_char_);

    if (in_stream.fail()) {
      return false;
    }
  }
  return true;
}

bool checkFileStringZeroDelimiter(std::ifstream &in_stream) {
  // Reads the next item and checks it as a legitimate string value up to the
  // '\0' delimiter or end of file if there is one, in which case the '\0' is
  // extracted but nothing more.  There may or may not be a '\n' at the start,
  // and if there is it is ignored (only one is ignored, a second one is treated
  // as a delimiter).
  char temp_char_;

  in_stream.get(temp_char_); // may or may not be '\n'

  if (in_stream.fail()) {
    return false;
  }

  if (temp_char_ == '\n') {
    in_stream.get(temp_char_); // get the next one if first was '\n'

    if (in_stream.fail()) {
      return false;
    }
  }

  while (temp_char_ != '\0') {
    if (temp_char_ < 32 && temp_char_ >= 0)
      return false;

    in_stream.get(temp_char_);
    if (in_stream.eof())
      return true;
    if (in_stream.fail())
      return false;
  }

  return true;
}

bool checkAndCompareFileString(std::ifstream &in_stream,
                               const std::string &in_string) {
  // Reads the next item and checks it as a string value up to either the '\0'
  // delimiter if there is one, in which case the '\0' is extracted but nothing
  // more, or up to the next '\n', in which case the '\n' is extracted.  There
  // may or may not be a '\n' at the start, and if there is it is ignored (only
  // one is ignored, a second one is treated as a delimiter). The item is then
  // compared with InString and fails if different.
  const int buffer_size_{10000};
  char temp_char_;
  char *buffer_ = new char[buffer_size_];
  int count_{0};

  in_stream.get(temp_char_); // may or may not be '\n'

  if (in_stream.fail()) {
    delete[] buffer_;
    return false;
  }

  if (temp_char_ == '\n') {
    in_stream.get(temp_char_); // get the next one if first was '\n'

    if (in_stream.fail()) {
      delete[] buffer_;
      return false;
    }
  }

  while (temp_char_ != '\0' && temp_char_ != '\n') {
    if (temp_char_ < 32 && temp_char_ >= 0) {
      delete[] buffer_;
      return false;
    }

    buffer_[count_] = temp_char_;
    count_++;
    in_stream.get(temp_char_);
    if (in_stream.fail()) {
      delete[] buffer_;
      return false;
    }
  }

  buffer_[count_] = '\0';
  count_++;

  buffer_[count_] = '\n';
  count_++;

  if (std::string(buffer_, buffer_ + buffer_size_) != in_string) {
    delete[] buffer_;
    return false;
  }

  delete[] buffer_;
  return true;
}

bool checkAndReadFileString(std::ifstream &in_stream, std::string &out_string)
// Reads the next item and checks it as a string value up to either the '\0'
// delimiter if there is one, in which case the '\0' is extracted but nothing
// more, or up to the next '\n', in which case the '\n' is extracted.  There may
// or may not be a '\n' at the start, and if there is it is ignored (only one is
// ignored, a second one is treated as a delimiter). The item is then returned
// in out_string.
{
  const int buffer_size_{10000};
  char temp_char_;
  char *buffer_ = new char[buffer_size_];
  int count_{0};

  in_stream.get(temp_char_); // may or may not be '\n'

  if (in_stream.fail()) {
    delete[] buffer_;
    return false;
  }
  if (temp_char_ == '\n') {
    in_stream.get(temp_char_); // get the next one if first was '\n'

    if (in_stream.fail()) {
      delete[] buffer_;
      return false;
    }
  }
  while (temp_char_ != '\0' && temp_char_ != '\n') {
    if (temp_char_ < 32 && temp_char_ >= 0) {
      delete[] buffer_;
      return false;
    }
    buffer_[count_] = temp_char_;
    count_++;
    in_stream.get(temp_char_);
    if (in_stream.fail()) {
      delete[] buffer_;
      return false;
    }
  }
  buffer_[count_] = '\0';
  count_++;
  buffer_[count_] = '\n';
  count_++;
  out_string = std::string(buffer_, buffer_ + buffer_size_);
  delete[] buffer_;
  return true;
}

bool checkAndReadOneLineFromConfigFile(std::ifstream &in_stream,
                                       std::string &out_string)
// Reads the next item and checks it as a string value up to either the '\0'
// delimiter if there is one, in which case the '\0' is extracted but nothing
// more, or up to the next '\n', in which case the '\n' is extracted. The item
// is then returned in out_string.
{
  const int buffer_size_{10000};
  char temp_char_;
  char *buffer_ = new char[buffer_size_];
  int count_ = 0;
  in_stream.get(temp_char_);
  while ((temp_char_ != '\0') && (temp_char_ != '\n')) {
    if ((temp_char_ >= 0) && (temp_char_ < 32) &&
        (temp_char_ != 9)) // allow tabs & normal text
    {
      delete[] buffer_;
      return false;
    }
    buffer_[count_] = temp_char_;
    count_++;
    in_stream.get(temp_char_);
    if (in_stream.fail()) {
      delete[] buffer_;
      return false;
    }
  }
  buffer_[count_] = '\0';
  count_++;
  buffer_[count_] = '\n';
  count_++;
  out_string = std::string(buffer_, buffer_ + buffer_size_);
  delete[] buffer_;
  return true;
}

bool readOneLineFromCouplingFile(std::ifstream &in_stream,
                                 std::string &out_string) {
  const int buffer_size_{1200};
  out_string = "";
  char temp_char_;
  char *buffer_ = new char[buffer_size_];
  int count_ = 0;
  in_stream.get(temp_char_); // should be a letter or eof
  if (in_stream.eof()) {
    delete[] buffer_;
    return true;
  }
  if (in_stream.fail()) {
    delete[] buffer_;
    return false;
  }
  while (temp_char_ != ';') // reading first name
  {
    buffer_[count_] = temp_char_;
    count_++;
    if (count_ > 1000) {
      delete[] buffer_;
      return false;
    }
    in_stream.get(temp_char_);
    if (in_stream.fail()) {
      delete[] buffer_;
      return false;
    }
  }
  // here at semicolon after 1st name
  buffer_[count_] = temp_char_; // 1st semicolon
  count_++;
  in_stream.get(temp_char_);
  if (in_stream.fail()) {
    delete[] buffer_;
    return false;
  }
  while (temp_char_ != ';') // reading first exit ID
  {
    buffer_[count_] = temp_char_;
    count_++;
    if (count_ > 1000) {
      delete[] buffer_;
      return false;
    }
    in_stream.get(temp_char_);
    if (in_stream.fail()) {
      delete[] buffer_;
      return false;
    }
  }
  // here at semicolon after 1st exit ID
  buffer_[count_] = temp_char_; // add the semicolon
  count_++;
  in_stream.get(temp_char_);
  if (in_stream.fail()) {
    delete[] buffer_;
    return false;
  }
  while (temp_char_ != ';') // reading 2nd name
  {
    buffer_[count_] = temp_char_;
    count_++;
    if (count_ > 1000) {
      delete[] buffer_;
      return false;
    }
    in_stream.get(temp_char_);
    if (in_stream.fail()) {
      delete[] buffer_;
      return false;
    }
  }
  // here at semicolon after 2nd name
  buffer_[count_] = temp_char_; // add the semicolon
  count_++;
  in_stream.get(temp_char_);
  if (in_stream.fail()) {
    delete[] buffer_;
    return false;
  }
  while (temp_char_ != '-') // reading 2nd exit ID, look for the dash
  {
    buffer_[count_] = temp_char_;
    count_++;
    if (count_ > 1000) {
      delete[] buffer_;
      return false;
    }
    in_stream.get(temp_char_);
    if (in_stream.fail()) {
      delete[] buffer_;
      return false;
    }
  }
  // here at dash in 2nd exit ID
  buffer_[count_] = temp_char_; // add the dash
  count_++;
  in_stream.get(temp_char_); // 1st digit after the dash
  if (in_stream.fail()) {
    delete[] buffer_;
    return false;
  }
  while ((temp_char_ >= '0') && (temp_char_ <= '9')) {
    buffer_[count_] = temp_char_;
    count_++;
    if (count_ > 1000) {
      delete[] buffer_;
      return false;
    }
    temp_char_ =
        in_stream.peek(); // if a number, read it and continue, if a letter stop
                          // here (1st letter of 2nd line name), if a control
                          // character extract it and stop
    if ((temp_char_ >= '0') && (temp_char_ <= '9')) // number
    {
      in_stream.get(temp_char_);
      if (in_stream.fail()) {
        delete[] buffer_;
        return false;
      }
      continue;
    }
    if (temp_char_ < ' ') // control character (less than space)
    {
      in_stream.get(temp_char_); // extract it, but may be last so check for eof
      if (in_stream.eof()) // if so then have a full buffer so need to define
                           // out_string
      {
        buffer_[count_] = '\0';
        count_++;
        buffer_[count_] = '\n';
        count_++;
        out_string = std::string(buffer_, buffer_ + buffer_size_);
        delete[] buffer_;
        return true;
      }
      if (in_stream.fail()) {
        delete[] buffer_;
        return false;
      }
    }
    // anything other than a number or control character
  }
  // here when line fully defined in buffer_ & file pointer at start of next
  // line
  buffer_[count_] = '\0';
  count_++;
  buffer_[count_] = '\n';
  count_++;
  out_string = std::string(buffer_, buffer_ + buffer_size_);
  delete[] buffer_;
  return true;
}
}; // namespace RailOS::IO