#include "hydra/utils/csv_reader.h"

#include <glog/logging.h>

#include <filesystem>
#include <fstream>

namespace hydra {

CsvReader::CsvReader(const std::string& file_name,
                     char separator,
                     bool skip_first_line) {
  setup(file_name, separator, skip_first_line);
}

bool CsvReader::setup(const std::string& file_name,
                      char separator,
                      bool skip_first_line) {
  // Check file can be read.
  if (!std::filesystem::exists(file_name)) {
    LOG(WARNING) << "CSV file '" << file_name << "' does not exist.";
    return false;
  }
  std::ifstream file(file_name);
  if (!file.is_open()) {
    LOG(WARNING) << "Could not open CSV file '" << file_name << "'.";
    return false;
  }

  // Clear previous data.
  headers_.clear();
  rows_.clear();
  header_to_index_ = std::make_shared<std::unordered_map<std::string, size_t>>();

  // Read all the data.
  std::vector<std::string> row;
  std::string line, word;
  bool is_header = skip_first_line;
  while (getline(file, line)) {
    row.clear();
    std::stringstream str(line);
    while (getline(str, word, separator)) {
      row.push_back(word);
    }
    char final_char = row.back().back();
    if (final_char == '\n' || final_char == '\r') {
      row.back().erase(row.back().length() - 1);  // remove the newline.
    }

    if (is_header) {
      headers_ = row;
      for (size_t i = 0; i < headers_.size(); i++) {
        header_to_index_->emplace(headers_[i], i);
      }
      is_header = false;
    } else {
      row.resize(headers_.size());
      rows_.emplace_back(row, header_to_index_);
    }
  }
  loaded_file_name_ = file_name;
  is_setup_ = true;
  return true;
}

size_t CsvReader::numRows() const { return rows_.size(); }

bool CsvReader::hasHeader(const std::string& header) const {
  if (!isSetup()) {
    return false;
  }
  return header_to_index_->find(header) != header_to_index_->end();
}

bool CsvReader::hasHeaders(const std::vector<std::string>& headers) const {
  if (!isSetup()) {
    return false;
  }
  for (const auto& header : headers) {
    if (!hasHeader(header)) {
      return false;
    }
  }
  return true;
}

const std::vector<std::string>& CsvReader::getHeaders() const { return headers_; }

const std::vector<CsvReader::Row>& CsvReader::getRows() const { return rows_; }

const CsvReader::Row& CsvReader::getRow(size_t row) const {
  if (row >= rows_.size()) {
    LOG(WARNING) << "Row index '" << row << +"' out of bounds.";
    return rows_[0];
  }
  return rows_[row];
}

const std::string& CsvReader::getEntry(const std::string& header, size_t row) const {
  if (row >= rows_.size()) {
    LOG(WARNING) << "Row index '" << row << +"' out of bounds.";
    return rows_[0][0];
  }
  auto it = header_to_index_->find(header);
  if (it == header_to_index_->end()) {
    LOG(WARNING) << "Header '" << header << "' not found.";
    return rows_[0][0];
  }
  return rows_[row][it->second];
}

bool CsvReader::checkRequiredHeaders(const std::vector<std::string>& headers) const {
  const std::vector<std::string> missing = missingHeaders(headers);
  if (missing.empty()) {
    return true;
  }

  std::stringstream msg;
  msg << "CSV file '" << loaded_file_name_ << "' is missing required headers: ";
  for (size_t i = 0; i < missing.size(); i++) {
    msg << "'" << missing[i] << "'";
    if (i < missing.size() - 1) {
      msg << ", ";
    }
  }
  LOG(ERROR) << msg.str();
  return false;
}

void CsvReader::checkOptionalHeaders(const std::vector<std::string>& headers) const {
  const std::vector<std::string> missing = missingHeaders(headers);
  if (missing.empty()) {
    return;
  }

  std::stringstream msg;
  msg << "CSV file '" << loaded_file_name_ << "' is missing optional headers: ";
  for (size_t i = 0; i < missing.size(); i++) {
    msg << "'" << missing[i] << "'";
    if (i < missing.size() - 1) {
      msg << ", ";
    }
  }
  LOG(WARNING) << msg.str();
}

std::vector<std::string> CsvReader::missingHeaders(
    const std::vector<std::string>& headers) const {
  std::vector<std::string> missing;
  for (const auto& header : headers) {
    if (!hasHeader(header)) {
      missing.push_back(header);
    }
  }
  return missing;
}

const std::string& CsvReader::Row::getEntry(const std::string& header) const {
  const auto it = header_to_index->find(header);
  if (it == header_to_index->end()) {
    LOG(WARNING) << "Header '" << header << "' not found.";
    return this->at(0);
  }
  return this->at(it->second);
}

}  // namespace hydra
