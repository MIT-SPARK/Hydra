#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace hydra {

class CsvReader {
 public:
  // Construction.
  CsvReader() = default;
  explicit CsvReader(const std::string& file_name,
                     char separator = ',',
                     bool skip_first_line = true);
  virtual ~CsvReader() = default;

  // Setup.
  /**
   * @brief Setup the CsvReader by reading a csv file.
   * @param file_name The name of the csv file to read.
   * @param separator The separator used in the csv file.
   * @return True if the file was read successfully, false otherwise.
   */
  bool setup(const std::string& file_name,
             char separator = ',',
             bool skip_first_line = true);
  bool isSetup() const { return is_setup_; }
  operator bool() const { return isSetup(); }

  struct Row : std::vector<std::string> {
   public:
    // Construction.
    Row() = default;
    Row(const std::vector<std::string>& row,
        std::shared_ptr<std::unordered_map<std::string, size_t>> _header_to_index)
        : std::vector<std::string>(row), header_to_index(std::move(_header_to_index)) {}
    // Queries.
    const std::string& getEntry(const std::string& header) const;

   private:
    std::shared_ptr<std::unordered_map<std::string, size_t>> header_to_index;
  };

  // Queries.
  size_t numRows() const;
  bool hasHeader(const std::string& header) const;
  bool hasHeaders(const std::vector<std::string>& headers) const;
  const std::vector<std::string>& getHeaders() const;
  const std::vector<Row>& getRows() const;
  const Row& getRow(size_t row) const;
  const std::string& getEntry(const std::string& header, size_t row) const;

  // Interface to verify the read file with required and optional headers.
  /**
   * @brief Require the csv file to have the given headers. An error about missing headers will be
   * printed.
   * @param headers List of headers that are required.
   * @return True if all required headers are present, false otherwise.
   */
  bool checkRequiredHeaders(const std::vector<std::string>& headers) const;

/**
 * @brief Check if the csv file has the given headers. A warning about missing headers will be
 * printed.
 * @param headers List of headers that are optional.
 */
  void checkOptionalHeaders(const std::vector<std::string>& headers) const;

 private:
  // State.
  bool is_setup_ = false;
  std::string loaded_file_name_;

  // Data.
  std::vector<std::string> headers_;
  std::vector<Row> rows_;
  std::shared_ptr<std::unordered_map<std::string, size_t>> header_to_index_;

  // Helper functions.
  std::vector<std::string> missingHeaders(const std::vector<std::string>& headers) const;
};

}  // namespace hydra
