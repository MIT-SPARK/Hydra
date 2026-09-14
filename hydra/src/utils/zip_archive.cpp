/* -----------------------------------------------------------------------------
 * Copyright 2022 Massachusetts Institute of Technology.
 * All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Research was sponsored by the United States Air Force Research Laboratory and
 * the United States Air Force Artificial Intelligence Accelerator and was
 * accomplished under Cooperative Agreement Number FA8750-19-2-1000. The views
 * and conclusions contained in this document are those of the authors and should
 * not be interpreted as representing the official policies, either expressed or
 * implied, of the United States Air Force or the U.S. Government. The U.S.
 * Government is authorized to reproduce and distribute reprints for Government
 * purposes notwithstanding any copyright notation herein.
 * -------------------------------------------------------------------------- */
#include "hydra/utils/zip_archive.h"

#include <config_utilities/config.h>
#include <config_utilities/validation.h>
#include <unistd.h>
#include <zip.h>

#include <cstdlib>
#include <fstream>
#include <limits>
#include <list>
#include <memory>
#include <stdexcept>

namespace hydra::io {
namespace {

void writeEmptyArchive(const std::filesystem::path& path) {
  // Older libzip versions cannot retain empty archives. Write the ZIP end of
  // central directory record with zero entries, directory size, and comment.
  constexpr char record[22] = {'P', 'K', 5, 6};
  std::ofstream output;
  output.exceptions(std::ios::failbit | std::ios::badbit);
  output.open(path, std::ios::binary | std::ios::trunc);
  output.write(record, sizeof(record));
  output.close();
}

class ZipWriter {
 public:
  ZipWriter(const std::filesystem::path& destination, const ArchiveOptions& options)
      : destination_(destination), options_(config::checkValid(options)) {
    const auto pattern = destination.string() + ".tmp.XXXXXX";
    std::vector<char> name(pattern.begin(), pattern.end());
    name.push_back('\0');
    const auto fd = mkstemp(name.data());
    if (fd < 0) {
      throw std::runtime_error("could not create temporary archive");
    }

    close(fd);
    temporary_ = name.data();
    file_ = zip_open(temporary_.c_str(), ZIP_CREATE | ZIP_TRUNCATE, nullptr);
    if (!file_) {
      std::filesystem::remove(temporary_);
      throw std::runtime_error("could not open temporary archive");
    }
  }

  ~ZipWriter() {
    if (file_) {
      zip_discard(file_);
    }

    std::error_code error;
    std::filesystem::remove(temporary_, error);
  }

  ZipWriter(const ZipWriter&) = delete;
  ZipWriter& operator=(const ZipWriter&) = delete;

  void write(const std::string& name, const Bytes& bytes) {
    // Libzip reads sources during commit, after the caller's buffers may be gone.
    entries_.push_back(bytes);
    const auto& data = entries_.back();
    const auto source = zip_source_buffer(file_, data.data(), data.size(), 0);
    if (!source) {
      throw std::runtime_error("could not create source for entry " + name);
    }

    const auto index = zip_file_add(file_, name.c_str(), source, ZIP_FL_ENC_UTF_8);
    if (index < 0) {
      zip_source_free(source);
      throw std::runtime_error("could not create entry " + name);
    }

    // Some OpenCV TIFF encoders leave int32 images uncompressed.
    const auto extension = std::filesystem::path(name).extension();
    const auto compress =
        options_.compression_level != 0 && extension != ".png" && extension != ".exr";
    const auto method = compress ? ZIP_CM_DEFLATE : ZIP_CM_STORE;
    const auto level = options_.compression_level > 0 ? options_.compression_level : 0;
    if (zip_set_file_compression(file_, index, method, level) < 0) {
      throw std::runtime_error("could not set compression for entry " + name);
    }
  }

  void commit() {
    if (entries_.empty()) {
      zip_discard(file_);
      file_ = nullptr;
      writeEmptyArchive(temporary_);
    } else if (zip_close(file_) < 0) {
      throw std::runtime_error("could not finish archive: " +
                               std::string(zip_strerror(file_)));
    }

    file_ = nullptr;
    std::filesystem::rename(temporary_, destination_);
  }

 private:
  std::filesystem::path destination_;
  std::filesystem::path temporary_;
  zip_t* file_ = nullptr;
  std::list<Bytes> entries_;
  const ArchiveOptions options_;
};

class ZipReader {
 public:
  explicit ZipReader(const std::filesystem::path& path)
      : file_(zip_open(path.c_str(), ZIP_RDONLY, nullptr)) {
    if (!file_) {
      throw std::runtime_error("could not open ZIP archive");
    }
  }

  ~ZipReader() { zip_discard(file_); }
  ZipReader(const ZipReader&) = delete;
  ZipReader& operator=(const ZipReader&) = delete;

  Bytes read(const std::string& name) {
    Bytes bytes(getEntrySize(name));
    const auto entry = std::unique_ptr<zip_file_t, decltype(&zip_fclose)>(
        zip_fopen(file_, name.c_str(), 0), zip_fclose);
    if (!entry) {
      throw std::runtime_error("could not open entry " + name);
    }

    size_t offset = 0;
    while (offset < bytes.size()) {
      const auto count =
          zip_fread(entry.get(), bytes.data() + offset, bytes.size() - offset);
      if (count <= 0) {
        throw std::runtime_error("truncated or corrupt entry " + name);
      }

      offset += count;
    }

    uint8_t extra;
    const auto trailing = zip_fread(entry.get(), &extra, 1);
    if (trailing != 0) {
      throw std::runtime_error("invalid size or checksum for entry " + name);
    }

    return bytes;
  }

 private:
  size_t getEntrySize(const std::string& name) {
    zip_stat_t info{};
    if (zip_stat(file_, name.c_str(), 0, &info) < 0) {
      throw std::runtime_error("missing entry " + name);
    }

    if (!(info.valid & ZIP_STAT_SIZE) || info.size > std::numeric_limits<int>::max()) {
      throw std::runtime_error("invalid or oversized entry " + name);
    }

    return info.size;
  }

  zip_t* file_;
};

}  // namespace

void declare_config(ArchiveOptions& config) {
  using namespace config;
  name("ArchiveOptions");
  field(config.compression_level, "compression_level");
  check(config.compression_level, GE, -1, "compression_level");
  check(config.compression_level, LE, 9, "compression_level");
}

void writeArchive(const std::filesystem::path& path,
                  const std::function<void(const WriteEntry&)>& write,
                  const ArchiveOptions& options) {
  try {
    ZipWriter archive(path, options);
    write([&archive](const auto& name, const auto& bytes) {
      archive.write(name, bytes);
    });
    archive.commit();
  } catch (const std::exception& e) {
    throw std::runtime_error("Writing archive '" + path.string() + "': " + e.what());
  }
}

void readArchive(const std::filesystem::path& path,
                 const std::function<void(const ReadEntry&)>& read) {
  try {
    ZipReader archive(path);
    read([&archive](const auto& name) { return archive.read(name); });
  } catch (const std::exception& e) {
    throw std::runtime_error("Reading archive '" + path.string() + "': " + e.what());
  }
}

}  // namespace hydra::io
