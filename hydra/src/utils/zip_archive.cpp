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
#include <minizip/unzip.h>
#include <minizip/zip.h>
#include <unistd.h>

#include <algorithm>
#include <cstdlib>
#include <limits>
#include <stdexcept>

namespace hydra::io {
namespace {

// Limit each Minizip read/write call to 1 MiB.
constexpr size_t kChunkSize = 1 << 20;

unsigned int getChunkSize(size_t remaining) {
  return static_cast<unsigned int>(std::min(remaining, kChunkSize));
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
    file_ = zipOpen64(temporary_.c_str(), APPEND_STATUS_CREATE);
    if (!file_) {
      std::filesystem::remove(temporary_);
      throw std::runtime_error("could not open temporary archive");
    }
  }

  ~ZipWriter() {
    if (file_) {
      zipClose(file_, nullptr);
    }

    std::error_code error;
    std::filesystem::remove(temporary_, error);
  }

  ZipWriter(const ZipWriter&) = delete;
  ZipWriter& operator=(const ZipWriter&) = delete;

  void write(const std::string& name, const Bytes& bytes) {
    openEntry(name, bytes.size());

    size_t offset = 0;
    while (offset < bytes.size()) {
      const auto size = getChunkSize(bytes.size() - offset);
      if (zipWriteInFileInZip(file_, bytes.data() + offset, size) != ZIP_OK) {
        zipCloseFileInZip(file_);
        throw std::runtime_error("could not write entry " + name);
      }

      offset += size;
    }

    if (zipCloseFileInZip(file_) != ZIP_OK) {
      throw std::runtime_error("could not finish entry " + name);
    }
  }

  void commit() {
    const auto status = zipClose(file_, nullptr);
    file_ = nullptr;
    if (status != ZIP_OK) {
      throw std::runtime_error("could not finish archive");
    }

    std::filesystem::rename(temporary_, destination_);
  }

 private:
  void openEntry(const std::string& name, size_t size) {
    // Some OpenCV TIFF encoders leave int32 images uncompressed.
    const auto extension = std::filesystem::path(name).extension();
    const auto compress =
        options_.compression_level != 0 && extension != ".png" && extension != ".exr";
    const auto method = compress ? Z_DEFLATED : 0;
    const auto level = compress ? options_.compression_level : 0;
    const auto zip64 = size >= std::numeric_limits<uint32_t>::max();
    const auto status = zipOpenNewFileInZip64(file_,
                                              name.c_str(),
                                              nullptr,
                                              nullptr,
                                              0,
                                              nullptr,
                                              0,
                                              nullptr,
                                              method,
                                              level,
                                              zip64);
    if (status != ZIP_OK) {
      throw std::runtime_error("could not create entry " + name);
    }
  }

  std::filesystem::path destination_;
  std::filesystem::path temporary_;
  zipFile file_ = nullptr;
  const ArchiveOptions options_;
};

class ZipReader {
 public:
  explicit ZipReader(const std::filesystem::path& path)
      : file_(unzOpen64(path.c_str())) {
    if (!file_) {
      throw std::runtime_error("could not open ZIP archive");
    }
  }

  ~ZipReader() { unzClose(file_); }
  ZipReader(const ZipReader&) = delete;
  ZipReader& operator=(const ZipReader&) = delete;

  Bytes read(const std::string& name) {
    Bytes bytes(getEntrySize(name));
    if (unzOpenCurrentFile(file_) != UNZ_OK) {
      throw std::runtime_error("could not open entry " + name);
    }

    size_t offset = 0;
    while (offset < bytes.size()) {
      const auto size = getChunkSize(bytes.size() - offset);
      const auto count = unzReadCurrentFile(file_, bytes.data() + offset, size);
      if (count <= 0) {
        unzCloseCurrentFile(file_);
        throw std::runtime_error("truncated or corrupt entry " + name);
      }

      offset += count;
    }

    uint8_t extra;
    const auto trailing = unzReadCurrentFile(file_, &extra, 1);
    const auto status = unzCloseCurrentFile(file_);
    if (trailing != 0 || status != UNZ_OK) {
      throw std::runtime_error("invalid size or checksum for entry " + name);
    }

    return bytes;
  }

 private:
  size_t getEntrySize(const std::string& name) {
    if (unzLocateFile(file_, name.c_str(), 1) != UNZ_OK) {
      throw std::runtime_error("missing entry " + name);
    }

    unz_file_info64 info{};
    const auto status =
        unzGetCurrentFileInfo64(file_, &info, nullptr, 0, nullptr, 0, nullptr, 0);
    if (status != UNZ_OK || info.uncompressed_size > std::numeric_limits<int>::max()) {
      throw std::runtime_error("invalid or oversized entry " + name);
    }

    return info.uncompressed_size;
  }

  unzFile file_;
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
