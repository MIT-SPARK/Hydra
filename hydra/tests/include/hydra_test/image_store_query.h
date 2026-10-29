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
#pragma once

#include <sqlite3.h>

#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>

namespace hydra::test {

using StoreRows = std::vector<std::vector<std::string>>;

//! Read-only query of an image store database (values as text, "NULL" for null)
inline StoreRows queryStore(const std::filesystem::path& store_dir,
                            const std::string& sql) {
  sqlite3* db = nullptr;
  const auto path = (store_dir / "store.sqlite").string();
  if (sqlite3_open_v2(path.c_str(), &db, SQLITE_OPEN_READONLY, nullptr) != SQLITE_OK) {
    sqlite3_close(db);
    throw std::runtime_error("failed to open " + path);
  }

  sqlite3_stmt* stmt = nullptr;
  if (sqlite3_prepare_v2(db, sql.c_str(), -1, &stmt, nullptr) != SQLITE_OK) {
    const std::string msg = sqlite3_errmsg(db);
    sqlite3_close(db);
    throw std::runtime_error("failed to prepare '" + sql + "': " + msg);
  }

  StoreRows rows;
  while (sqlite3_step(stmt) == SQLITE_ROW) {
    auto& row = rows.emplace_back();
    for (int i = 0; i < sqlite3_column_count(stmt); ++i) {
      const auto text = sqlite3_column_text(stmt, i);
      row.push_back(text ? reinterpret_cast<const char*>(text) : "NULL");
    }
  }

  sqlite3_finalize(stmt);
  sqlite3_close(db);
  return rows;
}

//! Single value of a query (empty if there are no rows)
inline std::string queryValue(const std::filesystem::path& store_dir,
                              const std::string& sql) {
  const auto rows = queryStore(store_dir, sql);
  return rows.empty() || rows.front().empty() ? "" : rows.front().front();
}

}  // namespace hydra::test
