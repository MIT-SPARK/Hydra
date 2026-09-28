#pragma once

#include <memory>

namespace hydra_multi {

/** Operations
 * UPDATE: updating the attributes (e.g. position) of existing entities.
 * REBASE: To rebase target on source means that overlapping parts of target and source
 *are updated to source, and then the non overlapping parts
 *of target are appended.
 * MERGE: To merge source to target means that the new parts of
 *source are appended to target. The rest of target are kept the same
 **/
enum class OperationType { UPDATE, REBASE, MERGE };

template <typename T, typename incT>
class InterfaceOperator {
 public:
  using Data = T;
  using IncData = incT;

  InterfaceOperator(const std::shared_ptr<Data>& data) : data_(data) {}
  virtual ~InterfaceOperator() = default;

  bool operator()(const Data& source, const OperationType& op) {
    switch (op) {
      case OperationType::UPDATE:
        return update(source);
        break;
      case OperationType::REBASE:
        return rebase(source);
        break;
      case OperationType::MERGE:
        return merge(source);
        break;
      default:
        throw std::runtime_error("Unknown operation in InterfaceOperator");
        break;
    }
    return false;
  }

  virtual bool incrementalAppend(const IncData& incremental_source) = 0;

 protected:
  virtual bool update(const Data& source) = 0;

  virtual bool rebase(const Data& source) = 0;

  virtual bool merge(const Data& source) = 0;

 protected:
  std::shared_ptr<Data> data_;
};

}  // namespace hydra_multi
