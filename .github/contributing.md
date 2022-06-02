## Code Contribution Guidelines

We welcome contributions in the form of pull-requests! If you're unsure on the design of a new feature, feel free to open an issue to discuss with a maintainer.

In general, contributing code should be a collaborative, respectful process for both the contributor and the maintainer.
We do try to follow a semi-consistent code style to encourage readability, which is detailed below.
A maintainer of the repo may reformat your code to better correspond to the style guide by running the mentioned linters below.

### Style Guide

In general, the use of auto-formatters is encouraged:

- Use `clang-format` to format C++ code

- Use [black](https://github.com/psf/black) to format python code

- Use [cmake-format](https://github.com/cheshirekow/cmake_format) to format cmake code

Also, avoid trailing whitespace (i.e. spaces at the end of lines).

#### Python

For python code, we follow these general principles:

- Follow pep8 style for python (`flake8` can be helpful here)

- We mostly follow the [napoleon](https://sphinxcontrib-napoleon.readthedocs.io/en/latest/) style docstring, and find `flake8-docstring` helpful.

- Use format strings when possible (unless the line-breaking looks better with the `format` method instead).

#### C++

For C++ code, we follow these general principles:

- Variables are `snake_case`

- Functions are mostly `camelCase`, though `snake_case` is also allowed

- Classes are `PascalCase` (like python)

- Private members of classes have an underscore as a suffix, e.g. `some_private_member_`

- Local includes use quotes, e.g. `#include "package/header_file.h"`

- System or non-local includes use triangle braces, e.g. `#include <package/header_file.h>`. We generally consider package code to be "non-local" to unit tests for that code.

- Always use curly-braces with if statements.

### Code Design

We find that several other generic choices make code more readable:

- Prefer to use `break`, `continue`, and `return` instead of `else if` or `elif`. See [here](https://llvm.org/docs/CodingStandards.html#use-early-exits-and-continue-to-simplify-code) for a more detailed discussion.

- Keep functions short, and use helper functions when possible.

- Keep variable names specific and short.

- Pay attention to how the auto-formatter creates line-breaks. `black` does a better job than `clang-format`, but it can still be worth it to tweak a longer line to either make it shorter or break more naturally.
