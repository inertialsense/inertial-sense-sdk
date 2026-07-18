# `ISLogReader` — segment reader example *(stub — D-11 fills out the full example)*

This directory is the placeholder slot reserved by **D-02 / SN-7893**
for the `ISLogReader` walkthrough example. The acceptance criteria
require the slot to exist now so that **D-11 / SN-7900** (Doxygen +
ExampleProjects for new API) can drop in the full code without
introducing a new top-level directory in the same PR.

For now the directory holds:

- this `README.md` — pointer to the source-of-truth API docs and the
  D-11 expansion task,
- a `CMakeLists.txt` stub that builds nothing (commented-out target)
  so a future `add_subdirectory(ExampleProjects/ISLogReader)` doesn't
  fail before D-11 lands.

## Quickstart (against `SDK/src/ISLogReader.h`)

```cpp
#include "ISLogReader.h"
#include "ISTimeStamp.h"

#include <iostream>

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <segment.raw>\n";
        return 1;
    }

    using namespace inertial_sense;
    auto reader = ISLogReader::openSegment(argv[1]);
    if (!reader) {
        std::cerr << "open failed: " << reader.error().message << "\n";
        return 2;
    }

    std::cout << "device " << reader->deviceId()
              << " — " << reader->recordCount() << " records, "
              << reader->presentDids().size() << " distinct DIDs\n";

    for (auto did : reader->presentDids()) {
        const auto count = reader->records(did).size();
        std::cout << "  DID " << did << ": " << count << " records\n";
    }
    return 0;
}
```

## Linkage

When D-11 finalizes this example, link against `InertialSenseSDK`:

```cmake
add_executable(islogreader_example main.cpp)
target_link_libraries(islogreader_example PRIVATE InertialSenseSDK)
```

## Related

- API: [`SDK/src/ISLogReader.h`](../../src/ISLogReader.h)
- Time-tagged values: [`SDK/src/ISTimeStamp.h`](../../src/ISTimeStamp.h) (D-06)
- Error model: [`SDK/src/ISError.h`](../../src/ISError.h) (D-10)
- Story: [SN-7893](https://inertialsense.atlassian.net/browse/SN-7893)
- Follow-up story: [SN-7900 (D-11)](https://inertialsense.atlassian.net/browse/SN-7900)
