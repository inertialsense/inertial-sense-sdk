# `ISLog` / `ISDeviceLog` — composition examples *(stub — D-11 fills out the full example)*

Placeholder slot reserved by **D-05 / SN-7896** for the `ISLog` and
`ISDeviceLog` walkthrough examples. Full code drops in with
**D-11 / SN-7900** (Doxygen + ExampleProjects).

## Quickstart

```cpp
#include "ISLog.h"

#include <iostream>

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <log_directory>\n";
        return 1;
    }

    using namespace inertial_sense;
    auto log = ISLog::openDirectory(argv[1]);
    if (!log) {
        std::cerr << "open failed: " << log.error().message << "\n";
        return 2;
    }

    std::cout << "directory: " << log->directory() << "\n";
    std::cout << "devices:   " << log->deviceIds().size() << "\n";
    std::cout << "records:   " << log->recordCount() << "\n";

    for (auto id : log->deviceIds()) {
        const auto& dl = log->device(id);
        std::cout << "  device " << id << "  segments=" << dl.segmentCount()
                  << "  records=" << dl.recordCount() << "\n";
    }
    return 0;
}
```

## Related

- API: [`SDK/src/ISLog.h`](../../src/ISLog.h),
        [`SDK/src/ISDeviceLog.h`](../../src/ISDeviceLog.h)
- Story: [SN-7896](https://inertialsense.atlassian.net/browse/SN-7896)
- Follow-up: [SN-7900 (D-11)](https://inertialsense.atlassian.net/browse/SN-7900)
