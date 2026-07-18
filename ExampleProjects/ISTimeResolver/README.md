# `ISTimeResolver` — piecewise-linear wall-clock reconstruction *(stub — D-11 fills out the full example)*

Placeholder slot reserved by **D-07 / SN-7897**. Full code drops in
with **D-11 / SN-7900**.

## Quickstart

```cpp
#include "ISDeviceLog.h"
#include "ISTimeResolver.h"

#include <iostream>

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <segment.raw>\n";
        return 1;
    }
    using namespace inertial_sense;

    auto log = ISDeviceLog::fromSegments({ argv[1] });
    if (!log) {
        std::cerr << "open failed: " << log.error().message << "\n";
        return 2;
    }

    auto resolver = ISTimeResolver::build(*log);
    if (!resolver) {
        std::cerr << "build failed: " << resolver.error().message << "\n";
        return 3;
    }

    std::cout << "sync points:    " << resolver->syncPoints().size() << '\n'
              << "discontinuities:" << resolver->discontinuities().size() << '\n';

    auto stats = resolver->computeStats(*log);
    std::cout << "stats: exact="   << stats.exact
              << " interp="        << stats.interpolated
              << " extrap_fwd="    << stats.extrapFwd
              << " extrap_back="   << stats.extrapBack
              << " unknown="       << stats.unknown << '\n';
    return 0;
}
```

## Related

- API: [`SDK/src/ISTimeResolver.h`](../../src/ISTimeResolver.h),
       [`SDK/src/ISSyncPoint.h`](../../src/ISSyncPoint.h)
- Story: [SN-7897](https://inertialsense.atlassian.net/browse/SN-7897)
- Follow-up: [SN-7900 (D-11)](https://inertialsense.atlassian.net/browse/SN-7900) — full example.
