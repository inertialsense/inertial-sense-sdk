# Built-in derivations — registry walk *(stub — D-11 fills out the full example)*

Placeholder slot reserved by **D-09 / SN-7899**. Full code drops in
with **D-11 / SN-7900** (Doxygen + ExampleProjects).

## Quickstart

```cpp
#include "ISLogReader.h"
#include "derivations/DerivationRegistry.h"

#include <iostream>

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0]
                  << " <segment.raw> <derivation_name>\n";
        return 1;
    }

    using namespace inertial_sense;
    auto reader = ISLogReader::openSegment(argv[1]);
    if (!reader) {
        std::cerr << "open failed: " << reader.error().message << "\n";
        return 2;
    }

    auto d = derivations::DerivationRegistry::instance().get(argv[2]);
    if (!d) {
        std::cerr << "unknown derivation: " << argv[2] << "\n";
        std::cerr << "available:\n";
        for (auto* e : derivations::DerivationRegistry::instance().all()) {
            std::cerr << "  " << e->name << "  (" << e->outputUnit << ")\n";
        }
        return 3;
    }

    derivations::DerivationContext ctx{ *reader };
    auto r = (*d)->evaluate(ctx);
    if (!r) {
        std::cerr << "evaluate failed: " << r.error().message << "\n";
        return 4;
    }

    std::cout << "samples: " << r->samples.size()
              << "  unit: "    << r->outputUnit
              << "  frame: '"  << r->outputFrame << "'\n";
    return 0;
}
```

## Related

- API: [`SDK/src/derivations/DerivationRegistry.h`](../../src/derivations/DerivationRegistry.h)
- Story: [SN-7899](https://inertialsense.atlassian.net/browse/SN-7899)
- Follow-up: [SN-7900 (D-11)](https://inertialsense.atlassian.net/browse/SN-7900)
