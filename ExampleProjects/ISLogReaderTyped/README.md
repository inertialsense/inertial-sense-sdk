# `ISLogReader` — templated sugar (`records<DID>()`) *(stub — D-11 fills out the full example)*

Placeholder slot reserved by **D-03 / SN-7894**. Full code drops in
with **D-11 / SN-7900**.

## Quickstart

```cpp
#include "ISLogReader.h"
#include "ISLogReaderSugar.h"   // opt-in: templated sugar

#include <iostream>

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <segment.raw>\n";
        return 1;
    }
    using namespace inertial_sense;
    auto r = ISLogReader::openSegment(argv[1]);
    if (!r) {
        std::cerr << "open failed: " << r.error().message << "\n";
        return 2;
    }

    // Compile-time-typed iteration — no casts:
    for (const ins_2_t& ins : r->records<DID_INS_2>()) {
        std::cout << "lat=" << ins.lla[0]
                  << " lon=" << ins.lla[1]
                  << " alt=" << ins.lla[2] << '\n';
    }
    return 0;
}
```

## Related

- API: [`SDK/src/DIDTraits.h`](../../src/DIDTraits.h),
       [`SDK/src/ISLogReaderSugar.h`](../../src/ISLogReaderSugar.h)
- Story: [SN-7894](https://inertialsense.atlassian.net/browse/SN-7894)
- Follow-up: [SN-7900 (D-11)](https://inertialsense.atlassian.net/browse/SN-7900) — full example.
- Auto-gen of `DIDTraits.h` from `data_sets.c` is tracked as a follow-up to D-03.
