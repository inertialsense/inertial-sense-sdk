# `ISLogWriter` — RAW + `.idx` v2 sidecar writer *(stub — D-11 fills out the full example)*

Placeholder slot reserved by **D-08 / SN-7898** for the `ISLogWriter`
walkthrough — typically a bake-trim-style demo: read records from a
source `.raw`, filter through a predicate, write the matches into a
new `.raw` + `.idx` pair. Full code drops in with **D-11 / SN-7900**
(Doxygen + ExampleProjects).

## Quickstart

```cpp
#include "ISLogReader.h"
#include "ISLogWriter.h"

#include <iostream>

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <in.raw> <out.raw>\n";
        return 1;
    }

    using namespace inertial_sense;
    auto reader = ISLogReader::openSegment(argv[1]);
    if (!reader) {
        std::cerr << "open failed: " << reader.error().message << "\n";
        return 2;
    }

    auto writer = ISLogWriter::create(argv[2], reader->deviceId());
    if (!writer) {
        std::cerr << "create failed: " << writer.error().message << "\n";
        return 3;
    }

    auto count = writer->appendFiltered(
        reader->allRecords(),
        [](const ISRecordView&) { return true; });  // example: copy all
    if (!count) {
        std::cerr << "append failed: " << count.error().message << "\n";
        return 4;
    }

    auto fin = writer->finalize();
    if (!fin) {
        std::cerr << "finalize failed: " << fin.error().message << "\n";
        return 5;
    }

    std::cout << "wrote " << count.value() << " records to " << argv[2] << "\n";
    return 0;
}
```

## Related

- API: [`SDK/src/ISLogWriter.h`](../../src/ISLogWriter.h)
- Story: [SN-7898](https://inertialsense.atlassian.net/browse/SN-7898)
- Follow-up: [SN-7900 (D-11)](https://inertialsense.atlassian.net/browse/SN-7900)
