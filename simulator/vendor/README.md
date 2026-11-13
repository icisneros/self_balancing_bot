Vendored, unmodified `PID_v1.cpp` and `PID_v1.h` from Brett Beauregard's
[Arduino PID Library](https://github.com/br3ttb/Arduino-PID-Library/tree/524a4268fc01e6ea397e7fc5b5d820741e9b662f),
commit `524a4268fc01e6ea397e7fc5b5d820741e9b662f`, library version 1.2.1.

The upstream source states that the library is licensed under the MIT License;
its original author and license notices remain in the files. Upstream does not
include a separate LICENSE file at this revision.

The project's historical installed PID version is unknown. The older example
in Git history uses 1.0.1; this sandbox explicitly uses 1.2.1. The sketch uses
the backwards-compatible proportional-on-error constructor.

The adapter renames the upstream class to `NativePID` during compilation and
subclasses it to record successful calculation times. It does not change the
PID equations, gating, clamping, or initialization.
