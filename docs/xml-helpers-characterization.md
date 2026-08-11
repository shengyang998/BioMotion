# XML helper characterization contract

`dart/utils/XmlHelpers.cpp` is on the iOS native archive path and historically
depends on Boost's header-only string algorithms and lexical conversion. Before
that dependency is removed, `tools/tests/xml_helpers_characterization_probe.cpp`
captures the behavior that callers already observe.

The probe covers:

- exact scalar formatting, including round-trip float and double precision;
- strict numeric parsing, integer and unsigned boundaries, overflow, hexadecimal
  floating point, NaN/Infinity, and the `std::bad_cast` base contract for every
  failed scalar conversion;
- preservation of caller `errno` on successful conversions and `ERANGE` on
  floating-point range failures;
- the existing unsigned `"-1"` wrap behavior and rejection of repeated or
  separated signs;
- literal-space vector tokenization, surrounding whitespace trimming, Eigen
  formatting, transforms, and tinyxml2 value/attribute wrappers; and
- simulator and device ordinary archive links, followed by simulator execution.

Some legacy behavior is intentionally surprising: boolean text is
case-insensitive but is not trimmed, a one-character space or tab is a valid
`char`, and parsing a subnormal float/double string reports a range failure.
These cases are pinned so a dependency refactor cannot silently "improve" them.

The characterization runs under the app's normal classic process locale. Boost
otherwise inherits a hostile global C++ locale, which can turn XML decimal dots
into locale-specific commas and grouping. That global-locale behavior is not a
portable XML contract and is deliberately excluded here; the refactor step must
pin XML conversion to the classic locale and add a separate hostile-locale test
that proves decimal-dot behavior remains stable.

The `ERANGE` observations are an Apple libc++ target contract, verified by the
simulator executable and the device archive link. Generic C++17 stream parsing
does not specify `errno`, so this receipt must not be generalized to another
standard library without rerunning the range cases there.

Run the contract directly:

```sh
tools/tests/xml_helpers_characterization_probe.sh
```

The expected sentinels are:

```text
XML_HELPERS_CHARACTERIZATION_PASS
XML_HELPERS_ARCHIVE_PROBE_PASS
```

The behavior-only characterization commit does not by itself prove that Boost
has been removed. The completed refactor gate in
`tools/tests/xml_helpers_refactor_probe.sh` therefore performs fresh simulator
and device source compiles, requires exactly one `XmlHelpers.cpp.o` in each
rebuilt archive, rejects Boost symbols in those members, runs a hostile-locale
probe, and finally reruns this same contract against the rebuilt artifacts.
