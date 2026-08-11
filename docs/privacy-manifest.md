# Privacy manifest contract

BioMotion's reviewed manifest is `BioMotion/PrivacyInfo.xcprivacy`. It:

- sets `NSPrivacyTracking = false`; and
- omits `NSPrivacyTrackingDomains`, `NSPrivacyCollectedDataTypes`, and
  `NSPrivacyAccessedAPITypes` because each reviewed inventory is empty.

The omitted keys are load-bearing. Apple's
[TN3181](https://developer.apple.com/documentation/technotes/tn3181-debugging-invalid-privacy-manifest)
classifies an empty `NSPrivacyAccessedAPITypes` array as an invalid privacy
manifest and says to remove that key when the list is empty. It also recommends
removing `NSPrivacyTrackingDomains` when tracking is false. The manifest does
not serialize “none” as speculative empty arrays.

The required-reason inventory is deliberately empty. The runtime no longer
reads a model file's modification time: it accepts only receipt-verified
precompiled models, so the previous File Timestamp evidence was removed with
the raw-model compile cache. The production source also contains no
`systemUptime` or `mach_absolute_time()` call, the APIs Apple lists for the
System Boot Time category.

There are 13 reviewed `CACurrentMediaTime()` calls: three in
`BodyTrackingSession.swift`, two in `MuscleSolver.mm`, four in
`NimbleEngine.swift`, and four in `OfflineSessionRunner.swift`. They measure
frame, solver, timeout, and tracking-loss elapsed time. Apple does not list
`CACurrentMediaTime()` or `clock_gettime()` in its System Boot Time
required-reason API category, so those calls are pinned by the audit but are not
misdeclared as reason `35F9.1`. The source audit separately found no UserDefaults,
available-disk-capacity, active-keyboard, AppTrackingTransparency, or
advertising-identifier use in the production target.

Required-reason API lists can change. Before release, compare Apple's current
[Required Reason API documentation](https://developer.apple.com/documentation/bundleresources/app-privacy-configuration/nsprivacyaccessedapitypes/nsprivacyaccessedapitype)
and the Xcode archive Privacy Report with this source inventory; do not add a
category or reason without matching production evidence.

Camera frames, selected photos/videos, AR body data, and pose/fitness inference
are processed on device. A TRC leaves the device only when the user explicitly
invokes the system share sheet; BioMotion does not operate a server and the
developer or a third-party analytics/advertising SDK does not receive this data.
This product-flow review—not merely a source-token search—is the basis for the
empty collected-data and tracking declarations.

Run the source/project contract with:

```sh
/bin/bash tools/tests/privacy_manifest_probe.sh
/bin/bash tools/tests/privacy_manifest_probe_tests.sh
```

Pass a built `.app` path to additionally require a byte-identical bundled
manifest and a strict code-signature verification:

```sh
/bin/bash tools/tests/privacy_manifest_probe.sh /path/to/BioMotion.app
```

The expected sentinel is `PRIVACY_MANIFEST_PROBE_PASS`. The gate requires
`NSPrivacyTracking = false`, rejects every other top-level key while the
reviewed inventories remain empty, and pins the exact 13-call non-required
elapsed-clock inventory. The **38/38** suite includes negative injections for
every spelling in Apple's current five-category API list, three language-layer
UserDefaults aliases, both reviewed elapsed clocks, an invalid empty API array,
and wrong-target/misdirected/duplicated project assignments. XcodeGen must copy
the manifest exactly once into BioMotion's own resources phase; the Background
Download extension does not currently use a required-reason API and does not
carry a separate manifest.

The release gate must pass a real archive app path. It then checks the bundle
identifier, root-manifest identity, every embedded manifest, every recursively
discovered Mach-O image, dynamic dependencies, and the code signature. Debug
builds are not allowed to hide code from the scan in Xcode's in-bundle
`*.debug.dylib` or `__preview.dylib`; only the exact reviewed self-named
`@rpath` images are accepted. The archive Privacy Report and App Store Connect
privacy answers remain human-reviewed release artifacts and must agree with
this contract.
