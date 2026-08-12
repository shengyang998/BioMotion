# Unattended TestFlight release

The global Codex Skill `ios-testflight-release` is the default workflow for
this and future iOS apps. Its installed source is
`~/.codex/skills/ios-testflight-release/`; it supplies the reusable project
contract, credential boundary, exact-IPA delivery tool, `VALID` polling, and
private receipt format. BioMotion keeps its repository-owned wrappers because
its nested native dependencies, Managed Background Asset Pack, manual/offline
signing fallback, resource allowlist, and dependency receipt need stricter
app-specific gates than the global delivery primitive.

This is BioMotion's default release path on the maintained workstation. It
requires no Xcode Organizer, Transporter, browser session, or UI click. Upload
is still an explicit action: use `--upload`; never make upload an implicit side
effect of a build.

## Stable workstation state

- App Store Connect private key:
  `~/.appstoreconnect/private_keys/AuthKey_<KEY_ID>.p8`, owner-only and mode
  0600; both parent directories must be owner-only.
- ASC Key ID reference: macOS Keychain generic password service
  `com.soleilyu.biomotion.appstoreconnect.key-id`, account `biomotion`.
- ASC Issuer reference: macOS Keychain generic password service
  `com.soleilyu.biomotion.appstoreconnect.issuer`, account `biomotion`.
- Global same-provider references for other iOS apps: Keychain services
  `com.soleilyu.ios-release.appstoreconnect.key-id` and
  `com.soleilyu.ios-release.appstoreconnect.issuer`, account `ios-release`.
- Persistent distribution certificate/key, app and extension profiles, and the
  pinned `rcodesign` binary:
  `~/.config/biomotion/release-signing/`, owner-only. This is the fallback when
  the login keychain cannot sign unattended; never copy it into Git.

The Key ID and Issuer are credential references, not the private key. Do not
print any of the three values into release logs or commit them. The release
script consults Keychain only after the source, dependency receipt, archive,
privacy, export, and byte-pinned IPA gates have passed. Supplying
`ASC_API_KEY_ID` and `ASC_API_ISSUER` explicitly remains a one-run override.
Apps owned by another App Store Connect provider must declare separate
Keychain account/service names in their project release contract; never
overwrite the global same-provider entries.

## Release sequence

1. Query App Store Connect and select an unused build number greater than every
   existing build. Set both target-level `CURRENT_PROJECT_VERSION` values in
   `project.yml`, regenerate with XcodeGen, run the source/resource/privacy
   gates, commit, and push `main`.
2. Produce a fresh archive with its adjacent dependency receipt through
   `tools/release/archive_release.sh`. If login-keychain signing is unavailable,
   archive with signing disabled, embed the two pinned App Store profiles,
   restore unmodified app icons, sign the extension and then the containing app
   with the pinned `rcodesign`, and require strict signature plus exact
   entitlement equality before continuing. Retain the archive and receipt
   together under `build/releases/<BUILD>/`.
3. Export/gate the IPA and explicitly authorize the external transaction:

   ```bash
   /bin/bash -p tools/release/testflight_release.sh --upload \
     --archive build/releases/BUILD/BioMotion-1.0.0-BUILD.xcarchive \
     --export-dir build/releases/BUILD/testflight-BUILD
   ```

   The wrapper rechecks dependencies and archive bytes, gates source/archive/
   privacy/IPA boundaries, writes a private SHA-256 receipt, validates the
   private byte-pinned snapshot, and uploads those same bytes.
4. Capture the delivery UUID and poll `altool --build-status --delivery-id ...`
   until App Store Connect reports `VALID`. `UPLOAD SUCCEEDED` alone is not the
   completion criterion. Record the build, IPA bytes/SHA, delivery UUID, final
   processing state, source commit, archive receipt, and device-smoke status in
   `README.md` and `STATUS.md`, then commit and push the documentation.
5. Install that exact TestFlight build on a physical device. For a hosted-model
   change, verify cellular/Wi-Fi download, pause/resume and relaunch recovery,
   complete `SAM3DBodyPose.mlmodelc` lookup, Core ML open, and one real
   inference. The 1.096 GB Managed Background Asset Pack is not ODR and is not
   in the main app/IPA bundle.

## Rotation and failure policy

Do not generate a new API key merely because a shell variable is absent. First
read the two Keychain references and verify the existing `.p8`. Create or revoke
credentials only as an explicit owner action. Renew the distribution
certificate/profiles before expiry, update the owner-only signing directory,
and prove certificate/key, profile entitlements, and app/extension signatures
again. Missing, expired, revoked, ambiguously selected, or broadly readable
credentials must stop the release.
