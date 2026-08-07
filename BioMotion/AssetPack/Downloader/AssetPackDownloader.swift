import BackgroundAssets
import ExtensionFoundation
import StoreKit

/// Background Download extension for the Apple-Hosted Managed Background Assets
/// pack that carries `SAM3DBodyPose.mlmodelc` (~1.31 GiB).
///
/// The system drives the download out-of-process. This extension's only job is
/// `shouldDownload`, the policy hook the OS calls when it is about to fetch a
/// pack on its own (the manifest marks the pack `prefetch`, so that is right
/// after install/update). Returning `false` does NOT make the pack unreachable —
/// the app still gets it on demand through
/// `AssetPackManager.ensureLocalAvailability`, which is what
/// `AssetPackModelStore` calls the first time a clip is imported.
///
/// This deliberately does NOT gate on Wi-Fi. The pack is the app's only way to
/// analyse anything, so a user on cellular who never joins Wi-Fi would otherwise
/// see "model unavailable" forever with no explanation. iOS already applies its
/// own system-level constraints to managed downloads (Low Data Mode, storage
/// pressure, user's App Store cellular-download setting), so a second private
/// policy here would only add a failure mode the user cannot see or change.
@main
struct AssetPackDownloader: StoreDownloaderExtension {
    func shouldDownload(_ assetPack: AssetPack) -> Bool {
        assetPack.id == AssetPackIdentity.modelPackID
    }
}

/// Duplicated (not shared) with `AssetPackModelStore` on purpose: the extension
/// is a separate binary in a separate target, and pulling the app's store — with
/// its Core ML, SwiftUI and Combine dependencies — into an extension that is
/// allowed only a few hundred milliseconds of wake time would be far worse than
/// two string constants. Changing one without the other breaks the pack lookup,
/// so both sites name this file.
enum AssetPackIdentity {
    static let modelPackID = "sam3d-body-pose"
}
