import SwiftUI

/// Draws the source frame at full size with the solved skeleton projected on
/// top, so the pose can be judged against the photo it came from.
///
/// The 3-D orbit view answers "what does the posture look like from another
/// angle". It cannot answer "is this right", because a skeleton floating in an
/// empty scene has nothing to be wrong against. This view answers that: the
/// joints are projected through the model's OWN camera
/// (`MHRRetarget.projectToImage`), not fitted to the image, so a visible gap
/// between a drawn joint and the corresponding body part is the model's error.
///
/// Everything is laid out in image pixel space and then mapped by one
/// `aspectFit` transform, so the skeleton cannot drift relative to the photo
/// under any view size — the failure the corner-thumbnail layout had, where the
/// two were rendered by unrelated paths at unrelated scales.
struct PhotoOverlayView: View {
    let frame: OfflineResultStore.FrameResult

    var body: some View {
        GeometryReader { geo in
            let img = frame.sourceImage
            let fit = Self.aspectFit(imageSize: img.size, into: geo.size)

            ZStack(alignment: .topLeading) {
                Image(uiImage: img)
                    .resizable()
                    .frame(width: fit.size.width, height: fit.size.height)
                    .offset(x: fit.origin.x, y: fit.origin.y)

                if let points = projectedPoints() {
                    Canvas { ctx, _ in
                        // Bones first so joint dots sit on top of the lines.
                        // `bones` holds index pairs into `JointMapping.primary`,
                        // not joint ids.
                        for (i, j) in JointMapping.bones {
                            guard i < JointMapping.primary.count, j < JointMapping.primary.count,
                                  let a = points[JointMapping.primary[i].arkitName],
                                  let b = points[JointMapping.primary[j].arkitName] else { continue }
                            var path = Path()
                            path.move(to: fit.map(a))
                            path.addLine(to: fit.map(b))
                            ctx.stroke(path, with: .color(.green.opacity(0.9)), lineWidth: 3)
                        }
                        for (_, p) in points {
                            let c = fit.map(p)
                            let r: CGFloat = 4
                            ctx.fill(Path(ellipseIn: CGRect(x: c.x - r, y: c.y - r,
                                                            width: r * 2, height: r * 2)),
                                     with: .color(.yellow))
                        }
                    }
                } else {
                    // Never silently show a bare photo as if it were an overlay.
                    Text("No camera data for this frame — cannot project")
                        .font(.caption)
                        .padding(6)
                        .background(.black.opacity(0.6), in: RoundedRectangle(cornerRadius: 6))
                        .foregroundStyle(.white)
                        .padding(8)
                }
            }
        }
    }

    private func projectedPoints() -> [String: CGPoint]? {
        guard let camT = frame.camT, let body = frame.bodyFrame else { return nil }
        var out: [String: CGPoint] = [:]
        for joint in body.joints where joint.isTracked {
            if let p = MHRRetarget.projectToImage(joint.worldPosition,
                                                  camT: camT,
                                                  imageSize: frame.sourceImage.size) {
                out[joint.id] = p
            }
        }
        return out.isEmpty ? nil : out
    }

    /// Letterboxes `imageSize` inside `container`, and maps image pixels into
    /// view coordinates. One transform for both the photo and the overlay is
    /// what guarantees they stay registered.
    struct Fit {
        let origin: CGPoint
        let size: CGSize
        let scale: CGFloat

        func map(_ p: CGPoint) -> CGPoint {
            CGPoint(x: origin.x + p.x * scale, y: origin.y + p.y * scale)
        }
    }

    static func aspectFit(imageSize: CGSize, into container: CGSize) -> Fit {
        guard imageSize.width > 0, imageSize.height > 0 else {
            return Fit(origin: .zero, size: container, scale: 1)
        }
        let scale = min(container.width / imageSize.width,
                        container.height / imageSize.height)
        let size = CGSize(width: imageSize.width * scale, height: imageSize.height * scale)
        let origin = CGPoint(x: (container.width - size.width) / 2,
                             y: (container.height - size.height) / 2)
        return Fit(origin: origin, size: size, scale: scale)
    }
}
