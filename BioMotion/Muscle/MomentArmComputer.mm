#import "MomentArmComputer.h"
#import "../Nimble/NimbleBridge+Internal.h"

#include <vector>
#include <string>
#include <map>
#include <set>
#include <cmath>
#include <cstdlib>
#include <sstream>
#include <algorithm>

#include <tinyxml2.h>

#include "dart/dynamics/Skeleton.hpp"
#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/DegreeOfFreedom.hpp"
#include "dart/math/MathTypes.hpp"

using namespace dart;

// MARK: - Internal muscle path structure

/// One location component (x, y or z) of a MovingPathPoint: a scalar function
/// of a driving coordinate, stored as its control points. `dofIndex < 0` with a
/// non-empty sample set never happens — such a point is rejected at parse time
/// rather than evaluated at a fabricated coordinate value.
struct MovingAxis {
    int dofIndex = -1;              // Skeleton DOF driving this component.
    std::vector<double> knotX;      // Control-point abscissae, ascending.
    std::vector<double> knotY;      // Control-point ordinates (meters).
    double constantValue = 0.0;     // Used when knotX is empty.
};

struct InternalPathPoint {
    std::string bodyName;
    Eigen::Vector3s localOffset;

    // ConditionalPathPoint gating. `conditionDofIndex < 0` means the point is
    // unconditional (a plain PathPoint, or a conditional one whose coordinate
    // is not a DOF of this skeleton) and is therefore always active.
    int conditionDofIndex = -1;
    double conditionMin = 0.0;
    double conditionMax = 0.0;

    // Latched by -refreshConditionalPathPointActivity. Held fixed across the
    // finite-difference perturbations so a via point switching state inside the
    // ±eps stencil cannot produce a spike in dL/dq.
    bool active = true;

    // MovingPathPoint: localOffset is recomputed from `axis` at every FK call.
    bool moving = false;
    MovingAxis axis[3];
};

struct InternalMusclePath {
    std::string name;
    std::vector<InternalPathPoint> points;
    double maxIsometricForce;
    double optimalFiberLength;
    double tendonSlackLength;
    double pennationAngle;
};

// MARK: - XML helpers

/// OpenSim sockets carry component paths ("/bodyset/tibia_r",
/// "/jointset/radioulnar_r/pro_sup_r"); the skeleton knows only the leaf name.
static std::string osimLeafName(const std::string& ref) {
    auto lastSlash = ref.find_last_of('/');
    return (lastSlash == std::string::npos) ? ref : ref.substr(lastSlash + 1);
}

/// Read a text child element, trying each candidate tag in order. Handles the
/// newer socket_* spelling alongside the pre-4.0 plain spelling. Surrounding
/// whitespace is stripped: these strings are matched exactly against skeleton
/// body / DOF names, and a stray newline would silently degrade the point.
static bool readLeafRef(tinyxml2::XMLElement* el,
                        std::initializer_list<const char*> tags,
                        std::string& out) {
    for (const char* tag : tags) {
        auto* child = el->FirstChildElement(tag);
        if (!child || !child->GetText()) continue;
        std::string text(child->GetText());
        const auto first = text.find_first_not_of(" \t\r\n");
        if (first == std::string::npos) continue;
        const auto last = text.find_last_not_of(" \t\r\n");
        out = osimLeafName(text.substr(first, last - first + 1));
        if (!out.empty()) return true;
    }
    return false;
}

/// Parse whitespace-separated doubles out of a child element's text.
static std::vector<double> readNumbers(tinyxml2::XMLElement* el, const char* tag) {
    std::vector<double> values;
    auto* child = el->FirstChildElement(tag);
    if (!child || !child->GetText()) return values;
    std::istringstream ss(child->GetText());
    double v = 0;
    while (ss >> v) values.push_back(v);
    return values;
}

/// Parse the location function of one MovingPathPoint component.
/// Returns false for function types we cannot represent — the caller drops the
/// whole point rather than substituting a guess.
/// `approximated` is set when the true function is a cubic spline that we will
/// evaluate by linear interpolation between its control points.
static bool parseMovingAxisFunction(tinyxml2::XMLElement* locationEl,
                                    MovingAxis& axis,
                                    bool& approximated) {
    auto* fn = locationEl->FirstChildElement();
    if (!fn) return false;
    const std::string kind = fn->Name();

    if (kind == "Constant") {
        auto values = readNumbers(fn, "value");
        if (values.empty()) return false;
        axis.constantValue = values[0];
        return true;
    }

    const bool isCubicSpline = (kind == "SimmSpline" ||
                                kind == "NaturalCubicSpline" ||
                                kind == "GCVSpline");
    if (!isCubicSpline && kind != "PiecewiseLinearFunction") return false;

    axis.knotX = readNumbers(fn, "x");
    axis.knotY = readNumbers(fn, "y");
    if (axis.knotX.size() < 2 || axis.knotX.size() != axis.knotY.size()) return false;

    // With only two control points a natural cubic spline degenerates to the
    // straight line through them, so linear interpolation is exact there.
    if (isCubicSpline && axis.knotX.size() > 2) approximated = true;
    return true;
}

/// Linear interpolation over the control points, clamped to the terminal
/// control point outside the knot span (OpenSim extrapolates; clamping keeps
/// the path bounded when a coordinate is driven past the tabulated range).
static double evaluateMovingAxis(const MovingAxis& axis, double coordValue) {
    if (axis.knotX.empty()) return axis.constantValue;
    if (coordValue <= axis.knotX.front()) return axis.knotY.front();
    if (coordValue >= axis.knotX.back()) return axis.knotY.back();
    for (size_t i = 1; i < axis.knotX.size(); i++) {
        if (coordValue <= axis.knotX[i]) {
            const double x0 = axis.knotX[i - 1], x1 = axis.knotX[i];
            const double span = x1 - x0;
            if (span <= 0) return axis.knotY[i];
            const double t = (coordValue - x0) / span;
            return axis.knotY[i - 1] + t * (axis.knotY[i] - axis.knotY[i - 1]);
        }
    }
    return axis.knotY.back();
}

/// Attachment offset in the parent body's frame. Fixed for PathPoint and
/// ConditionalPathPoint; for a MovingPathPoint it is evaluated live from the
/// driving coordinate rather than latched — unlike conditional gating this is
/// continuous in q, so it belongs inside the finite-difference stencil.
/// A free function rather than a method: this runs once per path point per FK
/// call, which is 2 × nDOFs × every point in the model per moment-arm solve.
static Eigen::Vector3s localOffsetForPathPoint(const InternalPathPoint& pp,
                                               const dynamics::Skeleton* skeleton) {
    if (!pp.moving || !skeleton) return pp.localOffset;

    Eigen::Vector3s offset = pp.localOffset;
    for (int a = 0; a < 3; a++) {
        const MovingAxis& axis = pp.axis[a];
        const double coord = (axis.dofIndex >= 0)
            ? skeleton->getDof(axis.dofIndex)->getPosition()
            : 0.0;  // only reached for constant axes, where coord is unused
        offset[a] = evaluateMovingAxis(axis, coord);
    }
    return offset;
}

// MARK: - MusclePathFidelityReport

NS_ASSUME_NONNULL_BEGIN

/// Write access for the parser only — the public interface stays read-only.
@interface MusclePathFidelityReport ()
@property (nonatomic, readwrite) NSInteger musclesParsed;
@property (nonatomic, readwrite) NSInteger pathPointsParsed;
@property (nonatomic, readwrite) NSInteger conditionalPathPointsParsed;
@property (nonatomic, readwrite) NSInteger conditionalPathPointsSkipped;
@property (nonatomic, readwrite) NSInteger conditionalPathPointsUnresolvedCoordinate;
@property (nonatomic, readwrite) NSInteger movingPathPointsParsed;
@property (nonatomic, readwrite) NSInteger movingPathPointsApproximated;
@property (nonatomic, readwrite) NSInteger movingPathPointsSkipped;
@property (nonatomic, readwrite) NSInteger unknownPathPointElementsSkipped;
@property (nonatomic, readwrite) NSInteger unmodelledPathWraps;
@property (nonatomic, readwrite) NSArray<NSString *> *musclesWithDefaultedTendonSlackLength;
@end

NS_ASSUME_NONNULL_END

@implementation MusclePathFidelityReport

- (instancetype)init {
    self = [super init];
    if (self) { _musclesWithDefaultedTendonSlackLength = @[]; }
    return self;
}

- (NSString *)summary {
    return [NSString stringWithFormat:
        @"%ld muscles | PathPoint %ld | Conditional %ld parsed (%ld skipped, "
        @"%ld ungated) | Moving %ld parsed (%ld approximated, %ld skipped) | "
        @"unknown elements %ld | UNMODELLED PathWraps %ld | "
        @"defaulted tendon_slack_length %lu",
        (long)_musclesParsed, (long)_pathPointsParsed,
        (long)_conditionalPathPointsParsed, (long)_conditionalPathPointsSkipped,
        (long)_conditionalPathPointsUnresolvedCoordinate,
        (long)_movingPathPointsParsed, (long)_movingPathPointsApproximated,
        (long)_movingPathPointsSkipped,
        (long)_unknownPathPointElementsSkipped,
        (long)_unmodelledPathWraps,
        (unsigned long)_musclesWithDefaultedTendonSlackLength.count];
}

- (NSString *)description {
    return [NSString stringWithFormat:@"<MusclePathFidelityReport: %@>", self.summary];
}

@end

// MARK: - MusclePathPoint

@implementation MusclePathPoint {
    NSString *_bodyName;
    double _x, _y, _z;
}

- (instancetype)initWithBody:(NSString *)body x:(double)x y:(double)y z:(double)z {
    self = [super init];
    if (self) { _bodyName = body; _x = x; _y = y; _z = z; }
    return self;
}
- (NSString *)bodyName { return _bodyName; }
- (double)x { return _x; }
- (double)y { return _y; }
- (double)z { return _z; }
@end

// MARK: - MusclePathData

@implementation MusclePathData {
    NSString *_name;
    NSArray<MusclePathPoint *> *_pathPoints;
    double _maxIsometricForce, _optimalFiberLength, _pennationAngle;
}

- (instancetype)initWithName:(NSString *)name
                  pathPoints:(NSArray<MusclePathPoint *> *)points
           maxIsometricForce:(double)f0
        optimalFiberLength:(double)l0
             pennationAngle:(double)alpha {
    self = [super init];
    if (self) {
        _name = name; _pathPoints = points;
        _maxIsometricForce = f0; _optimalFiberLength = l0; _pennationAngle = alpha;
    }
    return self;
}
- (NSString *)name { return _name; }
- (NSArray<MusclePathPoint *> *)pathPoints { return _pathPoints; }
- (double)maxIsometricForce { return _maxIsometricForce; }
- (double)optimalFiberLength { return _optimalFiberLength; }
- (double)pennationAngle { return _pennationAngle; }
@end

// MARK: - MomentArmComputer

@implementation MomentArmComputer {
    std::vector<InternalMusclePath> _musclePaths;
    std::shared_ptr<dynamics::Skeleton> _skeleton;
    BOOL _loaded;
    MusclePathFidelityReport *_fidelityReport;
}

- (instancetype)init {
    self = [super init];
    if (self) {
        _loaded = NO;
        _fidelityReport = [[MusclePathFidelityReport alloc] init];
    }
    return self;
}

- (MusclePathFidelityReport *)fidelityReport { return _fidelityReport; }

- (BOOL)parseMusclePathsFromOsimPath:(NSString *)path
                          fromBridge:(NimbleBridge *)bridge {
    std::string pathStr([path UTF8String]);

    // Adopt the skeleton that NimbleBridge has already loaded and scaled.
    // This is the single source of truth — any setBodyScales /
    // setPositions called on the bridge is instantly visible here.
    _skeleton = [bridge sharedSkeleton];
    if (!_skeleton) {
        NSLog(@"MomentArmComputer: NimbleBridge has no skeleton yet — "
              @"did you call loadModel first?");
        return NO;
    }

    // Parse muscle paths from XML (nimble OpenSimParser reads the
    // kinematic skeleton but doesn't expose the muscle GeometryPath
    // data we need for moment-arm FK, so we parse the relevant sections
    // directly).
    tinyxml2::XMLDocument doc;
    if (doc.LoadFile(pathStr.c_str()) != tinyxml2::XML_SUCCESS) return NO;

    auto* root = doc.FirstChildElement("OpenSimDocument");
    if (!root) return NO;
    auto* model = root->FirstChildElement("Model");
    if (!model) return NO;
    auto* forceSet = model->FirstChildElement("ForceSet");
    if (!forceSet) return NO;
    auto* objects = forceSet->FirstChildElement("objects");
    if (!objects) return NO;

    _musclePaths.clear();

    MusclePathFidelityReport *report = [[MusclePathFidelityReport alloc] init];
    NSMutableArray<NSString *> *defaultedTendonSlack = [NSMutableArray array];

    // Resolve OpenSim coordinate names to skeleton DOF indices once. Both
    // ConditionalPathPoint gating and MovingPathPoint location functions are
    // driven by coordinate values, and we refuse to evaluate either against a
    // fabricated value — an unresolvable coordinate downgrades the point
    // (unconditional / dropped) and is counted in the fidelity report.
    std::map<std::string, int> dofIndexByName;
    for (size_t i = 0; i < _skeleton->getNumDofs(); i++) {
        dofIndexByName[_skeleton->getDof(i)->getName()] = (int)i;
    }

    // Muscle parser that handles both Millard2012EquilibriumMuscle and
    // Thelen2003Muscle — they share the same XML fields (max_isometric_force,
    // optimal_fiber_length, tendon_slack_length, pennation_angle_at_optimal,
    // max_contraction_velocity, GeometryPath). Full-body models like
    // cyclistFullBodyMuscle mix the two classes (Millard for lower body,
    // Thelen for upper body + spine), so we iterate both XML tags under
    // <ForceSet>/<objects>.
    static const char* const muscleClasses[] = {
        "Millard2012EquilibriumMuscle",
        "Thelen2003Muscle",
        nullptr,
    };

    for (int ci = 0; muscleClasses[ci]; ci++) {
    for (auto* muscleEl = objects->FirstChildElement(muscleClasses[ci]);
         muscleEl;
         muscleEl = muscleEl->NextSiblingElement(muscleClasses[ci])) {

        InternalMusclePath mp;
        const char* name = muscleEl->Attribute("name");
        if (!name) continue;
        mp.name = name;

        // Read muscle params. strtod rather than std::stod: std::stod THROWS on
        // malformed text, and the caller distinguishing "absent" from "0" is
        // the whole point for tendon_slack_length.
        auto readDouble = [&](const char* tag, double* out) -> bool {
            auto* el = muscleEl->FirstChildElement(tag);
            if (!el || !el->GetText()) return false;
            const char* text = el->GetText();
            char* end = nullptr;
            double v = std::strtod(text, &end);
            if (end == text || !std::isfinite(v)) return false;
            *out = v;
            return true;
        };
        auto readDoubleOr = [&](const char* tag, double fallback) -> double {
            double v = 0;
            return readDouble(tag, &v) ? v : fallback;
        };

        mp.maxIsometricForce = readDoubleOr("max_isometric_force", 0.0);
        mp.optimalFiberLength = readDoubleOr("optimal_fiber_length", 0.0);
        mp.pennationAngle = readDoubleOr("pennation_angle_at_optimal", 0.0);

        // A defaulted tendon slack length is not a cosmetic gap: MuscleSolver
        // then computes fiber length as L_MT / cos(α), so that muscle sits on
        // the wrong part of its force-length curve forever.
        double tendonSlack = 0.0;
        const bool tendonSlackParsed = readDouble("tendon_slack_length", &tendonSlack);
        mp.tendonSlackLength = tendonSlackParsed ? tendonSlack : 0.0;

        // Parse GeometryPath → PathPointSet → PathPoint
        auto* geoPath = muscleEl->FirstChildElement("GeometryPath");
        if (!geoPath) continue;
        auto* pathPointSet = geoPath->FirstChildElement("PathPointSet");
        if (!pathPointSet) continue;
        auto* ppObjects = pathPointSet->FirstChildElement("objects");
        if (!ppObjects) continue;

        // Walk EVERY child element in document order rather than filtering by
        // tag name: path points are polyline vertices, so a via point inserted
        // at the wrong index is worse than a missing one, and tinyxml2 matches
        // tag names exactly (name-filtering silently drops the 418
        // ConditionalPathPoints and 4 MovingPathPoints of the production model).
        NSInteger nPlain = 0, nCond = 0, nCondSkipped = 0, nCondUngated = 0;
        NSInteger nMoving = 0, nMovingApprox = 0, nMovingSkipped = 0, nUnknown = 0;

        for (auto* pp = ppObjects->FirstChildElement();
             pp; pp = pp->NextSiblingElement()) {

            const std::string kind = pp->Name();
            const bool isPlain = (kind == "PathPoint");
            const bool isConditional = (kind == "ConditionalPathPoint");
            const bool isMoving = (kind == "MovingPathPoint");
            if (!isPlain && !isConditional && !isMoving) {
                nUnknown++;
                continue;
            }

            InternalPathPoint ipp;

            // Body name. Pre-4.0 models use <body>, 4.x uses a component path
            // in <socket_parent_frame> (e.g. "/bodyset/tibia_r" → "tibia_r").
            if (!readLeafRef(pp, {"body", "socket_parent_frame"}, ipp.bodyName)) {
                if (isConditional) nCondSkipped++;
                else if (isMoving) nMovingSkipped++;
                continue;
            }

            if (isMoving) {
                // Location is a function of a driving coordinate per axis;
                // localOffset is refreshed from those functions on every FK call.
                static const char* const axisTags[3] = {
                    "x_location", "y_location", "z_location"
                };
                static const char* const socketTags[3] = {
                    "socket_x_coordinate", "socket_y_coordinate", "socket_z_coordinate"
                };
                static const char* const legacyTags[3] = {
                    "x_coordinate", "y_coordinate", "z_coordinate"
                };

                bool usable = true;
                bool approximated = false;
                for (int a = 0; a < 3 && usable; a++) {
                    auto* locEl = pp->FirstChildElement(axisTags[a]);
                    if (!locEl ||
                        !parseMovingAxisFunction(locEl, ipp.axis[a], approximated)) {
                        usable = false;
                        break;
                    }
                    if (ipp.axis[a].knotX.empty()) continue;  // constant, no coordinate needed

                    std::string coordName;
                    if (!readLeafRef(pp, {socketTags[a], legacyTags[a]}, coordName)) {
                        usable = false;
                        break;
                    }
                    auto it = dofIndexByName.find(coordName);
                    if (it == dofIndexByName.end()) {
                        usable = false;  // never evaluate at an invented coordinate value
                        break;
                    }
                    ipp.axis[a].dofIndex = it->second;
                }

                if (!usable) {
                    nMovingSkipped++;
                    continue;
                }
                ipp.moving = true;
                ipp.localOffset = Eigen::Vector3s::Zero();  // filled in by FK
                nMoving++;
                if (approximated) nMovingApprox++;
            } else {
                auto locValues = readNumbers(pp, "location");
                ipp.localOffset = (locValues.size() >= 3)
                    ? Eigen::Vector3s(locValues[0], locValues[1], locValues[2])
                    : Eigen::Vector3s::Zero();
            }

            if (isConditional) {
                // Range gating needs both the coordinate and the range. Missing
                // either leaves the point unconditional — strictly better than
                // dropping a polyline vertex, but it must be visible.
                std::string coordName;
                auto range = readNumbers(pp, "range");
                bool gated = false;
                if (range.size() >= 2 &&
                    readLeafRef(pp, {"socket_coordinate", "coordinate"}, coordName)) {
                    auto it = dofIndexByName.find(coordName);
                    if (it != dofIndexByName.end()) {
                        ipp.conditionDofIndex = it->second;
                        ipp.conditionMin = std::min(range[0], range[1]);
                        ipp.conditionMax = std::max(range[0], range[1]);
                        gated = true;
                    }
                }
                nCond++;
                if (!gated) nCondUngated++;
            } else if (isPlain) {
                nPlain++;
            }

            mp.points.push_back(ipp);
        }

        // PathWrap geometry (WrapCylinder / WrapEllipsoid) is not implemented:
        // those muscles cut straight through bone instead of wrapping. Count
        // the references so nobody mistakes this path set for complete.
        NSInteger nWraps = 0;
        if (auto* wrapSet = geoPath->FirstChildElement("PathWrapSet")) {
            if (auto* wrapObjects = wrapSet->FirstChildElement("objects")) {
                for (auto* w = wrapObjects->FirstChildElement("PathWrap");
                     w; w = w->NextSiblingElement("PathWrap")) {
                    nWraps++;
                }
            }
        }

        if (mp.points.size() >= 2 && mp.maxIsometricForce > 0) {
            _musclePaths.push_back(mp);

            // Fold counters in only for retained muscles so the report
            // describes the geometry actually in use.
            report.musclesParsed += 1;
            report.pathPointsParsed += nPlain;
            report.conditionalPathPointsParsed += nCond;
            report.conditionalPathPointsSkipped += nCondSkipped;
            report.conditionalPathPointsUnresolvedCoordinate += nCondUngated;
            report.movingPathPointsParsed += nMoving;
            report.movingPathPointsApproximated += nMovingApprox;
            report.movingPathPointsSkipped += nMovingSkipped;
            report.unknownPathPointElementsSkipped += nUnknown;
            report.unmodelledPathWraps += nWraps;
            if (!tendonSlackParsed) {
                [defaultedTendonSlack
                    addObject:[NSString stringWithUTF8String:mp.name.c_str()]];
            }
        }
    }
    }  // end muscle-class loop

    report.musclesWithDefaultedTendonSlackLength = [defaultedTendonSlack copy];
    _fidelityReport = report;

    // Validate that every PathPoint resolves to a real body in the
    // shared skeleton. If not, the muscle-length FK will silently
    // fall back to raw local offsets and the muscle's moment arm
    // will be identically zero, pinning it to activation lower bound.
    // Collect the violations so the user sees them once at load time
    // instead of discovering them as "that muscle never activates".
    std::set<std::string> unresolvedBodies;
    std::map<std::string, int> unresolvedCounts;
    int unresolvedMuscles = 0;
    for (const auto& mp : _musclePaths) {
        bool anyMiss = false;
        for (const auto& pp : mp.points) {
            if (_skeleton->getBodyNode(pp.bodyName) == nullptr) {
                unresolvedBodies.insert(pp.bodyName);
                unresolvedCounts[pp.bodyName]++;
                anyMiss = true;
            }
        }
        if (anyMiss) unresolvedMuscles++;
    }

    _loaded = !_musclePaths.empty();
    NSLog(@"MomentArmComputer: Parsed %lu muscle paths (skeleton has %lu bodies)",
          (unsigned long)_musclePaths.size(),
          (unsigned long)_skeleton->getNumBodyNodes());
    if (!unresolvedBodies.empty()) {
        NSMutableArray<NSString *> *bodyList = [NSMutableArray array];
        for (const auto& name : unresolvedBodies) {
            int count = unresolvedCounts[name];
            [bodyList addObject:
                [NSString stringWithFormat:@"%s(×%d)", name.c_str(), count]];
        }
        NSLog(@"MomentArmComputer: ⚠ %d muscles reference %lu unresolved body names: %@",
              unresolvedMuscles,
              (unsigned long)unresolvedBodies.size(),
              [bodyList componentsJoinedByString:@", "]);
        NSLog(@"MomentArmComputer: ⚠ Those muscles will have zero moment arm "
              @"and will be pinned to activation lower bound. "
              @"Check for mismatched body names between muscle path points "
              @"and skeleton (socket path prefix handling, locale differences, "
              @"newer OpenSim XML formats).");
    }

    NSLog(@"MomentArmComputer: geometry fidelity — %@", report.summary);
    if (report.unmodelledPathWraps > 0) {
        NSLog(@"MomentArmComputer: ⚠ %ld PathWrap references are NOT modelled. "
              @"Those muscles take a straight-line shortcut where the real path "
              @"wraps around bone, so their L_MT and moment arms are wrong "
              @"(worst at flexed poses, where the sign can flip).",
              (long)report.unmodelledPathWraps);
    }
    if (report.movingPathPointsApproximated > 0) {
        NSLog(@"MomentArmComputer: ⚠ %ld MovingPathPoints evaluated by linear "
              @"interpolation between cubic-spline control points.",
              (long)report.movingPathPointsApproximated);
    }
    if (report.conditionalPathPointsUnresolvedCoordinate > 0) {
        NSLog(@"MomentArmComputer: ⚠ %ld ConditionalPathPoints kept "
              @"unconditionally — their gating coordinate is not a DOF of this "
              @"skeleton, so the <range> test is never applied.",
              (long)report.conditionalPathPointsUnresolvedCoordinate);
    }
    if (report.musclesWithDefaultedTendonSlackLength.count > 0) {
        NSLog(@"MomentArmComputer: ⚠ %lu muscles fell back to tendon_slack_length=0 "
              @"(fiber length becomes L_MT/cos α — force-length curve is "
              @"mis-scaled for them): %@",
              (unsigned long)report.musclesWithDefaultedTendonSlackLength.count,
              [report.musclesWithDefaultedTendonSlackLength
                  componentsJoinedByString:@", "]);
    }

    [self refreshConditionalPathPointActivity];
    return _loaded;
}

- (NSInteger)numMuscles {
    return (NSInteger)_musclePaths.size();
}

- (NSArray<NSString *> *)muscleNames {
    NSMutableArray *names = [NSMutableArray array];
    for (const auto& mp : _musclePaths) {
        [names addObject:[NSString stringWithUTF8String:mp.name.c_str()]];
    }
    return names;
}

- (nullable MusclePathData *)musclePathDataForName:(NSString *)name {
    std::string nameStr([name UTF8String]);
    for (const auto& mp : _musclePaths) {
        if (mp.name == nameStr) {
            NSMutableArray *points = [NSMutableArray array];
            for (const auto& pp : mp.points) {
                Eigen::Vector3s offset = localOffsetForPathPoint(pp, _skeleton.get());
                [points addObject:[[MusclePathPoint alloc]
                    initWithBody:[NSString stringWithUTF8String:pp.bodyName.c_str()]
                    x:offset.x() y:offset.y() z:offset.z()]];
            }
            return [[MusclePathData alloc] initWithName:name
                                             pathPoints:points
                                      maxIsometricForce:mp.maxIsometricForce
                                   optimalFiberLength:mp.optimalFiberLength
                                        pennationAngle:mp.pennationAngle];
        }
    }
    return nil;
}

/// Latch which ConditionalPathPoints are active at the skeleton's CURRENT pose.
/// Deliberately NOT called from inside the finite-difference loop: re-evaluating
/// the range test at q ± eps lets a via point switch state across a 1e-4 rad
/// stencil, which turns a finite path-length step into a moment arm of order
/// (segment length / 2e-4) — metres, not centimetres.
- (void)refreshConditionalPathPointActivity {
    if (!_skeleton) return;
    const NSInteger nDofs = (NSInteger)_skeleton->getNumDofs();
    for (auto& mp : _musclePaths) {
        for (auto& pp : mp.points) {
            if (pp.conditionDofIndex < 0 || pp.conditionDofIndex >= nDofs) {
                pp.active = true;  // unconditional, or coordinate never resolved
                continue;
            }
            const double q = _skeleton->getDof(pp.conditionDofIndex)->getPosition();
            pp.active = (q >= pp.conditionMin && q <= pp.conditionMax);
        }
    }
}

/// Compute total musculotendon path length for a muscle at the current skeleton configuration.
- (double)computeMuscleLengthForIndex:(NSInteger)muscleIdx {
    if (!_skeleton || muscleIdx < 0 || muscleIdx >= (NSInteger)_musclePaths.size()) return 0;

    const auto& mp = _musclePaths[muscleIdx];
    double totalLength = 0;

    // Inactive conditional via points drop out of the polyline entirely; the
    // segment then spans their neighbours, so the walk cannot use fixed indices.
    Eigen::Vector3s previous = Eigen::Vector3s::Zero();
    bool havePrevious = false;
    for (const auto& pp : mp.points) {
        if (!pp.active) continue;
        Eigen::Vector3s world = [self worldPositionForPathPoint:pp];
        if (havePrevious) totalLength += (world - previous).norm();
        previous = world;
        havePrevious = true;
    }

    return totalLength;
}

/// Transform a path point from body-local coordinates to world coordinates.
/// Load-time validation in `parseMusclePathsFromOsimPath:fromBridge:`
/// already reports every muscle with an unresolved path-point body, so
/// silent fallback here is still safe at runtime (no NSLog spam at 60 fps).
- (Eigen::Vector3s)worldPositionForPathPoint:(const InternalPathPoint&)pp {
    Eigen::Vector3s localOffset = localOffsetForPathPoint(pp, _skeleton.get());

    dynamics::BodyNode* body = _skeleton->getBodyNode(pp.bodyName);
    if (!body) return localOffset;  // already reported at load time

    Eigen::Isometry3s worldTransform = body->getWorldTransform();
    return worldTransform * localOffset;
}

- (nullable NSArray<NSNumber *> *)computeMomentArmsWithJointAngles:(NSArray<NSNumber *> *)jointAngles
                                                          dofNames:(NSArray<NSString *> *)dofNames {
    if (!_loaded || !_skeleton) return nil;

    NSInteger nMuscles = (NSInteger)_musclePaths.size();
    NSInteger nDOFs = dofNames.count;

    // Set skeleton to the given configuration
    Eigen::VectorXs q = _skeleton->getPositions();
    std::map<std::string, int> dofToSkeletonIdx;

    for (size_t i = 0; i < _skeleton->getNumDofs(); i++) {
        dofToSkeletonIdx[_skeleton->getDof(i)->getName()] = (int)i;
    }

    // Set the joint angles we have
    for (NSInteger i = 0; i < nDOFs; i++) {
        std::string dofName([dofNames[i] UTF8String]);
        auto it = dofToSkeletonIdx.find(dofName);
        if (it != dofToSkeletonIdx.end()) {
            q(it->second) = [jointAngles[i] doubleValue];
        }
    }
    _skeleton->setPositions(q);

    // Latch conditional via points at the UNPERTURBED pose and hold them for
    // the whole stencil, so dL/dq stays the derivative of a continuous path.
    [self refreshConditionalPathPointActivity];

    // Compute baseline muscle lengths
    std::vector<double> baselineLengths(nMuscles);
    for (NSInteger m = 0; m < nMuscles; m++) {
        baselineLengths[m] = [self computeMuscleLengthForIndex:m];
    }

    // Numerical differentiation: r_ij = -(L_i(q + eps*e_j) - L_i(q - eps*e_j)) / (2*eps)
    const double eps = 1e-4;  // radians
    NSMutableArray<NSNumber *> *momentArms = [NSMutableArray arrayWithCapacity:nMuscles * nDOFs];

    // Initialize with zeros
    for (NSInteger i = 0; i < nMuscles * nDOFs; i++) {
        [momentArms addObject:@(0.0)];
    }

    for (NSInteger j = 0; j < nDOFs; j++) {
        std::string dofName([dofNames[j] UTF8String]);
        auto it = dofToSkeletonIdx.find(dofName);
        if (it == dofToSkeletonIdx.end()) continue;
        int skelIdx = it->second;

        // Perturb +eps
        Eigen::VectorXs qPlus = q;
        qPlus(skelIdx) += eps;
        _skeleton->setPositions(qPlus);

        std::vector<double> lengthsPlus(nMuscles);
        for (NSInteger m = 0; m < nMuscles; m++) {
            lengthsPlus[m] = [self computeMuscleLengthForIndex:m];
        }

        // Perturb -eps
        Eigen::VectorXs qMinus = q;
        qMinus(skelIdx) -= eps;
        _skeleton->setPositions(qMinus);

        std::vector<double> lengthsMinus(nMuscles);
        for (NSInteger m = 0; m < nMuscles; m++) {
            lengthsMinus[m] = [self computeMuscleLengthForIndex:m];
        }

        // Compute moment arms: r = -dL/dq
        for (NSInteger m = 0; m < nMuscles; m++) {
            double r = -(lengthsPlus[m] - lengthsMinus[m]) / (2.0 * eps);
            momentArms[m * nDOFs + j] = @(r);
        }
    }

    // Restore original configuration
    _skeleton->setPositions(q);

    return momentArms;
}

- (NSArray<NSNumber *> *)currentMuscleLengths {
    if (!_loaded || !_skeleton) return @[];
    [self refreshConditionalPathPointActivity];
    NSMutableArray<NSNumber *> *lengths =
        [NSMutableArray arrayWithCapacity:_musclePaths.size()];
    for (NSInteger i = 0; i < (NSInteger)_musclePaths.size(); i++) {
        [lengths addObject:@([self computeMuscleLengthForIndex:i])];
    }
    return lengths;
}

- (NSArray<NSNumber *> *)maxIsometricForces {
    NSMutableArray<NSNumber *> *forces =
        [NSMutableArray arrayWithCapacity:_musclePaths.size()];
    for (const auto& mp : _musclePaths) [forces addObject:@(mp.maxIsometricForce)];
    return forces;
}

- (NSArray<NSNumber *> *)optimalFiberLengths {
    NSMutableArray<NSNumber *> *lens =
        [NSMutableArray arrayWithCapacity:_musclePaths.size()];
    for (const auto& mp : _musclePaths) [lens addObject:@(mp.optimalFiberLength)];
    return lens;
}

- (NSArray<NSNumber *> *)tendonSlackLengths {
    NSMutableArray<NSNumber *> *lens =
        [NSMutableArray arrayWithCapacity:_musclePaths.size()];
    for (const auto& mp : _musclePaths) [lens addObject:@(mp.tendonSlackLength)];
    return lens;
}

- (NSArray<NSNumber *> *)pennationAngles {
    NSMutableArray<NSNumber *> *angs =
        [NSMutableArray arrayWithCapacity:_musclePaths.size()];
    for (const auto& mp : _musclePaths) [angs addObject:@(mp.pennationAngle)];
    return angs;
}

- (NSArray<NSNumber *> *)muscleEndpointsWorld {
    NSMutableArray<NSNumber *> *out =
        [NSMutableArray arrayWithCapacity:_musclePaths.size() * 6];
    if (!_loaded || !_skeleton) return out;
    for (const auto& mp : _musclePaths) {
        if (mp.points.size() < 2) {
            for (int i = 0; i < 6; i++) [out addObject:@(0.0)];
            continue;
        }
        Eigen::Vector3s a = [self worldPositionForPathPoint:mp.points.front()];
        Eigen::Vector3s b = [self worldPositionForPathPoint:mp.points.back()];
        [out addObject:@(a.x())]; [out addObject:@(a.y())]; [out addObject:@(a.z())];
        [out addObject:@(b.x())]; [out addObject:@(b.y())]; [out addObject:@(b.z())];
    }
    return out;
}

@end
