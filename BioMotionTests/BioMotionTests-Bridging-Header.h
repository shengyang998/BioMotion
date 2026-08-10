#import "NimbleBridge.h"

NS_ASSUME_NONNULL_BEGIN

/// Test-only access to legacy inverse-dynamics diagnostics.
///
/// The plain selector assumes zero external force; the near-CoP selector lacks
/// validated foot-contact mechanics. Product code exposes neither raw path.
/// Selected numerical characterization tests call this category so their old
/// metrics remain measurable without widening the production API.
@interface NimbleBridge (UnvalidatedDynamicsDiagnostics)
- (nullable NimbleIDResult *)solveUnvalidatedIDForDiagnosticsWithJointAngles:(NSArray<NSNumber *> *)jointAngles
                                                                jointVelocities:(NSArray<NSNumber *> *)jointVelocities
                                                            jointAccelerations:(NSArray<NSNumber *> *)jointAccelerations;
- (nullable NimbleIDResult *)solveUnvalidatedIDGRFForDiagnosticsWithJointAngles:(NSArray<NSNumber *> *)jointAngles
                                                                  jointVelocities:(NSArray<NSNumber *> *)jointVelocities
                                                              jointAccelerations:(NSArray<NSNumber *> *)jointAccelerations;
@end

NS_ASSUME_NONNULL_END
