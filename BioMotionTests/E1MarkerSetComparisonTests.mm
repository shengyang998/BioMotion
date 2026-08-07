// E1 — marker-set observability comparison on synthetic ground truth.
//
// Implements the FROZEN pre-registration at
//   labs/sam-3d-body/findings/E1_PREREGISTRATION.md
// verbatim. Nothing here may be re-thresholded after a result is read; the
// PASS/FAIL verdict is produced exclusively by experiments/e1_check_gates.py.
//
// ObjC++ (not Swift) so it can call nimblephysics directly, the same way
// BioMotionTests/IKSolverInternalsTests.mm does. It does not modify any shipped
// code path.
//
// Output: a JSON blob written to NSTemporaryDirectory(); the absolute path is
// printed as `E1|RESULTS|<path>`.

#import <XCTest/XCTest.h>

#import "NimbleBridge.h"
#import "NimbleBridge+Internal.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "dart/dynamics/Skeleton.hpp"
#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/DegreeOfFreedom.hpp"
#include "dart/math/MathTypes.hpp"
#include "dart/math/IKSolver.hpp"

#include <Eigen/Dense>
#include <Eigen/SVD>

using namespace dart;

namespace e1 {

// ---------------------------------------------------------------------------
// section 5.1 — q_true. Defined BY COORDINATE NAME. Mirrors experiments/e1_qtrue.py.
// ---------------------------------------------------------------------------

static const int kFrames = 120;
static const double kDt = 1.0 / 30.0;
static const int kBurnIn = 10;       // section 6.2
static const int kSgHalf = 4;
static const int kSgCenterLo = 14;   // section 6.2
static const int kSgCenterHi = 115;

static const char* kIvdJoints[17] = {
    "L5_S1", "L4_L5", "L3_L4", "L2_L3", "L1_L2", "T12_L1",
    "T11_T12", "T10_T11", "T9_T10", "T8_T9", "T7_T8", "T6_T7",
    "T5_T6", "T4_T5", "T3_T4", "T2_T3", "T1_T2",
};

static double ivdFeAmplitude() { return (30.0 / 17.0) * M_PI / 180.0; }

static std::map<std::string, double> qTrueFrame(int k) {
  const double u = (double)k / 119.0;
  const double th = 4.0 * M_PI * u;
  const double s = 0.5 * (1.0 - std::cos(2.0 * M_PI * u));
  const double pi = M_PI;
  std::map<std::string, double> q;

  q["pelvis_tx"] = 0.30 * u;
  q["pelvis_ty"] = 0.93 + 0.010 * std::sin(2.0 * th);
  q["pelvis_tz"] = 0.05 * std::sin(2.0 * pi * u);
  q["pelvis_tilt"] = 0.05 * std::sin(2.0 * pi * u);
  q["pelvis_list"] = 0.04 * std::sin(th);
  q["pelvis_rotation"] = 0.06 * std::sin(th);

  q["hip_flexion_r"] = 0.10 + 0.35 * std::sin(th);
  q["hip_flexion_l"] = 0.10 + 0.35 * std::sin(th + pi);
  q["hip_adduction_r"] = 0.05 * std::sin(th);
  q["hip_adduction_l"] = 0.05 * std::sin(th + pi);
  q["hip_rotation_r"] = 0.04 * std::sin(th);
  q["hip_rotation_l"] = 0.04 * std::sin(th + pi);
  q["knee_angle_r"] = 0.05 + 0.30 * (1.0 - std::cos(th));
  q["knee_angle_l"] = 0.05 + 0.30 * (1.0 - std::cos(th + pi));
  q["ankle_angle_r"] = 0.20 * std::sin(th + pi / 2.0);
  q["ankle_angle_l"] = 0.20 * std::sin(th + 3.0 * pi / 2.0);
  q["subtalar_angle_r"] = 0.05 * std::sin(th);
  q["subtalar_angle_l"] = 0.05 * std::sin(th + pi);
  q["mtp_angle_r"] = 0.0;
  q["mtp_angle_l"] = 0.0;
  q["elbow_flex_r"] = 0.40 + 0.30 * std::sin(th + pi);
  q["elbow_flex_l"] = 0.40 + 0.30 * std::sin(th);
  q["pro_sup_r"] = 0.30 + 0.10 * std::sin(th);
  q["pro_sup_l"] = 0.30 + 0.10 * std::sin(th + pi);
  q["wrist_flex_r"] = 0.0;
  q["wrist_flex_l"] = 0.0;
  q["wrist_dev_r"] = 0.0;
  q["wrist_dev_l"] = 0.0;

  q["T1_head_neck_FE"] = 0.10 * s;
  q["T1_head_neck_LB"] = 0.03 * std::sin(2.0 * pi * u);
  q["T1_head_neck_AR"] = 0.03 * std::sin(4.0 * pi * u);

  const double a = ivdFeAmplitude();
  for (int i = 0; i < 17; i++) {
    std::string j(kIvdJoints[i]);
    q[j + "_FE"] = a * s;
    q[j + "_LB"] = 0.0;
    q[j + "_AR"] = 0.0;
  }

  q["Abs_FE"] = 0.0;
  q["Abs_LB"] = 0.0;
  q["Abs_AR"] = 0.0;

  const char* sides[2] = {"L", "R"};
  const char* axes[3] = {"X", "Y", "Z"};
  for (int i = 1; i <= 12; i++)
    for (int sI = 0; sI < 2; sI++)
      for (int aI = 0; aI < 3; aI++) {
        char buf[64];
        snprintf(buf, sizeof(buf), "T%d_r%d%s_%s", i, i, sides[sI], axes[aI]);
        q[std::string(buf)] = 0.0;
      }
  const char* stern[6] = {"SternumRotX", "SternumRotY", "SternumRotZ",
                          "SternumX", "SternumY", "SternumZ"};
  for (int i = 0; i < 6; i++) q[std::string(stern[i])] = 0.0;
  return q;
}

// ---------------------------------------------------------------------------
// section 5.2 — counter-based noise. std::normal_distribution is FORBIDDEN.
// ---------------------------------------------------------------------------

static inline uint64_t splitmix64(uint64_t v) {
  v += 0x9E3779B97F4A7C15ULL;
  v = (v ^ (v >> 30)) * 0xBF58476D1CE4E5B9ULL;
  v = (v ^ (v >> 27)) * 0x94D049BB133111EBULL;
  return v ^ (v >> 31);
}

static inline double gaussZ(int seed, int k, int j, int axis) {
  uint64_t key = (uint64_t)seed;
  key = key * 1000003ULL + (uint64_t)k;
  key = key * 1000003ULL + (uint64_t)j;
  key = key * 1000003ULL + (uint64_t)axis;
  uint64_t x1 = splitmix64(key);
  uint64_t x2 = splitmix64(key + 0x5DEECE66DULL);
  double u1 = (double)(x1 >> 11) * 0x1.0p-53;
  if (u1 < 0x1.0p-53) u1 = 0x1.0p-53;
  double u2 = (double)(x2 >> 11) * 0x1.0p-53;
  return std::sqrt(-2.0 * std::log(u1)) * std::cos(2.0 * M_PI * u2);
}

// ---------------------------------------------------------------------------
// Marker sets (sections 4.1 / 4.2)
// ---------------------------------------------------------------------------

struct MarkerSpec {
  std::string label;
  std::string body;
  double ox, oy, oz;
};

static std::string slotKey(const MarkerSpec& m) {
  char buf[256];
  snprintf(buf, sizeof(buf), "%s@%.6f,%.6f,%.6f", m.body.c_str(), m.ox, m.oy, m.oz);
  return std::string(buf);
}

/// section 4.1 — the 20 ARKit virtual markers of NimbleBridge.mm:345-390
/// (cyclist/FullBody branch), verbatim. FullBody.osim defines no marker with
/// any of these names, so the virtual table is exactly what the bridge installs.
static std::vector<MarkerSpec> armAMarkers() {
  return {
      {"PELVIS", "pelvis", 0, 0, 0},
      {"LHJC", "femur_l", 0, 0, 0},    {"RHJC", "femur_r", 0, 0, 0},
      {"LKJC", "tibia_l", 0, 0, 0},    {"RKJC", "tibia_r", 0, 0, 0},
      {"LAJC", "talus_l", 0, 0, 0},    {"RAJC", "talus_r", 0, 0, 0},
      {"LTOE", "toes_l", 0, 0, 0},     {"RTOE", "toes_r", 0, 0, 0},
      {"SPINE_L", "lumbar3", 0, 0, 0}, {"SPINE_M", "thoracic7", 0, 0, 0},
      {"C7", "thoracic1", 0, 0, 0},    {"NECK", "head_neck", 0, 0, 0},
      {"HEAD", "head_neck", 0, 0.15, 0},
      {"LSJC", "humerus_l", 0, 0, 0},  {"RSJC", "humerus_r", 0, 0, 0},
      {"LEJC", "ulna_l", 0, 0, 0},     {"REJC", "ulna_r", 0, 0, 0},
      {"LWJC", "hand_l", 0, 0, 0},     {"RWJC", "hand_r", 0, 0, 0},
  };
}

/// section 4.2 — MHR_POS_PLUS: 25 bodies, one marker each at the body origin,
/// except head_neck at (0, 0.15, 0).
static std::vector<MarkerSpec> armBMarkers() {
  const char* bodies[25] = {
      "pelvis", "lumbar5", "lumbar3", "lumbar2", "thoracic12", "thoracic7",
      "thoracic5", "thoracic1", "head_neck", "femur_l", "femur_r", "tibia_l",
      "tibia_r", "talus_l", "talus_r", "calcn_l", "calcn_r", "toes_l", "toes_r",
      "humerus_l", "humerus_r", "ulna_l", "ulna_r", "radius_l", "radius_r"};
  std::vector<MarkerSpec> out;
  for (int i = 0; i < 25; i++) {
    std::string b(bodies[i]);
    if (b == "head_neck") out.push_back({"MHR_" + b, b, 0, 0.15, 0});
    else out.push_back({"MHR_" + b, b, 0, 0, 0});
  }
  return out;
}

/// section 4.6 — bounds the body-origin confound: every marker +5 cm along local +Y.
static std::vector<MarkerSpec> offsetBy5cm(const std::vector<MarkerSpec>& in) {
  std::vector<MarkerSpec> out = in;
  for (auto& m : out) m.oy += 0.05;
  return out;
}

struct ArmSpec {
  std::string key;
  std::vector<MarkerSpec> markers;
  std::string solver;   // "plain" | "masked" | "damped"
  std::vector<std::string> mask;
  double sigma;
  double mu;
  bool singleSeed;
};

// ---------------------------------------------------------------------------
// Raw-loop dense linear algebra for the damped arms (section 4.5). Plain loops
// rather than Eigen expressions because this translation unit is compiled at
// -O0 in the Debug configuration the pre-registration froze, where Eigen's
// expression templates are ~2 orders of magnitude slower. IEEE double
// throughout; no fast-math, no reassociation.
// ---------------------------------------------------------------------------

/// Cholesky-solves A x = b in place. A is n*n column-major, lower triangle used.
static bool cholSolveInPlace(std::vector<double>& A, std::vector<double>& b, int n) {
  for (int c = 0; c < n; c++) {
    double d = A[(size_t)c * n + c];
    for (int k = 0; k < c; k++) {
      double v = A[(size_t)k * n + c];
      d -= v * v;
    }
    if (!(d > 0.0)) return false;
    d = std::sqrt(d);
    A[(size_t)c * n + c] = d;
    for (int r = c + 1; r < n; r++) {
      double s = A[(size_t)c * n + r];
      for (int k = 0; k < c; k++) s -= A[(size_t)k * n + r] * A[(size_t)k * n + c];
      A[(size_t)c * n + r] = s / d;
    }
  }
  for (int r = 0; r < n; r++) {
    double s = b[(size_t)r];
    for (int k = 0; k < r; k++) s -= A[(size_t)k * n + r] * b[(size_t)k];
    b[(size_t)r] = s / A[(size_t)r * n + r];
  }
  for (int r = n - 1; r >= 0; r--) {
    double s = b[(size_t)r];
    for (int k = r + 1; k < n; k++) s -= A[(size_t)r * n + k] * b[(size_t)k];
    b[(size_t)r] = s / A[(size_t)r * n + r];
  }
  return true;
}

// ---------------------------------------------------------------------------
// Savitzky-Golay, BioMotion/Nimble/SavitzkyGolayFilter.swift:28-33
// ---------------------------------------------------------------------------

static const double kSgAcc[9] = {28.0, 7.0, -8.0, -17.0, -20.0, -17.0, -8.0, 7.0, 28.0};
static const double kSgVel[9] = {86.0, -142.0, -193.0, -126.0, 0.0, 126.0, 193.0, 142.0, -86.0};

static void sgDerive(const std::vector<Eigen::VectorXs>& q, int center, int n,
                     Eigen::VectorXs* dq, Eigen::VectorXs* ddq) {
  dq->setZero(n);
  ddq->setZero(n);
  for (int i = 0; i < 9; i++) {
    const Eigen::VectorXs& s = q[(size_t)(center - kSgHalf + i)];
    for (int c = 0; c < n; c++) {
      (*dq)(c) += kSgVel[i] / 1188.0 * s(c);
      (*ddq)(c) += kSgAcc[i] / 462.0 * s(c);
    }
  }
  (*dq) /= kDt;
  (*ddq) /= (kDt * kDt);
}

static double rmsOverSet(const std::vector<Eigen::VectorXs>& err, int lo, int hi,
                         const std::vector<int>& idx) {
  double ss = 0.0;
  long cnt = 0;
  for (int k = lo; k <= hi; k++)
    for (int i : idx) { double e = (double)err[(size_t)k](i); ss += e * e; cnt++; }
  return cnt ? std::sqrt(ss / (double)cnt) : std::numeric_limits<double>::quiet_NaN();
}

static double medianOf(std::vector<double> v) {
  if (v.empty()) return std::numeric_limits<double>::quiet_NaN();
  std::sort(v.begin(), v.end());
  size_t m = v.size() / 2;
  return (v.size() % 2) ? v[m] : 0.5 * (v[m - 1] + v[m]);
}

static math::IKConfig e1Config(int numMarkers) {
  math::IKConfig c;                    // nimble defaults
  c.setMaxRestarts(1);                 // frozen DEVIATION, section 5.3
  c.setLossLowerBound((s_t)((double)numMarkers * 0.02 * 0.02));
  return c;
}

// ---------------------------------------------------------------------------
// JSON emission (hand-rolled so doubles keep %.17g)
// ---------------------------------------------------------------------------

static std::string jnum(double v) {
  if (!std::isfinite(v)) return "null";
  char buf[48];
  snprintf(buf, sizeof(buf), "%.17g", v);
  return std::string(buf);
}

static std::string jstr(const std::string& s) {
  std::string o = "\"";
  for (char c : s) {
    if (c == '"' || c == '\\') { o += '\\'; o += c; }
    else if (c == '\n') o += "\\n";
    else o += c;
  }
  return o + "\"";
}

static std::string jnums(const std::vector<double>& v) {
  std::string o = "[";
  for (size_t i = 0; i < v.size(); i++) { if (i) o += ","; o += jnum(v[i]); }
  return o + "]";
}

static std::string jints(const std::vector<int>& v) {
  std::string o = "[";
  for (size_t i = 0; i < v.size(); i++) { if (i) o += ","; o += std::to_string(v[i]); }
  return o + "]";
}

static std::string jstrs(const std::vector<std::string>& v) {
  std::string o = "[";
  for (size_t i = 0; i < v.size(); i++) { if (i) o += ","; o += jstr(v[i]); }
  return o + "]";
}

}  // namespace e1

// ---------------------------------------------------------------------------

@interface E1MarkerSetComparisonTests : XCTestCase
@end

@implementation E1MarkerSetComparisonTests {
  NimbleBridge* _bridge;
  std::shared_ptr<dynamics::Skeleton> _skel;
  int _n;
  std::vector<std::string> _dofNames;
  std::map<std::string, int> _dofIndex;
  std::vector<Eigen::VectorXs> _qTrue;
  Eigen::VectorXs _seedDefault;
  std::map<std::string, std::vector<int>> _sets;
  std::vector<e1::MarkerSpec> _slots;
  std::map<std::string, int> _slotIndex;
  std::vector<std::vector<Eigen::Vector3s>> _slotPos;
  std::vector<std::string> _unreachableNames;
  std::vector<std::string> _notes;
}

- (void)setUp {
  [super setUp];
  _bridge = [[NimbleBridge alloc] init];
  NSString* path = [[NSBundle bundleForClass:[self class]] pathForResource:@"FullBody"
                                                                    ofType:@"osim"];
  XCTAssertNotNil(path, @"FullBody.osim missing from the test bundle");
  XCTAssertTrue([_bridge loadModelFromPath:path]);
  _skel = [_bridge sharedSkeleton];
  XCTAssertTrue(_skel != nullptr);
  _n = (int)_skel->getNumDofs();
  _dofNames.clear();
  _dofIndex.clear();
  for (int i = 0; i < _n; i++) {
    std::string nm = _skel->getDof(i)->getName();
    _dofNames.push_back(nm);
    _dofIndex[nm] = i;
  }
  _seedDefault = _skel->getPositions();
  printf("E1|MODEL|dofs=%d\n", _n);
  fflush(stdout);
}

// ---------------------------------------------------------------------------

- (std::vector<int>)indicesFor:(const std::vector<std::string>&)names {
  std::vector<int> out;
  for (const auto& nm : names) {
    auto it = _dofIndex.find(nm);
    if (it != _dofIndex.end()) out.push_back(it->second);
    else {
      printf("E1|WARN|set name not a DOF: %s\n", nm.c_str());
      _notes.push_back("coordinate-set name absent from model: " + nm);
    }
  }
  return out;
}

- (void)buildCoordinateSets {
  std::vector<std::string> ivdFe, ivdOff, ivd51, rib72, stern6, abs3, neck3,
      limb19, pelT3, locked6, movrot82;
  for (int i = 0; i < 17; i++) {
    std::string j(e1::kIvdJoints[i]);
    ivdFe.push_back(j + "_FE");
    ivdOff.push_back(j + "_LB");
    ivdOff.push_back(j + "_AR");
  }
  ivd51 = ivdFe;
  ivd51.insert(ivd51.end(), ivdOff.begin(), ivdOff.end());
  const char* sides[2] = {"L", "R"};
  const char* axes[3] = {"X", "Y", "Z"};
  for (int i = 1; i <= 12; i++)
    for (int s = 0; s < 2; s++)
      for (int a = 0; a < 3; a++) {
        char buf[64];
        snprintf(buf, sizeof(buf), "T%d_r%d%s_%s", i, i, sides[s], axes[a]);
        rib72.push_back(buf);
      }
  stern6 = {"SternumRotX", "SternumRotY", "SternumRotZ",
            "SternumX", "SternumY", "SternumZ"};
  abs3 = {"Abs_FE", "Abs_LB", "Abs_AR"};
  neck3 = {"T1_head_neck_FE", "T1_head_neck_LB", "T1_head_neck_AR"};
  limb19 = {"pelvis_tilt", "pelvis_list", "pelvis_rotation",
            "hip_flexion_r", "hip_flexion_l", "hip_adduction_r", "hip_adduction_l",
            "hip_rotation_r", "hip_rotation_l", "knee_angle_r", "knee_angle_l",
            "ankle_angle_r", "ankle_angle_l", "subtalar_angle_r", "subtalar_angle_l",
            "elbow_flex_r", "elbow_flex_l", "pro_sup_r", "pro_sup_l"};
  pelT3 = {"pelvis_tx", "pelvis_ty", "pelvis_tz"};
  locked6 = {"mtp_angle_l", "mtp_angle_r", "wrist_flex_l", "wrist_flex_r",
             "wrist_dev_l", "wrist_dev_r"};
  std::vector<std::string> shoulder6 =
      {"shoulder_elv_r", "shoulder_elv_l", "shoulder_rot_r", "shoulder_rot_l",
       "elv_angle_r", "elv_angle_l"};
  movrot82 = ivd51;
  movrot82.insert(movrot82.end(), stern6.begin(), stern6.end());
  movrot82.push_back("T1_r1R_X");
  movrot82.push_back("T1_r1R_Y");
  movrot82.push_back("T1_r1R_Z");
  movrot82.insert(movrot82.end(), limb19.begin(), limb19.end());
  movrot82.insert(movrot82.end(), neck3.begin(), neck3.end());

  _sets["IVD_FE17"] = [self indicesFor:ivdFe];
  _sets["IVD_OFF34"] = [self indicesFor:ivdOff];
  _sets["IVD51"] = [self indicesFor:ivd51];
  _sets["RIB72"] = [self indicesFor:rib72];
  _sets["STERNUM6"] = [self indicesFor:stern6];
  _sets["ABS3"] = [self indicesFor:abs3];
  _sets["NECK3"] = [self indicesFor:neck3];
  _sets["LIMB_ROT19"] = [self indicesFor:limb19];
  _sets["PELVIS_T3"] = [self indicesFor:pelT3];
  _sets["LOCKED6"] = [self indicesFor:locked6];
  // The six glenohumeral coordinates. They did not exist as DOFs when this
  // experiment ran: FullBody.osim's shoulder CustomJoints had non-orthogonal
  // rotation axes, so nimble's crash-guard substituted a WeldJoint and the
  // model parsed to 163 DOFs. The 2026-08-06 axis unit-snap made them real,
  // and the patellofemoral weld removed the two knee_angle_*_beta coordinates,
  // so the model is now 169. Without this block the partition below covers 163
  // of 169 and the completeness assertion fails for a reason that has nothing
  // to do with E1.
  _sets["SHOULDER6"] = [self indicesFor:shoulder6];
  _sets["MOVROT82"] = [self indicesFor:movrot82];

  XCTAssertEqual((int)_sets["IVD_FE17"].size(), 17);
  XCTAssertEqual((int)_sets["IVD_OFF34"].size(), 34);
  XCTAssertEqual((int)_sets["IVD51"].size(), 51);
  XCTAssertEqual((int)_sets["RIB72"].size(), 72);
  XCTAssertEqual((int)_sets["STERNUM6"].size(), 6);
  XCTAssertEqual((int)_sets["ABS3"].size(), 3);
  XCTAssertEqual((int)_sets["NECK3"].size(), 3);
  XCTAssertEqual((int)_sets["LIMB_ROT19"].size(), 19);
  XCTAssertEqual((int)_sets["PELVIS_T3"].size(), 3);
  XCTAssertEqual((int)_sets["LOCKED6"].size(), 6);
  XCTAssertEqual((int)_sets["SHOULDER6"].size(), 6);
  XCTAssertEqual((int)_sets["MOVROT82"].size(), 82);

  std::set<int> cover;
  const char* blocks[9] = {"IVD51", "RIB72", "STERNUM6", "ABS3", "NECK3",
                           "LIMB_ROT19", "PELVIS_T3", "LOCKED6", "SHOULDER6"};
  for (int i = 0; i < 9; i++)
    for (int idx : _sets[blocks[i]]) cover.insert(idx);
  XCTAssertEqual((int)cover.size(), _n);
}

// ---------------------------------------------------------------------------

- (std::vector<std::pair<dynamics::BodyNode*, Eigen::Vector3s>>)
        resolve:(const std::vector<e1::MarkerSpec>&)specs
           kept:(std::vector<e1::MarkerSpec>*)kept
        missing:(std::vector<std::string>*)missing {
  std::vector<std::pair<dynamics::BodyNode*, Eigen::Vector3s>> out;
  for (const auto& m : specs) {
    dynamics::BodyNode* b = _skel->getBodyNode(m.body);
    if (b == nullptr) { if (missing) missing->push_back(m.body); continue; }
    out.push_back({b, Eigen::Vector3s(m.ox, m.oy, m.oz)});
    if (kept) kept->push_back(m);
  }
  return out;
}

// ---------------------------------------------------------------------------
// Solvers
// ---------------------------------------------------------------------------

- (void)solvePlain:(const std::vector<std::pair<dynamics::BodyNode*, Eigen::Vector3s>>&)mk
           targets:(const Eigen::VectorXs&)tgt
           weights:(const Eigen::VectorXs&)w {
  _skel->fitMarkersToWorldPositions(mk, tgt, w, false, e1::e1Config((int)mk.size()));
}

/// Reparameterised masked solve — the construction of
/// `NimbleBridge -solveMaskedIKWithMarkers:` (NimbleBridge.mm:744-799).
- (void)solveMasked:(const std::vector<std::pair<dynamics::BodyNode*, Eigen::Vector3s>>&)mk
            targets:(const Eigen::VectorXs&)tgt
            weights:(const Eigen::VectorXs&)w
            freeIdx:(const std::vector<int>&)freeIdx
             pinned:(const Eigen::VectorXs&)pinned {
  const int f = (int)freeIdx.size();
  auto gather = [freeIdx, f](const Eigen::VectorXs& full) {
    Eigen::VectorXs sub(f);
    for (int k = 0; k < f; k++) sub(k) = full(freeIdx[(size_t)k]);
    return sub;
  };
  auto scatter = [freeIdx, f, pinned](const Eigen::VectorXs& sub) {
    Eigen::VectorXs full = pinned;
    for (int k = 0; k < f; k++) full(freeIdx[(size_t)k]) = sub(k);
    return full;
  };
  Eigen::VectorXs initial = gather(_skel->getPositions());
  Eigen::VectorXs upper = gather(_skel->getPositionUpperLimits());
  Eigen::VectorXs lower = gather(_skel->getPositionLowerLimits());
  auto* skel = _skel.get();
  math::solveIK(
      initial, upper, lower, (int)mk.size() * 3,
      [skel, scatter, gather](const Eigen::VectorXs pos, bool clamp) {
        skel->setPositions(scatter(pos));
        if (clamp) {
          skel->clampPositionsToLimits();
          skel->setPositions(scatter(gather(skel->getPositions())));
          return gather(skel->getPositions());
        }
        return pos;
      },
      [skel, tgt, mk, w, freeIdx, f](Eigen::Ref<Eigen::VectorXs> diff,
                                     Eigen::Ref<Eigen::MatrixXs> jac) {
        diff = skel->getMarkerWorldPositions(mk) - tgt;
        for (int j = 0; j < w.size(); j++) diff.segment<3>(j * 3) *= w(j);
        Eigen::MatrixXs fullJac =
            skel->getMarkerWorldPositionsJacobianWrtJointPositions(mk);
        for (int k = 0; k < f; k++) jac.col(k) = fullJac.col(freeIdx[(size_t)k]);
      },
      [skel, gather](Eigen::Ref<Eigen::VectorXs> val) { val = gather(skel->getRandomPose()); },
      e1::e1Config((int)mk.size()));
}

/// section 4.5 — refineIK with the damped step
///   delta = (J^T J + lambda I + mu I)^-1 (J^T diff + mu (q - q_prev))
/// Everything else (loss, line search, lr schedule, clamp schedule, convergence
/// and termination tests, and the transpose fallback) is IKSolver.cpp:291-493
/// verbatim. The transpose branch is left untouched: the frozen formula has the
/// damped-least-squares structure and lambda IS that branch's parameter.
- (double)refineDamped:(const std::vector<std::pair<dynamics::BodyNode*, Eigen::Vector3s>>&)mk
               targets:(const Eigen::VectorXs&)tgt
               weights:(const Eigen::VectorXs&)w
                 qPrev:(const Eigen::VectorXs&)qPrev
                    mu:(double)mu
              maxSteps:(int)maxSteps
               outPose:(Eigen::VectorXs*)outPose
                config:(const math::IKConfig&)config {
  const int n = _n;
  const int m = (int)mk.size() * 3;
  const double lambda = (double)config.leastSquaresDamping;
  const double alpha = lambda + mu;
  auto* skel = _skel.get();

  auto setPosAndClamp = [skel](const Eigen::VectorXs& p, bool clamp) {
    skel->setPositions(p);
    if (clamp) { skel->clampPositionsToLimits(); return skel->getPositions(); }
    return p;
  };

  Eigen::VectorXs pos = setPosAndClamp(_skel->getPositions(), config.startClamped);
  Eigen::VectorXs diff(m);
  Eigen::MatrixXs J(m, n);
  double lastError = std::numeric_limits<double>::infinity();
  double lr = 1.0;
  bool useTranspose = false;
  bool clamp = config.startClamped;
  Eigen::VectorXs lastPos = pos;

  std::vector<double> A((size_t)n * (size_t)n), b((size_t)n);
  Eigen::VectorXs delta(n);

  for (int i = 0; i < maxSteps; i++) {
    if (i > maxSteps - 5) clamp = true;

    diff = _skel->getMarkerWorldPositions(mk) - tgt;
    for (int j = 0; j < w.size(); j++) diff.segment<3>(j * 3) *= w(j);
    J = _skel->getMarkerWorldPositionsJacobianWrtJointPositions(mk);
    double currentError = (double)diff.squaredNorm();

    if (i > 0) {
      double errorChange = currentError - lastError;
      if (currentError < 1e-21) { lastError = currentError; break; }
      if (errorChange > 0) {
        lr *= 0.5;
        if (lr < 1e-4) useTranspose = true;
        else if (!config.dontExitTranspose) useTranspose = false;
        if (config.lineSearch) pos = setPosAndClamp(lastPos, clamp);
        if (lr < 1e-10) { lastError = currentError; break; }
      } else if (errorChange > -(double)config.convergenceThreshold) {
        if (!useTranspose) {
          if (lr > 5e-5) lr = 5e-5;
          useTranspose = true;
        } else {
          if (!clamp) clamp = true;
          else break;
        }
      } else {
        lr *= 1.1;
        lastError = currentError;
      }
    }

    const double* Jd = J.data();   // column-major, m rows
    if (useTranspose) {
      for (int c = 0; c < n; c++) {
        const double* col = Jd + (size_t)c * (size_t)m;
        double s = 0.0;
        for (int r = 0; r < m; r++) s += col[r] * (double)diff(r);
        delta(c) = s;
      }
    } else {
      for (int c = 0; c < n; c++) {
        const double* cc = Jd + (size_t)c * (size_t)m;
        for (int r = c; r < n; r++) {
          const double* rc = Jd + (size_t)r * (size_t)m;
          double s = 0.0;
          for (int t = 0; t < m; t++) s += rc[t] * cc[t];
          A[(size_t)c * (size_t)n + (size_t)r] = s + (r == c ? alpha : 0.0);
        }
        double s = 0.0;
        for (int r = 0; r < m; r++) s += cc[r] * (double)diff(r);
        b[(size_t)c] = s + mu * ((double)pos(c) - (double)qPrev(c));
      }
      if (!e1::cholSolveInPlace(A, b, n)) {
        for (int c = 0; c < n; c++) b[(size_t)c] = 0.0;
        _notes.push_back("cholesky failed in refineDamped");
      }
      for (int c = 0; c < n; c++) delta(c) = b[(size_t)c];
    }
    lastPos = pos;
    pos = setPosAndClamp(pos - (lr * delta), clamp);
  }
  if (outPose) *outPose = pos;
  return lastError;
}

/// math::solveIK's enclosing structure at maxRestarts == 1: a 20-step probe
/// followed by the full refine (IKSolver.cpp:225-288).
- (void)solveDamped:(const std::vector<std::pair<dynamics::BodyNode*, Eigen::Vector3s>>&)mk
            targets:(const Eigen::VectorXs&)tgt
            weights:(const Eigen::VectorXs&)w
              qPrev:(const Eigen::VectorXs&)qPrev
                 mu:(double)mu {
  math::IKConfig cfg = e1::e1Config((int)mk.size());
  Eigen::VectorXs probePose;
  [self refineDamped:mk targets:tgt weights:w qPrev:qPrev mu:mu
            maxSteps:20 outPose:&probePose config:cfg];
  _skel->setPositions(probePose);
  _skel->clampPositionsToLimits();
  Eigen::VectorXs finalPose;
  [self refineDamped:mk targets:tgt weights:w qPrev:qPrev mu:mu
            maxSteps:cfg.maxStepCount outPose:&finalPose config:cfg];
}

// ---------------------------------------------------------------------------
// One arm x seed x seed-condition trajectory
// ---------------------------------------------------------------------------

- (std::string)runArm:(const e1::ArmSpec&)arm
                 seed:(int)seed
            condition:(const std::string&)cond
              tauTrue:(const std::vector<Eigen::VectorXs>&)tauTrue {
  std::vector<std::string> missing;
  std::vector<e1::MarkerSpec> kept;
  auto mk = [self resolve:arm.markers kept:&kept missing:&missing];
  const int nm = (int)mk.size();
  Eigen::VectorXs w = Eigen::VectorXs::Ones(nm);

  std::vector<int> freeIdx, maskedIdx;
  {
    std::set<std::string> ms(arm.mask.begin(), arm.mask.end());
    for (int i = 0; i < _n; i++) {
      if (ms.count(_dofNames[(size_t)i])) maskedIdx.push_back(i);
      else freeIdx.push_back(i);
    }
  }

  Eigen::VectorXs qSeed = (cond == "SEED-TRUTH") ? _qTrue[0] : _seedDefault;

  std::vector<Eigen::VectorXs> qhat((size_t)e1::kFrames);
  std::vector<double> markerLoss((size_t)e1::kFrames, 0.0);
  std::vector<int> coldFallbackFrames;
  bool nonFinite = false;
  double maxLockedAbs = 0.0, maxUnreachMove = 0.0, maxMaskedMove = 0.0;
  const double lossReject = (double)nm * 0.15 * 0.15;

  std::vector<int> unreachIdx;
  for (const auto& s : _unreachableNames) {
    auto it = _dofIndex.find(s);
    if (it != _dofIndex.end()) unreachIdx.push_back(it->second);
  }

  _skel->setPositions(qSeed);
  Eigen::VectorXs qPrev = qSeed;

  for (int k = 0; k < e1::kFrames; k++) {
    Eigen::VectorXs tgt(nm * 3);
    for (int i = 0; i < nm; i++) {
      const int j = _slotIndex[e1::slotKey(kept[(size_t)i])];
      Eigen::Vector3s p = _slotPos[(size_t)k][(size_t)j];
      for (int a = 0; a < 3; a++)
        tgt(i * 3 + a) = (s_t)((double)p(a) + arm.sigma * e1::gaussZ(seed, k, j, a));
    }
    if (k == 0) _skel->setPositions(qSeed);

    if (arm.solver == "plain") {
      [self solvePlain:mk targets:tgt weights:w];
    } else if (arm.solver == "masked") {
      Eigen::VectorXs pinned = _skel->getPositions();
      [self solveMasked:mk targets:tgt weights:w freeIdx:freeIdx pinned:pinned];
    } else {
      [self solveDamped:mk targets:tgt weights:w qPrev:qPrev mu:arm.mu];
    }

    Eigen::VectorXs q = _skel->getPositions();
    if (!q.allFinite()) nonFinite = true;
    qhat[(size_t)k] = q;

    Eigen::VectorXs d = _skel->getMarkerWorldPositions(mk) - tgt;
    markerLoss[(size_t)k] = (double)d.squaredNorm();
    if (markerLoss[(size_t)k] > lossReject) coldFallbackFrames.push_back(k);

    for (int i : _sets["LOCKED6"])
      maxLockedAbs = std::max(maxLockedAbs, std::abs((double)q(i)));
    for (int i : unreachIdx)
      maxUnreachMove = std::max(maxUnreachMove, std::abs((double)(q(i) - qSeed(i))));
    for (int i : maskedIdx)
      maxMaskedMove = std::max(maxMaskedMove, std::abs((double)(q(i) - qSeed(i))));

    qPrev = q;
  }

  // ---- metrics ----
  std::vector<Eigen::VectorXs> err((size_t)e1::kFrames);
  for (int k = 0; k < e1::kFrames; k++)
    err[(size_t)k] = qhat[(size_t)k] - _qTrue[(size_t)k];

  std::vector<std::string> setOrder = {"IVD_FE17", "IVD_OFF34", "IVD51", "RIB72",
                                       "STERNUM6", "ABS3", "NECK3", "LIMB_ROT19",
                                       "PELVIS_T3", "LOCKED6", "MOVROT82"};
  std::ostringstream E, W, ADD, T;
  E << "{"; W << "{"; ADD << "{"; T << "{";
  for (size_t si = 0; si < setOrder.size(); si++) {
    if (si) { E << ","; W << ","; }
    const std::vector<int>& idx = _sets[setOrder[si]];
    E << e1::jstr(setOrder[si]) << ":"
      << e1::jnum(e1::rmsOverSet(err, e1::kBurnIn, e1::kFrames - 1, idx));
    std::vector<double> wv;
    for (int k = e1::kBurnIn + 1; k < e1::kFrames; k++) {
      double ss = 0.0;
      for (int i : idx) {
        double dd = (double)((qhat[(size_t)k](i) - qhat[(size_t)(k - 1)](i)) -
                             (_qTrue[(size_t)k](i) - _qTrue[(size_t)(k - 1)](i)));
        ss += dd * dd;
      }
      wv.push_back(std::sqrt(ss));
    }
    W << e1::jstr(setOrder[si]) << ":" << e1::jnum(e1::medianOf(wv));
  }

  std::vector<double> addSs(setOrder.size(), 0.0), addCnt(setOrder.size(), 0.0);
  std::vector<double> tSs(setOrder.size(), 0.0), tCnt(setOrder.size(), 0.0);
  Eigen::VectorXs dqh(_n), ddqh(_n);
  for (int c = e1::kSgCenterLo; c <= e1::kSgCenterHi; c++) {
    e1::sgDerive(qhat, c, _n, &dqh, &ddqh);
    Eigen::VectorXs dqt(_n), ddqt(_n);
    e1::sgDerive(_qTrue, c, _n, &dqt, &ddqt);
    for (size_t si = 0; si < setOrder.size(); si++)
      for (int i : _sets[setOrder[si]]) {
        double dd = (double)(ddqh(i) - ddqt(i));
        addSs[si] += dd * dd;
        addCnt[si] += 1.0;
      }
    _skel->setPositions(qhat[(size_t)c]);
    _skel->setVelocities(dqh);
    Eigen::VectorXs tauHat = _skel->getInverseDynamics(ddqh);
    const Eigen::VectorXs& tt = tauTrue[(size_t)(c - e1::kSgCenterLo)];
    for (size_t si = 0; si < setOrder.size(); si++)
      for (int i : _sets[setOrder[si]]) {
        double dd = (double)(tauHat(i) - tt(i));
        tSs[si] += dd * dd;
        tCnt[si] += 1.0;
      }
  }
  for (size_t si = 0; si < setOrder.size(); si++) {
    if (si) { ADD << ","; T << ","; }
    ADD << e1::jstr(setOrder[si]) << ":" << e1::jnum(std::sqrt(addSs[si] / addCnt[si]));
    T << e1::jstr(setOrder[si]) << ":" << e1::jnum(std::sqrt(tSs[si] / tCnt[si]));
  }
  E << "}"; W << "}"; ADD << "}"; T << "}";

  // observable/null split, sampled at 5 frames (see protocol note in the report)
  std::ostringstream split;
  split << "{";
  {
    int frames[5] = {10, 30, 60, 90, 119};
    for (int fi = 0; fi < 5; fi++) {
      if (fi) split << ",";
      int k = frames[fi];
      _skel->setPositions(_qTrue[(size_t)k]);
      Eigen::MatrixXs J = _skel->getMarkerWorldPositionsJacobianWrtJointPositions(mk);
      Eigen::JacobiSVD<Eigen::MatrixXs> svd(J, Eigen::ComputeThinV);
      Eigen::VectorXs sv = svd.singularValues();
      Eigen::MatrixXs V = svd.matrixV();
      double obs2 = 0.0;
      const Eigen::VectorXs& e = err[(size_t)k];
      for (int i = 0; i < sv.size(); i++)
        if ((double)sv(i) > 1e-3) { double c = (double)V.col(i).dot(e); obs2 += c * c; }
      double tot2 = (double)e.squaredNorm();
      split << e1::jstr(std::to_string(k)) << ":{\"obs_norm\":" << e1::jnum(std::sqrt(obs2))
            << ",\"null_norm\":" << e1::jnum(std::sqrt(std::max(0.0, tot2 - obs2))) << "}";
    }
  }
  split << "}";

  double lossMax = *std::max_element(markerLoss.begin(), markerLoss.end());
  std::ostringstream o;
  o << "{\"E\":" << E.str() << ",\"W\":" << W.str() << ",\"A_dd\":" << ADD.str()
    << ",\"T\":" << T.str() << ",\"obs_null_split\":" << split.str()
    << ",\"nonfinite\":" << (nonFinite ? "true" : "false")
    << ",\"cold_fallback_frames\":" << e1::jints(coldFallbackFrames)
    << ",\"max_abs_locked_q\":" << e1::jnum(maxLockedAbs)
    << ",\"max_unreachable_move\":" << e1::jnum(maxUnreachMove)
    << ",\"max_masked_move\":" << e1::jnum(maxMaskedMove)
    << ",\"marker_loss_frame0\":" << e1::jnum(markerLoss[0])
    << ",\"marker_loss_median\":" << e1::jnum(e1::medianOf(markerLoss))
    << ",\"marker_loss_max\":" << e1::jnum(lossMax)
    << ",\"marker_rms_median_m\":" << e1::jnum(std::sqrt(e1::medianOf(markerLoss) / (double)nm))
    << ",\"markers_resolved\":" << nm
    << ",\"markers_missing\":" << e1::jstrs(missing) << "}";
  return o.str();
}

// ---------------------------------------------------------------------------
// Rank block (gate G0)
// ---------------------------------------------------------------------------

- (std::string)rankFor:(const std::vector<e1::MarkerSpec>&)specs {
  std::vector<std::string> missing;
  auto mk = [self resolve:specs kept:nullptr missing:&missing];
  std::vector<int> spineIdx = _sets["IVD51"];
  for (int i : _sets["ABS3"]) spineIdx.push_back(i);
  std::set<int> spineSet(spineIdx.begin(), spineIdx.end());
  std::vector<int> nonSpine;
  for (int i = 0; i < _n; i++) if (!spineSet.count(i)) nonSpine.push_back(i);

  const double taus[5] = {1e-1, 1e-2, 1e-3, 1e-6, 1e-12};
  const char* tauNames[5] = {"1e-1", "1e-2", "1e-3", "1e-6", "1e-12"};
  const int frames[5] = {0, 30, 60, 90, 119};

  auto countAbove = [](const Eigen::VectorXs& s, double t) {
    int c = 0;
    for (int i = 0; i < s.size(); i++) if ((double)s(i) > t) c++;
    return c;
  };

  std::ostringstream o;
  o << "{\"n_markers\":" << mk.size() << ",\"missing\":" << e1::jstrs(missing);
  for (int fi = 0; fi < 5; fi++) {
    int k = frames[fi];
    _skel->setPositions(_qTrue[(size_t)k]);
    Eigen::MatrixXs J = _skel->getMarkerWorldPositionsJacobianWrtJointPositions(mk);
    Eigen::JacobiSVD<Eigen::MatrixXs> svd(J);
    Eigen::VectorXs sv = svd.singularValues();

    Eigen::MatrixXs Jsp(J.rows(), (int)spineIdx.size());
    for (size_t c = 0; c < spineIdx.size(); c++) Jsp.col((int)c) = J.col(spineIdx[c]);
    Eigen::JacobiSVD<Eigen::MatrixXs> svdSp(Jsp);
    Eigen::VectorXs svSp = svdSp.singularValues();

    Eigen::MatrixXs Jns(J.rows(), (int)nonSpine.size());
    for (size_t c = 0; c < nonSpine.size(); c++) Jns.col((int)c) = J.col(nonSpine[c]);
    Eigen::JacobiSVD<Eigen::MatrixXs> svdNs(Jns);
    Eigen::VectorXs svNs = svdNs.singularValues();

    std::vector<double> svv, svspv;
    for (int i = 0; i < sv.size(); i++) svv.push_back((double)sv(i));
    for (int i = 0; i < svSp.size(); i++) svspv.push_back((double)svSp(i));

    std::vector<std::string> zeroCols;
    for (int c = 0; c < J.cols(); c++)
      if ((double)J.col(c).cwiseAbs().maxCoeff() == 0.0)
        zeroCols.push_back(_dofNames[(size_t)c]);

    o << "," << e1::jstr(std::to_string(k)) << ":{"
      << "\"rows\":" << J.rows() << ",\"cols\":" << J.cols()
      << ",\"R_all\":" << countAbove(sv, 1e-3)
      << ",\"R_spine\":" << countAbove(svSp, 1e-3)
      << ",\"R_nonspine\":" << countAbove(svNs, 1e-3)
      << ",\"R_spine_cond\":" << (countAbove(sv, 1e-3) - countAbove(svNs, 1e-3))
      << ",\"R_all_at_tau\":{";
    for (int t = 0; t < 5; t++) {
      if (t) o << ",";
      o << e1::jstr(tauNames[t]) << ":" << countAbove(sv, taus[t]);
    }
    o << "},\"R_spine_at_tau\":{";
    for (int t = 0; t < 5; t++) {
      if (t) o << ",";
      o << e1::jstr(tauNames[t]) << ":" << countAbove(svSp, taus[t]);
    }
    o << "},\"singular_values\":" << e1::jnums(svv)
      << ",\"spine_singular_values\":" << e1::jnums(svspv)
      << ",\"n_zero_columns\":" << zeroCols.size()
      << ",\"zero_column_names\":" << e1::jstrs(zeroCols) << "}";
  }
  o << "}";
  return o.str();
}

// ---------------------------------------------------------------------------
// Masks
// ---------------------------------------------------------------------------

- (std::vector<std::string>)xmlLocked {
  // Measured, not hard-coded: <locked>true</locked> is parsed into a zero-width
  // position limit (OpenSimParser.cpp:5923-5946).
  std::vector<std::string> out;
  Eigen::VectorXs lo = _skel->getPositionLowerLimits();
  Eigen::VectorXs hi = _skel->getPositionUpperLimits();
  for (int i = 0; i < _n; i++)
    if (std::abs((double)(hi(i) - lo(i))) < 1e-12) out.push_back(_dofNames[(size_t)i]);
  return out;
}

- (std::vector<std::string>)protectedGirdle {
  std::vector<std::string> p = {"SternumRotX", "SternumRotY", "SternumRotZ",
                                "SternumX", "SternumY", "SternumZ"};
  for (int i = 1; i <= 12; i++)
    for (const char* side : {"L", "R"}) {
      char buf[64];
      snprintf(buf, sizeof(buf), "T%d_r%d%s_X", i, i, side);
      p.push_back(buf);
    }
  return p;
}

- (std::vector<std::string>)runtimeMask {
  std::set<std::string> u;
  for (const auto& s : [self xmlLocked]) u.insert(s);
  for (const auto& s : _unreachableNames) u.insert(s);
  std::set<std::string> prot;
  for (const auto& s : [self protectedGirdle]) prot.insert(s);
  std::vector<std::string> out;
  for (const auto& s : u) if (!prot.count(s)) out.push_back(s);
  return out;
}

- (std::vector<std::string>)structuralMask {
  std::set<std::string> u;
  for (const auto& s : [self xmlLocked]) u.insert(s);
  for (const auto& s : _unreachableNames) u.insert(s);
  return std::vector<std::string>(u.begin(), u.end());
}

- (std::vector<std::string>)zeroColumnsUnder:(const std::vector<e1::MarkerSpec>&)specs {
  auto mk = [self resolve:specs kept:nullptr missing:nullptr];
  std::set<std::string> zeroAlways;
  const int frames[3] = {0, 60, 119};
  for (int fi = 0; fi < 3; fi++) {
    _skel->setPositions(_qTrue[(size_t)frames[fi]]);
    Eigen::MatrixXs J = _skel->getMarkerWorldPositionsJacobianWrtJointPositions(mk);
    std::set<std::string> z;
    for (int c = 0; c < J.cols(); c++)
      if ((double)J.col(c).cwiseAbs().maxCoeff() == 0.0) z.insert(_dofNames[(size_t)c]);
    if (fi == 0) zeroAlways = z;
    else {
      std::set<std::string> inter;
      for (const auto& s : zeroAlways) if (z.count(s)) inter.insert(s);
      zeroAlways = inter;
    }
  }
  return std::vector<std::string>(zeroAlways.begin(), zeroAlways.end());
}

// ---------------------------------------------------------------------------
// V4: does the harness reproduce the shipped bridge?
// ---------------------------------------------------------------------------

- (std::vector<std::pair<dynamics::BodyNode*, Eigen::Vector3s>>)
    resolveOn:(std::shared_ptr<dynamics::Skeleton>)sk
        specs:(const std::vector<e1::MarkerSpec>&)specs {
  std::vector<std::pair<dynamics::BodyNode*, Eigen::Vector3s>> out;
  for (const auto& m : specs) {
    dynamics::BodyNode* b = sk->getBodyNode(m.body);
    if (b) out.push_back({b, Eigen::Vector3s(m.ox, m.oy, m.oz)});
  }
  return out;
}

- (std::string)runV4 {
  auto specs = e1::armAMarkers();
  const int nm = (int)specs.size();

  NSMutableArray<NSNumber*>* posA = [NSMutableArray array];
  NSMutableArray<NSString*>* nameA = [NSMutableArray array];
  Eigen::VectorXs tgt(nm * 3);
  for (int i = 0; i < nm; i++) {
    int j = _slotIndex[e1::slotKey(specs[(size_t)i])];
    Eigen::Vector3s p = _slotPos[0][(size_t)j];
    for (int a = 0; a < 3; a++) {
      double v = (double)p(a) + 0.008 * e1::gaussZ(1, 0, j, a);
      tgt(i * 3 + a) = (s_t)v;
      [posA addObject:@(v)];
    }
    [nameA addObject:[NSString stringWithUTF8String:specs[(size_t)i].label.c_str()]];
  }

  NimbleBridge* b2 = [[NimbleBridge alloc] init];
  NSString* path = [[NSBundle bundleForClass:[self class]] pathForResource:@"FullBody"
                                                                    ofType:@"osim"];
  if (![b2 loadModelFromPath:path]) return "{\"status\":\"NOT EVALUATED\"}";
  std::shared_ptr<dynamics::Skeleton> sk2 = [b2 sharedSkeleton];
  sk2->setPositions(_seedDefault);
  [b2 solveIKWithMarkerPositions:posA markerNames:nameA];   // cold, 5 restarts
  Eigen::VectorXs qCold = sk2->getPositions();
  [b2 solveIKWithMarkerPositions:posA markerNames:nameA];   // warm, 1 restart
  Eigen::VectorXs qBridgeWarm = sk2->getPositions();

  auto weightFor = [](const std::string& n) -> double {
    if (n == "PELVIS" || n == "SPINE_L" || n == "SPINE_M" || n == "C7" ||
        n == "NECK" || n == "HEAD") return 1.00;
    if (n == "LHJC" || n == "RHJC" || n == "LSJC" || n == "RSJC") return 0.85;
    if (n == "LKJC" || n == "RKJC" || n == "LEJC" || n == "REJC") return 0.70;
    if (n == "LAJC" || n == "RAJC" || n == "LWJC" || n == "RWJC") return 0.55;
    if (n == "LTOE" || n == "RTOE") return 0.40;
    return 1.00;
  };
  Eigen::VectorXs wProd(nm);
  for (int i = 0; i < nm; i++) wProd(i) = (s_t)weightFor(specs[(size_t)i].label);
  double ss = (double)wProd.squaredNorm();
  if (ss > 0) wProd *= (s_t)std::sqrt((double)nm / ss);

  auto mk2 = [self resolveOn:sk2 specs:specs];
  math::IKConfig cfg;
  cfg.setLossLowerBound((s_t)((double)nm * 0.02 * 0.02));
  cfg.setMaxRestarts(1);

  sk2->setPositions(qCold);
  sk2->fitMarkersToWorldPositions(mk2, tgt, wProd, false, cfg);
  Eigen::VectorXs qHarnessProd = sk2->getPositions();

  sk2->setPositions(qCold);
  Eigen::VectorXs wUni = Eigen::VectorXs::Ones(nm);
  sk2->fitMarkersToWorldPositions(mk2, tgt, wUni, false, cfg);
  Eigen::VectorXs qHarnessE1 = sk2->getPositions();

  double dProd = (double)(qBridgeWarm - qHarnessProd).cwiseAbs().maxCoeff();
  double dE1 = (double)(qBridgeWarm - qHarnessE1).cwiseAbs().maxCoeff();

  std::ostringstream o;
  o << "{\"status\":\"RAN\",\"markers_resolved\":" << mk2.size()
    << ",\"max_abs_dq_bridge_vs_harness_production_weights\":" << e1::jnum(dProd)
    << ",\"max_abs_dq_bridge_vs_harness_frozen_e1_config\":" << e1::jnum(dE1)
    << ",\"note\":"
    << e1::jstr("Production's cold path uses 5 random restarts and a reliability "
                "weight prior; neither is settable through the public API. The "
                "comparison is therefore made against the bridge's WARM solve "
                "(kIKWarmRestarts=1) with the harness configured identically. The "
                "second number is the residual deviation of the frozen E1 config "
                "(uniform weights) from the shipped one.")
    << "}";
  return o.str();
}

// ---------------------------------------------------------------------------
// The run
// ---------------------------------------------------------------------------

- (void)testE1RunAll {
  [self buildCoordinateSets];

  // -- q_true ---------------------------------------------------------------
  _qTrue.assign((size_t)e1::kFrames, Eigen::VectorXs::Zero(_n));
  std::vector<std::string> qtrueNamesNotInModel;
  for (int k = 0; k < e1::kFrames; k++) {
    auto m = e1::qTrueFrame(k);
    if (k == 0) XCTAssertEqual((int)m.size(), 163);
    Eigen::VectorXs q = Eigen::VectorXs::Zero(_n);
    for (const auto& kv : m) {
      auto it = _dofIndex.find(kv.first);
      if (it == _dofIndex.end()) {
        if (k == 0) qtrueNamesNotInModel.push_back(kv.first);
        continue;
      }
      q(it->second) = (s_t)kv.second;
    }
    _qTrue[(size_t)k] = q;
  }
  XCTAssertEqual((int)qtrueNamesNotInModel.size(), 0);

  // -- V2a ------------------------------------------------------------------
  Eigen::VectorXs lo = _skel->getPositionLowerLimits();
  Eigen::VectorXs hi = _skel->getPositionUpperLimits();
  int v2aViolations = 0;
  double v2aWorst = 0.0;
  std::vector<std::string> v2aNames;
  for (int k = 0; k < e1::kFrames; k++)
    for (int i = 0; i < _n; i++) {
      double v = (double)_qTrue[(size_t)k](i);
      double over = std::max((double)lo(i) - v, v - (double)hi(i));
      if (over > 1e-12) {
        v2aViolations++;
        v2aWorst = std::max(v2aWorst, over);
        if (v2aNames.size() < 30) v2aNames.push_back(_dofNames[(size_t)i]);
      }
    }
  printf("E1|V2A|violations=%d|worst=%.3e\n", v2aViolations, v2aWorst);
  fflush(stdout);

  // -- structurally unreachable under arm A ---------------------------------
  _unreachableNames = [self zeroColumnsUnder:e1::armAMarkers()];
  printf("E1|UNREACHABLE_A|%d\n", (int)_unreachableNames.size());
  fflush(stdout);

  // -- slot table + FK targets ----------------------------------------------
  {
    std::map<std::string, e1::MarkerSpec> uniq;
    for (const auto& m : e1::armAMarkers()) uniq[e1::slotKey(m)] = m;
    for (const auto& m : e1::armBMarkers()) uniq[e1::slotKey(m)] = m;
    for (const auto& kv : uniq) _slots.push_back(kv.second);   // std::map = sorted
    for (size_t i = 0; i < _slots.size(); i++)
      _slotIndex[e1::slotKey(_slots[i])] = (int)i;
  }
  XCTAssertEqual((int)_slots.size(), 28);
  {
    auto mkSlots = [self resolve:_slots kept:nullptr missing:nullptr];
    XCTAssertEqual((int)mkSlots.size(), (int)_slots.size());
    _slotPos.assign((size_t)e1::kFrames, {});
    for (int k = 0; k < e1::kFrames; k++) {
      _skel->setPositions(_qTrue[(size_t)k]);
      Eigen::VectorXs p = _skel->getMarkerWorldPositions(mkSlots);
      std::vector<Eigen::Vector3s> row;
      for (int i = 0; i < (int)_slots.size(); i++) row.push_back(p.segment<3>(i * 3));
      _slotPos[(size_t)k] = row;
    }
  }

  // -- V3 -------------------------------------------------------------------
  double v3Worst = 0.0;
  {
    std::vector<std::vector<e1::MarkerSpec>> both = {e1::armAMarkers(), e1::armBMarkers()};
    for (const auto& specs : both) {
      std::vector<e1::MarkerSpec> kept;
      auto mk = [self resolve:specs kept:&kept missing:nullptr];
      for (int k = 0; k < e1::kFrames; k++) {
        _skel->setPositions(_qTrue[(size_t)k]);
        Eigen::VectorXs p = _skel->getMarkerWorldPositions(mk);
        for (size_t i = 0; i < kept.size(); i++) {
          int j = _slotIndex[e1::slotKey(kept[i])];
          for (int a = 0; a < 3; a++)
            v3Worst = std::max(v3Worst,
                               std::abs((double)(p((int)i * 3 + a) -
                                                 _slotPos[(size_t)k][(size_t)j](a))));
        }
      }
    }
  }
  printf("E1|V3|worst=%.3e\n", v3Worst);
  fflush(stdout);

  // -- true torques ---------------------------------------------------------
  std::vector<Eigen::VectorXs> tauTrue;
  {
    Eigen::VectorXs dq(_n), ddq(_n);
    for (int c = e1::kSgCenterLo; c <= e1::kSgCenterHi; c++) {
      e1::sgDerive(_qTrue, c, _n, &dq, &ddq);
      _skel->setPositions(_qTrue[(size_t)c]);
      _skel->setVelocities(dq);
      tauTrue.push_back(_skel->getInverseDynamics(ddq));
    }
  }

  // -- masks ----------------------------------------------------------------
  std::vector<std::string> locked = [self xmlLocked];
  std::vector<std::string> maskAprime = [self runtimeMask];
  std::vector<std::string> maskAdouble = [self structuralMask];
  printf("E1|MASKS|locked=%d|Aprime=%d|Adouble=%d\n",
         (int)locked.size(), (int)maskAprime.size(), (int)maskAdouble.size());
  fflush(stdout);

  // -- arms -----------------------------------------------------------------
  std::vector<e1::ArmSpec> arms;
  arms.push_back({"A", e1::armAMarkers(), "plain", {}, 0.008, 0.0, false});
  arms.push_back({"Aprime", e1::armAMarkers(), "masked", maskAprime, 0.008, 0.0, false});
  arms.push_back({"Adoubleprime", e1::armAMarkers(), "masked", maskAdouble, 0.008, 0.0, false});
  arms.push_back({"A_damp", e1::armAMarkers(), "damped", {}, 0.008, 1e-3, false});
  arms.push_back({"B", e1::armBMarkers(), "plain", {}, 0.008, 0.0, false});
  arms.push_back({"B_damp", e1::armBMarkers(), "damped", {}, 0.008, 1e-3, false});
  arms.push_back({"B_ideal", e1::armBMarkers(), "plain", {}, 0.0, 0.0, true});
  arms.push_back({"B_adv", e1::armBMarkers(), "plain", {}, 0.004, 0.0, false});
  arms.push_back({"A_damp_mu1em4", e1::armAMarkers(), "damped", {}, 0.008, 1e-4, true});
  arms.push_back({"A_damp_mu1em2", e1::armAMarkers(), "damped", {}, 0.008, 1e-2, true});
  arms.push_back({"B_damp_mu1em4", e1::armBMarkers(), "damped", {}, 0.008, 1e-4, true});
  arms.push_back({"B_damp_mu1em2", e1::armBMarkers(), "damped", {}, 0.008, 1e-2, true});

  std::ostringstream armsJson;
  armsJson << "{";
  bool firstArm = true;
  for (const auto& arm : arms) {
    if (!firstArm) armsJson << ",";
    firstArm = false;
    NSDate* t0 = [NSDate date];
    armsJson << e1::jstr(arm.key) << ":{\"status\":\"RAN\",\"solver\":" << e1::jstr(arm.solver)
             << ",\"sigma_m\":" << e1::jnum(arm.sigma) << ",\"mu\":" << e1::jnum(arm.mu)
             << ",\"n_markers\":" << arm.markers.size()
             << ",\"masked_dof_count\":" << arm.mask.size()
             << ",\"masked_dofs\":" << e1::jstrs(arm.mask) << ",\"marker_slots\":[";
    for (size_t i = 0; i < arm.markers.size(); i++) {
      if (i) armsJson << ",";
      armsJson << e1::jstr(e1::slotKey(arm.markers[i]));
    }
    armsJson << "],\"per_seed\":{";
    bool firstSeed = true;
    for (int seed = 1; seed <= (arm.singleSeed ? 1 : 5); seed++) {
      if (!firstSeed) armsJson << ",";
      firstSeed = false;
      armsJson << e1::jstr(std::to_string(seed)) << ":{";
      std::vector<std::string> conds = {"SEED-DEFAULT", "SEED-TRUTH"};
      for (size_t ci = 0; ci < conds.size(); ci++) {
        if (ci) armsJson << ",";
        armsJson << e1::jstr(conds[ci]) << ":"
                 << [self runArm:arm seed:seed condition:conds[ci] tauTrue:tauTrue];
      }
      armsJson << "}";
    }
    armsJson << "}}";
    printf("E1|ARM_DONE|%s|%.1fs\n", arm.key.c_str(), -[t0 timeIntervalSinceNow]);
    fflush(stdout);
  }
  armsJson << "}";

  // -- rank block -----------------------------------------------------------
  std::ostringstream rankJson;
  rankJson << "{\"A\":" << [self rankFor:e1::armAMarkers()]
           << ",\"B\":" << [self rankFor:e1::armBMarkers()]
           << ",\"A_off\":" << [self rankFor:e1::offsetBy5cm(e1::armAMarkers())]
           << ",\"B_off\":" << [self rankFor:e1::offsetBy5cm(e1::armBMarkers())] << "}";
  printf("E1|RANK_DONE\n");
  fflush(stdout);

  std::string v4 = [self runV4];
  std::vector<std::string> unreachB = [self zeroColumnsUnder:e1::armBMarkers()];

  // -- assemble -------------------------------------------------------------
  std::ostringstream out;
  out << "{\n";
  out << "\"preregistration\":"
      << e1::jstr("labs/sam-3d-body/findings/E1_PREREGISTRATION.md") << ",\n";
  out << "\"nimble_dof_count\":" << _n << ",\n";
  out << "\"dof_names\":" << e1::jstrs(_dofNames) << ",\n";
  out << "\"config\":{\"sigma_m\":0.008,\"noise_seeds\":[1,2,3,4,5],"
      << "\"seed_conditions\":[\"SEED-DEFAULT\",\"SEED-TRUTH\"],\"burn_in_frame\":10,"
      << "\"sg_center_lo\":14,\"sg_center_hi\":115,\"fps\":30,"
      << "\"ik_config\":{\"convergenceThreshold\":1e-7,\"maxStepCount\":100,"
      << "\"leastSquaresDamping\":0.01,\"maxRestarts\":1,\"startClamped\":false,"
      << "\"lineSearch\":true,\"marker_weights\":\"uniform 1.0\","
      << "\"lossLowerBound\":\"numMarkers*0.02^2\","
      << "\"warm_reject_cold_fallback\":\"disabled\"}},\n";
  out << "\"seed_default_pose\":[";
  for (int i = 0; i < _n; i++) { if (i) out << ","; out << e1::jnum((double)_seedDefault(i)); }
  out << "],\n";
  out << "\"noise_slots\":[";
  for (size_t i = 0; i < _slots.size(); i++) {
    if (i) out << ",";
    out << e1::jstr(e1::slotKey(_slots[i]));
  }
  out << "],\n";
  out << "\"coordinate_sets\":{";
  {
    bool f = true;
    for (const auto& kv : _sets) {
      if (!f) out << ",";
      f = false;
      std::vector<std::string> nm;
      for (int i : kv.second) nm.push_back(_dofNames[(size_t)i]);
      out << e1::jstr(kv.first) << ":" << e1::jstrs(nm);
    }
  }
  out << "},\n";
  out << "\"xml_locked_coordinates\":" << e1::jstrs(locked) << ",\n";
  out << "\"mask_runtime57\":" << e1::jstrs(maskAprime) << ",\n";
  out << "\"mask_structural80\":" << e1::jstrs(maskAdouble) << ",\n";
  out << "\"q_true\":{\"formula_id\":\"E1-QTRUE-v1\",\"per_frame\":[";
  for (int k = 0; k < e1::kFrames; k++) {
    if (k) out << ",";
    out << "[";
    for (int i = 0; i < _n; i++) {
      if (i) out << ",";
      out << e1::jnum((double)_qTrue[(size_t)k](i));
    }
    out << "]";
  }
  out << "]},\n";
  out << "\"per_arm\":" << armsJson.str() << ",\n";
  out << "\"rank\":" << rankJson.str() << ",\n";
  out << "\"structurally_unreachable_under_A\":" << e1::jstrs(_unreachableNames) << ",\n";
  out << "\"structurally_unreachable_under_B\":" << e1::jstrs(unreachB) << ",\n";
  out << "\"v2a\":{\"violations\":" << v2aViolations
      << ",\"worst_overshoot\":" << e1::jnum(v2aWorst)
      << ",\"names\":" << e1::jstrs(v2aNames) << "},\n";
  out << "\"v3_worst_abs_fk_minus_target\":" << e1::jnum(v3Worst) << ",\n";
  out << "\"v4\":" << v4 << ",\n";
  out << "\"null_model\":{\"E_IVD_FE17_null\":0.0196158570056737,"
      << "\"E_IVD_FE17_half_variance\":0.0138684109030113,"
      << "\"E_IVD51_null\":0.011325220322610921},\n";
  out << "\"harness_notes\":" << e1::jstrs(_notes) << "\n}\n";

  NSString* p = [NSTemporaryDirectory() stringByAppendingPathComponent:@"E1_results_raw.json"];
  std::string s = out.str();
  NSData* d = [NSData dataWithBytes:s.data() length:s.size()];
  BOOL ok = [d writeToFile:p atomically:YES];
  printf("E1|RESULTS|%s|written=%d|bytes=%d\n", [p UTF8String], (int)ok, (int)s.size());
  fflush(stdout);
  XCTAssertTrue(ok);
}

@end
