"""Standalone OpenSim .osim forward-kinematics + moment-arm engine.

WHY THIS EXISTS
---------------
The two structural defects fixed in the 2026-08-06 snapshot (patella skipped,
shoulders welded) live in how *nimblephysics* turns FullBody.osim into a
Skeleton, and their effect is visible in muscle moment arms. Measuring them
normally meant running the iOS app, which that workstream was forbidden from
building. So this module re-implements, in Python, the two pieces of C++ that
decided the historical answer:

  1. `nimblephysics/dart/biomechanics/OpenSimParser.cpp :: readOsim40()`
     — which bodies/joints get built, and which joint class each XML joint
       becomes.  Ported rules, with source line references, in `_classify_joint`
       and `parse_model`.
  2. `BioMotion/BioMotion/Muscle/MomentArmComputer.mm`
     — muscle path polyline -> length -> moment arm by central difference.
       Ported in `Model.muscle_length` / `Model.moment_arm`, including the
       ConditionalPathPoint latching behaviour and eps = 1e-4 rad.

It is therefore a wrap-disabled structural diagnostic for the historical
model edit, not an independent biomechanics implementation and not an exact
snapshot of either the 2026-08-06 or current app. It keeps the old straight-line
muscle paths so the before/after structural numbers remain apples-to-apples,
while its SimmSpline evaluator includes the later endpoint-linear correction.
The current iOS `MomentArmComputer` solves both wrapped surfaces; this
diagnostic does not.

CONVENTIONS VERIFIED AGAINST NIMBLE SOURCE (not assumed)
--------------------------------------------------------
* PhysicalOffsetFrame orientation -> `math::eulerXYZToMatrix` = Rx*Ry*Rz
  (OpenSimParser.cpp:6694).
* CustomJoint transform: `mT = T_parentToJoint * T(q) * T_childToJoint^-1`
  with `T(q).linear() = EulerJoint::convertToTransform(euler, order, flips)`
  and `T(q).translation()` set (NOT rotated) from the three translation
  functions (CustomJoint.cpp:797-807).
* `eulerXYZToMatrix(a) = Rx(a0)Ry(a1)Rz(a2)`, `eulerZYXToMatrix(a) =
  Rz(a0)Ry(a1)Rx(a2)`, likewise ZXY / XZY (Geometry.cpp:1767, 2122, ...).
  Combined with `getAxisFlips` this is identical to composing
  Rot(axis_i, f_i(q)) for i = 0,1,2 in document order, which is what this file
  does.
* SimmSpline: exact port of BioMotion's patched `dart/math/SimmSpline.cpp`
  (itself a port of OpenSim's), including endpoint-tangent linear
  extrapolation, the SIMM end conditions and TINY_NUMBER = 1e-7.
* PinJoint -> DART RevoluteJoint with default axis = +Z (OpenSimParser.cpp:5610
  never calls setAxis).  UniversalJoint -> DART defaults axis1=+X, axis2=+Y.

LIMITATIONS OF THIS HISTORICAL DIAGNOSTIC
-----------------------------------------
* PathWrap (WrapCylinder / WrapEllipsoid) is not implemented here. The app's
  MomentArmComputer now solves both surfaces. Each of the 8 quadriceps measured
  by this historical diagnostic carries exactly 1 PathWrap, so its absolute
  values remain only approximate here.
* MovingPathPoint location functions use the same OpenSim-compatible SimmSpline
  semantics here and in the app. No muscle measured in this report carries one
  (asserted in measure.py).
* No constraint solver: CoordinateCouplerConstraint is applied here only when a
  caller explicitly asks for the `coupled` reference model.  nimble never
  applies it, which is the whole point of defect 1.
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field

import numpy as np

TINY_NUMBER = 1e-7
ROUNDOFF_ERROR = 2e-13


# --------------------------------------------------------------------------
# Functions (ported from dart/math/*)
# --------------------------------------------------------------------------


class Constant:
    kind = "Constant"

    def __init__(self, value):
        self.value = float(value)

    def __call__(self, x):
        return self.value


class Linear:
    kind = "Linear"

    def __init__(self, a, b):
        self.a, self.b = float(a), float(b)

    def __call__(self, x):
        return self.a * x + self.b


class Polynomial:
    kind = "Polynomial"

    def __init__(self, coeffs):
        # OpenSim lists highest power first
        self.coeffs = [float(c) for c in coeffs]

    def __call__(self, x):
        return float(np.polyval(self.coeffs, x))


class PiecewiseLinear:
    kind = "PiecewiseLinear"

    def __init__(self, x, y):
        self.x = [float(v) for v in x]
        self.y = [float(v) for v in y]

    def __call__(self, t):
        return float(np.interp(t, self.x, self.y))


class SimmSpline:
    """OpenSim-compatible port of SimmSpline coefficients and value evaluation."""

    kind = "SimmSpline"

    def __init__(self, x, y):
        self._x = [float(v) for v in x]
        self._y = [float(v) for v in y]
        self._b = []
        self._c = []
        self._d = []
        self._calc_coefficients()

    def _calc_coefficients(self):
        x, y = self._x, self._y
        n = len(x)
        if n < 2:
            return
        b = [0.0] * n
        c = [0.0] * n
        d = [0.0] * n
        if n == 2:
            t = max(TINY_NUMBER, x[1] - x[0])
            b[0] = b[1] = (y[1] - y[0]) / t
            self._b, self._c, self._d = b, c, d
            return
        nm1, nm2 = n - 1, n - 2
        d[0] = max(TINY_NUMBER, x[1] - x[0])
        c[1] = (y[1] - y[0]) / d[0]
        for i in range(1, nm1):
            d[i] = max(TINY_NUMBER, x[i + 1] - x[i])
            b[i] = 2.0 * (d[i - 1] + d[i])
            c[i + 1] = (y[i + 1] - y[i]) / d[i]
            c[i] = c[i + 1] - c[i]
        b[0] = -d[0]
        b[nm1] = -d[nm2]
        c[0] = 0.0
        c[nm1] = 0.0
        if n > 3:
            d31 = max(TINY_NUMBER, x[3] - x[1])
            d20 = max(TINY_NUMBER, x[2] - x[0])
            d1 = max(TINY_NUMBER, x[nm1] - x[n - 3])
            d2 = max(TINY_NUMBER, x[nm2] - x[n - 4])
            d30 = max(TINY_NUMBER, x[3] - x[0])
            d3 = max(TINY_NUMBER, x[nm1] - x[n - 4])
            c[0] = c[2] / d31 - c[1] / d20
            c[nm1] = c[nm2] / d1 - c[n - 3] / d2
            c[0] = c[0] * d[0] * d[0] / d30
            c[nm1] = -c[nm1] * d[nm2] * d[nm2] / d3
        for i in range(1, n):
            t = d[i - 1] / b[i - 1]
            b[i] -= t * d[i - 1]
            c[i] -= t * c[i - 1]
        c[nm1] /= b[nm1]
        for j in range(0, nm1):
            i = nm2 - j
            c[i] = (c[i] - d[i] * c[i + 1]) / b[i]
        b[nm1] = (y[nm1] - y[nm2]) / d[nm2] + d[nm2] * (c[nm2] + 2.0 * c[nm1])
        for i in range(0, nm1):
            b[i] = (y[i + 1] - y[i]) / d[i] - d[i] * (c[i + 1] + 2.0 * c[i])
            d[i] = (c[i + 1] - c[i]) / d[i]
            c[i] *= 3.0
        c[nm1] *= 3.0
        d[nm1] = d[nm2]
        self._b, self._c, self._d = b, c, d

    def __call__(self, ax):
        x = self._x
        n = len(x)
        if ax < x[0]:
            return self._y[0] + (ax - x[0]) * self._b[0]
        if ax > x[n - 1]:
            return self._y[n - 1] + (ax - x[n - 1]) * self._b[n - 1]
        if n < 3:
            k = 0
        else:
            if abs(ax - x[0]) <= ROUNDOFF_ERROR or ax < x[0]:
                k = 0
            elif abs(ax - x[n - 1]) <= ROUNDOFF_ERROR or ax > x[n - 1]:
                k = n - 1
            else:
                i, j = 0, n
                while True:
                    k = (i + j) // 2
                    if ax < x[k]:
                        j = k
                    elif ax > x[k + 1]:
                        i = k
                    else:
                        break
        dx = ax - x[k]
        return self._y[k] + dx * (self._b[k] + dx * (self._c[k] + dx * self._d[k]))


def _text_vec(el, n=3):
    if el is None or el.text is None:
        return np.zeros(n)
    parts = el.text.split()
    v = np.zeros(n)
    for i in range(min(n, len(parts))):
        v[i] = float(parts[i])
    return v


def _text_list(el):
    if el is None or el.text is None:
        return []
    return [float(p) for p in el.text.split()]


def parse_function(el):
    """Parse one <TransformAxis> function element the way OpenSimParser does.

    Returns (callable, kind) where kind is one of Constant/Linear/SimmSpline/...
    `MultiplierFunction` wrapping is unwrapped and the scale folded in, exactly
    as OpenSimParser.cpp:5093-5113 does.
    """
    scale = 1.0
    node = el
    mult = el.find("MultiplierFunction")
    if mult is not None:
        inner = mult.find("function")
        sc = mult.find("scale")
        if sc is not None and sc.text:
            scale = float(sc.text)
        node = inner if inner is not None else mult

    c = node.find("Constant")
    if c is not None:
        return Constant(float(c.find("value").text) * scale), "Constant"
    lf = node.find("LinearFunction")
    if lf is not None:
        co = _text_list(lf.find("coefficients"))
        return Linear(co[0] * scale, co[1] * scale), "Linear"
    ss = node.find("SimmSpline")
    if ss is not None:
        y = [v * scale for v in _text_list(ss.find("y"))]
        return SimmSpline(_text_list(ss.find("x")), y), "SimmSpline"
    pl = node.find("PiecewiseLinearFunction")
    if pl is not None:
        y = [v * scale for v in _text_list(pl.find("y"))]
        return PiecewiseLinear(_text_list(pl.find("x")), y), "PiecewiseLinear"
    pf = node.find("PolynomialFunction")
    if pf is not None:
        co = [v * scale for v in _text_list(pf.find("coefficients"))]
        return Polynomial(co), "Polynomial"
    raise ValueError("Unrecognized function type under %r" % el.tag)


# --------------------------------------------------------------------------
# Rigid transforms
# --------------------------------------------------------------------------


def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def euler_xyz(v):
    """math::eulerXYZToMatrix — Rx(v0) Ry(v1) Rz(v2)."""
    return rot_x(v[0]) @ rot_y(v[1]) @ rot_z(v[2])


def rot_about(axis, angle):
    a = np.asarray(axis, dtype=float)
    n = np.linalg.norm(a)
    if n == 0:
        return np.eye(3)
    a = a / n
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(angle) * K + (1 - math.cos(angle)) * (K @ K)


def iso(R=None, p=None):
    T = np.eye(4)
    if R is not None:
        T[:3, :3] = R
    if p is not None:
        T[:3, 3] = p
    return T


def iso_inv(T):
    R = T[:3, :3]
    p = T[:3, 3]
    out = np.eye(4)
    out[:3, :3] = R.T
    out[:3, 3] = -R.T @ p
    return out


# --------------------------------------------------------------------------
# Model
# --------------------------------------------------------------------------


@dataclass
class Joint:
    name: str
    xml_type: str
    built_type: str  # what nimble actually builds
    parent_body: str
    child_body: str
    T_from_parent: np.ndarray
    T_from_child: np.ndarray
    coords: list = field(default_factory=list)  # coordinate names, in XML order
    rot_axes: list = field(default_factory=list)
    rot_funcs: list = field(default_factory=list)
    rot_driven_by: list = field(default_factory=list)  # index into self.coords
    trans_axes: list = field(default_factory=list)
    trans_funcs: list = field(default_factory=list)
    trans_driven_by: list = field(default_factory=list)
    defaults: dict = field(default_factory=dict)
    note: str = ""


@dataclass
class PathPoint:
    kind: str
    body: str
    location: np.ndarray
    cond_coord: str | None = None
    cond_min: float = -math.inf
    cond_max: float = math.inf
    moving_axes: list | None = None  # [(func|None, coordname|None)] * 3
    active: bool = True


@dataclass
class Muscle:
    name: str
    cls: str
    points: list
    n_wraps: int
    max_iso_force: float


NIMBLE_SKIPPED_BODIES = ("patella_r", "patella_l")
NIMBLE_SKIPPED_JOINTS = ("patellofemoral_r", "patellofemoral_l")
NIMBLE_SKIPPED_CHILD_PATHS = ("/bodyset/patella_r", "/bodyset/patella_l")


class Model:
    def __init__(self, path, nimble_rules=True, couple_patella=False, verbose=False):
        """
        nimble_rules   -- emulate readOsim40's patella skip + the crash-guard
                          WeldJoint substitution for joints nimble cannot build.
                          Set False to get the "what OpenSim itself would do"
                          reference model.
        couple_patella -- apply the CoordinateCouplerConstraint (beta = knee
                          angle).  Only meaningful with nimble_rules=False; this
                          is the anatomically-correct reference nimble can never
                          produce.
        """
        self.path = str(path)
        self.nimble_rules = nimble_rules
        self.couple_patella = couple_patella
        self.verbose = verbose
        self.warnings = []
        self.parse_model()
        self.q = {c: self.coord_defaults.get(c, 0.0) for c in self.coord_names}
        self._cache = {}

    # ---------------- parsing ----------------

    def parse_model(self):
        root = ET.parse(self.path).getroot()
        model = root.find("Model")
        self.model_name = model.get("name")

        bodies = []
        for b in model.find("./BodySet/objects"):
            if b.tag != "Body":
                continue
            name = b.get("name").strip()
            if self.nimble_rules and name in NIMBLE_SKIPPED_BODIES:
                self.warnings.append("nimble skips body %r (OpenSimParser.cpp:6562)" % name)
                continue
            bodies.append(name)
        self.body_names = bodies

        joints = []
        for j in model.find("./JointSet/objects"):
            jn = j.get("name").strip()
            jt = j.tag
            pfe = j.find("socket_parent_frame")
            cfe = j.find("socket_child_frame")
            parent_offset_frame = pfe.text.strip()
            child_offset_frame = cfe.text.strip()

            parent_body = ""
            child_body = ""
            T_par = np.eye(4)
            T_chi = np.eye(4)
            frames = j.find("frames")
            if frames is not None:
                for f in frames:
                    sp = f.find("socket_parent")
                    pb = sp.text.strip() if sp is not None and sp.text else ""
                    T = iso(euler_xyz(_text_vec(f.find("orientation"))),
                            _text_vec(f.find("translation")))
                    fname = f.get("name").strip()
                    if fname == parent_offset_frame:
                        parent_body, T_par = pb, T
                    elif fname == child_offset_frame:
                        child_body, T_chi = pb, T
            if not parent_body:
                parent_body = parent_offset_frame
            if not child_body:
                child_body = child_offset_frame

            if self.nimble_rules and (
                jn in NIMBLE_SKIPPED_JOINTS or child_body in NIMBLE_SKIPPED_CHILD_PATHS
            ):
                self.warnings.append(
                    "nimble skips joint %r (OpenSimParser.cpp:6737-6739)" % jn
                )
                continue

            parent_body = _leaf(parent_body)
            child_body = _leaf(child_body)

            joint = Joint(
                name=jn,
                xml_type=jt,
                built_type=jt,
                parent_body=parent_body,
                child_body=child_body,
                T_from_parent=T_par,
                T_from_child=T_chi,
            )

            coords_el = j.find("coordinates")
            if coords_el is not None:
                for c in coords_el:
                    if c.tag != "Coordinate":
                        continue
                    cn = c.get("name").strip()
                    joint.coords.append(cn)
                    dv = c.find("default_value")
                    joint.defaults[cn] = float(dv.text) if dv is not None else 0.0

            st = j.find("SpatialTransform")
            if st is not None:
                axes = list(st.findall("TransformAxis"))
                for i, ta in enumerate(axes):
                    axis = _text_vec(ta.find("axis"))
                    fn_el = ta.find("function")
                    fn, kind = parse_function(fn_el if fn_el is not None else ta)
                    ce = ta.find("coordinates")
                    cname = ce.text.strip() if (ce is not None and ce.text) else ""
                    drv = joint.coords.index(cname) if cname in joint.coords else 0
                    if i < 3:
                        joint.rot_axes.append(axis)
                        joint.rot_funcs.append((fn, kind))
                        joint.rot_driven_by.append(drv)
                    else:
                        joint.trans_axes.append(axis)
                        joint.trans_funcs.append((fn, kind))
                        joint.trans_driven_by.append(drv)

            joint.built_type = self._classify_joint(joint)
            joints.append(joint)

        self.joints = joints

        # dof list, in the order nimble would build them (joint order, coord order)
        self.coord_names = []
        self.coord_defaults = {}
        self.coord_owner = {}
        for jt in joints:
            if jt.built_type == "Weld":
                continue
            n = _ndofs_for(jt)
            for cn in jt.coords[:n]:
                self.coord_names.append(cn)
                self.coord_defaults[cn] = jt.defaults.get(cn, 0.0)
                self.coord_owner[cn] = jt.name

        # kinematic tree
        self.children = {}
        self.parent_joint = {}
        for jt in joints:
            self.children.setdefault(jt.parent_body, []).append(jt)
            self.parent_joint[jt.child_body] = jt
        self.root_bodies = [
            jt.child_body for jt in joints if jt.parent_body not in self.parent_joint
        ]

        # muscles
        self.muscles = {}
        fs = model.find("./ForceSet/objects")
        for m in fs:
            if m.tag not in ("Thelen2003Muscle", "Millard2012EquilibriumMuscle"):
                continue
            gp = m.find("GeometryPath")
            if gp is None:
                continue
            pps = gp.find("./PathPointSet/objects")
            if pps is None:
                continue
            pts = []
            for pp in pps:
                if pp.tag not in ("PathPoint", "ConditionalPathPoint", "MovingPathPoint"):
                    continue
                bref = pp.find("socket_parent_frame")
                if bref is None:
                    bref = pp.find("body")
                body = _leaf(bref.text.strip()) if bref is not None and bref.text else ""
                p = PathPoint(kind=pp.tag, body=body, location=_text_vec(pp.find("location")))
                if pp.tag == "ConditionalPathPoint":
                    ce = pp.find("socket_coordinate")
                    if ce is None:
                        ce = pp.find("coordinate")
                    rng = _text_list(pp.find("range"))
                    if ce is not None and ce.text and len(rng) >= 2:
                        p.cond_coord = _leaf(ce.text.strip())
                        p.cond_min, p.cond_max = min(rng), max(rng)
                if pp.tag == "MovingPathPoint":
                    ax = []
                    for tag, sock, legacy in (
                        ("x_location", "socket_x_coordinate", "x_coordinate"),
                        ("y_location", "socket_y_coordinate", "y_coordinate"),
                        ("z_location", "socket_z_coordinate", "z_coordinate"),
                    ):
                        le = pp.find(tag)
                        if le is None:
                            ax.append((None, None))
                            continue
                        fn, kind = parse_function(le)
                        ce = pp.find(sock) or pp.find(legacy)
                        cn = _leaf(ce.text.strip()) if ce is not None and ce.text else None
                        ax.append((fn, cn))
                    p.moving_axes = ax
                pts.append(p)
            nwrap = 0
            ws = gp.find("./PathWrapSet/objects")
            if ws is not None:
                nwrap = len(list(ws.findall("PathWrap")))
            mif = m.find("max_isometric_force")
            self.muscles[m.get("name")] = Muscle(
                name=m.get("name"),
                cls=m.tag,
                points=pts,
                n_wraps=nwrap,
                max_iso_force=float(mif.text) if mif is not None else 0.0,
            )

    def _classify_joint(self, jt: Joint) -> str:
        """Port of the joint-type dispatch in OpenSimParser.cpp:5030-5560 + the
        BioMotion crash-guard at :5791.  Returns one of
        Weld / Weld(crash-guard) / EulerFree / Euler / Euler(R-basis) /
        Revolute / Prismatic / Universal / Custom.
        """
        if jt.xml_type == "WeldJoint":
            return "Weld"
        if jt.xml_type == "PinJoint":
            return "Revolute"
        if jt.xml_type == "UniversalJoint":
            return "Universal"
        if jt.xml_type != "CustomJoint":
            return jt.xml_type

        kinds = [k for _, k in jt.rot_funcs] + [k for _, k in jt.trans_funcs]
        rot_kinds = kinds[:3]
        n_const = sum(1 for k in kinds if k == "Constant")
        any_spline = any(k in ("SimmSpline", "PiecewiseLinear", "Polynomial") for k in kinds)
        all_linear = all(k == "Linear" for k in kinds)
        first3_linear = all(k == "Linear" for k in rot_kinds)
        # allLocked: every function is a Constant with value 0
        all_locked = True
        for fn, k in jt.rot_funcs + jt.trans_funcs:
            if k != "Constant" or fn.value != 0:
                all_locked = False
                break
        ndof = len(jt.coords)

        if all_locked:
            return "Weld"
        if all_linear and not any_spline and ndof == 6:
            return "EulerFree"
        if first3_linear and not any_spline and ndof in (3, 6):
            unit = [_is_pm_unit(a) for a in jt.rot_axes]
            if all(unit):
                return "Euler"
            # non-unit: needs mutual orthogonality within 1e-4
            a = [np.asarray(x, float) for x in jt.rot_axes]
            d01 = abs(float(a[0] @ a[1]))
            d12 = abs(float(a[1] @ a[2]))
            d20 = abs(float(a[2] @ a[0]))
            if d01 < 1e-4 and d12 < 1e-4 and d20 < 1e-4:
                return "Euler(R-basis)"
            jt.note = "non-orthogonal axes |dots|=(%.6f, %.6f, %.6f)" % (d01, d12, d20)
            return "Weld(crash-guard)"
        n_linear = sum(1 for k in kinds if k == "Linear")
        last_linear = max((i for i, k in enumerate(kinds) if k == "Linear"), default=0)
        if n_linear == 1 and n_const == 5 and not any_spline:
            return "Revolute" if last_linear < 3 else "Prismatic"
        if n_linear == 2 and n_const == 4 and last_linear < 3 and not any_spline:
            return "Universal"
        return "Custom"

    # ---------------- kinematics ----------------

    def set_positions(self, q: dict):
        self.q = dict(q)
        self._cache = {}

    def set_coord(self, name, value):
        self.q[name] = value
        self._cache = {}

    def _coord_value(self, name):
        if self.couple_patella and name.endswith("_beta"):
            base = name[: -len("_beta")]
            if base in self.q:
                return self.q[base]
        return self.q.get(name, 0.0)

    def joint_transform(self, jt: Joint) -> np.ndarray:
        """T(q) for one joint: the OpenSim / nimble parametric transform."""
        bt = jt.built_type
        if bt.startswith("Weld"):
            return np.eye(4)
        if bt == "Revolute":
            if jt.xml_type == "PinJoint":
                return iso(rot_z(self._coord_value(jt.coords[0])))
            # CustomJoint reduced to a revolute: single linear rotation axis
            for i, (fn, k) in enumerate(jt.rot_funcs):
                if k == "Linear":
                    q = self._coord_value(jt.coords[jt.rot_driven_by[i]])
                    return iso(rot_about(jt.rot_axes[i], fn(q)))
            return np.eye(4)
        if bt == "Universal" and jt.xml_type == "UniversalJoint":
            q0 = self._coord_value(jt.coords[0])
            q1 = self._coord_value(jt.coords[1])
            return iso(rot_x(q0) @ rot_y(q1))
        # everything else is the general OpenSim CustomJoint composition:
        # R = Rot(a0, f0) Rot(a1, f1) Rot(a2, f2), p = f3*a3 + f4*a4 + f5*a5
        R = np.eye(3)
        for i, (fn, _k) in enumerate(jt.rot_funcs):
            q = self._coord_value(jt.coords[jt.rot_driven_by[i]]) if jt.coords else 0.0
            R = R @ rot_about(jt.rot_axes[i], fn(q))
        p = np.zeros(3)
        for i, (fn, _k) in enumerate(jt.trans_funcs):
            q = self._coord_value(jt.coords[jt.trans_driven_by[i]]) if jt.coords else 0.0
            p = p + fn(q) * np.asarray(jt.trans_axes[i], float)
        return iso(R, p)

    def body_world(self, body: str) -> np.ndarray | None:
        if body in self._cache:
            return self._cache[body]
        jt = self.parent_joint.get(body)
        if jt is None:
            if body in self.body_names or body == "ground":
                self._cache[body] = np.eye(4)
                return self._cache[body]
            return None
        parent_T = self.body_world(jt.parent_body)
        if parent_T is None:
            parent_T = np.eye(4)
        T = parent_T @ jt.T_from_parent @ self.joint_transform(jt) @ iso_inv(jt.T_from_child)
        self._cache[body] = T
        return T

    # ---------------- muscle geometry (MomentArmComputer.mm port) ----------

    def _local_offset(self, p: PathPoint) -> np.ndarray:
        if p.moving_axes is None:
            return p.location
        out = np.array(p.location, dtype=float)
        for i, (fn, cn) in enumerate(p.moving_axes):
            if fn is None:
                continue
            out[i] = fn(self._coord_value(cn)) if cn else fn(0.0)
        return out

    def _world_point(self, p: PathPoint) -> np.ndarray:
        loc = self._local_offset(p)
        T = self.body_world(p.body)
        if T is None:
            # MomentArmComputer.mm:worldPositionForPathPoint — unresolved body
            # falls back to the RAW LOCAL OFFSET, i.e. a point pinned in world
            # space.  This is the exact failure mode of the skipped patella.
            return loc
        return (T @ np.append(loc, 1.0))[:3]

    def latch_conditionals(self):
        for m in self.muscles.values():
            for p in m.points:
                if p.kind != "ConditionalPathPoint" or p.cond_coord is None:
                    p.active = True
                    continue
                if p.cond_coord not in self.q and not (
                    self.couple_patella and p.cond_coord.endswith("_beta")
                ):
                    p.active = True  # coordinate not a DOF -> ungated
                    continue
                v = self._coord_value(p.cond_coord)
                p.active = (v >= p.cond_min) and (v <= p.cond_max)

    def muscle_length(self, name) -> float:
        m = self.muscles[name]
        total = 0.0
        prev = None
        for p in m.points:
            if not p.active:
                continue
            w = self._world_point(p)
            if prev is not None:
                total += float(np.linalg.norm(w - prev))
            prev = w
        return total

    def moment_arm(self, muscle, coord, eps=1e-4) -> float:
        """r = -dL/dq by central difference, eps = 1e-4 rad — identical to
        MomentArmComputer.mm:computeMomentArmsWithJointAngles."""
        q0 = self.q.get(coord, 0.0)
        self.latch_conditionals()  # latched at the UNPERTURBED pose, then held
        saved = dict(self.q)
        self.q[coord] = q0 + eps
        self._cache = {}
        lp = self.muscle_length(muscle)
        self.q[coord] = q0 - eps
        self._cache = {}
        lm = self.muscle_length(muscle)
        self.q = saved
        self._cache = {}
        return -(lp - lm) / (2.0 * eps)

    # ---------------- reporting helpers ----------------

    @property
    def n_dofs(self):
        return len(self.coord_names)

    def joint_dofs(self, joint_name):
        for jt in self.joints:
            if jt.name == joint_name:
                if jt.built_type.startswith("Weld"):
                    return 0
                return _ndofs_for(jt)
        return None

    def unresolved_path_bodies(self):
        out = {}
        for m in self.muscles.values():
            for p in m.points:
                if self.body_world(p.body) is None:
                    out.setdefault(p.body, 0)
                    out[p.body] += 1
        return out


def _leaf(s):
    return s.split("/")[-1] if s else s


def _is_pm_unit(a):
    a = np.asarray(a, float)
    for j in range(3):
        e = np.zeros(3)
        e[j] = 1.0
        if np.array_equal(a, e) or np.array_equal(a, -e):
            return True
    return False


def _ndofs_for(jt: Joint) -> int:
    bt = jt.built_type
    if bt.startswith("Weld"):
        return 0
    if bt in ("Revolute", "Prismatic"):
        return 1
    if bt == "Universal":
        return 2
    if bt in ("Euler", "Euler(R-basis)"):
        return 3
    if bt == "EulerFree":
        return 6
    return len(jt.coords)  # Custom<N>
