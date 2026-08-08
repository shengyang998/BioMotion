"""Is OpenSim's moment arm the finite difference of OpenSim's own path length?

`MomentArmComputer` computes r = -dL/dq by central difference. OpenSim's
`GeometryPath::computeMomentArm` does NOT: it asks `MomentArmSolver` for the
generalized force a unit tension along the CURRENT path produces, holding the
wrap points fixed on their bodies. The two agree when the wrapped path is a true
geodesic that varies smoothly with q, and they need not agree where the wrap
solution is marginal.

This measures the gap in OpenSim's own numbers, so a disagreement between the
app and the reference can be attributed rather than guessed at.
"""
from __future__ import annotations
import argparse, sys
import osim_model, inspect_wrap

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pose", default="neutral")
    ap.add_argument("--muscle", required=True)
    ap.add_argument("--coordinates", nargs="+", required=True)
    ap.add_argument("--eps", type=float, default=1e-4)
    args = ap.parse_args()

    model, state, _ = osim_model.load_model()
    values = inspect_wrap.read_pose(args.pose)
    cs = model.getCoordinateSet()
    for i in range(cs.getSize()):
        c = cs.get(i)
        if c.getName() in values:
            c.setValue(state, values[c.getName()], False)
    model.assemble(state)
    model.realizePosition(state)
    path = model.getMuscles().get(args.muscle).getGeometryPath()

    print(f"pose={args.pose} muscle={args.muscle} eps={args.eps}")
    for name in args.coordinates:
        coord = cs.get(name)
        base = coord.getValue(state)
        analytic = path.computeMomentArm(state, coord)
        model.realizePosition(state)
        length0 = path.getLength(state)

        coord.setValue(state, base + args.eps, False)
        model.realizePosition(state)
        plus = path.getLength(state)
        coord.setValue(state, base - args.eps, False)
        model.realizePosition(state)
        minus = path.getLength(state)
        coord.setValue(state, base, False)
        model.realizePosition(state)

        central = -(plus - minus) / (2 * args.eps)
        forward = -(plus - length0) / args.eps
        backward = -(length0 - minus) / args.eps
        print(f"  {name:16s} analytic {analytic:+.6f}  central {central:+.6f}  "
              f"forward {forward:+.6f}  backward {backward:+.6f}  "
              f"L- {minus:.6f} L0 {length0:.6f} L+ {plus:.6f}")

if __name__ == "__main__":
    main()
