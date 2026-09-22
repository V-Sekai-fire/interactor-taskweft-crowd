#!/usr/bin/env python3
"""Generate the transit-station crowd MJCF from an XML AST. See the README.

    python scripts/make_crowd.py [--agents N] [--self-test]
"""

import argparse
import math
import pathlib
import sys
import xml.etree.ElementTree as ET

OUT = pathlib.Path(__file__).resolve().parent.parent / "project" / "plans" / "crowd.xml"

AGENTS = 128
AGENT_R = 0.25           # 0.5 m across, about an adult's shoulders
AGENT_H = 1.70           # a person's height, about an interior door
HALL_LEN = 24.0          # entrance-to-exit run
HALL_WID = 12.0
SPAWN_PITCH = 0.60       # centres 0.6 m apart at spawn, so nobody starts overlapping
PILLARS = 4              # a row of columns the crowd must part around

TIMESTEP = 0.004
ITERATIONS = 30

_ANCHORS = [(0.76, "a credit card's thickness"), (1.52, "a penny"),
            (7.0, "a pencil"), (14.5, "an AA battery"), (21.2, "a nickel"),
            (42.7, "a golf ball"), (57.0, "an adult wrist"), (66.0, "a soda can")]


def like(metres):
    """A rough household-object equivalent for a length in metres."""
    mm = metres * 1000.0
    if mm >= 1000.0:
        return "about %.1f m, roughly %.1f soda cans end to end" % (metres, mm / 66.0)
    best = min(_ANCHORS, key=lambda a: abs(a[0] - mm))
    return "about %s (%.0f mm)" % (best[1], mm)


def spawn_positions(n):
    """A block of agents at the entrance end, on a fixed grid so no two overlap."""
    cols = max(1, int(HALL_WID / SPAWN_PITCH))
    out = []
    for i in range(n):
        r, c = divmod(i, cols)
        x = -HALL_LEN / 2.0 + 1.0 + r * SPAWN_PITCH
        y = -((cols - 1) * SPAWN_PITCH) / 2.0 + c * SPAWN_PITCH
        out.append((x, y))
    return out


def build_tree(n=AGENTS):
    m = ET.Element("mujoco", model="transit_crowd")
    ET.SubElement(m, "compiler", angle="radian")
    ET.SubElement(m, "option", timestep=str(TIMESTEP), gravity="0 0 -9.81",
                  integrator="implicitfast", iterations=str(ITERATIONS),
                  ls_iterations="20", cone="pyramidal", tolerance="1e-10")
    default = ET.SubElement(m, "default")
    ET.SubElement(default, "geom", type="capsule", condim="1",
                  solref="0.01 1", solimp="0.9 0.95 0.001",
                  friction="0.5 0.005 0.0001")

    world = ET.SubElement(m, "worldbody")
    ET.SubElement(world, "geom", name="floor", type="plane",
                  size="%.2f %.2f 0.1" % (HALL_LEN / 2.0, HALL_WID / 2.0), rgba="0.2 0.2 0.22 1")
    for side, y in (("N", HALL_WID / 2.0), ("S", -HALL_WID / 2.0)):
        ET.SubElement(world, "geom", name="wall%s" % side, type="box",
                      pos="0 %.3f 1.5" % y, size="%.2f 0.1 1.5" % (HALL_LEN / 2.0),
                      rgba="0.35 0.36 0.4 1")
    for p in range(PILLARS):
        px = -HALL_LEN / 4.0 + p * (HALL_LEN / 2.0) / max(1, PILLARS - 1)
        ET.SubElement(world, "geom", name="pillar%d" % p, type="box",
                      pos="%.3f 0 1.5" % px, size="0.4 0.4 1.5", rgba="0.5 0.5 0.55 1")

    pos = spawn_positions(n)
    for i, (x, y) in enumerate(pos):
        body = ET.SubElement(world, "body", name="ped%d" % i,
                             pos="%.4f %.4f %.4f" % (x, y, AGENT_H / 2.0))
        ET.SubElement(body, "joint", name="x%d" % i, type="slide", axis="1 0 0",
                      damping="8")
        ET.SubElement(body, "joint", name="y%d" % i, type="slide", axis="0 1 0",
                      damping="8")
        ET.SubElement(body, "geom", name="ped%d" % i, size="%.3f" % AGENT_R,
                      fromto="0 0 %.3f 0 0 %.3f" % (-AGENT_H / 2.0 + AGENT_R,
                                                    AGENT_H / 2.0 - AGENT_R),
                      density="120", rgba="0.8 0.7 0.55 1")

    act = ET.SubElement(m, "actuator")
    for i in range(n):
        ET.SubElement(act, "motor", name="fx%d" % i, joint="x%d" % i, gear="200",
                      ctrlrange="-1 1")
        ET.SubElement(act, "motor", name="fy%d" % i, joint="y%d" % i, gear="200",
                      ctrlrange="-1 1")

    key = ET.SubElement(m, "keyframe")
    qpos = " ".join("0" for _ in range(2 * n))
    qvel = " ".join("0" for _ in range(2 * n))
    ET.SubElement(key, "key", name="at_entrance", qpos=qpos, qvel=qvel)
    return m


def build(n=AGENTS):
    m = build_tree(n)
    ET.indent(m, space="  ")
    return ET.tostring(m, encoding="unicode") + "\n"


def self_test():
    controls = []

    def control(name, ok, detail=""):
        controls.append((name, ok, detail))

    n = 64
    m = build_tree(n)
    pos = spawn_positions(n)

    control("every agent spawns inside the hall",
            all(abs(x) < HALL_LEN / 2.0 and abs(y) < HALL_WID / 2.0 for x, y in pos))

    dmin = min(math.dist(pos[i], pos[j])
               for i in range(n) for j in range(i + 1, n))
    control("no two agents start overlapping",
            dmin >= 2 * AGENT_R - 1e-9, "closest pair %s" % like(dmin))

    joints = m.findall(".//joint[@type='slide']")
    control("two planar joints per agent, no toppling DOF", len(joints) == 2 * n)
    control("no free or hinge joints exist (a toppling DOF would break determinism)",
            len(m.findall(".//joint[@type='free']")) == 0 and
            len(m.findall(".//joint[@type='hinge']")) == 0)

    control("one motor per planar joint", len(m.findall(".//motor")) == 2 * n)

    control("the keyframe covers every DOF",
            len(m.find(".//key").get("qpos").split()) == 2 * n)

    walls = [g for g in m.iter("geom") if (g.get("name") or "").startswith("wall")]
    control("the hall has both side walls", len(walls) == 2)
    control("the concourse has its pillars", len(
        [g for g in m.iter("geom") if (g.get("name") or "").startswith("pillar")]) == PILLARS)

    control("agents spawn at the entrance end, not on top of the exit",
            all(x < 0 for x, y in pos), "max spawn x %s" % like(max(x for x, y in pos) + HALL_LEN / 2.0))

    bad = [(0.0, 0.0), (0.0, 0.0)]
    bad_min = math.dist(bad[0], bad[1])
    control("a stacked spawn would be caught", not (bad_min >= 2 * AGENT_R))

    for name, ok, detail in controls:
        print(("PASS" if ok else "FAIL") + "  " + name + ("  [" + detail + "]" if detail else ""))
    passed = sum(1 for _, ok, _ in controls if ok)
    print("%d/%d controls" % (passed, len(controls)))
    return 0 if passed == len(controls) else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--agents", type=int, default=AGENTS)
    ap.add_argument("--self-test", action="store_true")
    args = ap.parse_args()
    if args.self_test:
        return self_test()
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(build(args.agents), encoding="utf-8", newline="\n")
    print("wrote %s (%d agents, hall %s long)" % (OUT, args.agents, like(HALL_LEN)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
