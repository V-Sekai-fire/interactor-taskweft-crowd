# interactor-taskweft-crowd

A deterministic transit-station crowd: thousands of taskweft agents steer one shared MuJoCo world, bit-identical on every host.

## What this is

A crowd simulation where the *determinism* is the demo. N pedestrians cross a
pillared concourse, each a taskweft HTN agent deciding where to go, MuJoCo
stepping how they move. Because the physics runs as a RISC-V guest under
libriscv, the same inputs produce a bit-identical world on every host — x86_64,
arm64, Windows, Linux — so every client agrees on the crowd down to the last
step. The scale is the point: 128 agents is the default, and the count is a
command-line knob, not a rewrite.

This is the easy half of a two-repo pair. Crowd steering is sparse planar motion
and soft avoidance, which reproduces across platforms far more readily than
dense stacking. Its sibling, [`interactor-mujoco-cloth-sim`](https://github.com/V-Sekai-fire/interactor-mujoco-cloth-sim),
takes the hard half: iterative cloth constraint solving, the most order-sensitive
physics there is, made bit-identical anyway.

## Why the agents cannot topple

Each pedestrian is a vertical capsule on two slide joints — x and y in the floor
plane, with no vertical or rotational degree of freedom. It can be pushed and
push back, but it can never fall over. Toppling is a rotational contact response,
which is exactly where a convex solver's iteration order changes the answer
between CPUs; removing that degree of freedom is what keeps the crowd
deterministic at density.

## Layout

    scripts/make_crowd.py      the model generator (XML AST) and its self-test
    project/plans/crowd.xml     the generated MJCF

## Build the model

    python scripts/make_crowd.py                 # default 128 agents
    python scripts/make_crowd.py --agents 1024   # turn up the scale
    python scripts/make_crowd.py --self-test     # 10 controls, incl. a negative control

The steering policy is not in the model. It is the taskweft HTN agent that drives
the per-agent actuators from inside the guest.

## Credit

V-Sekai-fire and chibifire.
