# interactor-taskweft-crowd

A deterministic transit-station crowd: planning agents steer one shared physics world that steps bit-identically on every host.

## What it is for

Pedestrians cross a pillared concourse, each a taskweft HTN agent choosing where to go while MuJoCo, running as a RISC-V sandbox guest, moves them. Each pedestrian is a capsule that slides in the floor plane and cannot rotate, which keeps contact resolution identical across CPUs at high density. `interactor-mujoco-cloth-sim` takes the harder case, cloth, with the same guarantee.

## Building and running

```sh
python scripts/make_crowd.py
```

That regenerates the crowd model, and its `--help` lists the options. Open `project` in Godot 4 and run the main scene to watch the crowd.

## Licence

MIT, as [CITATION.cff](CITATION.cff) declares. The addons in the Godot project keep their own licences.
