# interactor-taskweft-crowd

A transit-station crowd model in one shared physics world, built to be driven by taskweft planning agents.

## What it is for

Pedestrians cross a pillared concourse while MuJoCo, running as a RISC-V sandbox guest, moves them. Each pedestrian is a capsule that slides in the floor plane and cannot rotate, which keeps contact resolution simple at high density. Steering is a stand-in that pushes every pedestrian along +x; taskweft HTN agents are the intended driver.

## Building and running

```sh
python scripts/make_crowd.py
```

That regenerates the crowd model, and its `--help` lists the options. Open `project` in Godot 4 on Windows or Linux x86_64 and run the main scene to watch the crowd; the sandbox and video addons ship no macOS binaries.

## Licence

MIT. See [LICENSE](LICENSE). The addons in the Godot project keep their own licences.
