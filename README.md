# chrono-mcity

The [Mcity Test Facility digital twin](https://github.com/mcity/mcity-digital-twin) as a drivable
[PyChrono](https://projectchrono.org) scene, in one Python script. It runs on stock PyChrono.
Nothing is converted and Chrono is not modified.

![An Audi on a Mcity road in the Chrono VSG window](docs/mcity.jpg)

## Run it

```sh
conda install projectchrono::pychrono -c conda-forge
curl -LO https://raw.githubusercontent.com/ksha23/chrono-mcity/main/mcity.py
python mcity.py
```

The first run downloads the scene (200 MB) into `scene/` beside the script, checks its SHA-256
and unpacks it. Later runs start in a few seconds.

Drive with **W/S** for throttle and brake and **A/D** to steer.

| Option | |
| --- | --- |
| `--foliage LEVEL` | vegetation: `none`, `trees`, `trees-leaf`, `shrubs` or `full`. See below |
| `--data DIR` | keep the scene somewhere else |
| `--tire pac02\|tmeasy\|rigid` | tire model, default `pac02` |
| `--tire-step S` | tire internal step in seconds, default `1e-4` |
| `--speed-limit V` | speed the throttle ramp is scaled toward, default 20 m/s |
| `--duration S` | stop after S simulated seconds |
| `--headless` | no window. Sets the car down with the brakes on and checks that it rests on the road |
| `--force` | load a vegetation level even if it looks too big for this machine |

PyChrono has to come from the `projectchrono` conda channel. The `conda-forge` package of the
same name has no vehicle or VSG module.

## Vegetation

Trees and shrubs are optional. Asking for a level downloads a second archive (160 MB) the first
time, into the same `scene/` directory:

```sh
python mcity.py --foliage trees
```

| Level | Plants | Triangles drawn | On stock PyChrono (build 1187, M4 Pro, 48 GB) |
| --- | --- | --- | --- |
| `none` | none | 1.4 M | real time, 5 GB |
| `trees` | 383 trees, bare branches | 13 M | 0.26x real time, 10 GB |
| `trees-leaf` | 447 trees with leaves | 22 M | about 0.15x real time, 14 GB |
| `shrubs` | 1,623 trees and shrubs, bare branches | 60 M | does not run: the GPU ran out of memory at 32 GB |
| `full` | 2,009 trees and shrubs with leaves | 143 M | not tried, about 70 GB by extrapolation |

The plants are scan-grade models, and stock Chrono::VSG gives every placement of a mesh its own
vertex buffer and draws all of them every frame. That is why the cost follows the triangle count
so closely, and why the two heavy levels are out of reach on stock PyChrono. The script checks
the level against the machine's memory before downloading anything and stops if it will not fit.
`--force` overrides that.

The heavy levels were built for a Chrono with two renderer changes, vertex welding and frustum
culling, which are on the `feature/mcity-vehicle-demo` branch of
[ksha23/chrono](https://github.com/ksha23/chrono/tree/feature/mcity-vehicle-demo) and not in
upstream Chrono. With a PyChrono built from that branch, this same script loads `shrubs` in
12.5 GB at 0.15x real time.

## Use Mcity in your own simulation

`mcity.py` is also a module. Put it next to your script:

```python
import pychrono as chrono
import pychrono.vehicle as veh
import mcity

system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

scene = mcity.fetch()                      # scene directory, downloaded if it is not there
mcity.add_scenery(system, scene)           # what you see: visual shapes, no collision
terrain = mcity.add_ground(system, scene)  # what the wheels touch: one collision mesh

z = mcity.ground_height(scene, mcity.START_X, mcity.START_Y)   # road height at the start pose
```

For vegetation, pass the level to both calls: `mcity.fetch(foliage="trees")` and
`mcity.add_scenery(system, scene, foliage="trees")`.

- `add_scenery` puts every placement on a few fixed bodies as visual shapes. Pass
  `groups=["Static", "Terrain"]` to load only some of `Static`, `TrafficPoles`, `TrafficLights`,
  `StreetLights`, `TrafficLightCables`, `Terrain` and, with vegetation, `Foliage_Instanced`.
- `add_ground` returns an initialized `RigidTerrain` over the road, sidewalk, curb and island
  surfaces. They are the same triangles the scenery draws, so what you see is what you drive on.
- The site keeps its real elevation. The road is near **z = 274 m**, not z = 0. Use
  `ground_height` to place things: `RigidTerrain.GetHeight` returns 0 until the first
  `DoStepDynamics`.
- `START_X`, `START_Y`, `START_YAW` are a pose on a lane, facing along it.

The scene is plain files, so C++ Chrono or any other tool can read it too. `mcity_ground.obj`
works directly with `RigidTerrain::AddPatch`.

## What is in the scene

```
mcity_scene.json     placement manifest: 229 assets, 859 placements in 6 groups
mcity_ground.obj     drivable surfaces merged in world space, 111,258 triangles
assets/              527 OBJ meshes, one per (asset, material)
textures/            570 PNG maps: base colour, normal, roughness, metallic
LICENSE.mcity        the upstream MIT notice
README.txt           the upstream and converter commits this copy was built from
```

Lengths are metres and Z is up, Chrono's own frame. The manifest looks like this:

```json
{
  "assets": [
    { "name": "SM_BarrierNose_s001_v01",
      "parts": [
        { "mesh": "assets/SM_BarrierNose_s001_v01__MI_BarrierNose_s001_Metal.obj",
          "texture": "textures/T_BarrierNose_s001_Metal_BC.png",
          "normal": "textures/T_BarrierNose_s001_Metal_NRM.png",
          "roughness": "textures/T_BarrierNose_s001_Metal_RGH.png",
          "metallic": "textures/T_BarrierNose_s001_Metal_MET.png",
          "colour": [1.0, 1.0, 1.0], "ks": [0.05, 0.05, 0.05], "ns": 10.0 } ] } ],
  "instances": [
    { "asset": 0, "group": "Static",
      "pos": [156.7052, -64.9924, 271.0522],
      "rot": [0.999323, 0.011311, -0.033883, -0.008853],
      "scale": [1.0, 1.0, 1.0] } ]
}
```

An asset is a list of single-material meshes. An instance places asset number `asset` at `pos`
with `rot` as a (w, x, y, z) quaternion and a per-axis `scale`. Paths are relative to the
manifest, and a texture entry is `null` where upstream published no map for that material.

The vegetation archive adds one manifest per level (`mcity_scene_trees_bare.json`,
`mcity_scene_trees_leaf.json`, `mcity_scene_all_bare.json`, `mcity_scene_full.json`) and a
`lod_*` mesh directory for each. It unpacks over the base scene.

To fetch the scene without the script:

```sh
curl -LO https://github.com/ksha23/chrono-mcity/releases/download/v1/mcity_scene_base.tar.gz
echo "41b0e14eb0a10609fde95621a2085ab194d8aa4de45054bb8f09a76a766a41f7  mcity_scene_base.tar.gz" | shasum -a 256 -c
mkdir scene && tar -xzf mcity_scene_base.tar.gz -C scene

# optional vegetation, over the top
curl -LO https://github.com/ksha23/chrono-mcity/releases/download/v1/mcity_scene_foliage.tar.gz
echo "246434ba4e3249fd50b08cf50411b38f48bd6d451575a3731401139c35995c87  mcity_scene_foliage.tar.gz" | shasum -a 256 -c
tar -xzf mcity_scene_foliage.tar.gz -C scene
```

## What is not here

- **The road network.** Upstream also publishes an OpenDRIVE file. This scene has no lanes or
  junctions, only surfaces. Get `McityMap_Main.xodr` from the upstream repository if you need it.

## Tested with

PyChrono 10.0.0 from the `projectchrono` channel, conda builds `py313_1187` and `py312_677`, on
macOS (Apple silicon). Linux and Windows have not been tried.

## How the scene was built

Converted once from the upstream USD stage, so nobody else has to:

| | |
| --- | --- |
| Upstream | [`mcity/mcity-digital-twin@3e8096b`](https://github.com/mcity/mcity-digital-twin/tree/3e8096b8ea2e48762cd512839d9dc8559814f6e6), stage `Omniverse/Collected_McityMap_NSR_v4_1_6/McityMap_Main.usdc` |
| Converter | [`ksha23/chrono@529de85`](https://github.com/ksha23/chrono/tree/529de857033aa420fe456fae0180737832224f2f/src/demos/vehicle/terrain/mcity), `usd_to_chrono.py` and `resolve_textures.py` |

Re-running the converter at that commit reproduces the base scene's manifest, its ground mesh
and all 527 of its meshes byte for byte. The vegetation levels come from `decimate_foliage.py`
in the same directory and were not re-run for this check.

## Licence and credit

The scene is a converted copy of the Mcity digital twin, which is MIT licensed, copyright
Quantum Signal AI LLC and The Regents of the University of Michigan. That notice is in
[`LICENSE.mcity`](LICENSE.mcity) and travels inside the archive. The 3D content was created by
[Quantum Signal AI](https://quantumsignalai.com/) for the University of Michigan's
[Mcity Test Facility](https://mcity.umich.edu/what-we-do/mcity-test-facility/). The upstream
README notes that some traffic sign meshes and textures come from the CARLA asset library, which
CARLA distributes under CC-BY.

`mcity.py` is under the BSD 3-Clause licence in [`LICENSE`](LICENSE), the same terms as Chrono.

This repository is not affiliated with Mcity, Quantum Signal AI or the University of Michigan.
