#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""Drive an Audi around the Mcity digital twin in PyChrono.

    conda install projectchrono::pychrono -c conda-forge    # once, from the projectchrono channel
    python mcity.py

The first run downloads the scene (200 MB) into scene/ beside this file and checks its hash.
After that it starts straight away. Nothing is converted and Chrono is not modified: this needs
only a stock PyChrono with the vehicle and VSG modules.

Controls: W/S throttle and brake, A/D steer, plus the usual VSG camera keys.

To put Mcity in a simulation of your own, import this file:

    import mcity
    scene = mcity.fetch()                      # scene directory, downloaded if it is not there
    mcity.add_scenery(system, scene)           # what you see: visual shapes, no collision
    terrain = mcity.add_ground(system, scene)  # what the wheels touch: one collision mesh
    z = mcity.ground_height(scene, x, y)       # road height, usable before the first step

The scene is a converted copy of https://github.com/mcity/mcity-digital-twin (MIT).
"""

import argparse
import hashlib
import json
import math
import os
import shutil
import sys
import tarfile
import time
import urllib.request

# The published scene. One pinned archive, checked against its hash before anything is unpacked,
# so a changed or truncated download fails here and not later as a half-loaded scene.
SCENE_URL = "https://github.com/ksha23/chrono-mcity/releases/download/v1/mcity_scene_base.tar.gz"
SCENE_SHA256 = "41b0e14eb0a10609fde95621a2085ab194d8aa4de45054bb8f09a76a766a41f7"
SCENE_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "scene")

MANIFEST = "mcity_scene.json"
GROUND = "mcity_ground.obj"

# A pose on a real Mcity lane, facing along the carriageway: x, y in metres and yaw in radians.
# The site keeps its real elevation, so the road here is near z = 274 m and not z = 0.
START_X, START_Y, START_YAW = 158.923, 62.991, 1.285431


# --------------------------------------------------------------------------------------------
# Getting the scene
# --------------------------------------------------------------------------------------------


def fetch(scene_dir=SCENE_DIR, url=None):
    """Return a directory holding the Mcity scene, downloading and unpacking it if needed."""
    scene_dir = os.path.abspath(scene_dir)
    if os.path.isfile(os.path.join(scene_dir, MANIFEST)) and os.path.isfile(os.path.join(scene_dir, GROUND)):
        return scene_dir
    if os.path.isdir(scene_dir) and os.listdir(scene_dir):
        raise SystemExit(f"{scene_dir} exists but holds no Mcity scene. Remove it, or pass another --data directory.")

    url = url or os.environ.get("MCITY_SCENE_URL", SCENE_URL)
    parent = os.path.dirname(scene_dir)
    os.makedirs(parent, exist_ok=True)
    archive = scene_dir + ".download"
    partial = scene_dir + ".partial"

    print(f"Mcity scene not found in {scene_dir}")
    print(f"  downloading {url}")
    digest = hashlib.sha256()
    try:
        with urllib.request.urlopen(url) as src, open(archive, "wb") as dst:
            total = int(src.headers.get("Content-Length") or 0)
            done, shown = 0, -1
            while True:
                block = src.read(1 << 20)
                if not block:
                    break
                dst.write(block)
                digest.update(block)
                done += len(block)
                percent = 100 * done // total if total else 0
                if percent != shown and sys.stdout.isatty():
                    print(f"\r  {done >> 20} MB  {percent:3d}%", end="", flush=True)
                    shown = percent
        if sys.stdout.isatty():
            print()
    except Exception as err:
        _remove(archive)
        raise SystemExit(f"  download failed: {err}\n  Fetch it by hand (see the README) and pass --data DIR.")

    if digest.hexdigest() != SCENE_SHA256:
        _remove(archive)
        raise SystemExit(f"  checksum mismatch\n    expected {SCENE_SHA256}\n    got      {digest.hexdigest()}")

    # Unpack beside the target and rename at the end, so an interrupted run cannot leave behind
    # something that looks like a scene but is missing half its files.
    print("  unpacking")
    shutil.rmtree(partial, ignore_errors=True)
    with tarfile.open(archive) as tar:
        try:
            tar.extractall(partial, filter="data")
        except TypeError:  # Python older than 3.12 has no extraction filters
            tar.extractall(partial)
    if os.path.isdir(scene_dir):
        os.rmdir(scene_dir)
    os.replace(partial, scene_dir)
    _remove(archive)
    print(f"  scene ready in {scene_dir}")
    return scene_dir


def _remove(path):
    try:
        os.remove(path)
    except OSError:
        pass


# --------------------------------------------------------------------------------------------
# Loading it into a Chrono system
# --------------------------------------------------------------------------------------------


def add_scenery(system, scene_dir, manifest=MANIFEST, groups=None, verbose=True):
    """Add everything you see: buildings, poles, signals, signs, barriers and the road surface.

    The manifest lists a few hundred meshes and the placements they appear at. Each mesh becomes
    one visual shape, added to a body once per placement, so its triangles are loaded once however
    often it appears. The bodies are fixed and carry no collision geometry. Scenery never reaches
    the solver, and the driving surface is a separate object: see add_ground.

    groups, if given, keeps only those manifest groups. Returns the bodies, one per group.
    """
    import pychrono as chrono

    with open(os.path.join(scene_dir, manifest)) as f:
        doc = json.load(f)
    assets = doc["assets"]

    meshes = {}     # mesh path -> ChTriangleMeshConnected, or None if unusable
    materials = {}  # mesh path -> ChVisualMaterial
    shapes = {}     # (asset index, scale) -> the asset's parts as visual shapes
    bodies = {}     # group -> ChBody
    placed = {}     # group -> number of placements
    textured = skipped = 0

    for inst in doc["instances"]:
        group = inst.get("group", "default")
        if groups is not None and group not in groups:
            continue

        # A shape is shared by every placement of the same asset at the same scale. Scale lives
        # on the shape and not on the placement frame, so differing scales need their own shape.
        # Rounding keeps near-identical scales together.
        scale = inst.get("scale", [1.0, 1.0, 1.0])
        key = (inst["asset"], tuple(round(s * 1000) for s in scale))
        parts = shapes.get(key)
        if parts is None:
            parts = []
            for part in assets[inst["asset"]]["parts"]:
                path = os.path.join(scene_dir, part["mesh"])
                if path not in meshes:
                    mesh = None
                    if os.path.isfile(path):
                        # Texture coordinates are not loaded by default and a texture needs them.
                        mesh = chrono.ChTriangleMeshConnected.CreateFromWavefrontFile(path, True, True)
                        if mesh is not None and mesh.GetNumTriangles() == 0:
                            mesh = None
                    meshes[path] = mesh
                mesh = meshes[path]
                if mesh is None:
                    continue

                # One material per part, shared by every scale of the asset.
                material = materials.get(path)
                if material is None:
                    material = chrono.ChVisualMaterial()
                    ns = float(part.get("ns", 10.0))
                    material.SetDiffuseColor(chrono.ChColor(*part.get("colour", [0.62, 0.62, 0.64])))
                    material.SetSpecularColor(chrono.ChColor(*part.get("ks", [0.05, 0.05, 0.05])))
                    material.SetSpecularExponent(ns)
                    # Wavefront Ns is the inverse sense of PBR roughness.
                    material.SetRoughness(1.0 - min(1.0, ns / 100.0))
                    texture = _existing(scene_dir, part.get("texture"))
                    if texture:
                        material.SetKdTexture(texture)
                        textured += 1
                    normal = _existing(scene_dir, part.get("normal"))
                    if normal:
                        material.SetNormalMapTexture(normal)
                    # Chrono's VSG backend packs these two into one texture and needs both.
                    roughness = _existing(scene_dir, part.get("roughness"))
                    metallic = _existing(scene_dir, part.get("metallic"))
                    if roughness and metallic:
                        material.SetRoughnessTexture(roughness)
                        material.SetMetallicTexture(metallic)
                    materials[path] = material

                shape = chrono.ChVisualShapeTriangleMesh()
                # load_materials=False matters. The default re-parses the OBJ on every call to
                # look for an MTL, which costs minutes over a scene this size and finds nothing,
                # because the material is attached explicitly below.
                shape.SetMesh(mesh, False)
                shape.SetScale(chrono.ChVector3d(*scale))
                shape.SetMutable(False)
                shape.AddMaterial(material)
                parts.append(shape)
            shapes[key] = parts
        if not parts:
            skipped += 1
            continue

        body = bodies.get(group)
        if body is None:
            body = chrono.ChBody()
            body.SetName("scenery_" + group)
            body.SetFixed(True)
            body.EnableCollision(False)
            system.Add(body)
            bodies[group] = body

        rot = chrono.ChQuaterniond(*inst.get("rot", [1.0, 0.0, 0.0, 0.0]))
        rot.Normalize()
        frame = chrono.ChFramed(chrono.ChVector3d(*inst.get("pos", [0.0, 0.0, 0.0])), rot)
        for shape in parts:
            body.AddVisualShape(shape, frame)
        placed[group] = placed.get(group, 0) + 1

    if verbose:
        used = sum(1 for m in meshes.values() if m is not None)
        print(f"  scenery: {sum(placed.values())} placements of {used} meshes, {textured} textured")
        for group in sorted(placed):
            print(f"    {placed[group]:5d}  {group}")
        if skipped:
            print(f"    ({skipped} placements skipped: their meshes are missing)")
    return list(bodies.values())


def _existing(scene_dir, relative):
    if not relative:
        return None
    path = os.path.join(scene_dir, relative)
    return path if os.path.isfile(path) else None


def add_ground(system, scene_dir, friction=0.9, restitution=0.01, young_modulus=2e7):
    """Add the driving surface and return it as an initialized RigidTerrain.

    Roads, sidewalks, curbs and traffic islands, merged into one mesh in world coordinates. These
    are the same triangles add_scenery draws, so what you see and what the wheels touch are the
    same surface. It is collision only, since drawing it a second time would z-fight.
    Keep the returned object alive for as long as the system uses it.
    """
    import pychrono as chrono
    import pychrono.vehicle as veh

    info = chrono.ChContactMaterialData()
    info.mu = friction
    info.cr = restitution
    info.Y = young_modulus
    material = info.CreateMaterial(system.GetContactMethod())

    terrain = veh.RigidTerrain(system)
    # connected_mesh=True is the one to use. Measured on PyChrono build 1187, the triangle-soup
    # alternative takes 4.5 s to enter the collision system instead of 0.2 s, and its height
    # queries land 30 mm below the real surface.
    terrain.AddPatch(material, chrono.CSYSNORM, os.path.join(scene_dir, GROUND), True, 0.0, False)
    terrain.Initialize()
    return terrain


def ground_height(scene_dir, x, y, radius=2.0):
    """Height of the road near (x, y), read straight from the ground mesh.

    RigidTerrain.GetHeight raycasts the collision system and returns 0 on a miss. Asked during
    setup, before the first step, it misses, and 0 is 274 m below this road. Reading the mesh
    works at any time. Returns None if no vertex lies within radius of the point.
    """
    best = None
    with open(os.path.join(scene_dir, GROUND)) as f:
        for line in f:
            if line.startswith("v "):
                vx, vy, vz = map(float, line.split()[1:4])
                if abs(vx - x) < radius and abs(vy - y) < radius and (best is None or vz > best):
                    best = vz
    return best


# --------------------------------------------------------------------------------------------
# The demo
# --------------------------------------------------------------------------------------------

TIRES = {"pac02": "audi/json/audi_Pac02Tire.json", "tmeasy": "audi/json/audi_TMeasyTire.json", "rigid": "audi/json/audi_RigidTire.json"}


def main():
    parser = argparse.ArgumentParser(description="Drive an Audi around the Mcity digital twin. W/S throttle and brake, A/D steer.")
    parser.add_argument("--data", metavar="DIR", default=SCENE_DIR, help="scene directory, downloaded into if empty (default: scene/ beside this file)")
    parser.add_argument("--tire", choices=sorted(TIRES), default="pac02", help="tire model (default: pac02)")
    parser.add_argument("--tire-step", metavar="S", type=float, default=1e-4, help="tire internal step in seconds (default: 1e-4)")
    parser.add_argument("--speed-limit", metavar="V", type=float, default=20.0, help="speed the throttle ramp is scaled toward, m/s (default: 20)")
    parser.add_argument("--duration", metavar="S", type=float, default=None, help="stop after this many simulated seconds (default: run until the window closes)")
    parser.add_argument("--headless", action="store_true", help="no window: simulate with the vehicle parked and print where it is")
    args = parser.parse_args()

    try:
        import pychrono as chrono
        import pychrono.vehicle as veh
        if not args.headless:
            import pychrono.vsg3d  # noqa: F401, checked here so a missing module fails before the download
    except ImportError as err:
        raise SystemExit(
            f"This needs PyChrono with the vehicle and VSG modules ({err}).\n"
            "The projectchrono conda channel ships them. The conda-forge pychrono package does not.\n"
            "  conda install projectchrono::pychrono -c conda-forge"
        )

    scene = fetch(args.data)
    boot = time.perf_counter()

    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    # The default solver under-solves a vehicle's suspension constraints, which shows up as the
    # suspension juddering for no visible reason. These are the settings Chrono's own road demos use.
    system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
    system.GetSolver().AsIterative().SetMaxIterations(150)
    system.SetMaxPenetrationRecoverySpeed(4.0)

    if not args.headless:
        add_scenery(system, scene)
    terrain = add_ground(system, scene)

    z = ground_height(scene, START_X, START_Y)
    if z is None:
        raise SystemExit(f"no ground found under the start pose in {os.path.join(scene, GROUND)}")

    audi = veh.WheeledVehicle(system, veh.GetVehicleDataFile("audi/json/audi_Vehicle.json"))
    audi.Initialize(chrono.ChCoordsysd(chrono.ChVector3d(START_X, START_Y, z + 0.5), chrono.QuatFromAngleZ(START_YAW)))
    audi.SetChassisVisualizationType(chrono.VisualizationType_MESH)
    audi.SetSuspensionVisualizationType(chrono.VisualizationType_MESH)
    audi.SetSteeringVisualizationType(chrono.VisualizationType_MESH)
    audi.SetWheelVisualizationType(chrono.VisualizationType_MESH)

    engine = veh.ReadEngineJSON(veh.GetVehicleDataFile("audi/json/audi_EngineSimpleMap.json"))
    transmission = veh.ReadTransmissionJSON(veh.GetVehicleDataFile("audi/json/audi_AutomaticTransmissionSimpleMap.json"))
    audi.InitializePowertrain(veh.ChPowertrainAssembly(engine, transmission))

    for axle in audi.GetAxles():
        for wheel in axle.GetWheels():
            tire = veh.ReadTireJSON(veh.GetVehicleDataFile(TIRES[args.tire]))
            tire.SetStepsize(args.tire_step)
            audi.InitializeTire(tire, wheel, chrono.VisualizationType_MESH)

    step = 1e-3
    if args.headless:
        run_headless(system, audi, terrain, veh, step, args.duration if args.duration is not None else 5.0, z)
    else:
        run_window(system, audi, terrain, chrono, veh, step, args, boot)

    # Tearing the visual system down from Python can crash on the way out, after everything has
    # already worked. Leave without running destructors.
    sys.stdout.flush()
    os._exit(0)


def run_window(system, audi, terrain, chrono, veh, step, args, boot):
    driver = veh.ChInteractiveDriver(audi)
    driver.SetSteeringDelta(0.04)
    driver.SetThrottleDelta(1.0 / max(1.0, args.speed_limit))
    driver.SetBrakingDelta(0.3)
    driver.Initialize()

    vis = veh.ChWheeledVehicleVisualSystemVSG()
    vis.SetWindowTitle("Mcity")
    vis.SetWindowSize(1600, 900)
    vis.AttachVehicle(audi)
    vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 7.0, 0.6)
    vis.SetLightIntensity(1.0)
    vis.SetLightDirection(1.5 * chrono.CH_PI_2, chrono.CH_PI_4)
    vis.EnableShadows()
    vis.AttachDriver(driver)
    vis.Initialize()

    print(f"\n  [{time.perf_counter() - boot:.1f} s to build the scene and open the window]")
    print("W/S throttle and brake, A/D steer.\n")

    render_step = 1.0 / 50  # physics wants 1 kHz, the display does not
    next_render = next_report = 0.0
    realtime = chrono.ChRealtimeStepTimer()
    start = time.perf_counter()

    while vis.Run():
        now = system.GetChTime()
        if args.duration is not None and now >= args.duration:
            break

        if now >= next_render:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
            next_render += render_step

        if now >= next_report:
            wall = time.perf_counter() - start
            print(f"  t={now:5.1f} s   {now / wall if wall > 0 else 0:.2f}x real time   {audi.GetSpeed():5.1f} m/s")
            next_report += 2.0

        inputs = driver.GetInputs()
        driver.Synchronize(now)
        terrain.Synchronize(now)
        audi.Synchronize(now, inputs, terrain)
        vis.Synchronize(now, inputs)

        driver.Advance(step)
        terrain.Advance(step)
        audi.Advance(step)
        vis.Advance(step)
        system.DoStepDynamics(step)

        # Pace once per rendered frame and not once per physics step, so the fast steps between
        # two frames can absorb the cost of drawing one.
        if now >= next_render - step:
            realtime.Spin(render_step)


def run_headless(system, audi, terrain, veh, step, duration, road_z):
    inputs = veh.DriverInputs()
    inputs.m_braking = 1.0
    print(f"  headless: {duration:g} s with the brakes on, road at z = {road_z:.2f} m")
    next_report = 0.0
    while system.GetChTime() < duration:
        now = system.GetChTime()
        if now >= next_report:
            pos = audi.GetPos()
            print(f"  t={now:4.1f} s   x={pos.x:8.2f}  y={pos.y:8.2f}  z={pos.z:7.2f}   {audi.GetSpeed():5.2f} m/s")
            next_report += 1.0
        terrain.Synchronize(now)
        audi.Synchronize(now, inputs, terrain)
        terrain.Advance(step)
        audi.Advance(step)
        system.DoStepDynamics(step)
    pos = audi.GetPos()
    sunk = road_z - pos.z
    if sunk > 0.5 or abs(pos.z - road_z) > 2.0:
        raise SystemExit(f"  FAILED: the vehicle is at z = {pos.z:.2f} m, the road is at {road_z:.2f} m")
    print(f"  ok: resting {pos.z - road_z:.2f} m above the road surface (chassis reference height)")


if __name__ == "__main__":
    main()
