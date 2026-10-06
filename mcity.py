#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""Drive an Audi around the Mcity digital twin in PyChrono.

    conda install projectchrono::pychrono -c conda-forge    # once, from the projectchrono channel
    python mcity.py

The first run downloads the scene (211 MB) into scene/ beside this file and checks its hash.
After that it starts straight away. Nothing is converted and Chrono is not modified: this needs
only a stock PyChrono with the vehicle and VSG modules.

Controls: W/S throttle and brake, A/D steer, plus the usual VSG camera keys.

Vegetation is optional, a second download that happens the first time you ask for it:

    python mcity.py --foliage trees     # or trees-leaf, shrubs, full

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

# The published scene. Pinned archives, each checked against its hash before anything is unpacked,
# so a changed or truncated download fails here and not later as a half-loaded scene.
RELEASE = "https://github.com/ksha23/chrono-mcity/releases/download/v2/"
SCENE_URL = RELEASE + "mcity_scene_base.tar.gz"
SCENE_SHA256 = "daf79764350bba37878437541de591e152d8187e5a35aa0878054559b154735a"
# Vegetation, as an add-on that unpacks over the base scene. Only fetched when asked for.
FOLIAGE_URL = RELEASE + "mcity_scene_foliage.tar.gz"
FOLIAGE_SHA256 = "443f33b83a76f4d8158f441d238473087a9e779f3d194407a307ad9daad33527"
SCENE_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "scene")

MANIFEST = "mcity_scene.json"
GROUND = "mcity_ground.obj"

# Vegetation levels and the manifest each one loads. The source plants are scan-grade models, so
# every level is a different answer to what to give up: shrubs, leaves, or neither.
FOLIAGE = {
    "none": MANIFEST,
    "trees": "mcity_scene_trees_bare.json",
    "trees-leaf": "mcity_scene_trees_leaf.json",
    "shrubs": "mcity_scene_all_bare.json",
    "full": "mcity_scene_full.json",
}

# What each level needs in memory on stock PyChrono, in GB, measured on build 1187. Stock
# Chrono::VSG gives every placement of a mesh its own vertex buffer and draws all of them again
# for each shadow map, so every level is built to stay under 6 M triangles: 1.4 M with no
# vegetation, then 4.0, 5.5, 5.9 and 5.9 M.
FOLIAGE_MEMORY_GB = {"none": 5, "trees": 7, "trees-leaf": 8, "shrubs": 8, "full": 8}

# The scene format this script expects. fetch() replaces an older scene it installed itself.
SCENE_VERSION = 2
MARKER = ".mcity-scene"

# A pose on a real Mcity lane, facing along the carriageway: x, y in metres and yaw in radians.
# The site keeps its real elevation, so the road here is near z = 274 m and not z = 0.
START_X, START_Y, START_YAW = 158.923, 62.991, 1.285431


# --------------------------------------------------------------------------------------------
# Getting the scene
# --------------------------------------------------------------------------------------------


def fetch(scene_dir=SCENE_DIR, foliage="none"):
    """Return a directory holding the Mcity scene, downloading and unpacking it if needed.

    foliage is one of the FOLIAGE levels. Anything but "none" also needs the vegetation add-on,
    which is fetched into the same directory the first time it is asked for.
    """
    if foliage not in FOLIAGE:
        raise ValueError(f"unknown foliage level {foliage!r}, expected one of {', '.join(FOLIAGE)}")
    scene_dir = os.path.abspath(scene_dir)

    have = _has(scene_dir, MANIFEST) and _has(scene_dir, GROUND)
    if have and _version(scene_dir) < SCENE_VERSION:
        if _has(scene_dir, MARKER):
            # An older scene that an earlier version of this script put here. It is a download
            # cache and nothing else, so it is set aside and fetched again.
            old = f"{scene_dir}.v{_version(scene_dir)}"
            print(f"The scene in {scene_dir} is an older version. Moving it to {old} and fetching the current one.")
            if os.path.exists(old):
                raise SystemExit(f"{old} is in the way. Remove it and run again.")
            os.replace(scene_dir, old)
            have = False
        else:
            print(f"Note: {scene_dir} holds an older scene (version {_version(scene_dir)}, this script expects "
                  f"{SCENE_VERSION}). It was not installed by this script, so it is used as it is.")

    if not have:
        if os.path.isdir(scene_dir) and os.listdir(scene_dir):
            raise SystemExit(f"{scene_dir} exists but holds no Mcity scene. Remove it, or pass another --data directory.")
        print(f"Mcity scene not found in {scene_dir}")
        partial = _download_and_unpack(os.environ.get("MCITY_SCENE_URL", SCENE_URL), SCENE_SHA256, scene_dir)
        with open(os.path.join(partial, MARKER), "w") as f:
            f.write(f"installed by mcity.py, scene version {SCENE_VERSION}\n")
        if os.path.isdir(scene_dir):
            os.rmdir(scene_dir)
        os.replace(partial, scene_dir)
        print(f"  scene ready in {scene_dir}")

    if foliage != "none" and not all(_has(scene_dir, m) for m in FOLIAGE.values()):
        print(f"Mcity vegetation not found in {scene_dir}")
        partial = _download_and_unpack(os.environ.get("MCITY_FOLIAGE_URL", FOLIAGE_URL), FOLIAGE_SHA256, scene_dir)
        _merge(partial, scene_dir)
        print("  vegetation ready")

    return scene_dir


def _version(scene_dir):
    """The scene format version a manifest declares. The first published scene declared none."""
    try:
        with open(os.path.join(scene_dir, MANIFEST)) as f:
            return int(json.load(f).get("version", 1))
    except (OSError, ValueError):
        return 0


def _has(scene_dir, name):
    return os.path.isfile(os.path.join(scene_dir, name))


def _download_and_unpack(url, sha256, scene_dir):
    """Download one archive, check it, and unpack it beside scene_dir. Returns the unpacked path.

    Unpacking beside the target and moving into place afterwards means an interrupted run cannot
    leave behind something that looks like a scene but is missing half its files.
    """
    os.makedirs(os.path.dirname(scene_dir), exist_ok=True)
    archive = scene_dir + ".download"
    partial = scene_dir + ".partial"

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

    if digest.hexdigest() != sha256:
        _remove(archive)
        raise SystemExit(f"  checksum mismatch\n    expected {sha256}\n    got      {digest.hexdigest()}")

    print("  unpacking")
    shutil.rmtree(partial, ignore_errors=True)
    with tarfile.open(archive) as tar:
        try:
            tar.extractall(partial, filter="data")
        except TypeError:  # Python older than 3.12 has no extraction filters
            tar.extractall(partial)
    _remove(archive)
    return partial


def _merge(src, dst):
    """Move an unpacked add-on into an existing scene, manifests last.

    A manifest is what says its meshes are present, so it must not arrive before they do.
    """
    manifests = []
    for root, _, files in os.walk(src):
        rel = os.path.relpath(root, src)
        os.makedirs(os.path.join(dst, rel), exist_ok=True)
        for name in files:
            pair = (os.path.join(root, name), os.path.join(dst, rel, name))
            if rel == "." and name.endswith(".json"):
                manifests.append(pair)
            else:
                os.replace(*pair)
    for pair in manifests:
        os.replace(*pair)
    shutil.rmtree(src, ignore_errors=True)


def _remove(path):
    try:
        os.remove(path)
    except OSError:
        pass


# --------------------------------------------------------------------------------------------
# Loading it into a Chrono system
# --------------------------------------------------------------------------------------------


def add_scenery(system, scene_dir, foliage="none", groups=None, signals=None, verbose=True):
    """Add everything you see: buildings, poles, signals, signs, barriers and the road surface.

    The manifest lists a few hundred meshes and the placements they appear at. Each mesh becomes
    one visual shape, added to a body once per placement, so its triangles are loaded once however
    often it appears. The bodies are fixed and carry no collision geometry. Scenery never reaches
    the solver, and the driving surface is a separate object: see add_ground.

    foliage picks the vegetation level, one of FOLIAGE. Anything but "none" needs the add-on
    that fetch(foliage=...) downloads. groups, if given, keeps only those manifest groups.

    signals lights the traffic signal lenses: "red", "amber", "green", or "all". The scene is
    static and has no signal phases, so by default every lens is dark.

    Every part's material also gets a class id for Chrono::Sensor's segmentation camera, taken
    from the label upstream gave the asset. labels(scene_dir) lists what the ids mean.

    Returns the bodies, one per group.
    """
    import pychrono as chrono

    manifest = os.path.join(scene_dir, FOLIAGE[foliage])
    if not os.path.isfile(manifest):
        raise SystemExit(f"{manifest} is missing. Call mcity.fetch(foliage={foliage!r}) first.")
    with open(manifest) as f:
        doc = json.load(f)
    assets = doc["assets"]
    class_ids = _class_ids(doc)

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
                    material.SetRoughness(part.get("roughness_value", 1.0 - min(1.0, ns / 100.0)))
                    if "metallic_value" in part:
                        material.SetMetallic(part["metallic_value"])
                    if "uv_scale" in part:
                        material.SetTextureScale(*part["uv_scale"])
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
                    ao = _existing(scene_dir, part.get("ao"))
                    if ao:
                        material.SetAmbientOcclusionTexture(ao)
                    # Glass and netting. Without this they draw as solid panels.
                    opacity = _existing(scene_dir, part.get("opacity"))
                    if opacity:
                        material.SetOpacityTexture(opacity)
                    if "emissive" in part and _lit(part["name"], signals):
                        material.SetEmissiveColor(chrono.ChColor(*part["emissive"]))
                        glow = _existing(scene_dir, part.get("emissive_texture"))
                        if glow:
                            material.SetKeTexture(glow)
                    if inst["asset"] in class_ids:
                        material.SetClassID(class_ids[inst["asset"]])
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


def _lit(material_name, signals):
    """Whether a lens material is one the caller asked to light. They are named emit_<colour>."""
    if not signals:
        return False
    return signals == "all" or material_name.lower().endswith("_" + signals)


def _class_ids(doc):
    """asset index -> class id, numbering the manifest's labels from 1 in their listed order."""
    order = {q: n for n, q in enumerate(doc.get("labels", {}), start=1)}
    out = {}
    for inst in doc["instances"]:
        if inst.get("label") in order:
            out.setdefault(inst["asset"], order[inst["label"]])
    return out


def manifest(scene_dir, foliage="none"):
    """The scene manifest as a dict, for the parts of it that are data and not geometry.

    "instances" name every placement (a traffic light's name ends in its OpenDRIVE signal id) and
    give its label. "lights" lists the signal lamps with position, direction and colour.
    "labels" maps label ids to names. "road_network" and "sky" are paths relative to scene_dir.
    """
    with open(os.path.join(scene_dir, FOLIAGE[foliage])) as f:
        return json.load(f)


def labels(scene_dir):
    """class id -> (Wikidata id, name) for the class ids add_scenery puts on materials."""
    doc = manifest(scene_dir)
    return {n: (q, name) for n, (q, name) in enumerate(doc.get("labels", {}).items(), start=1)}


def sky(scene_dir):
    """Path of the sky panorama, or None for a scene that has none."""
    relative = manifest(scene_dir).get("sky")
    return _existing(scene_dir, relative)


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

FOLIAGE_HELP = """vegetation level (default: none). The first use downloads the vegetation archive.
Memory is what stock PyChrono needs.
  none        no vegetation                          5 GB
  trees       447 trees, bare branches               7 GB
  trees-leaf  447 trees with leaves                  8 GB
  shrubs      2009 trees and shrubs, bare branches   8 GB
  full        2009 trees and shrubs with leaves      8 GB"""

TIRES = {"pac02": "audi/json/audi_Pac02Tire.json", "tmeasy": "audi/json/audi_TMeasyTire.json", "rigid": "audi/json/audi_RigidTire.json"}


def main():
    parser = argparse.ArgumentParser(
        description="Drive an Audi around the Mcity digital twin. W/S throttle and brake, A/D steer.",
        formatter_class=argparse.RawTextHelpFormatter,
    )
    parser.add_argument("--data", metavar="DIR", default=SCENE_DIR, help="scene directory, downloaded into if empty (default: scene/ beside this file)")
    parser.add_argument("--foliage", choices=list(FOLIAGE), default="none", help=FOLIAGE_HELP)
    parser.add_argument("--signals", choices=["red", "amber", "green", "all"], default=None, help="light the traffic signal lenses of that colour (default: all dark)")
    parser.add_argument("--no-sky", action="store_true", help="plain background instead of the sky dome")
    parser.add_argument("--no-shadows", action="store_true", help="do not draw shadows. Worth trying on a slow GPU: stock Chrono redraws the scene for every shadow map")
    parser.add_argument("--tire", choices=sorted(TIRES), default="pac02", help="tire model (default: pac02)")
    parser.add_argument("--tire-step", metavar="S", type=float, default=1e-4, help="tire internal step in seconds (default: 1e-4)")
    parser.add_argument("--speed-limit", metavar="V", type=float, default=20.0, help="speed the throttle ramp is scaled toward, m/s (default: 20)")
    parser.add_argument("--duration", metavar="S", type=float, default=None, help="stop after this many simulated seconds (default: run until the window closes)")
    parser.add_argument("--headless", action="store_true", help="no window: simulate with the vehicle parked and print where it is")
    parser.add_argument("--force", action="store_true", help="load a vegetation level even if it looks too big for this machine's memory")
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

    # A headless run draws nothing, so it has no use for vegetation.
    foliage = "none" if args.headless else args.foliage
    if foliage != "none":
        # Checked before the download, so nobody fetches an archive only to be turned away.
        need, have = FOLIAGE_MEMORY_GB[foliage], _physical_memory_gb()
        if have is not None and need > 0.6 * have and not args.force:
            raise SystemExit(
                f"Vegetation level '{foliage}' needs about {need} GB of memory on stock PyChrono, which is too much\n"
                f"for the {have:.0f} GB in this machine. Stock Chrono::VSG keeps a separate copy of the geometry\n"
                "for every plant. Pick a lighter level, or pass --force to try anyway."
            )
    scene = fetch(args.data, foliage)
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
        add_scenery(system, scene, foliage, signals=args.signals)
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
        run_window(system, audi, terrain, chrono, veh, step, args, boot, None if args.no_sky else sky(scene))

    # Tearing the visual system down from Python can crash on the way out, after everything has
    # already worked. Leave without running destructors.
    sys.stdout.flush()
    os._exit(0)


def _physical_memory_gb():
    try:
        return os.sysconf("SC_PAGE_SIZE") * os.sysconf("SC_PHYS_PAGES") / 2**30
    except (AttributeError, ValueError, OSError):  # not available everywhere, Windows for one
        return None


def run_window(system, audi, terrain, chrono, veh, step, args, boot, sky_texture):
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
    if not args.no_shadows:
        vis.EnableShadows()
    if sky_texture:
        # The second argument is where the sun sits in the picture. This sky has no visible sun.
        vis.SetSkyDomeTexture(sky_texture, 0.0)
        vis.EnableSkyTexture()
    vis.AttachDriver(driver)
    vis.Initialize()

    print(f"\n  [{time.perf_counter() - boot:.1f} s to build the scene and open the window]")
    print("W/S throttle and brake, A/D steer.\n")

    render_step = 1.0 / 50  # physics wants 1 kHz, the display does not
    next_render = next_report = 0.0
    last_render = -1.0
    frames = 0
    start = time.perf_counter()
    clock = start  # the wall-clock instant that simulated time 0 is held against

    while vis.Run():
        now = system.GetChTime()
        if args.duration is not None and now >= args.duration:
            break

        # How far the simulation has fallen behind the wall clock. A stall that long is the
        # window being dragged or the machine being busy, and is written off instead of chased.
        lag = (time.perf_counter() - clock) - now
        if lag > 0.5:
            clock += lag - 0.05
            lag = 0.05

        # Draw only when the simulation is keeping up. Heavy vegetation can take longer to draw
        # than a 50 Hz frame lasts, and drawing every frame regardless turns that into slow
        # motion. Skipping frames keeps the car at real time and lets the frame rate drop
        # instead, down to a floor of five a second.
        if now >= next_render and (lag < render_step or now - last_render >= 0.2):
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
            frames += 1
            last_render = now
            next_render = now + render_step

        if now >= next_report:
            wall = time.perf_counter() - start
            print(f"  t={now:5.1f} s   {now / wall if wall > 0 else 0:.2f}x real time   "
                  f"{frames / wall if wall > 0 else 0:4.1f} frames/s   {audi.GetSpeed():5.1f} m/s")
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

        # Physics alone runs several times faster than real time, so wait for the clock.
        ahead = (now + step) - (time.perf_counter() - clock)
        if ahead > 0.002:
            time.sleep(ahead - 0.001)


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
