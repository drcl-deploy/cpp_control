#!/usr/bin/env python3
"""Generate an mj_sim scene with the G1 standing on the ground.

The shipped scene (`assets: g1/scene_flat.xml`) holds the pelvis in the air:

    <equality>
        <weld name="world_root" active="true" body1="pelvis" relpose="0 0 -1.0 ..."/>
    </equality>

which is right for bring-up — you can watch a controller move the limbs without
it falling over — and is what the readme's "to keep robot on/off ground, right
side bar under Equality, toggle `world_root`" refers to. It is also exactly the
kind of thing that must not be left on for a transfer result: anything a weld
holds up is not a locomotion measurement. mj_sim can only clear it from the
viewer (the `Y` key), so an unattended run needs a scene that never had it.

This writes such a scene, by copying the robot model with two surgical edits and
adding a scene file beside it:

    <out>/g1/g1_bm_ground.xml      weld inactive; meshdir made absolute so the
                                   copy still finds the original meshes
    <out>/g1/scene_flat_ground.xml includes the stock terrain + that model

Point mj_sim at it with SIM_ASSETS_PATH=<out> and
`model_path: g1/scene_flat_ground.xml`, which is what
`run_difftrack_sim2sim.sh` does.

The output is generated, machine-local and disposable: regenerate it rather than
editing it, and never commit it.

    python3 scripts/make_sim2sim_scene.py [--assets $SIM_ASSETS_PATH] [--out DIR]
"""

import argparse
import os
import re
import sys
import tempfile


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--flavor", choices=("mj_sim", "unitree"), default="mj_sim",
                    help="which simulator the scene is for. mj_sim (default) is "
                         "the drcl_deploy plant described above; unitree is "
                         "unitree_mujoco, whose model needs no weld cleared and "
                         "which takes ONE absolute scene path on its -s flag")
    ap.add_argument("--assets", default=os.environ.get("SIM_ASSETS_PATH", ""),
                    help="asset root (default: $SIM_ASSETS_PATH)")
    ap.add_argument("--out", default=os.path.join(tempfile.gettempdir(),
                                                  "cpp_control_sim_scene"),
                    help="where to write the generated scene")
    ap.add_argument("--model", default="g1/g1_bm.xml",
                    help="robot model to copy, relative to --assets")
    ap.add_argument("--terrain", default="terrain/flat_ground.xml",
                    help="terrain to include, relative to --assets")
    ap.add_argument("--unitree-scene",
                    default=os.path.join(
                        os.environ.get("UNITREE_MUJOCO", ""),
                        "unitree_robots", "g1", "scene_29dof.xml"),
                    help="--flavor unitree: the stock unitree_mujoco scene to "
                         "start from (default: $UNITREE_MUJOCO/unitree_robots/"
                         "g1/scene_29dof.xml)")
    ap.add_argument("--init-pose", choices=("rsi", "default"), default="rsi",
                    help="which pose --init-from bakes. rsi (default) is the "
                         "clip's first frame; default is the policy's own "
                         "nominal pose, standing level with the feet solved "
                         "onto the ground -- the posture the STAND entry "
                         "assumes it is handed")
    ap.add_argument("--init-from", default="",
                    help="difftrack_config.json: start the robot ON the clip's first "
                         "frame (reference-state initialisation) instead of the "
                         "model's own pose")
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args()

    if args.flavor == "unitree":
        return _make_unitree_scene(args)

    if not args.assets:
        sys.exit("no asset root: pass --assets or set SIM_ASSETS_PATH "
                 "(source workflows/conda_env/runenv.sh)")
    src = os.path.join(args.assets, args.model)
    terrain = os.path.join(args.assets, args.terrain)
    for path in (src, terrain):
        if not os.path.isfile(path):
            sys.exit(f"no such asset: {path}")

    xml = open(src).read()

    # 1. The weld. Matched on the name so a scene without it (or with a second
    #    equality) is not silently mangled.
    xml, n = re.subn(r'(<weld\b[^>]*\bname="world_root"[^>]*\bactive=")true(")',
                     r"\1false\2", xml)
    if n != 1:
        sys.exit(f"expected exactly one active world_root weld in {src}, found {n}. "
                 "The scene changed upstream — read it before trusting this script.")

    # 2. meshdir. The copy lives somewhere else, so a relative mesh directory
    #    would resolve against the wrong place: MuJoCo then fails to load with a
    #    missing-mesh error rather than anything about this script.
    mesh_abs = os.path.join(os.path.dirname(src), "meshes") + os.sep
    xml, n = re.subn(r'(<compiler\b[^>]*\bmeshdir=")[^"]*(")',
                     lambda m: m.group(1) + mesh_abs + m.group(2), xml, count=1)
    if n != 1:
        sys.exit(f"no <compiler meshdir=...> in {src}; cannot relocate the model")

    banner = (f"<!-- GENERATED by cpp_control/scripts/make_sim2sim_scene.py from\n"
              f"     {src}\n"
              f"     world_root weld disabled (the robot stands on the ground).\n"
              f"     Machine-local and disposable: regenerate, do not edit. -->\n")

    out_g1 = os.path.join(args.out, "g1")
    os.makedirs(out_g1, exist_ok=True)
    model_out = os.path.join(out_g1, "g1_bm_ground.xml")
    with open(model_out, "w") as f:
        f.write(banner + xml)

    scene_out = os.path.join(out_g1, "scene_flat_ground.xml")
    with open(scene_out, "w") as f:
        f.write(banner + f"""<mujoco model="g1_flat_ground">
<include file="{terrain}"/>
<include file="g1_bm_ground.xml"/>
</mujoco>
""")

    if args.init_from:
        scene_out = _bake_initial_pose(scene_out, args.init_from, banner)

    rel = os.path.relpath(scene_out, args.out)
    if not args.quiet:
        print(f"[scene] {scene_out}")
        print(f"[scene] SIM_ASSETS_PATH={args.out}  model_path={rel}")
    else:
        # One line the caller can read: the root mj_sim wants, and the path to
        # put in its config, which differs between the plain and the RSI scene.
        print(f"{args.out} {rel}")


# ── the follow camera ─────────────────────────────────────────────────────────
#
# A chase camera for the unitree scene, as (bearing, distance, eye height, aim
# height). Overridable from the environment because taste in camera angles is
# not worth a rebuild:
#
#   DRCL_CAM_AZ    bearing in degrees, measured from BEHIND the robot (which
#                  faces +x at reset) and rotating toward its right:
#
#                      0   dead astern, over the robot's shoulder
#                      90  full profile, from its right
#                      125 three-quarter FRONT, from its right   <- default
#                      180 head-on
#
#                  A three-quarter shows stride length the way a profile does,
#                  and unlike a profile it also shows heading drift, which is
#                  the failure these tracking policies actually have. From the
#                  front rather than the back because that is the side with a
#                  face, a chest and both arms on it -- the robot's posture and
#                  where it is looking are legible, and a stumble reads
#                  immediately. 55 is the same shot from behind.
#   DRCL_CAM_DIST  metres from the robot, horizontally.
#   DRCL_CAM_EYE   camera height, metres.
#   DRCL_CAM_AIM   the height the camera POINTS at, metres. This is the one
#                  that was wrong: aiming at the robot's middle rather than
#                  down at the floor is the difference between a G1 framed
#                  head-to-toe and a G1 with its head cut off. The shipped
#                  camera pointed 26.6 deg down from 1.2 m at 3 m range, so its
#                  optical axis crossed the robot's plane at z = -0.3 m --
#                  BELOW the ground -- putting the robot in the top third of
#                  the frame and two thirds of the picture on empty floor.
CAM_AZ = float(os.environ.get("DRCL_CAM_AZ", 125.0))
CAM_DIST = float(os.environ.get("DRCL_CAM_DIST", 2.8))
CAM_EYE = float(os.environ.get("DRCL_CAM_EYE", 1.15))
CAM_AIM = float(os.environ.get("DRCL_CAM_AIM", 0.72))


def _chase_camera(az_deg=None, dist=None, eye=None, aim=None):
    """Position and orientation for the `track` camera, as (pos, quat).

    Solved from a look-at rather than written down, because the orientation and
    the position are not independent: a camera that is moved and not re-aimed
    is exactly how the shipped one ended up pointing under the floor.

    `trackcom` translates the camera with the model's centre of mass and leaves
    its ORIENTATION alone, so this quaternion is the direction the camera looks
    for the whole run. That is the mode's virtue -- the robot holds one place
    on screen at a constant angle, and only the background moves -- and its one
    limitation: a clip that turns the robot turns it relative to the camera,
    because no fixed orientation can follow a heading. It stays in frame either
    way, which is what matters.
    """
    import math

    import mujoco
    import numpy as np

    az = math.radians(CAM_AZ if az_deg is None else az_deg)
    L = CAM_DIST if dist is None else dist
    eye = CAM_EYE if eye is None else eye
    aim = CAM_AIM if aim is None else aim

    pos = np.array([-L * math.cos(az), -L * math.sin(az), eye])
    fwd = np.array([0.0, 0.0, aim]) - pos
    fwd /= np.linalg.norm(fwd)
    # MuJoCo cameras look down their own -z with +y up, so the frame is
    # [right, up, -forward] and `right` comes from the world vertical.
    right = np.cross(fwd, [0.0, 0.0, 1.0])
    right /= np.linalg.norm(right)
    up = np.cross(right, fwd)
    quat = np.zeros(4)
    mujoco.mju_mat2Quat(quat, np.column_stack([right, up, -fwd]).flatten())
    return pos.tolist(), quat.tolist()


def _make_unitree_scene(args):
    """Generate the unitree_mujoco scene, optionally on the clip's first frame.

    Much less surgery than the mj_sim flavour needs, and for one good reason:
    unitree_mujoco's shipped G1 scene has no bring-up weld holding the pelvis in
    the air, so there is nothing to switch off. What it does need is a copy that
    lives outside the upstream checkout (never write into unitree_robots/) with
    the mesh directory made absolute so the copy still finds its meshes, and the
    same reference-state bake the mj_sim flavour does.

    Two things are added that the stock scene does not have:

      * a `track` camera on the world body (mode `trackcom`), framed by
        _chase_camera() and SELECTED as the scene's initial camera, because
        MuJoCo's own `simulate` UI — which is unitree_mujoco's viewer — boots
        on a free camera that does not move, and a G1 at 1 m/s leaves the
        frame in about four seconds. Being a camera rather than a viewer
        feature, it also survives the model reload `simulate` does on a scene
        edit, and it does not depend on the keyboard — which in unitree_mujoco
        is not MuJoCo's (src/main.cc replaces the GLFW key callback, so `[`,
        `]` and Esc do nothing unless its patch is applied). (The mj_sim
        flavour does not need any of this: run_mj_sim.py carries its own
        follow camera.)
      * nothing else. In particular the physics options are left exactly as
        unitree ships them (timestep 2 ms, Euler), because the whole point of
        running this plant is that it is THEIR plant.

    Prints one absolute path: unitree_mujoco resolves `-s` against
    unitree_robots/<robot>/ only when it is relative.
    """
    import mujoco

    src = args.unitree_scene
    if not src or not os.path.isfile(src):
        sys.exit(f"no unitree_mujoco scene at '{src}': pass --unitree-scene or set "
                 "UNITREE_MUJOCO to the unitree_mujoco checkout")

    spec = mujoco.MjSpec.from_file(src)

    # The meshes stay where they are; only the scene moves. A relative meshdir
    # would resolve against the output directory and MuJoCo would fail with a
    # missing-mesh error rather than anything about this script.
    spec.meshdir = os.path.join(os.path.dirname(src), "meshes") + os.sep

    cam = spec.worldbody.add_camera()
    cam.name = "track"
    cam.mode = mujoco.mjtCamLight.mjCAMLIGHT_TRACKCOM
    cam.pos, cam.quat = _chase_camera()

    # `trackcom` holds the camera at a CONSTANT world-axis offset from the
    # model's centre of mass with a fixed orientation, so the robot sits at the
    # same place on screen whatever direction it walks and however far it goes
    # -- only the background moves. Verified: displacing the free joint by
    # (+5, +2) m moves cam_xpos from (0, -3, 1.2) to (5, -1, 1.2).
    #
    # Selecting it is a separate step, and without it the camera above is dead
    # weight: simulate's AlignAndScaleView() reads vis.global.cameraid and only
    # sets cam.type = mjCAMERA_FIXED when it names a real camera. The default,
    # -1, is the free camera -- which does not move. Resolved by name rather
    # than assumed to be 0, because the id is a compile-order index and the
    # stock scene is free to grow a camera of its own.
    spec.visual.global_.cameraid = mujoco.mj_name2id(
        spec.compile(), mujoco.mjtObj.mjOBJ_CAMERA, "track")

    # The offscreen framebuffer the F9 recorder renders into is sized by the
    # MODEL, not by the recorder, and MuJoCo's default is 640x480 -- a viewport
    # larger than it is silently clipped, so a 720p recording off a stock scene
    # would come out cropped rather than scaled. 1920x1080 leaves room for a
    # 1080p DRCL_RECORD_SIZE and costs one buffer of GPU memory.
    spec.visual.global_.offwidth = 1920
    spec.visual.global_.offheight = 1080

    banner = (f"<!-- GENERATED by cpp_control/scripts/make_sim2sim_scene.py "
              f"--flavor unitree from\n     {src}\n"
              f"     Machine-local and disposable: regenerate, do not edit. -->\n")

    out_dir = os.path.join(args.out, "g1")
    os.makedirs(out_dir, exist_ok=True)
    scene_out = os.path.join(out_dir, "scene_29dof_sim2sim.xml")
    with open(scene_out, "w") as f:
        f.write(banner + spec.to_xml())

    if args.init_from:
        scene_out = _bake_initial_pose(
            scene_out, args.init_from, banner,
            out_name=f"scene_29dof_{args.init_pose}.xml", pose=args.init_pose)

    scene_out = os.path.abspath(scene_out)
    if not args.quiet:
        print(f"[scene] {scene_out}")
        print(f"[scene] unitree_mujoco -r g1 -t 1 -s {scene_out}")
    else:
        print(scene_out)
    return 0


def _bake_initial_pose(scene_path, config_path, banner,
                       out_name="scene_flat_rsi.xml", pose="rsi"):
    """Rewrite the scene so `mj_resetData` lands on the clip's first frame.

    Reference-state initialisation is how these policies were trained and
    evaluated: the episode starts with the robot ON the reference, not standing
    beside it. There is no way to ask mj_sim for that — it resets to the model's
    own pose and offers the operator a keyframe only behind a viewer key — so
    the pose has to be the model's own.

    MuJoCo has no single "initial joint angle" that does this, and each half of
    the answer is wrong on its own:

      * `ref` alone moves qpos0 to theta but shifts the joint's zero with it, so
        the robot at reset is still in the XML pose while the encoders claim
        theta. The observation would be right and the robot wrong.
      * baking theta into the body frame alone puts the robot in the clip's pose
        but leaves qpos at 0, so the encoders — which is all the controller
        reads — report a robot that is not the one standing there.

    Both together are exact. Set `ref = theta` AND `quat <- quat * R(axis,
    theta)`: the joint contributes R(axis, qpos - ref), which is identity at
    reset, so the configuration is the body frames alone, i.e. the clip's pose;
    and qpos reads theta. Every other joint coordinate keeps its old meaning —
    at any qpos = x the configuration is quat * R(axis, theta) * R(axis, x -
    theta) = quat * R(axis, x), exactly as before — so ranges, limits and the
    position servos are untouched. This needs the hinge to pass through the body
    origin, which every G1 joint does (the exporter asserts it) and which is
    checked below.

    What cannot be baked is VELOCITY: qvel is always zero at reset, while these
    clips start moving (g1_walk enters at ~0.97 m/s). The controller's
    `lead_in_duration` exists for exactly that gap — it ramps the reference from
    the robot's actual velocity up to the clip's.
    """
    import json
    import math

    import mujoco

    conf = json.load(open(config_path))
    names = conf["policy_joint_names"]
    if pose == "rsi":
        dof = conf["rsi"]["dof_pos"]
        root_pos = list(conf["rsi"]["root_pos"])
        root_quat = list(conf["rsi"]["root_quat_wxyz"])
    elif pose == "default":
        # The posture the policy expects to be handed, standing level and
        # facing +x. Not a frame of any clip: the STAND entry's premise is that
        # the robot is already up in the nominal pose and the controller then
        # picks the clip up from wherever that is.
        #
        # The height is solved rather than taken from the config, because the
        # config has no height for this pose — see _ground_root_z().
        dof = conf["default_angles"]
        root_pos = None          # solved below, once the joints are baked
        root_quat = [1.0, 0.0, 0.0, 0.0]
    else:
        sys.exit(f"unknown --init-pose '{pose}'")

    spec = mujoco.MjSpec.from_file(scene_path)

    for name, angle in zip(names, dof):
        joint = spec.joint(name)
        if joint is None:
            sys.exit(f"{scene_path} has no joint '{name}' — this scene is not the "
                     "robot the policy was exported for")
        if int(joint.type) != int(mujoco.mjtJoint.mjJNT_HINGE):
            sys.exit(f"joint '{name}' is not a hinge; cannot bake an initial angle")
        if max(abs(v) for v in joint.pos) > 1e-9:
            sys.exit(f"joint '{name}' is not at its body's origin ({list(joint.pos)}); "
                     "rotating the body frame would not be the joint rotation")
        axis = list(joint.axis)
        n = math.sqrt(sum(v * v for v in axis))
        half = 0.5 * angle
        sin = math.sin(half) / n
        jq = [math.cos(half), axis[0] * sin, axis[1] * sin, axis[2] * sin]
        joint.ref = angle
        body = joint.parent
        q = list(body.quat)
        body.quat = [
            q[0] * jq[0] - q[1] * jq[1] - q[2] * jq[2] - q[3] * jq[3],
            q[0] * jq[1] + q[1] * jq[0] + q[2] * jq[3] - q[3] * jq[2],
            q[0] * jq[2] - q[1] * jq[3] + q[2] * jq[0] + q[3] * jq[1],
            q[0] * jq[3] + q[1] * jq[2] - q[2] * jq[1] + q[3] * jq[0],
        ]

    root = spec.body(spec.joint(names[0]).parent.name)   # any actuated body's chain
    while root.parent is not None and root.parent.name not in ("world", ""):
        root = root.parent
    root.quat = list(root_quat)
    if root_pos is None:
        root_pos = [0.0, 0.0, _ground_root_z(spec, root)]
    root.pos = list(root_pos)

    out = os.path.join(os.path.dirname(scene_path), out_name)
    with open(out, "w") as f:
        f.write(banner + spec.to_xml())

    # Prove it: the compiled reset pose must be the clip's frame, to round-off.
    model = mujoco.MjModel.from_xml_path(out)
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)
    mujoco.mj_forward(model, data)
    got = [data.qpos[model.jnt_qposadr[mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_JOINT, n)]] for n in names]
    err = max(abs(a - b) for a, b in zip(got, dof))
    root_err = max(abs(data.qpos[i] - root_pos[i]) for i in range(3))
    # 1e-5 rather than round-off: MjSpec writes the XML in float32-ish decimal,
    # so a bake that is exact in memory comes back ~5e-7 off. Anything larger
    # than this is a real disagreement, not serialisation.
    if err > 1e-5 or root_err > 1e-5:
        sys.exit(f"baked initial pose is off by {err:.2e} rad / {root_err:.2e} m; "
                 "the scene is not what the config describes")
    if pose == "rsi":
        print(f"[scene] reference-state start: clip frame 0 to {err:.1e} rad / "
              f"{root_err:.1e} m (velocity cannot be baked; qvel starts at zero)")
    else:
        print(f"[scene] nominal-pose start: the policy's default pose to {err:.1e} rad, "
              f"feet on the ground at pelvis z = {root_pos[2]:.3f} m")
    return out


def _ground_root_z(spec, root):
    """Lowest root height at which nothing touches the floor, by bisection.

    The export carries a default POSE but no height to hold it at, and the
    number is not guessable: it is whatever puts the feet on the floor with
    those knee and ankle angles, and it moves with every checkpoint.

    Bisected on MuJoCo's own collision rather than computed from geometry.
    `mj_forward` runs the collision pass, so `data.ncon` answers "is anything
    touching" exactly, for meshes and boxes alike -- where a bounding-sphere
    estimate would lift the robot several centimetres and hand it a drop to
    start from.
    """
    import mujoco

    lo, hi = 0.0, 2.0          # hi must be clear of the floor; a G1 is ~1.3 m
    root.pos = [0.0, 0.0, hi]
    model = spec.compile()
    data = mujoco.MjData(model)

    def touching(z):
        data.qpos[:] = model.qpos0
        data.qpos[2] = z
        mujoco.mj_forward(model, data)
        return data.ncon > 0

    if touching(hi):
        sys.exit(f"the robot still collides at z = {hi} m; this is not a G1-shaped model")
    for _ in range(40):
        mid = 0.5 * (lo + hi)
        if touching(mid):
            lo = mid
        else:
            hi = mid
    # `hi` is the lowest height that is CLEAR. Settle onto the floor by the
    # contact margin MuJoCo would take up anyway, so the robot starts in
    # contact rather than in a millimetre of free fall.
    return hi - 1e-3


if __name__ == "__main__":
    main()
