"""A generic humanoid rig for the FBT IK lab: no game needed, and the template for exporting your own skeleton.

Writes humanoid_rig.txt in the rig file format (README.md, "Using it with another skeleton"): a 1.75 m body in the
reference pose the IK expects -- a T-pose, arms straight out along x, palms down, thumbs forward, the elbows a
hair back and the knees a little forward (the way they bend) -- in the lab's model space (x right, y forward, z up, metres). Every bone frame
is the model frame (identity rotations): the IK derives each bone's axes from its reference pose, so any frames work.
It has no eye bones on purpose: the IK then takes the eyes 8.5 cm forward and 5.5 cm up of the head joint.

    python make_humanoid_rig.py [output]        (default: humanoid_rig.txt next to this script)
"""
import os
import sys

# name, parent name, model position (x right, y forward, z up)
BONES = [
    ("Root", None, (0.0, 0.0, 0.0)),
    ("Hips", "Root", (0.0, 0.0, 0.95)),
    # The spine's natural S-curve, back at the chest and forward again to the head: a perfectly straight spine leaves
    # the direction it bends in undefined at its first compression (the IK's continuity test catches the jump).
    ("Spine", "Hips", (0.0, -0.02, 1.04)),
    ("Spine1", "Spine", (0.0, -0.025, 1.12)),
    ("Spine2", "Spine1", (0.0, -0.035, 1.22)),
    ("Spine3", "Spine2", (0.0, -0.06, 1.33)),
    ("Neck", "Spine3", (0.0, -0.06, 1.48)),
    ("Neck1", "Neck", (0.0, -0.04, 1.54)),
    ("Head", "Neck1", (0.0, -0.025, 1.60)),
]
for side, s in (("Left", -1.0), ("Right", 1.0)):
    BONES += [
        (side + "Shoulder", "Spine3", (s * 0.04, -0.045, 1.42)),
        (side + "Arm", side + "Shoulder", (s * 0.17, -0.045, 1.43)),
        (side + "ForeArm", side + "Arm", (s * 0.45, -0.055, 1.43)),   # the elbow a hair back: it bends back
        (side + "Hand", side + "ForeArm", (s * 0.70, -0.045, 1.43)),
        (side + "HandEnd", side + "Hand", (s * 0.88, -0.045, 1.43)),
        (side + "HandThumb1", side + "Hand", (s * 0.735, -0.01, 1.41)),   # forward and a little down: palm down
    ]
for side, s in (("Left", -1.0), ("Right", 1.0)):
    BONES += [
        (side + "UpLeg", "Hips", (s * 0.09, 0.0, 0.92)),
        (side + "Leg", side + "UpLeg", (s * 0.09, 0.02, 0.50)),   # the knee a little forward: it bends forward
        (side + "Foot", side + "Leg", (s * 0.09, 0.0, 0.08)),
        (side + "ToeBase", side + "Foot", (s * 0.09, 0.14, 0.02)),
    ]

ROLES = {
    "hips": "Hips", "spine0": "Spine", "spine1": "Spine1", "spine2": "Spine2", "spine3": "Spine3",
    "neck": "Neck", "neck1": "Neck1", "head": "Head",
    "left_upperarm": "LeftArm", "left_forearm": "LeftForeArm", "left_hand": "LeftHand",
    "left_thumb1": "LeftHandThumb1", "left_hand_end": "LeftHandEnd",
    "right_upperarm": "RightArm", "right_forearm": "RightForeArm", "right_hand": "RightHand",
    "right_thumb1": "RightHandThumb1", "right_hand_end": "RightHandEnd",
    "left_upleg": "LeftUpLeg", "left_leg": "LeftLeg", "left_foot": "LeftFoot",
    "right_upleg": "RightUpLeg", "right_leg": "RightLeg", "right_foot": "RightFoot",
}


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(os.path.abspath(__file__)), "humanoid_rig.txt")
    index = {name: i for i, (name, _, _) in enumerate(BONES)}
    pos = {name: p for name, _, p in BONES}
    lines = ["# A generic humanoid, reference pose (fbt_ik_lab/samples/make_humanoid_rig.py). Model space: x right, "
             "y forward, z up, metres.", "# roles: name index"]
    lines += [f"role {role} {index[bone]}" for role, bone in ROLES.items()]
    lines.append("# bone: index parent name  local pos xyz  local rot xyzw  model pos xyz  model rot xyzw")
    for i, (name, parent, p) in enumerate(BONES):
        q = pos[parent] if parent else (0.0, 0.0, 0.0)
        local = tuple(a - b for a, b in zip(p, q))
        lines.append(f"bone {i} {index[parent] if parent else -1} {name}  "
                     f"{local[0]:.6f} {local[1]:.6f} {local[2]:.6f}  0.0000000 0.0000000 0.0000000 1.0000000  "
                     f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}  0.0000000 0.0000000 0.0000000 1.0000000")
    with open(out, "w", encoding="utf-8", newline="\n") as f:
        f.write("\n".join(lines) + "\n")
    print(f"{out}: {len(BONES)} bones, {len(ROLES)} roles")


if __name__ == "__main__":
    main()
