// Maps the protocol's joints onto the Xbot rig and applies incoming rotations.
//
// The server sends, per joint, a `local_anim_rot` quaternion: 
// the joint's rotation relative to the captured rest pose (a delta). 
// A three.js bone's local quaternion is also parent-relative, so we drive each bone as:
//
//     bone.quaternion = bindQuaternion * animDelta
//

import * as THREE from 'three';

// Indexed by exo.proto.JointId (0..6). Sanitized bone names (no colon).
export const BONE_BY_JOINT_ID = [
    'mixamorigHips',       // 0 Pelvis
    'mixamorigRightUpLeg', // 1 RKnee  (thigh)
    'mixamorigLeftUpLeg',  // 2 LKnee
    'mixamorigRightLeg',   // 3 RAnkle (shin)
    'mixamorigLeftLeg',    // 4 LAnkle
    'mixamorigRightFoot',  // 5 RFoot
    'mixamorigLeftFoot',   // 6 LFoot
];

// Snapshot each driven bone's rest (bind) quaternion so deltas apply on top of it.
export function captureBindPose(bones) {
    const bind = {};
    for (const name of BONE_BY_JOINT_ID) {
        if (bones[name]) { bind[name] = bones[name].quaternion.clone(); }
    }
    return bind;
}

const _delta = new THREE.Quaternion();

// Apply one decoded PoseFrame to the rig. `frame` is the generated flatbuffers object.
export function applyPoseFrame(bones, bindPose, frame) {
    for (let i = 0; i < frame.jointsLength(); i++) {
        const jp = frame.joints(i);
        const name = BONE_BY_JOINT_ID[jp.id()];
        const bone = bones[name];
        if (!bone) { continue; }

        const q = jp.localAnimRot();
        if (!q) { continue; } // lost joint (null rotation) keeps its rest pose

        _delta.set(q.x(), q.y(), q.z(), q.w());
        bone.quaternion.copy(bindPose[name]).multiply(_delta);
    }
}

const _q = new THREE.Quaternion();
const _euler = new THREE.Euler();

// Convert a flatbuffers Quat to euler angles [deg] in the given euler order.
// (for display-only; the rig is driven by the quaternion)
export function quatToEulerDeg(quat, order = 'XYZ') {
    _q.set(quat.x(), quat.y(), quat.z(), quat.w());
    _euler.setFromQuaternion(_q, order);
    const deg = THREE.MathUtils.radToDeg;
    return { x: deg(_euler.x), y: deg(_euler.y), z: deg(_euler.z) };
}
