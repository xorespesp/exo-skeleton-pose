// Entry point: build the scene, load the rig, and wire a lil-gui panel for the
// connection and camera/rest-pose commands.

import { GUI } from 'three/addons/libs/lil-gui.module.min.js';
import { createScene, loadCharacter } from './scene.js';
import { captureBindPose, applyPoseFrame, quatToEulerDeg, BONE_BY_JOINT_ID } from './skeleton.js';
import { PoseClient } from './pose-client.js';

const { scene, startRenderLoop } = createScene(document.getElementById('container'));

const { model, bones } = await loadCharacter(scene);
const bindPose = captureBindPose(bones);
startRenderLoop();

// --- ui state -----------------------------------------------------------------
const ui = {
    url: 'ws://localhost:9002',
    connection: 'disconnected',
    // Open options
    source: '0', // device index or recording path
    tag_size_m: 0.05,
    manual_exposure: false,
    exposure_us: 8000,
    manual_gain: false,
    gain: 0,
    // readouts
    source_name: '(none)',
    rest_pose: 'none',
    frame_id: 0,
    euler_order: 'XYZ',
    joints: Object.fromEntries(BONE_BY_JOINT_ID.map((n) => [n, '-'])),
};

// Transient control flags (not shown in the panel).
let connected = false;   // WebSocket open
let connecting = false;  // connect in flight
let opened = false;      // a source is open on the server
let pending = false;     // a command is awaiting its Ack

// --- pose client --------------------------------------------------------------
const client = new PoseClient();

client.onOpen = () => {
    connecting = false; connected = true;
    ui.connection = 'connected';
    startStatusPoll(); // resync ServerStatus while connected
    refresh();
};
client.onClose = () => {
    connecting = false; connected = false; opened = false; pending = false;
    ui.connection = 'disconnected';
    ui.source_name = '(none)';
    ui.rest_pose = 'none';
    stopStatusPoll();
    refresh();
};
client.onAck = (ack, requestId) => {
    pending = false;
    console.log(`ack[#${requestId}]: ok=${ack.ok()} "${ack.message()}"`);
    refresh();
};
client.onSourceEnded = (ev) => {
    // Stream stopped on its own (recording EOF or device lost).
    console.log(`source ended: is_error=${ev.isError()} "${ev.message()}"`);
};
client.onStatus = (st) => {
    opened = st.isStreamOpened();
    ui.source_name = opened ? `${st.sourceName()} (${st.width()}x${st.height()})` : '(none)';
    ui.rest_pose = st.hasRestPose() ? 'calibrated' : 'none';
    refresh();
};
client.onPoseFrame = (frame) => {
    applyPoseFrame(bones, bindPose, frame);
    ui.frame_id = frame.frameId();
    for (let i = 0; i < frame.jointsLength(); i++) {
        const jp = frame.joints(i);
        const e = quatToEulerDeg(jp.localAnimRot(), ui.euler_order); // convert on the client
        ui.joints[BONE_BY_JOINT_ID[jp.id()]] =
            jp.visible() ? `${e.x.toFixed(0)}, ${e.y.toFixed(0)}, ${e.z.toFixed(0)}` : '-';
    }
};

// --- actions ------------------------------------------------------------------
function doConnect() {
    if (connected || connecting) { return; }
    connecting = true;
    client.connect(ui.url);
    refresh();
}
function doDisconnect() { client.disconnect(); }
function doOpen() {
    pending = true;
    client.sendOpen({
        source: ui.source,
        tagSizeM: ui.tag_size_m,
        exposureUs: ui.manual_exposure ? ui.exposure_us : null,
        gain: ui.manual_gain ? ui.gain : null,
    });
    refresh();
}
function doClose() { pending = true; client.sendClose(); refresh(); }
function doCalibrate() { pending = true; client.sendCalibrateRestPose(); refresh(); }
function doClearRest() { pending = true; client.sendClearRestPose(); refresh(); }

// Low-frequency ServerStatus poll (for resync; status is also pushed on every change)
let statusPollTimer = null;
function startStatusPoll() {
    stopStatusPoll();
    statusPollTimer = setInterval(() => client.sendGetServerStatus(), 2000);
}
function stopStatusPoll() {
    if (statusPollTimer !== null) { clearInterval(statusPollTimer); statusPollTimer = null; }
}

// --- gui panel ----------------------------------------------------------------
const acts = {
    connect: doConnect, 
    disconnect: doDisconnect, 
    open: doOpen, 
    close: doClose,
    calibrate: doCalibrate, 
    clearRest: doClearRest,
};

const gui = new GUI({ title: 'exo-skeleton-pose', width: 300 });
gui.add(ui, 'url').name('server url');
const cConnect = gui.add(acts, 'connect').name('Connect');
const cDisconnect = gui.add(acts, 'disconnect').name('Disconnect');
gui.add(ui, 'connection').name('status').listen().disable();

const src = gui.addFolder('Source');
src.add(ui, 'source').name('index or path');
src.add(ui, 'tag_size_m').name('tag size [m]').min(0.01).max(0.5).step(0.001);
src.add(ui, 'manual_exposure').name('manual exposure').onChange(refresh);
const cExposure = src.add(ui, 'exposure_us').name('exposure [us]').min(100).max(100000).step(100);
src.add(ui, 'manual_gain').name('manual gain').onChange(refresh);
const cGain = src.add(ui, 'gain').name('gain').min(0).max(255).step(1);
const cOpen = src.add(acts, 'open').name('Open');
const cClose = src.add(acts, 'close').name('Close');
src.add(ui, 'source_name').name('opened').listen().disable();

const rest = gui.addFolder('Rest Pose');
const cCalibrate = rest.add(acts, 'calibrate').name('Calibrate');
const cClear = rest.add(acts, 'clearRest').name('Clear');
rest.add(ui, 'rest_pose').name('state').listen().disable();

const joints = gui.addFolder('Joints (euler deg)').close();
joints.add(ui, 'frame_id').name('frame').listen().disable();
joints.add(ui, 'euler_order', ['XYZ', 'XZY', 'YXZ', 'YZX', 'ZXY', 'ZYX']).name('euler order');
for (const name of BONE_BY_JOINT_ID) {
    joints.add(ui.joints, name).listen().disable();
}

// Enable/disable controls to match the current connection + source state.
function refresh() {
    model.visible = opened; // only show the rig while a source is streaming
    cConnect.enable(!connected && !connecting);
    cDisconnect.enable(connected);
    cExposure.enable(ui.manual_exposure);
    cGain.enable(ui.manual_gain);
    cOpen.enable(connected && !opened && !pending);
    cClose.enable(connected && opened && !pending);
    cCalibrate.enable(connected && opened && !pending);
    cClear.enable(connected && opened && !pending);
}

refresh();
