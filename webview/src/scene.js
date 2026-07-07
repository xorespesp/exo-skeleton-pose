// three.js scene setup and character loading.
// The character is the rigged Xbot glTF.
// We collect its bones by name so the pose protocol can drive them later.

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';

// `?url` lets Vite bundle the (large, binary) model and hand us a served URL.
import xbotUrl from '../models/gltf/Xbot.glb?url';

export function createScene(container) {
  const renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(window.devicePixelRatio);
  renderer.setSize(window.innerWidth, window.innerHeight);
  renderer.shadowMap.enabled = true;
  container.appendChild(renderer.domElement);

  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x2a2a30);

  const camera = new THREE.PerspectiveCamera(45, window.innerWidth / window.innerHeight, 0.1, 100);
  camera.position.set(-1.5, 2, 3);

  const controls = new OrbitControls(camera, renderer.domElement);
  controls.target.set(0, 1, 0);
  controls.update();

  scene.add(new THREE.HemisphereLight(0xffffff, 0x8d8d8d, 3));
  const sun = new THREE.DirectionalLight(0xffffff, 3);
  sun.position.set(3, 10, 10);
  sun.castShadow = true;
  scene.add(sun);

  const ground = new THREE.Mesh(
    new THREE.PlaneGeometry(50, 50),
    new THREE.MeshPhongMaterial({ color: 0x3a3a42, depthWrite: false }),
  );
  ground.rotation.x = -Math.PI / 2;
  ground.receiveShadow = true;
  scene.add(ground);
  scene.add(new THREE.GridHelper(50, 50, 0x555555, 0x333333));

  window.addEventListener('resize', () => {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
  });

  function startRenderLoop() {
    renderer.setAnimationLoop(() => renderer.render(scene, camera));
  }

  return { scene, camera, renderer, controls, startRenderLoop };
}

// Lower-body bones (Hips + both leg chains). Mesh skinned above these is dropped.
const LOWER_BODY_BONES = new Set([
  'mixamorigHips',
  'mixamorigLeftUpLeg', 'mixamorigRightUpLeg',
  'mixamorigLeftLeg', 'mixamorigRightLeg',
  'mixamorigLeftFoot', 'mixamorigRightFoot',
  'mixamorigLeftToeBase', 'mixamorigRightToeBase',
  'mixamorigLeftToe_End', 'mixamorigRightToe_End',
]);

// Xbot is a single whole-body skinned mesh, so we cannot just hide a child.
// Instead drop every triangle whose vertices are skinned mainly to non-lower-body bones,
// leaving a clean cut at the pelvis. Skinning is unaffected. (only the index shrinks)
function hideUpperBody(model) {
  model.traverse((obj) => {
    if (!obj.isSkinnedMesh || !obj.geometry.index) { return; }
    const geo = obj.geometry;
    const si = geo.attributes.skinIndex;
    const sw = geo.attributes.skinWeight;
    const bones = obj.skeleton.bones;

    const vertexIsLower = (v) => {
      // The dominant (max-weight) bone decides which half the vertex belongs to.
      const idx = [si.getX(v), si.getY(v), si.getZ(v), si.getW(v)];
      const w = [sw.getX(v), sw.getY(v), sw.getZ(v), sw.getW(v)];
      let best = 0;
      for (let k = 1; k < 4; k++) { if (w[k] > w[best]) { best = k; } }
      const bone = bones[idx[best]];
      return bone && LOWER_BODY_BONES.has(bone.name);
    };

    const index = geo.index;
    const kept = [];
    for (let i = 0; i < index.count; i += 3) {
      const a = index.getX(i), b = index.getX(i + 1), c = index.getX(i + 2);
      if (vertexIsLower(a) && vertexIsLower(b) && vertexIsLower(c)) { kept.push(a, b, c); }
    }
    geo.setIndex(kept);
  });
}

// Loads Xbot.glb, adds it to the scene, and returns { model, bones }
// where `bones` maps sanitized bone name -> THREE.Bone.
// NOTE: GLTFLoader strips the ':' from the raw "mixamorig:Hips" names,
//       so keys look like "mixamorigHips".
export async function loadCharacter(scene) {
  const gltf = await new GLTFLoader().loadAsync(xbotUrl);
  const model = gltf.scene;
  scene.add(model);

  const bones = {};
  model.traverse((obj) => {
    if (obj.isBone) { bones[obj.name] = obj; }
    if (obj.isMesh) { obj.castShadow = true; }
  });

  hideUpperBody(model); // lower-limb demo: drop the mesh above the pelvis

  // One-time check that the expected sanitized names are present.
  console.log('loaded bones:', Object.keys(bones));
  return { model, bones };
}
