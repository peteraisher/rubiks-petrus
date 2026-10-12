// three.js meshes for a CubeModel: one group per cubie (a rounded black
// body plus its stickers), shared by the video and the thumbnail.
import * as THREE from 'three';
import { RoundedBoxGeometry } from 'three/addons/geometries/RoundedBoxGeometry.js';

export const STICKER_COLORS = { U: 0xf4f4ef, R: 0xd4202c, F: 0x12a150, D: 0xffd500, L: 0xff6b00, B: 0x1b5fe3 };

function stickerGeometry() {
  const size = 0.82, r = 0.14, h = size / 2;
  const s = new THREE.Shape();
  s.moveTo(-h + r, -h);
  s.lineTo(h - r, -h);
  s.quadraticCurveTo(h, -h, h, -h + r);
  s.lineTo(h, h - r);
  s.quadraticCurveTo(h, h, h - r, h);
  s.lineTo(-h + r, h);
  s.quadraticCurveTo(-h, h, -h, h - r);
  s.lineTo(-h, -h + r);
  s.quadraticCurveTo(-h, -h, -h + r, -h);
  return new THREE.ShapeGeometry(s, 6);
}
const STICKER_GEO = stickerGeometry();
const BODY_GEO = new RoundedBoxGeometry(0.97, 0.97, 0.97, 4, 0.1);
const BODY_MAT = new THREE.MeshStandardMaterial({ color: 0x0b0b0c, roughness: 0.55 });
const STICKER_OFFSET = 0.487;

export function mat4(rot) {
  const m = new THREE.Matrix4();
  m.set(rot[0], rot[1], rot[2], 0, rot[3], rot[4], rot[5], 0, rot[6], rot[7], rot[8], 0, 0, 0, 0, 1);
  return m;
}

/// Adds the cubies to the scene. Each group's matrix is set by the caller
/// (matrixAutoUpdate is off); `group.mats` are its sticker materials, each
/// with its full colour in `base` for dimming.
export function buildCubies(scene, cubies) {
  return cubies.map((c) => {
    const g = new THREE.Group();
    g.matrixAutoUpdate = false;
    const body = new THREE.Mesh(BODY_GEO, BODY_MAT);
    body.position.set(...c.home);
    g.add(body);
    g.mats = c.stickers.map((st) => {
      const m = new THREE.MeshStandardMaterial({ color: STICKER_COLORS[st.color], roughness: 0.32 });
      m.base = new THREE.Color(STICKER_COLORS[st.color]);
      const mesh = new THREE.Mesh(STICKER_GEO, m);
      const n = new THREE.Vector3(...st.n);
      mesh.quaternion.setFromUnitVectors(new THREE.Vector3(0, 0, 1), n);
      mesh.position.set(...c.home).addScaledVector(n, STICKER_OFFSET);
      g.add(mesh);
      return m;
    });
    scene.add(g);
    return g;
  });
}
