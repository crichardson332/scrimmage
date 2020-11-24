var CesiumAir = {
  uri: './models/CesiumAir/Cesium_Air.gltf',
  // rotation: new Cesium.Quaternion(0, 0, 0, 1),
  // rotation: new Cesium.Quaternion(0, 0, 0.707, 0.707),
  rotation: {
    x: 0,
    y: 0,
    z: 0.707,
    w: 0.707,
  },
  runAnimations: true,
  minimumPixelSize: 64,
  maximumScale: 20000,
}

var model_map = {
  'default': CesiumAir,
}
