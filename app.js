/**
 * Autonomous Vehicle Digital Twin for Unstructured Kutcha Roads
 * -------------------------------------------------------------
 * Layers:
 *  1. Procedural 3D Kutcha Road & Eroded Terrain Generator (with 3D Pothole Craters)
 *  2. Perception Engine (Road Width Scanlines, Pothole 3D BBoxes, Virtual Lane Synthesizer)
 *  3. Frenet Local Path Deflector & Stanley Lateral Steering Controller
 *  4. 3rd-Person Spring-Arm Chase PoV & Bird's-Eye View (BEV) Telemetry Radar
 */

(() => {
  // =========================================================================
  // 1. CONFIGURATION & SIMULATION STATE
  // =========================================================================
  const CONFIG = {
    roadLength: 600,
    roadSegments: 480,
    carWidth: 1.82,
    carLength: 4.3,
    wheelbase: 2.65,
    lookaheadDist: 14.0,
    perceptionRange: 32.0,
    scanDistances: [6, 12, 18, 24, 30]
  };

  const state = {
    s: 20.0,               // Longitudinal position along road spine (m)
    d: -0.9,               // Lateral offset from road spine (m, negative = left side)
    headingError: 0.0,     // Yaw angle relative to road tangent (rad)
    speedKmh: 28.0,        // Actual velocity (km/h)
    targetSpeedKmh: 35.0,  // User-configured cruise speed (km/h)
    steerAngle: 0.0,       // Front wheel steering angle (rad)
    stanleyGain: 0.75,     // Stanley cross-track gain k
    laneMode: 'left',      // 'left' (Keep-Left LHD) or 'center' (Center Spine)
    cameraMode: 'chase',   // 'chase', 'bev', 'sensor', 'suspension'
    paused: false,
    showScanlines: true,
    showPotholeBBox: true,
    showVirtualLane: true,
    showTrajectory: true,
    currentRoadWidth: 6.4,
    activeHazards: 0,
    plannerStatus: 'NOMINAL CRUISE'
  };

  // =========================================================================
  // 2. DETERMINISTIC NOISE & PROCEDURAL KUTCHA ROAD MATH
  // =========================================================================
  function pseudoNoise(x) {
    return (
      Math.sin(x * 0.045) * 0.45 +
      Math.sin(x * 0.11 + 1.7) * 0.30 +
      Math.cos(x * 0.27 + 4.2) * 0.15 +
      Math.sin(x * 0.63 + 0.9) * 0.10
    );
  }

  // Spine centerline X(s) in world coordinates (s is along -Z)
  function getSpineX(s) {
    return (
      Math.sin(s * 0.022) * 14.0 +
      Math.cos(s * 0.051 + 1.2) * 5.5 +
      Math.sin(s * 0.011) * 8.0
    );
  }

  // Spine tangent angle in radians
  function getSpineYaw(s) {
    const ds = 0.25;
    const dx = getSpineX(s + ds) - getSpineX(s - ds);
    return Math.atan2(dx, 2 * ds);
  }

  // Unmarked, eroded left and right road half-widths (meters from spine)
  function getLeftHalfWidth(s) {
    const base = 3.35 + pseudoNoise(s + 100) * 0.95;
    // Occasional rural pinch bottleneck
    const pinch = Math.exp(-Math.pow(((s % 140) - 75) / 14, 2)) * 0.95;
    return Math.max(1.85, base - pinch);
  }

  function getRightHalfWidth(s) {
    const base = 3.35 + pseudoNoise(s + 260) * 0.95;
    const pinch = Math.exp(-Math.pow(((s % 140) - 75) / 14, 2)) * 0.85;
    return Math.max(1.85, base - pinch);
  }

  // Generate Potholes along the Kutcha road
  const potholes = [];
  function generatePotholes(densityCount = 34) {
    potholes.length = 0;
    for (let i = 0; i < densityCount; i++) {
      const s = 32 + ((i * (CONFIG.roadLength - 60)) / densityCount) + (Math.sin(i * 7.3) * 4.5);
      const wL = getLeftHalfWidth(s);
      const wR = getRightHalfWidth(s);
      // Bias potholes within the drivable earthen surface
      const normLat = Math.sin(i * 3.91 + 0.4) * 0.65;
      const d = normLat * Math.min(wL, wR) * 0.78;
      const radius = 0.42 + (Math.abs(Math.cos(i * 5.1)) * 0.55);
      const depth = 0.07 + (Math.abs(Math.sin(i * 2.7)) * 0.13);
      potholes.push({ id: i, s, d, radius, depth });
    }
  }
  generatePotholes(34);

  // Convert Frenet (s, d) -> World (x, y, z)
  function frenetToWorld(s, d) {
    const cx = getSpineX(s);
    const yaw = getSpineYaw(s);
    const nx = Math.cos(yaw);
    const nz = -Math.sin(yaw);
    return {
      x: cx + d * nx,
      y: 0,
      z: -s + d * nz,
      yaw
    };
  }

  // =========================================================================
  // 3. THREE.JS SCENE, LIGHTING & CAMERA SETUP
  // =========================================================================
  const container = document.getElementById('canvas-container');
  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x161f2c);
  scene.fog = new THREE.FogExp2(0x161f2c, 0.009);

  const camera = new THREE.PerspectiveCamera(
    55,
    window.innerWidth / window.innerHeight,
    0.1,
    350
  );

  const renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setSize(window.innerWidth, window.innerHeight);
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  renderer.shadowMap.enabled = true;
  container.appendChild(renderer.domElement);

  const hemiLight = new THREE.HemisphereLight(0xffeedd, 0x3b2d1f, 0.85);
  scene.add(hemiLight);

  const dirLight = new THREE.DirectionalLight(0xfff3e0, 1.1);
  dirLight.position.set(45, 80, 30);
  dirLight.castShadow = true;
  scene.add(dirLight);

  // =========================================================================
  // 4. PROCEDURAL KUTCHA TERRAIN & VERTEX-CARVED POTHOLES
  // =========================================================================
  let roadMesh = null;
  function buildKutchaRoadMesh() {
    if (roadMesh) {
      scene.remove(roadMesh);
      roadMesh.geometry.dispose();
    }

    const widthSegs = 44;
    const lengthSegs = CONFIG.roadSegments;
    const planeWidth = 28;
    const geom = new THREE.PlaneGeometry(
      planeWidth,
      CONFIG.roadLength,
      widthSegs,
      lengthSegs
    );
    geom.rotateX(-Math.PI / 2);

    const pos = geom.attributes.position;
    const colors = [];
    const colorAttr = new THREE.BufferAttribute(new Float32Array(pos.count * 3), 3);

    for (let i = 0; i < pos.count; i++) {
      const localX = pos.getX(i);
      const localZ = pos.getZ(i);
      const s = -localZ + CONFIG.roadLength / 2;
      const cx = getSpineX(s);

      const wL = getLeftHalfWidth(s);
      const wR = getRightHalfWidth(s);
      const d = localX;

      let worldX = cx + d;
      let worldZ = -s;
      let y = 0.0;

      // Determine whether vertex is on Kutcha road, eroded shoulder, or scrubland
      const insideLeft = d >= -wL;
      const insideRight = d <= wR;
      const onRoad = insideLeft && insideRight;

      let r = 0.46, g = 0.34, b = 0.23; // Base earthen soil color

      if (onRoad) {
        // Subtle wheel rut & gravel micro-roughness
        const micro = Math.sin(s * 0.9 + d * 3.2) * 0.022;
        y += micro;

        // Carve 3D pothole craters into the mesh
        for (let p = 0; p < potholes.length; p++) {
          const ph = potholes[p];
          const ds = s - ph.s;
          const dd = d - ph.d;
          const distSq = ds * ds + dd * dd;
          if (distSq < ph.radius * ph.radius * 1.35) {
            const factor = Math.max(0, 1 - distSq / (ph.radius * ph.radius));
            y -= ph.depth * factor * factor * 2.2;
            // Darken crater interior (muddy depression)
            r *= (1 - 0.45 * factor);
            g *= (1 - 0.48 * factor);
            b *= (1 - 0.42 * factor);
          }
        }
      } else {
        // Unstructured rough shoulder / rural scrubland ditch
        const distOutside = d < -wL ? (-wL - d) : (d - wR);
        y = Math.sin(s * 0.25 + d * 0.5) * 0.28 + Math.min(1.4, distOutside * 0.14);
        r = 0.28 + Math.sin(s * 0.3) * 0.04;
        g = 0.33 + Math.cos(d * 0.5) * 0.04;
        b = 0.18;
      }

      pos.setXYZ(i, worldX, y, worldZ);
      colorAttr.setXYZ(i, r, g, b);
    }

    geom.setAttribute('color', colorAttr);
    geom.computeVertexNormals();

    const mat = new THREE.MeshStandardMaterial({
      vertexColors: true,
      roughness: 0.94,
      metalness: 0.04,
      flatShading: true
    });

    roadMesh = new THREE.Mesh(geom, mat);
    roadMesh.receiveShadow = true;
    scene.add(roadMesh);
  }
  buildKutchaRoadMesh();

  // Add roadside boulders & rural scrub trees for visual speed reference
  const propsGroup = new THREE.Group();
  scene.add(propsGroup);
  (function populateRoadsideProps() {
    const rockGeo = new THREE.DodecahedronGeometry(0.65, 0);
    const rockMat = new THREE.MeshStandardMaterial({ color: 0x5c5346, roughness: 0.9 });
    const trunkGeo = new THREE.CylinderGeometry(0.14, 0.22, 2.2, 6);
    const trunkMat = new THREE.MeshStandardMaterial({ color: 0x4a3525 });
    const canopyGeo = new THREE.ConeGeometry(1.6, 2.5, 7);
    const canopyMat = new THREE.MeshStandardMaterial({ color: 0x2e4c2b, flatShading: true });

    for (let s = 10; s < CONFIG.roadLength - 10; s += 9) {
      const side = (s % 18 === 0) ? -1 : 1;
      const offset = (side < 0 ? -getLeftHalfWidth(s) : getRightHalfWidth(s)) + side * (2.6 + (s % 5));
      const pt = frenetToWorld(s, offset);

      const rock = new THREE.Mesh(rockGeo, rockMat);
      rock.position.set(pt.x, 0.35, pt.z);
      rock.scale.setScalar(0.6 + (s % 7) * 0.15);
      propsGroup.add(rock);

      if (s % 27 === 0) {
        const tree = new THREE.Group();
        const trunk = new THREE.Mesh(trunkGeo, trunkMat);
        trunk.position.y = 1.1;
        const canopy = new THREE.Mesh(canopyGeo, canopyMat);
        canopy.position.y = 2.7;
        tree.add(trunk, canopy);
        tree.position.set(pt.x + side * 2.2, 0, pt.z);
        propsGroup.add(tree);
      }
    }
  })();

  // =========================================================================
  // 5. AUTONOMOUS VEHICLE RIG (CHASSIS, STEERABLE WHEELS & ROOF LIDAR/STEREO)
  // =========================================================================
  const carGroup = new THREE.Group();
  scene.add(carGroup);

  const chassisMesh = new THREE.Mesh(
    new THREE.BoxGeometry(CONFIG.carWidth, 0.85, CONFIG.carLength),
    new THREE.MeshStandardMaterial({ color: 0x1e88e5, roughness: 0.35, metalness: 0.4 })
  );
  chassisMesh.position.y = 0.72;
  chassisMesh.castShadow = true;
  carGroup.add(chassisMesh);

  const cabinMesh = new THREE.Mesh(
    new THREE.BoxGeometry(CONFIG.carWidth * 0.84, 0.62, CONFIG.carLength * 0.52),
    new THREE.MeshStandardMaterial({ color: 0x102030, roughness: 0.2, metalness: 0.6 })
  );
  cabinMesh.position.set(0, 1.42, -0.15);
  carGroup.add(cabinMesh);

  // Roof-mounted Autonomous Sensor Pod (LiDAR + Stereo RGB-D)
  const sensorPod = new THREE.Mesh(
    new THREE.CylinderGeometry(0.22, 0.26, 0.28, 16),
    new THREE.MeshStandardMaterial({ color: 0x00e5ff, emissive: 0x005566 })
  );
  sensorPod.position.set(0, 1.88, 0.1);
  carGroup.add(sensorPod);

  // 4 Wheels (Front two pivot with steering angle)
  const wheels = [];
  const wheelGeo = new THREE.CylinderGeometry(0.38, 0.38, 0.26, 18);
  wheelGeo.rotateZ(Math.PI / 2);
  const wheelMat = new THREE.MeshStandardMaterial({ color: 0x1a1a1a, roughness: 0.85 });

  const wheelOffsets = [
    { x: -0.92, z: -1.32, front: true },
    { x:  0.92, z: -1.32, front: true },
    { x: -0.92, z:  1.32, front: false },
    { x:  0.92, z:  1.32, front: false }
  ];
  wheelOffsets.forEach((wo) => {
    const wPivot = new THREE.Group();
    wPivot.position.set(wo.x, 0.38, wo.z);
    const wMesh = new THREE.Mesh(wheelGeo, wheelMat);
    wPivot.add(wMesh);
    carGroup.add(wPivot);
    wheels.push({ pivot: wPivot, mesh: wMesh, front: wo.front });
  });

  // =========================================================================
  // 6. REAL-TIME 3D PERCEPTION OVERLAYS (SCANLINES, VIRTUAL LANE, BBOXES)
  // =========================================================================
  const overlayGroup = new THREE.Group();
  scene.add(overlayGroup);

  // A. Segmented Left & Right Eroded Boundary Lines + Virtual Lane Target Line
  function createLineStrip(maxPts, hexColor) {
    const geom = new THREE.BufferGeometry();
    const positions = new Float32Array(maxPts * 3);
    geom.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    const mat = new THREE.LineBasicMaterial({ color: hexColor, linewidth: 2 });
    const line = new THREE.Line(geom, mat);
    overlayGroup.add(line);
    return { line, positions, geom };
  }

  const leftEdgeLine = createLineStrip(35, 0xffab00);
  const rightEdgeLine = createLineStrip(35, 0xffab00);
  const virtualLaneLine = createLineStrip(35, 0x00e676);
  const trajectoryLine = createLineStrip(35, 0x00e5ff);

  // B. Road Width Scanlines (+6m, +12m, +18m, +24m, +30m)
  const scanlineMeshes = CONFIG.scanDistances.map(() => {
    const lineObj = createLineStrip(2, 0x64ffda);
    return lineObj;
  });

  // C. Pothole 3D Wireframe Bounding Boxes & LiDAR Rays
  const potholeBoxPool = [];
  for (let i = 0; i < 12; i++) {
    const boxGeo = new THREE.BoxGeometry(1, 0.45, 1);
    const edges = new THREE.EdgesGeometry(boxGeo);
    const line = new THREE.LineSegments(
      edges,
      new THREE.LineBasicMaterial({ color: 0xff5252 })
    );
    line.visible = false;
    overlayGroup.add(line);
    potholeBoxPool.push(line);
  }

  // =========================================================================
  // 7. PERCEPTION, VIRTUAL LANE SYNTHESIS & FRENET-STANLEY PLANNER
  // =========================================================================
  function evaluateVirtualLaneOffset(sQuery) {
    const wL = getLeftHalfWidth(sQuery);
    const wR = getRightHalfWidth(sQuery);
    const totalW = wL + wR;

    // Synthesize nominal lateral target d_nominal
    let dTarget = 0.0;
    if (state.laneMode === 'left') {
      // Keep-Left convention with 0.65m safety margin from eroded left boundary
      const safetyMargin = 0.65;
      dTarget = -wL + safetyMargin + CONFIG.carWidth * 0.5;
      // If road narrows into a single-track bottleneck, smoothly blend to center spine
      if (totalW < 4.5) {
        dTarget = (-wL + wR) * 0.5;
      }
    } else {
      dTarget = (-wL + wR) * 0.5;
    }

    // Deflect path around detected potholes within lookahead horizon
    let hazardSlowdown = false;
    for (let i = 0; i < potholes.length; i++) {
      const ph = potholes[i];
      const ds = Math.abs(sQuery - ph.s);
      const longitudinalInfluence = ph.radius + 3.2;
      if (ds < longitudinalInfluence) {
        const lateralDist = Math.abs(dTarget - ph.d);
        const clearanceNeeded = ph.radius + CONFIG.carWidth * 0.52;
        if (lateralDist < clearanceNeeded) {
          const weight = Math.cos((ds / longitudinalInfluence) * (Math.PI / 2));
          // Check available clearance on right vs left of pothole
          const spaceRight = wR - (ph.d + ph.radius);
          const spaceLeft = (ph.d - ph.radius) - (-wL);

          if (spaceRight >= CONFIG.carWidth + 0.35) {
            dTarget += (clearanceNeeded - lateralDist + 0.35) * weight;
          } else if (spaceLeft >= CONFIG.carWidth + 0.35) {
            dTarget -= (clearanceNeeded - lateralDist + 0.35) * weight;
          } else {
            // Road is too narrow to swerve safely -> command slow crawl over crater
            hazardSlowdown = true;
          }
        }
      }
    }

    // Clamp strictly inside drivable earthen road boundaries
    const minD = -wL + CONFIG.carWidth * 0.5 + 0.25;
    const maxD =  wR - CONFIG.carWidth * 0.5 - 0.25;
    dTarget = Math.max(minD, Math.min(maxD, dTarget));

    return { dTarget, totalW, wL, wR, hazardSlowdown };
  }

  // =========================================================================
  // 8. SIMULATION STEP & 3D OVERLAY UPDATE
  // =========================================================================
  const bevCanvas = document.getElementById('bev-canvas');
  const bevCtx = bevCanvas.getContext('2d');

  function updateSimulation(dt) {
    if (state.paused) return;

    // 1. Evaluate local perception & path planner ahead of vehicle
    const currentEval = evaluateVirtualLaneOffset(state.s);
    const lookaheadEval = evaluateVirtualLaneOffset(state.s + CONFIG.lookaheadDist);
    state.currentRoadWidth = currentEval.totalW;

    // Count potholes inside 32m forward perception cone & update 3D BBoxes
    potholeBoxPool.forEach((b) => (b.visible = false));
    let activeCount = 0;
    let imminentCrawl = false;

    for (let i = 0; i < potholes.length; i++) {
      const ph = potholes[i];
      const relS = ph.s - state.s;
      if (relS > -1.0 && relS <= CONFIG.perceptionRange) {
        if (state.showPotholeBBox && activeCount < potholeBoxPool.length) {
          const box = potholeBoxPool[activeCount];
          const pt = frenetToWorld(ph.s, ph.d);
          box.position.set(pt.x, 0.22, pt.z);
          box.scale.set(ph.radius * 2, 1, ph.radius * 2);
          box.rotation.y = -pt.yaw;
          box.visible = true;
        }
        activeCount++;
        if (relS > 0 && relS < 14 && evaluateVirtualLaneOffset(ph.s).hazardSlowdown) {
          imminentCrawl = true;
        }
      }
    }
    state.activeHazards = activeCount;

    // 2. Longitudinal Speed Control (Adaptive slow-down for narrow bottlenecks or craters)
    let desiredSpeed = state.targetSpeedKmh;
    if (imminentCrawl) {
      desiredSpeed = Math.min(desiredSpeed, 12.0);
      state.plannerStatus = 'CRATER CRAWL SLOWDOWN';
    } else if (activeCount > 0 && Math.abs(lookaheadEval.dTarget - state.d) > 0.45) {
      desiredSpeed = Math.min(desiredSpeed, state.targetSpeedKmh * 0.78);
      state.plannerStatus = 'SWERVING AROUND POTHOLE';
    } else if (currentEval.totalW < 4.6) {
      desiredSpeed = Math.min(desiredSpeed, 22.0);
      state.plannerStatus = 'NARROW BOTTLENECK';
    } else {
      state.plannerStatus = 'NOMINAL VIRTUAL LANE';
    }

    state.speedKmh += (desiredSpeed - state.speedKmh) * Math.min(1, dt * 3.0);
    const velocityMs = state.speedKmh / 3.6;

    // 3. Stanley Lateral Steering Controller
    const targetD = (currentEval.dTarget * 0.35) + (lookaheadEval.dTarget * 0.65);
    const crossTrackError = targetD - state.d;
    const desiredHeading = Math.atan2(
      lookaheadEval.dTarget - currentEval.dTarget,
      CONFIG.lookaheadDist
    );
    const headingErr = desiredHeading - state.headingError;

    const stanleySteer =
      headingErr +
      Math.atan2(state.stanleyGain * crossTrackError, Math.max(velocityMs, 1.0));
    state.steerAngle = Math.max(-0.55, Math.min(0.55, stanleySteer));

    // 4. Kinematic Bicycle Model Integration in Frenet Coordinates
    state.s += velocityMs * Math.cos(state.headingError) * dt;
    state.d += velocityMs * Math.sin(state.headingError) * dt;
    state.headingError += (velocityMs / CONFIG.wheelbase) * Math.tan(state.steerAngle) * dt;
    state.headingError *= 0.92; // Damping

    // Loop road seamlessly
    if (state.s > CONFIG.roadLength - 45) {
      state.s = 20.0;
    }

    // 5. Update 3D Vehicle Pose & Suspension Pitch/Roll on Rough Soil
    const carWorld = frenetToWorld(state.s, state.d);
    const pitchBounce = Math.sin(state.s * 1.9) * 0.015;
    const rollBounce = Math.cos(state.s * 1.4 + state.d) * 0.02;

    carGroup.position.set(carWorld.x, Math.abs(pitchBounce) * 2, carWorld.z);
    carGroup.rotation.set(pitchBounce, -carWorld.yaw - state.headingError, rollBounce);

    wheels.forEach((w) => {
      w.mesh.rotation.x -= velocityMs * dt * 2.4;
      if (w.front) {
        w.pivot.rotation.y = -state.steerAngle;
      }
    });
    sensorPod.rotation.y += dt * 8.0;

    // 6. Update 3D Perception Ribbons & Road Width Scanlines
    for (let i = 0; i < 35; i++) {
      const sAhead = state.s + i * 1.0;
      const ev = evaluateVirtualLaneOffset(sAhead);

      const pLeft = frenetToWorld(sAhead, -ev.wL);
      const pRight = frenetToWorld(sAhead, ev.wR);
      const pVirt = frenetToWorld(sAhead, ev.dTarget);

      // Blend vehicle current d to target d for cyan planned trajectory
      const blend = 1 - Math.exp(-i * 0.16);
      const dTraj = state.d * (1 - blend) + ev.dTarget * blend;
      const pTraj = frenetToWorld(sAhead, dTraj);

      leftEdgeLine.positions.set([pLeft.x, 0.12, pLeft.z], i * 3);
      rightEdgeLine.positions.set([pRight.x, 0.12, pRight.z], i * 3);
      virtualLaneLine.positions.set([pVirt.x, 0.15, pVirt.z], i * 3);
      trajectoryLine.positions.set([pTraj.x, 0.18, pTraj.z], i * 3);
    }

    leftEdgeLine.geom.attributes.position.needsUpdate = true;
    rightEdgeLine.geom.attributes.position.needsUpdate = true;
    virtualLaneLine.geom.attributes.position.needsUpdate = true;
    trajectoryLine.geom.attributes.position.needsUpdate = true;

    leftEdgeLine.line.visible = state.showVirtualLane;
    rightEdgeLine.line.visible = state.showVirtualLane;
    virtualLaneLine.line.visible = state.showVirtualLane;
    trajectoryLine.line.visible = state.showTrajectory;

    CONFIG.scanDistances.forEach((dist, idx) => {
      const scanObj = scanlineMeshes[idx];
      scanObj.line.visible = state.showScanlines;
      if (state.showScanlines) {
        const sScan = state.s + dist;
        const wL = getLeftHalfWidth(sScan);
        const wR = getRightHalfWidth(sScan);
        const pL = frenetToWorld(sScan, -wL);
        const pR = frenetToWorld(sScan, wR);
        scanObj.positions.set([pL.x, 0.14, pL.z, pR.x, 0.14, pR.z], 0);
        scanObj.geom.attributes.position.needsUpdate = true;
      }
    });

    // 7. Update Camera Mode (3rd Person Chase, BEV Drone, Roof Sensor, Suspension)
    updateCamera(carWorld);
    updateHUD(crossTrackError);
    drawBEVRadar();
  }

  function updateCamera(carWorld) {
    const totalYaw = carWorld.yaw + state.headingError;
    const forwardX = Math.sin(totalYaw);
    const forwardZ = -Math.cos(totalYaw);

    if (state.cameraMode === 'chase') {
      const targetCamX = carWorld.x - forwardX * 7.2;
      const targetCamZ = carWorld.z - forwardZ * 7.2;
      camera.position.lerp(new THREE.Vector3(targetCamX, 3.3, targetCamZ), 0.12);
      camera.lookAt(carWorld.x + forwardX * 8.5, 0.9, carWorld.z + forwardZ * 8.5);
    } else if (state.cameraMode === 'bev') {
      camera.position.set(carWorld.x, 28.0, carWorld.z + 4.0);
      camera.lookAt(carWorld.x + forwardX * 8.0, 0, carWorld.z + forwardZ * 8.0);
    } else if (state.cameraMode === 'sensor') {
      camera.position.set(carWorld.x, 1.95, carWorld.z);
      camera.lookAt(carWorld.x + forwardX * 18.0, 0.6, carWorld.z + forwardZ * 18.0);
    } else if (state.cameraMode === 'suspension') {
      camera.position.set(carWorld.x - 2.3, 0.85, carWorld.z + 1.8);
      camera.lookAt(carWorld.x + forwardX * 5.0, 0.35, carWorld.z + forwardZ * 5.0);
    }
  }

  // =========================================================================
  // 9. BIRD'S-EYE VIEW (BEV) RADAR CANVAS & HUD TELEMETRY
  // =========================================================================
  function drawBEVRadar() {
    const w = bevCanvas.width;
    const h = bevCanvas.height;
    bevCtx.clearRect(0, 0, w, h);

    const originX = w / 2;
    const originY = h - 26;
    const scaleX = 11.5;
    const scaleY = 5.5;

    // Draw segmented road boundaries in BEV
    bevCtx.strokeStyle = '#ffab00';
    bevCtx.lineWidth = 2;
    ['left', 'right'].forEach((side) => {
      bevCtx.beginPath();
      for (let dist = 0; dist <= 32; dist += 2) {
        const sQ = state.s + dist;
        const edgeD = side === 'left' ? -getLeftHalfWidth(sQ) : getRightHalfWidth(sQ);
        const px = originX + edgeD * scaleX;
        const py = originY - dist * scaleY;
        if (dist === 0) bevCtx.moveTo(px, py);
        else bevCtx.lineTo(px, py);
      }
      bevCtx.stroke();
    });

    // Draw planned trajectory in BEV
    bevCtx.strokeStyle = '#00e5ff';
    bevCtx.beginPath();
    for (let dist = 0; dist <= 30; dist += 2) {
      const ev = evaluateVirtualLaneOffset(state.s + dist);
      const blend = 1 - Math.exp(-dist * 0.16);
      const dTraj = state.d * (1 - blend) + ev.dTarget * blend;
      const px = originX + dTraj * scaleX;
      const py = originY - dist * scaleY;
      if (dist === 0) bevCtx.moveTo(px, py);
      else bevCtx.lineTo(px, py);
    }
    bevCtx.stroke();

    // Draw detected potholes in BEV
    potholes.forEach((ph) => {
      const relS = ph.s - state.s;
      if (relS >= 0 && relS <= 32) {
        const px = originX + ph.d * scaleX;
        const py = originY - relS * scaleY;
        bevCtx.strokeStyle = '#ff5252';
        bevCtx.fillStyle = 'rgba(255, 82, 82, 0.28)';
        bevCtx.beginPath();
        bevCtx.arc(px, py, ph.radius * scaleX, 0, Math.PI * 2);
        bevCtx.fill();
        bevCtx.stroke();
      }
    });

    // Draw Ego Vehicle in BEV
    const egoX = originX + state.d * scaleX;
    bevCtx.fillStyle = '#00e676';
    bevCtx.fillRect(egoX - 7, originY - 12, 14, 22);
  }

  function updateHUD(crossTrackError) {
    document.getElementById('kpi-width').textContent = state.currentRoadWidth.toFixed(2) + ' m';
    document.getElementById('kpi-speed').textContent = state.speedKmh.toFixed(1) + ' km/h';
    document.getElementById('kpi-steer').textContent =
      ((state.steerAngle * 180) / Math.PI).toFixed(1) + '°';
    document.getElementById('kpi-cte').textContent = crossTrackError.toFixed(2) + ' m';
    document.getElementById('kpi-hazards').textContent = state.activeHazards;

    const banner = document.getElementById('status-banner');
    banner.textContent = state.plannerStatus;
    banner.className = 'hazard-banner';
    if (state.plannerStatus.includes('SWERVING') || state.plannerStatus.includes('BOTTLENECK')) {
      banner.classList.add('warn');
    } else if (state.plannerStatus.includes('CRAWL')) {
      banner.classList.add('danger');
    }
  }

  // =========================================================================
  // 10. UI EVENT LISTENERS & MAIN ANIMATION LOOP
  // =========================================================================
  document.querySelectorAll('[data-cam]').forEach((btn) => {
    btn.addEventListener('click', () => {
      document.querySelectorAll('[data-cam]').forEach((b) => b.classList.remove('active'));
      btn.classList.add('active');
      state.cameraMode = btn.getAttribute('data-cam');
    });
  });

  document.querySelectorAll('[data-lane]').forEach((btn) => {
    btn.addEventListener('click', () => {
      document.querySelectorAll('[data-lane]').forEach((b) => b.classList.remove('active'));
      btn.classList.add('active');
      state.laneMode = btn.getAttribute('data-lane');
    });
  });

  const speedSlider = document.getElementById('slider-speed');
  speedSlider.addEventListener('input', (e) => {
    state.targetSpeedKmh = parseFloat(e.target.value);
    document.getElementById('val-speed').textContent = state.targetSpeedKmh + ' km/h';
  });

  const gainSlider = document.getElementById('slider-gain');
  gainSlider.addEventListener('input', (e) => {
    state.stanleyGain = parseFloat(e.target.value);
    document.getElementById('val-gain').textContent = state.stanleyGain.toFixed(2);
  });

  const potholeSlider = document.getElementById('slider-potholes');
  potholeSlider.addEventListener('change', (e) => {
    const count = parseInt(e.target.value, 10);
    document.getElementById('val-potholes').textContent = count;
    generatePotholes(count);
    buildKutchaRoadMesh();
  });

  document.getElementById('chk-scanlines').addEventListener('change', (e) => {
    state.showScanlines = e.target.checked;
  });
  document.getElementById('chk-bboxes').addEventListener('change', (e) => {
    state.showPotholeBBox = e.target.checked;
  });
  document.getElementById('chk-vlane').addEventListener('change', (e) => {
    state.showVirtualLane = e.target.checked;
  });
  document.getElementById('chk-traj').addEventListener('change', (e) => {
    state.showTrajectory = e.target.checked;
  });

  window.addEventListener('resize', () => {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
  });

  let lastTime = performance.now();
  function animate(now) {
    requestAnimationFrame(animate);
    const dt = Math.min(0.05, (now - lastTime) / 1000);
    lastTime = now;
    updateSimulation(dt);
    renderer.render(scene, camera);
  }
  requestAnimationFrame(animate);
})();
