/**
 * Adapted from this tutorial: https://amebouslabs.com/developing-physics-based-vr-hands-in-unity/
 */

AFRAME.registerComponent('physics-hand', {
  schema: {
    hand: { type: 'string', oneOf: ['left', 'right'] },
    positionStrength: { type: 'float', default: 20 },
    positionThreshold: { type: 'float', default: 0.005 },
    maxDistance: { type: 'float', default: 1 },
    rotationStregth: { type: 'float', default: 30 },
    rotationThreshold: { type: 'float', default: .3 }
  },

  init: function () {
    // three.js objects
    this.obj = this.el.object3D;
    this.controllerObj = document.querySelector(`#${this.data.hand}Controller`).object3D;

    // Physics vars
    this.body;
    this.velocity = new THREE.Vector3();
    this.distance = 0;
    this.angleDistance = 0;
    this.kp = 0;
    this.kd = 0;
    this.ax = new THREE.Vector4();
    this.axMag = 0;
    this.axVec = new THREE.Vector3();
    this.q = new THREE.Quaternion();
    this.q2 = new THREE.Quaternion();
    this.pidv = new THREE.Vector3();
    this.rotInertia2World = new THREE.Quaternion();
    this.angularVelocity = new THREE.Vector3();
    this.inertiaTensor = new THREE.Vector3();
    
    // Event listeners
    this.el.sceneEl.addEventListener('physx-started', () => {
      this.body = this.el.components['physx-body'];
    });
  },

  tick: function (time, timeDelta) {
    if (!this.controllerObj || !this.body) return;
    
    // Get distance between real hand position and physics hand position
    this.distance = this.controllerObj.position.distanceTo(this.obj.position);
    if (this.distance > this.data.maxDistance || this.distance < this.data.positionThreshold) {
      // Manually just set position if too far away or under threshold
      // NOTE: Attempting to warp while still colliding with an object may cause the hand
      // to bounce away and become unresponsive to setGlobalPose until shaken (don't know why),
      // so keep the maxDistance reasonably high for now to try and mitigate this.
      this.body.rigidBody.setGlobalPose({
        translation: {
          x: this.controllerObj.position.x,
          y: this.controllerObj.position.y,
          z: this.controllerObj.position.z
        },
        rotation: { 
          w: this.obj.quaternion.w, 
          x: this.obj.quaternion.x, 
          y: this.obj.quaternion.y, 
          z: this.obj.quaternion.z
        }
      }, true);
    } else {
      // Get normalized direction vector
      this.velocity.subVectors(this.controllerObj.position, this.obj.position).normalize();
      // Set physics hand velocity to normalized direction vector multiplied by follow strength
      this.velocity.multiplyScalar(this.data.positionStrength * this.distance);
      this.body.rigidBody.setLinearVelocity({
        x: this.velocity.x,
        y: this.velocity.y,
        z: this.velocity.z
      }, true);
    }

    // Get magnitude of angle between real hand rotation and physics hand rotation
    this.angleDistance = this.obj.quaternion.angleTo(this.controllerObj.quaternion);
    if (this.angleDistance < this.data.rotationThreshold || this.angleDistance > 2) {
      // Manually set rotation if distance too high or under threshold
      this.body.rigidBody.setGlobalPose({
        translation: {
          x: this.obj.position.x,
          y: this.obj.position.y,
          z: this.obj.position.z
        },
        rotation: {
          w: this.controllerObj.quaternion.w,
          x: this.controllerObj.quaternion.x,
          y: this.controllerObj.quaternion.y,
          z: this.controllerObj.quaternion.z
        }
      }, true);
      this.body.rigidBody.setAngularVelocity({ x: 0, y: 0, z: 0 }, true);
    } else {
      // This is essentially the PD code from the tutorial with two changes:
      // 1 - inertiaTensorRotation is actually just the identity quaternion in Unity, so that's subbed in instead
      // 2 - No Deg2Rad statement needed since three.js natively works with radians.
      this.kp = ((6 * this.data.rotationStregth) ** 2) * 0.25;
      this.kd = 4.5 * this.data.rotationStregth;
      this.q.copy(this.controllerObj.quaternion).multiply(this.q2.copy(this.obj.quaternion).invert());
      this.ax.setAxisAngleFromQuaternion(this.q);
      this.axVec.set(this.ax.x, this.ax.y, this.ax.z);
      this.axMag = this.ax.w;
      this.axVec.normalize();
      const { x: x1, y: y1, z: z1 } = this.body.rigidBody.getAngularVelocity()
      this.angularVelocity.set(x1, y1, z1);
      this.pidv.copy(this.axVec).multiplyScalar(this.kp).multiplyScalar(this.axMag).sub(this.angularVelocity.multiplyScalar(this.kd));
      const { x: x2, y: y2, z: z2 } = this.body.rigidBody.getMassSpaceInertiaTensor();
      this.inertiaTensor.set(x2, y2, z2);
      this.rotInertia2World.copy(this.q.identity()).multiply(this.obj.quaternion);
      this.pidv.applyQuaternion(this.rotInertia2World.invert());
      this.pidv.multiplyVectors(this.pidv, this.inertiaTensor);
      this.pidv.applyQuaternion(this.rotInertia2World);
      this.body.rigidBody.addTorque({ x: this.pidv.x, y: this.pidv.y, z: this.pidv.z });
    }
  }
});
