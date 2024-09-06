using System;
using Player.Core;
using UnityEngine;

namespace Player
{
    public class MyCharacter : MonoBehaviour, ICharacterController
    {
        public KinematicCharacterMotor motor;

        private void Start()
        {
            motor.CharacterController = this;
        }

        public void UpdateRotation(ref Quaternion currentRotation, float deltaTime)
        {
            throw new NotImplementedException();
        }

        public void UpdateVelocity(ref Vector3 currentVelocity, float deltaTime)
        {
            throw new NotImplementedException();
        }

        public void BeforeCharacterUpdate(float deltaTime)
        {
            throw new NotImplementedException();
        }

        public void PostGroundingUpdate(float deltaTime)
        {
            throw new NotImplementedException();
        }

        public void AfterCharacterUpdate(float deltaTime)
        {
            throw new NotImplementedException();
        }

        public bool IsColliderValidForCollisions(Collider coll)
        {
            throw new NotImplementedException();
        }

        public void OnGroundHit(Collider hitCollider, Vector3 hitNormal, Vector3 hitPoint, ref HitStabilityReport hitStabilityReport)
        {
            throw new NotImplementedException();
        }

        public void OnMovementHit(Collider hitCollider, Vector3 hitNormal, Vector3 hitPoint,
            ref HitStabilityReport hitStabilityReport)
        {
            throw new NotImplementedException();
        }

        public void ProcessHitStabilityReport(Collider hitCollider, Vector3 hitNormal, Vector3 hitPoint, Vector3 atCharacterPosition,
            Quaternion atCharacterRotation, ref HitStabilityReport hitStabilityReport)
        {
            throw new NotImplementedException();
        }

        public void OnDiscreteCollisionDetected(Collider hitCollider)
        {
            throw new NotImplementedException();
        }
    }
}
