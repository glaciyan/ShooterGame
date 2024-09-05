using UnityEngine;

namespace Player
{
    public struct CollisionInfo
    {
        public Vector3 NearPoint;
        public float ShortDistance;
        public Vector3 RemainderVelocity;
        public Vector3 RelevantOffset;
        public bool HasHit;
        public RaycastHit HitInfo;
        public bool Failed;
    }
}