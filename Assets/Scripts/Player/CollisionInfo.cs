using UnityEngine;

namespace Player
{
    public struct CollisionInfo
    {
        public Vector3 NearPoint; // 12 bytes
        public float ShortDistance; // 4 bytes
        public Vector3 RemainderVelocity; // 12 bytes
        public Vector3 RelevantOffset; // 12 bytes
        public bool HasHit; // 1 byte
        public RaycastHit HitInfo; // 40 bytes
    }
}