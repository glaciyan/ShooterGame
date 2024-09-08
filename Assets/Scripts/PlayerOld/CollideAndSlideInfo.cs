using UnityEngine;

namespace Player
{
    public struct CollideAndSlideInfo
    {
        public int NumHits;
        public Vector3 Crease;
        public Vector3 FirstPlaneNormal;
        public Vector3 SecondPlaneNormal;
        public Vector3 ThirdPlaneNormal;

        public float GetBestFloor(Vector3 down)
        {
            var downDirection = -down;
            var best = -1f;
            
            if (NumHits >= 1)
            {
                best = Mathf.Max(Vector3.Dot(FirstPlaneNormal, downDirection), best);
            }

            if (NumHits >= 2)
            {
                best = Mathf.Max(Vector3.Dot(SecondPlaneNormal, downDirection), best);
            }

            if (NumHits >= 3)
            {
                best = Mathf.Max(Vector3.Dot(ThirdPlaneNormal, downDirection), best);
            }

            return best;
        }
    }
}