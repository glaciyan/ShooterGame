using UnityEngine;

namespace Player
{
    public class FirstPersonCamera : MonoBehaviour
    {
        private Transform _follow;

        public void SetFollow(Transform value)
        {
            _follow = value;
            UpdatePosition();
        }

        private void LateUpdate()
        {
            UpdatePosition();
        }

        private void UpdatePosition()
        {
            var interpolationFraction = (Time.time - Time.fixedTime) / Time.fixedDeltaTime;
            transform.position =
                Vector3.Lerp(transform.position, _follow.position, interpolationFraction);
        }

        public void UpdateRotation(Vector2 rotation)
        {
            transform.rotation =
                Quaternion.Euler(new Vector3(-rotation.y, rotation.x, 0f));
        }
    }
}