using UnityEngine;

namespace Player
{
    public class FirstPersonCamera : MonoBehaviour
    {
        public Transform Follow { private get; set; }

        private void LateUpdate()
        {
            var interpolationFraction = (Time.time - Time.fixedTime) / Time.fixedDeltaTime;
            transform.position =
                Vector3.Lerp(transform.position, Follow.position, interpolationFraction);
        }

        public void UpdateRotation(Vector2 rotation)
        {
            transform.rotation =
                Quaternion.Euler(new Vector3(-rotation.y, rotation.x, 0f));
        }
    }
}