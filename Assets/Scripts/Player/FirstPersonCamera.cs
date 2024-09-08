using UnityEngine;

namespace Player
{
    public class FirstPersonCamera : MonoBehaviour, IInputObserver
    {
        private Transform _follow;
        private PlayerInput _input;
        
        public void SetFollow(Transform value)
        {
            _follow = value;
            transform.position = value.position;
        }

        private void LateUpdate()
        {
            UpdatePosition();
        }

        public void OnInputUpdate(in PlayerInput input)
        {
            transform.rotation =
                Quaternion.Euler(new Vector3(-input.VirtualViewAngle.y, input.VirtualViewAngle.x, 0f));
        }

        private void UpdatePosition()
        {
            var interpolationFraction = (Time.time - Time.fixedTime) / Time.fixedDeltaTime;
            transform.position =
                Vector3.Lerp(transform.position, _follow.position, interpolationFraction);
        }
    }
}