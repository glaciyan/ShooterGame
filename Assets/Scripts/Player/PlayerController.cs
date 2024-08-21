using UnityEngine;

namespace Player
{
    public class PlayerController : MonoBehaviour
    {
        public Camera playerCamera;

        private PlayerInputHandler _inputHandler;

        private Vector2 _velocity;

        private void Awake()
        {
            _inputHandler = GetComponent<PlayerInputHandler>();
        }

        private void Update()
        {
            // Visually move the Camera
            playerCamera.transform.rotation =
                Quaternion.Euler(new Vector3(-_inputHandler.VirtualViewAngle.y, _inputHandler.VirtualViewAngle.x, 0f));
        }

        private void FixedUpdate()
        {
            Debug.DrawRay(gameObject.transform.position, _inputHandler.Forward, Color.red);
            Debug.DrawRay(gameObject.transform.position, _inputHandler.RequestedMovementDirection, Color.green);
            if (_inputHandler.ConsumeJump())
            {
                Debug.Log("Jumped");
            }
        }
    }
}