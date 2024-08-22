using UnityEngine;

namespace Player
{
    public class SmoothFollow : MonoBehaviour
    {
        public GameObject subject;
        public GameObject target;

        private void Awake()
        {
            subject.transform.position = target.transform.position;
        }

        private void LateUpdate()
        {
            var interpolationFraction = (Time.time - Time.fixedTime) / Time.fixedDeltaTime;

            subject.transform.position =
                Vector3.Lerp(subject.transform.position, target.transform.position, interpolationFraction);
        }
    }
}