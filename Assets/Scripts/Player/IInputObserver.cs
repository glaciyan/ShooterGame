namespace Player
{
    public interface IInputObserver
    {
        void OnInputUpdate(in PlayerInput input);
    }
}