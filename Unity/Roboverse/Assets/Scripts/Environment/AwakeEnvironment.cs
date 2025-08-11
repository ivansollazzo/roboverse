using UnityEngine;

public class AwakeEnvironment : MonoBehaviour
{
    void Awake()
    {
        Application.runInBackground = true;
    }
}
