/*
    Roboverse - Unicycle Controller
    This script controls the unicycle's movement by subscribing to ROS topics and publishing the unicycle's current pose.
*/

using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class UnicycleController : MonoBehaviour
{
    // Topics
    public string ctrlTopic = "/dynamics/ctrl";
    public string currentPoseTopic = "/dynamics/current_pose";

    // Reference to dynamics handler
    private UnicycleDynamics unicycleDynamics;

    // ROS connection
    private ROSConnection ros;

    void Start()
    {
        // Get unicycle dynamics script
        unicycleDynamics = GetComponent<UnicycleDynamics>();

        // Get or create ROS connection
        ros = ROSConnection.GetOrCreateInstance();

        // Register publisher
        ros.RegisterPublisher<PoseMsg>(currentPoseTopic);

        // Subscribe to /dynamics/ctrl
        ros.Subscribe<TwistMsg>(ctrlTopic, CmdCallback);
    }

    void Update()
    {
        // Publish current pose each frame
        PublishCurrentPose();
    }

    void PublishCurrentPose()
    {
        // Get position and rotation from Unity
        Vector3 pos = transform.position;
        var rotation = new QuaternionMsg(
            transform.eulerAngles.x * Mathf.Deg2Rad,
            transform.eulerAngles.y * Mathf.Deg2Rad,
            transform.eulerAngles.z * Mathf.Deg2Rad,
            1.0f
        );
        
        var msg = new PoseMsg
        {
            position = new PointMsg
            {
                x = pos.x,
                y = pos.y,
                z = pos.z
            },
            orientation = rotation
        };


        ros.Publish(currentPoseTopic, msg);
    }

    void CmdCallback(TwistMsg msg)
    {
        // Apply received commands to unicycle dynamics
        // Make sure you match the fields your ROS node uses!
        unicycleDynamics.linearSpeed = (float) msg.linear.z;
        unicycleDynamics.angularSpeed = (float) msg.angular.y;

        // Log the received command for debugging
        Debug.Log($"Received command from Roboverse Server: linearSpeed={unicycleDynamics.linearSpeed}, angularSpeed={unicycleDynamics.angularSpeed}");
    }
}
