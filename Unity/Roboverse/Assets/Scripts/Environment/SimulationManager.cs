/*
    Roboverse - Simulation Manager

    This script is responsible for initializing and starting the simulation environment in the Roboverse project.
*/

using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

public class SimulationManager : MonoBehaviour
{
    // ROS connection
    private ROSConnection ros;

    private float simulationTimer = 0.0f;
    private float simulationInterval = 1.0f; // Publish every second

    // Topic to publish simulation start signal
    public string startSimulationTopic = "/simulation/ready";

    void Start()
    {
        // Get or create ROS connection
        ros = ROSConnection.GetOrCreateInstance();

        // Register publisher for starting the simulation
        ros.RegisterPublisher<BoolMsg>(startSimulationTopic);
    }
    
    void Update()
    {
        simulationTimer += Time.deltaTime;

        if (simulationTimer >= simulationInterval)
        {
            PublishSimulationReady();
            simulationTimer = 0.0f;
        }
    }

    void OnApplicationQuit()
    {
        PublishSimulationNotReady();
    }

    void PublishSimulationReady()
    {
        // Create a message to signal the readiness of the simulation.
        var msg = new BoolMsg
        {
            data = true
        };

        // Publish the message to the topic
        ros.Publish(startSimulationTopic, msg);
    }

    void PublishSimulationNotReady()
    {
        // Create an empty message to signal the exit of the simulation
        var msg = new BoolMsg
        {
            data = false
        };

        // Publish the message to the topic
        ros.Publish(startSimulationTopic, msg);
    }
}