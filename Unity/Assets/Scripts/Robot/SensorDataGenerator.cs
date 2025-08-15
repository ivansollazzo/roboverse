using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

public class SensorDataGenerator : MonoBehaviour
{
    [Header("Current Values")]
    [SerializeField] private float temperature;
    [SerializeField] private float humidity;
    [SerializeField] private float airQuality;

    private ROSConnection ros;

    public string rawDataTopic = "/simulation/raw_data_0";

    private void Start()
    {
        // ROS connection
        ros = ROSConnection.GetOrCreateInstance();

        // Register
        ros.RegisterPublisher<Float32MultiArrayMsg>(rawDataTopic);

        GenerateRandomValues();
        InvokeRepeating("GenerateRandomValues", 1f, 2f);
    }

    private void GenerateRandomValues()
    {
        temperature = Random.Range(19.5f, 20.5f);
        humidity = Random.Range(60.0f, 75.0f);
        airQuality = Random.Range(50.0f, 55.0f);

        PublishSensorRawData();
    }

    private void PublishSensorRawData()
    {
        // Create a message with the sensor data
        var msg = new Float32MultiArrayMsg
        {
            data = new float[] { temperature, humidity, airQuality }
        };

        // Publish the message to the topic
        ros.Publish(rawDataTopic, msg);
    }
}