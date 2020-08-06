using Microsoft.MixedReality.Toolkit.Input;
using Microsoft.MixedReality.Toolkit.UI;
using MixedRealityRobot;
using MixedRealityRobot.Publishers;
using MixedRealityRobot.Subscribers;
using RosSharp.RosBridgeClient;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class StartMessages : MonoBehaviour
{
    public GameObject robotplate;
    public GameObject chassis_link;
    public GameObject StartButton;
    public GameObject Button1;
    bool flag = false;
   void Start()
    {
        chassis_link = GameObject.Find("chassis_link");
        StartButton.active = true;
    }

    public void startconnection()
    {
        //Destroy Jackal's startplate and the start button, enabled all scripts needed for operation, make the jackal non-interactable so it isn't accidentally moved and show the navigation target button.
        Destroy(robotplate);
        Destroy(StartButton);
        GetComponent<JointStatePublisher>().enabled = true;
        GetComponent<LaserScanPublisher>().enabled = true;
        GetComponent<OdomTimestampSync>().enabled = true;
        GetComponent<OdomTfPublisher>().enabled = true;
        GetComponent<OdometryPublisher>().enabled = true;
        GetComponent<DiffDriveController>().enabled = true;
        GetComponent<PathSubscriber>().enabled = true;
        GetComponent<NavGoalController>().enabled = true;
        chassis_link.GetComponent<NearInteractionGrabbable>().enabled = false;
        chassis_link.GetComponent<ManipulationHandler>().enabled = false;
        Button1.active = true;
        Destroy(this);
    }
}
