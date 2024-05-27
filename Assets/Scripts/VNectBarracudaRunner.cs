using UnityEngine;
using UnityEngine.UI;
using System.Collections;
using System.Collections.Generic;
using UnityEngine.Networking;

public class VNectBarracudaRunner : MonoBehaviour
{
    public VNectModel VNectModel;
    public VideoCapture videoCapture;

    private VNectModel.JointPoint[] jointPoints;
    private const int JointNum = 24;
    public float KalmanParamQ;
    public float KalmanParamR;
    private bool Lock = true;
    public bool UseLowPassFilter;
    public float LowPassParam;
    public Text Msg;
    public float WaitTimeModelLoad = 10f;
    private float Countdown = 0;

    private void Start()
    {
        jointPoints = VNectModel.Init();
        StartCoroutine("WaitLoad");
    }

    private void Update()
    {
        if (!Lock)
        {
            StartCoroutine(FetchPoseDataFromServer());
        }
    }

    private IEnumerator WaitLoad()
    {
        // Wait for the initial setup
        yield return new WaitForSeconds(WaitTimeModelLoad);
        int bgWidth = 640;  // Replace with actual width
        int bgHeight = 480; // Replace with actual height
        videoCapture.Init(bgWidth, bgHeight);
        Lock = false;
        Msg.gameObject.SetActive(false);
    }

    private IEnumerator FetchPoseDataFromServer()
    {
        using (UnityWebRequest webRequest = UnityWebRequest.Get("http://localhost:5000/api/getposedata"))
        {
            // Request and wait for the desired page.
            yield return webRequest.SendWebRequest();

            if (webRequest.isNetworkError || webRequest.isHttpError)
            {
                Debug.LogError(": Error: " + webRequest.error);
            }
            else
            {
                ProcessPoseData(webRequest.downloadHandler.text);
            }
        }
    }

    private void ProcessPoseData(string jsonData)
    {
        PoseData poseData = JsonUtility.FromJson<PoseData>(jsonData);

        for (int j = 0; j < JointNum; j++)
        {
            jointPoints[j].Now3D = poseData.jointPositions[j];
        }

        PredictPose();
    }

    private void PredictPose()
    {
        // Calculate hip location
        var lc = (jointPoints[PositionIndex.rThighBend.Int()].Now3D + jointPoints[PositionIndex.lThighBend.Int()].Now3D) / 2f;
        jointPoints[PositionIndex.hip.Int()].Now3D = (jointPoints[PositionIndex.abdomenUpper.Int()].Now3D + lc) / 2f;

        // Calculate neck location
        jointPoints[PositionIndex.neck.Int()].Now3D = (jointPoints[PositionIndex.rShldrBend.Int()].Now3D + jointPoints[PositionIndex.lShldrBend.Int()].Now3D) / 2f;

        // Calculate head location
        var cEar = (jointPoints[PositionIndex.rEar.Int()].Now3D + jointPoints[PositionIndex.lEar.Int()].Now3D) / 2f;
        var hv = cEar - jointPoints[PositionIndex.neck.Int()].Now3D;
        var nhv = Vector3.Normalize(hv);
        var nv = jointPoints[PositionIndex.Nose.Int()].Now3D - jointPoints[PositionIndex.neck.Int()].Now3D;
        jointPoints[PositionIndex.head.Int()].Now3D = jointPoints[PositionIndex.neck.Int()].Now3D + nhv * Vector3.Dot(nhv, nv);

        // Calculate spine location
        jointPoints[PositionIndex.spine.Int()].Now3D = jointPoints[PositionIndex.abdomenUpper.Int()].Now3D;

        // Kalman filter
        foreach (var jp in jointPoints)
        {
            KalmanUpdate(jp);
        }

        // Low pass filter
        if (UseLowPassFilter)
        {
            foreach (var jp in jointPoints)
            {
                jp.PrevPos3D[0] = jp.Pos3D;
                for (var i = 1; i < jp.PrevPos3D.Length; i++)
                {
                    jp.PrevPos3D[i] = jp.PrevPos3D[i] * LowPassParam + jp.PrevPos3D[i - 1] * (1f - LowPassParam);
                }
                jp.Pos3D = jp.PrevPos3D[jp.PrevPos3D.Length - 1];
            }
        }
    }

    void KalmanUpdate(VNectModel.JointPoint measurement)
    {
        measurementUpdate(measurement);
        measurement.Pos3D.x = measurement.X.x + (measurement.Now3D.x - measurement.X.x) * measurement.K.x;
        measurement.Pos3D.y = measurement.X.y + (measurement.Now3D.y - measurement.X.y) * measurement.K.y;
        measurement.Pos3D.z = measurement.X.z + (measurement.Now3D.z - measurement.X.z) * measurement.K.z;
        measurement.X = measurement.Pos3D;
    }

    void measurementUpdate(VNectModel.JointPoint measurement)
    {
        measurement.K.x = (measurement.P.x + KalmanParamQ) / (measurement.P.x + KalmanParamQ + KalmanParamR);
        measurement.K.y = (measurement.P.y + KalmanParamQ) / (measurement.P.y + KalmanParamQ + KalmanParamR);
        measurement.K.z = (measurement.P.z + KalmanParamQ) / (measurement.P.z + KalmanParamQ + KalmanParamR);
        measurement.P.x = KalmanParamR * (measurement.P.x + KalmanParamQ) / (KalmanParamR + measurement.P.x + KalmanParamQ);
        measurement.P.y = KalmanParamR * (measurement.P.y + KalmanParamQ) / (KalmanParamR + measurement.P.y + KalmanParamQ);
        measurement.P.z = KalmanParamR * (measurement.P.z + KalmanParamQ) / (KalmanParamR + measurement.P.z + KalmanParamQ);
    }

    [System.Serializable]
    public class PoseData
    {
        public Vector3[] jointPositions;
    }
}
