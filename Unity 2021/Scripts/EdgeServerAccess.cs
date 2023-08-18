using System;
using System.Collections;
using System.Diagnostics;
using System.Text;
using System.Timers;
using OpenCVForUnity.CoreModule;
using OpenCVForUnity.ImgcodecsModule;
using UnityEngine;
using UnityEngine.Networking;
using Vuforia;
using Debug = UnityEngine.Debug;

public class EdgeServerAccess : MonoBehaviour
{
    private const string CV_SERVER = "http://192.168.1.3:8080/";
    private const string EINK_SERVER = "http://192.168.1.2:8080/";
    private const string LIGHT_SERVER = "http://192.168.1.6:8080/";

    private static readonly double[] ValidBrightness = {0, 0.05, 0.1, 0.15, 0.2, 0.3, 0.4, 0.5, 0.75, 1};
    private static readonly int[] ValidDistance = {0, 50, 100, 150, 200, 250, 300, 350, 400, 450, 500, 550, 600, 650, 700, 750, 800};
    private static readonly int[] ValidTilt = { 75, 60, 45, 30, 15, 0};
    private static readonly int[] currValid = ValidTilt;

    [SerializeField] [TooltipAttribute("Unity Script for Accessing Camera Stream")]
    private CameraImageAccess cameraStream;
    
    [SerializeField] [TooltipAttribute("Requested FPS of the image")]
    private int fps;
    
    private readonly Timer _serverTimer = new();
    private readonly Timer _fpsTimer = new();
    private int _currIndex;
    private bool _markerFound;
    private bool _getFrame;
    private bool _postBrightness;
    private DateTime _elasped; 

    private void Start()
    {
        _serverTimer.Interval = 15000;
        _serverTimer.Enabled = true;
        _serverTimer.Elapsed += ServerTimerEventProcessor;
        _fpsTimer.Interval = 1000 / fps;
        _fpsTimer.Enabled = true;
        _fpsTimer.Elapsed += FpsTimerEventProcessor;
        _elasped = DateTime.Now;
        StartCoroutine(PostTilt());
    }

    private void Update()
    {
        if (_markerFound) return;
        if (!_getFrame) return;
        var bgramat = cameraStream.GetOpenCVMat();
        if (bgramat == null) return;
        _getFrame = false;
        var matByteStream = new MatOfByte();
        Imgcodecs.imencode(".jpg", bgramat, matByteStream);
        bgramat.Dispose();
        StartCoroutine(PutPlainMat(matByteStream, DateTimeOffset.UtcNow.ToUnixTimeMilliseconds()));
        if (!_postBrightness) return;
        _postBrightness = false;
        StartCoroutine(PostTilt());
    }

    public void OnDetection(string markerName)
    {
        _markerFound = true;
        var data = new StringBuilder();
        data.Append(markerName.Remove(markerName.Length - 1,1));
        data.Append('\t');
        data.Append(currValid[_currIndex]);
        data.Append('\t');
        data.Append(markerName[^1]);
        data.Append('\t');
        data.Append((DateTime.Now -_elasped).Milliseconds);
        data.Append('\t');
        var marker = GameObject.Find(markerName);
        var cameraAR = Camera.main; 
        var markerPos = marker.transform.position;
        var cameraPos = cameraAR!.transform.position;
        data.Append(markerPos);
        data.Append('\t');
        data.Append(cameraPos);
        data.Append('\t');
        data.Append(Vector3.Distance(markerPos, cameraPos));
        data.Append('\t');
        data.Append(marker.transform.lossyScale);
        data.Append('\t');
        data.Append(marker.transform.right);
        data.Append('\t');
        data.Append(marker.transform.up);
        data.Append('\t');
        data.Append(marker.transform.forward);
        data.Append('\t');
        data.Append(cameraAR.transform.right);
        data.Append('\t');
        data.Append(cameraAR.transform.up);
        data.Append('\t');
        data.Append(cameraAR.transform.forward);
        data.Append('\t');
        StartCoroutine(PostDetection(data.ToString()));
    }

    private void ServerTimerEventProcessor(object sender, ElapsedEventArgs e)
    {
        if (!_markerFound) _postBrightness = true;
    }
    
    private void FpsTimerEventProcessor(object sender, ElapsedEventArgs e)
    {
        _getFrame = true;
    }

    private IEnumerator PostDetection(String data)
    {
        using var www = UnityWebRequest.Put(LIGHT_SERVER + "store", data);
        yield return www.SendWebRequest();
    }
    private IEnumerator PutPlainMat(MatOfByte stream, long start)
    {
        var data = stream.toArray();
        stream.Dispose();
        using var www = UnityWebRequest.Put(CV_SERVER + "frame?value=" + getCurrent(), data);
        yield return www.SendWebRequest();
        var end = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();
        var response = www.downloadHandler.text;
        if (response.Equals("0")) yield break;
        response = response + "," + start + "," + end + "," + getCurrent();
        Debug.Log(response);
        using var store =
            UnityWebRequest.Put(CV_SERVER + "store", Encoding.UTF8.GetBytes(response));
        yield return store.SendWebRequest();
    }

    private IEnumerator PostBrightness()
    {
        var form = new WWWForm();
        _elasped = DateTime.Now;
        using var www =
            UnityWebRequest.Post(CV_SERVER + "light/brightness?value=" + currValid[_currIndex], form);
        {
            yield return www.SendWebRequest();
            if (www.result == UnityWebRequest.Result.Success) IncrementIndex();
        }
    }

    private void IncrementIndex()
    {
        _currIndex = _currIndex < currValid.Length - 1 ? _currIndex + 1 : _currIndex;
    }

    private int getCurrent()
    {
        return _currIndex == 0 ? -1 : currValid[_currIndex - 1];
    }

    private IEnumerator PostTilt()
    {
        var form = new WWWForm();
        _elasped = DateTime.Now;
        using var www =
            UnityWebRequest.Post(EINK_SERVER + "eink/tilt?value=" + currValid[_currIndex], form);
        {
            yield return www.SendWebRequest();
            if (www.result == UnityWebRequest.Result.Success) IncrementIndex();
        }
    }
    
    private IEnumerator PostDistance()
    {
        var form = new WWWForm();
        _elasped = DateTime.Now;
        using var www =
            UnityWebRequest.Post(EINK_SERVER + "eink/size?value=" + currValid[_currIndex], form);
        {
            yield return www.SendWebRequest();
            if (www.result == UnityWebRequest.Result.Success) IncrementIndex();
        }
    }
}