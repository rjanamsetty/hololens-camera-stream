using System.Numerics;
using OpenCVForUnity.CoreModule;
using OpenCVForUnity.ImgprocModule;
using UnityEngine;
using UnityEngine.UI;
using Vuforia;
using Image = Vuforia.Image;
using Vector3 = UnityEngine.Vector3;


public class CameraImageAccess : MonoBehaviour
{
    [SerializeField] [TooltipAttribute("Unity UI element to view the camera image frame")]
    private RawImage rawImage;

    [SerializeField] [TooltipAttribute("Toggle the camera image frame view")]
    private bool viewRawImage;

    private Image _image;

    private readonly PixelFormat mPixelFormat = PixelFormat.RGB888; // Editor passes in a RGBA8888 texture instead of RGB888
    private bool _mFormatRegistered;
    private Texture2D _texture;

    private int _width;
    private int _height;


    private void Start()
    {
        // Register Vuforia life-cycle callbacks:
        VuforiaApplication.Instance.OnVuforiaStarted += RegisterFormat;
        VuforiaApplication.Instance.OnVuforiaPaused += OnPause;
        VuforiaBehaviour.Instance.World.OnStateUpdated += OnVuforiaUpdated;
        if (!viewRawImage) rawImage.transform.localScale += new Vector3(-1f, -1f, -1f);
        
    }

    private void OnDestroy()
    {
        // Unregister Vuforia life-cycle callbacks:
        VuforiaApplication.Instance.OnVuforiaStarted -= RegisterFormat;
        VuforiaApplication.Instance.OnVuforiaPaused -= OnPause;
        VuforiaBehaviour.Instance.World.OnStateUpdated -= OnVuforiaUpdated;
    }

    /// Called each time the Vuforia state is updated
     void OnVuforiaUpdated()
    {
        //if (!_mFormatRegistered) return;
        //if(_texture) Destroy(_texture);
        //_texture = new Texture2D(_width, _height, TextureFormat.BGRA32, false);
        //_image = VuforiaBehaviour.Instance.CameraDevice.GetCameraImage(mPixelFormat);
        //_image.CopyBufferToTexture(_texture);
        //_texture.Apply();

        /*
        Debug.Log(
            "\nImage Format: " + _image.PixelFormat +
            "\nImage Size: " + _image.Width + " x " + _image.Height +
            "\nBuffer Size: " + _image.BufferWidth + " x " + _image.BufferHeight +
            "\nImage Stride: " + _image.Stride + "\n"
        );
        */
        /*
        if (!_texture || !viewRawImage) return;
        if(rawImage.texture) Destroy(rawImage.texture);
        rawImage.texture = _texture;
        rawImage.material.mainTexture = _texture;
        */
    }

    /// Called when app is paused / resumed
     void OnPause(bool paused)
    {
        if (paused)
        {
            Debug.Log("App was paused");
            UnregisterFormat();
        }
        else
        {
            Debug.Log("App was resumed");
            RegisterFormat();
        }
    }

    /// Register the camera pixel format
     void RegisterFormat()
    {
        // Vuforia has started, now register camera image format
        var success = VuforiaBehaviour.Instance.CameraDevice.SetFrameFormat(mPixelFormat, true);
        if (success)

        {
            Debug.Log("Successfully registered pixel format " + mPixelFormat);
            _mFormatRegistered = true;
        }
        else
        {
            Debug.LogError(
                "Failed to register pixel format " + mPixelFormat +
                "\n the format may be unsupported by your device;" +
                "\n consider using a different pixel format.");
            _mFormatRegistered = false;
        }
    }

    /// Unregister the camera pixel format (e.g. call this when app is paused)
    private void UnregisterFormat()
    {
        Debug.Log("Unregistering camera pixel format " + mPixelFormat);
        VuforiaBehaviour.Instance.CameraDevice.SetFrameFormat(mPixelFormat, false);
        _mFormatRegistered = false;
    }

    // https://gist.github.com/dtaillard/7d7efa6ff44e9e69e12d8309476627f7
    // https://library.vuforia.com/platform-support/working-camera-unity
    // https://github.com/EnoxSoftware/VuforiaWithOpenCVForUnityExample/blob/master/Assets/VuforiaWithOpenCVForUnityExample/CameraImageToMatExample.cs
    public Mat GetOpenCVMat()
    {
        if (!_mFormatRegistered) return null;
        _image = VuforiaBehaviour.Instance.CameraDevice.GetCameraImage(mPixelFormat);
        if (_image == null || _image.Height == 0 || _image.Width == 0) return null;
        var mat = new Mat(_image.Height, _image.Width, CvType.CV_8UC3);
        mat.put(0, 0, _image.Pixels);
        Imgproc.resize(mat, mat, new Size(), 0.5, 0.5);
        return mat;
    }
}