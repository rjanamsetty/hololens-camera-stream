using System;
using System.Collections;
using System.Collections.Generic;
using System.Data;
using HoloLensCameraStream;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.SceneManagement;
using OpenCVForUnity.CoreModule;
using OpenCVForUnity.ImgprocModule;
using OpenCVForUnity.UnityUtils.Helper;
using OpenCVForUnity.UnityUtils;
using Rect = OpenCVForUnity.CoreModule.Rect;
using TMPro;
using Microsoft.MixedReality.Toolkit;
using OpenCVForUnity.Features2dModule;
using OpenCVForUnity.UtilsModule;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Text;
using Microsoft.MixedReality.Toolkit.Utilities.Solvers;
using UnityEngine.Windows.Speech;

#if ENABLE_WINMD_SUPPORT
using Windows.Media.Devices.Core;
#endif

using Microsoft.MixedReality.Toolkit.Input;
using OpenCVForUnity.ImgcodecsModule;
using UnityEngine.Networking;
using Debug = UnityEngine.Debug;


namespace Script
{
    /// <summary>
    /// HoloLens Red Area Detection Example
    /// An example of image processing (Red area detection) using OpenCVForUnity on Hololens.
    /// Comic filter processing is implemented when running on Unity.
    /// </summary>
    [RequireComponent(typeof(HololensCameraStreamToMatHelper))]
    public class HololensLaserPointerDetection : MonoBehaviour
    {
        [SerializeField]
        [TooltipAttribute(
            "Unprojection Calibration: Set offset of unprojected pixel coordinates; Offset unit is meter.")]
        private Vector2 unprojectionOffset = new Vector2(0.0f, -0.05f);

        [SerializeField]
        [TooltipAttribute(
            "Centered portion of the image used as detection area. Factors for x and y coordinates between 0 and 1.")]
        private Vector2 detectionArea = new Vector2(0.5f, 0.5f);

        [SerializeField] [TooltipAttribute("Capture data display flag for debugging.")]
        private bool isVisibleImage = false;

        [SerializeField]
        [TooltipAttribute("Enable display of processed frames per second for performance measurements.")]
        private bool showFPS = false;

        [SerializeField]
        [TooltipAttribute("Use fast pointer detection algorithm optimized for performance rather than accuracy.")]
        private bool fastDetection = false;

        [SerializeField] [TooltipAttribute("Prefab used for the display in current algorithm")]
        private GameObject pointPrefab;

        [SerializeField] [TooltipAttribute("Prefab used for the marker area in current algorithm")]
        private MarkerArea markerPrefab;

        [SerializeField] private HololensCameraStreamToMatHelper cameraStream;

        /// <summary>
        /// The texture.
        /// </summary>
        private Texture2D texture;

        /// <summary>
        /// The quad renderer.
        /// </summary>
        private Renderer quad_renderer;

        // Mask to RayCast only the spatial recognition layer
        private readonly int SpatialAwarnessLayerMask = 1 << 31;

        private GameObject redSphere;
        private TextMeshPro toolTipText;
        private PhysicsScene physicsScene;
        private Camera _mainCamera;
        private List<GameObject> _points = new List<GameObject>();
        private List<double> toSave;
        private const string URI = "http://172.20.10.4:5000/brightness?value=65535";

        private static readonly Queue<Action> ExecuteOnMainThread = new Queue<Action>();

        private float timeSpan = 0.0f; // time keeping
        private float prevTime = -1.0f; // used for
        private int numFrames = 0; // calculating FPS 
        private static readonly int WorldToCameraMatrix = Shader.PropertyToID("_WorldToCameraMatrix");
        private Matrix4x4 _webcamMatrix;
        private DateTime referenceTime = DateTime.Now;
        private KeywordRecognizer keywordRecognizer;
        private Dictionary<string, Action> keywords = new Dictionary<string, Action>();
        private SquareDetection _squares;
        private bool sendRequest = true;

        // Use this for initialization
        protected void Start()
        {
            _mainCamera = Camera.main;
            if (!isVisibleImage) gameObject.transform.localScale = new Vector3(0.0f, 0.0f, 0.0f);
            InitWebCamHelper();
            PointerUtils.SetGazePointerBehavior(PointerBehavior.AlwaysOn);
            keywords.Add("save data", () => StartCoroutine(SaveData()));
            keywords.Add("create data section", () => StartCoroutine(CreateDataSection()));
            keywordRecognizer = new KeywordRecognizer(keywords.Keys.ToArray());
            keywordRecognizer.OnPhraseRecognized += KeywordRecognizer_OnPhraseRecognized;
            keywordRecognizer.Start();
            _squares = new SquareDetection();
        }

        private void KeywordRecognizer_OnPhraseRecognized(PhraseRecognizedEventArgs args)
        {
            Action keywordAction;
            // if the keyword recognized is in our dictionary, call that Action.
            if (keywords.TryGetValue(args.text, out keywordAction))
            {
                keywordAction.Invoke();
            }
        }

        private void InitializeLaserPointer()
        {
            if (redSphere != null) return;
            redSphere = Instantiate(pointPrefab);
            toolTipText = redSphere.transform.Find("ToolTip/Label").GetComponent<TextMeshPro>();
        }

        /// <summary>
        /// Registers callback for incoming video frames and starts capturing video frames 
        /// </summary>
        private void InitWebCamHelper()
        {
#if ENABLE_WINMD_SUPPORT
                cameraStream.FrameMatAcquired += OnFrameMatAcquired;
#endif
            cameraStream.Initialize();
        }

        /// <summary>
        /// Raises the web cam texture to mat helper initialized event.
        /// </summary>
        public void OnWebCamTextureToMatHelperInitialized()
        {
            Debug.Log("OnWebCamTextureToMatHelperInitialized");
            _webcamMatrix = cameraStream.GetCameraToWorldMatrix();
            var webCamTextureMat = cameraStream.GetMat();

            // Require all images to be in BGRA format
            texture = new Texture2D(webCamTextureMat.cols(), webCamTextureMat.rows(), TextureFormat.BGRA32, false);
            texture.wrapMode = TextureWrapMode.Clamp;

            Debug.Log("Screen.width " + Screen.width + " Screen.height " + Screen.height + " Screen.orientation " +
                      Screen.orientation);

            Matrix4x4 projectionMatrix;

#if ENABLE_WINMD_SUPPORT
            projectionMatrix = cameraStream.GetProjectionMatrix();
#else
            //This value is obtained from PhotoCapture's TryGetProjectionMatrix() method.I do not know whether this method is good.
            //Please see the discussion of this thread.Https://forums.hololens.com/discussion/782/live-stream-of-locatable-camera-webcam-in-unity
            projectionMatrix = Matrix4x4.identity;
            projectionMatrix.m00 = 2.31029f;
            projectionMatrix.m01 = 0.00000f;
            projectionMatrix.m02 = 0.09614f;
            projectionMatrix.m03 = 0.00000f;
            projectionMatrix.m10 = 0.00000f;
            projectionMatrix.m11 = 4.10427f;
            projectionMatrix.m12 = -0.06231f;
            projectionMatrix.m13 = 0.00000f;
            projectionMatrix.m20 = 0.00000f;
            projectionMatrix.m21 = 0.00000f;
            projectionMatrix.m22 = -1.00000f;
            projectionMatrix.m23 = 0.00000f;
            projectionMatrix.m30 = 0.00000f;
            projectionMatrix.m31 = 0.00000f;
            projectionMatrix.m32 = -1.00000f;
            projectionMatrix.m33 = 0.00000f;
#endif
            quad_renderer = gameObject.GetComponent<Renderer>();
            quad_renderer.sharedMaterial.SetTexture("_MainTex", texture);
            quad_renderer.sharedMaterial.SetMatrix("_CameraProjectionMatrix", projectionMatrix);

            var halfOfVerticalFov = Mathf.Atan(1.0f / projectionMatrix.m11);
            var aspectRatio = 1.0f / Mathf.Tan(halfOfVerticalFov) / projectionMatrix.m00;
            Debug.Log("halfOfVerticalFov " + halfOfVerticalFov);
            Debug.Log("aspectRatio " + aspectRatio);
        }

        /// <summary>
        /// Raises the web cam texture to mat helper disposed event.
        /// </summary>
        public void OnWebCamTextureToMatHelperDisposed()
        {
            Debug.Log("OnWebCamTextureToMatHelperDisposed");

            lock (ExecuteOnMainThread)
            {
                ExecuteOnMainThread.Clear();
            }
        }

        /// <summary>
        /// Raises the web cam texture to mat helper error occurred event.
        /// </summary>
        /// <param name="errorCode">Error code.</param>
        public void OnWebCamTextureToMatHelperErrorOccurred(WebCamTextureToMatHelper.ErrorCode errorCode)
        {
            Debug.Log("OnWebCamTextureToMatHelperErrorOccurred " + errorCode);
        }

        /// <summary>
        /// Fast version: Find brightest red pixel in a masked image 
        /// Use Laplacian filter to detect areas where the red color component has changed more than blue or green
        /// </summary>
        /// <param name="bgraMat">Image data in BGRA format</param>
        /// <param name="mask">mask of relavant areas to search</param>
        /// <param name="originalWidth">width of the original (non-cropped) image used for parameter tuning</param>
        private Point FastFindBrightestPoint(Mat bgraMat, Mat mask, int originalWidth)
        {
            var ksize = originalWidth > 1600 ? 15 : 11;
            var blurred = new Mat();
            Imgproc.blur(bgraMat, blurred, new Size(ksize, ksize));

            var redBlurred = new Mat();
            Core.extractChannel(blurred, redBlurred, 2);

            var red = new Mat();
            Core.extractChannel(bgraMat, red, 2);

            red -= redBlurred;

            var minMax =
                Core.minMaxLoc(red, mask); // get pixel with strongest contrast towards red among reddish things

            // experimental size detection: only accept small objects as laser pointer
            //Scalar zero = new Scalar(0);
            //Mat fillMask = new Mat(red.height(), red.width(), CvType.CV_8UC1, zero);
            //Rect rect = new Rect();
            //Imgproc.floodFill(red, fillMask, minMax.maxLoc, zero, rect, zero /*new Scalar(minMax.maxVal/2)*/, new Scalar(minMax.maxVal), 4 | Imgproc.FLOODFILL_FIXED_RANGE);
            //if (rect.area() > 1)
            //{
            //    return null;
            //}

            blurred.Dispose();
            redBlurred.Dispose();
            red.Dispose();

            return minMax.maxLoc;
        }


        /// <summary>
        /// Find brightest red pixel in a masked image 
        /// Use Laplacian filter to detect areas where the red color component has changed more than blue or green
        /// </summary>
        /// <param name="bgraMat">Image data in BGRA format</param>
        /// <param name="mask">mask of relavant areas to search</param>
        /// <param name="originalWidth">width of the original (non-cropped) image used for parameter tuning</param>
        private Point FindBrightestPoint(Mat bgraMat, Mat mask, int originalWidth)
        {
            var channel = new Mat(bgraMat.height(), bgraMat.width(), CvType.CV_8UC1);
            var blueContrast = new Mat(bgraMat.height(), bgraMat.width(), CvType.CV_16SC1);
            var greenContrast = new Mat(bgraMat.height(), bgraMat.width(), CvType.CV_16SC1);
            var redContrast = new Mat(bgraMat.height(), bgraMat.width(), CvType.CV_16SC1);
            var sum = new Mat(bgraMat.height(), bgraMat.width(), CvType.CV_16SC1);

            var kernelSize = originalWidth > 1600 ? 5 : 3;
            Core.extractChannel(bgraMat, channel, 0);
            Imgproc.Laplacian(channel, blueContrast, CvType.CV_16S, kernelSize, 1, 0,
                Core.BORDER_REPLICATE); // calculate contrast in blue channel to detect sudden increase of red component
            Core.extractChannel(bgraMat, channel, 1);
            Imgproc.Laplacian(channel, greenContrast, CvType.CV_16S, kernelSize, 1, 0,
                Core.BORDER_REPLICATE); // calculate contrast in green channel to detect sudden increase of red component
            Core.extractChannel(bgraMat, channel, 2);
            Imgproc.Laplacian(channel, redContrast, CvType.CV_16S, kernelSize, 1, 0,
                Core.BORDER_REPLICATE); // calculate contrast in red channel to detect sudden increase of red component

            channel.convertTo(sum, CvType.CV_16S);
            sum *= 2; // give absolute brightness of red component double weight
            sum -= 3 * redContrast; // boost weight of pixels with a negative red curvature, i.e. whose neighbors are less red.
            sum += blueContrast; // oposite weights for changes in blue and green contrast, to give pixels a stronger boost whose red component changed but blue and green remained constant 
            sum += greenContrast;

            var minMax =
                Core.minMaxLoc(sum, mask); // get pixel with strongest contrast towards red among reddish things

            sum.Dispose();
            redContrast.Dispose();
            greenContrast.Dispose();
            blueContrast.Dispose();
            channel.Dispose();

            return minMax.maxLoc;
        }

        /// <summary>
        /// Extract detection area from camera image 
        /// </summary>
        /// <param name="img">Image data</param>
        private Mat GetROI(Mat img)
        {
            var w = img.width();
            var h = img.height();
            var ROIwidth = Math.Max(0, Math.Min(w, (int) Math.Round(w * detectionArea.x * 0.5) * 2));
            var ROIheight = Math.Max(0, Math.Min(h, (int) Math.Round(h * detectionArea.y * 0.5) * 2));
            var ROI = new Rect((w - ROIwidth) / 2, (h - ROIheight) / 2, ROIwidth, ROIheight);

            return new Mat(img, ROI);
        }

        private Point ROItoOriginal(Mat img, Point pt)
        {
            var w = img.width();
            var h = img.height();
            var ROIwidth = Math.Max(0, Math.Min(w, (int) Math.Round(w * detectionArea.x * 0.5) * 2));
            var ROIheight = Math.Max(0, Math.Min(h, (int) Math.Round(h * detectionArea.y * 0.5) * 2));
            var x = (w - ROIwidth) / 2;
            var y = (h - ROIheight) / 2;
            return new Point(pt.x + x, pt.y + y);
        }

        /// <summary>
        /// Find location of red laser pointer in image 
        /// </summary>
        /// <param name="bgraMat">Image data in BGRA format</param>
        private Point FindLaserPointer(Mat bgraMat)
        {
            var croppedBgra = GetROI(bgraMat);

            if (!fastDetection && bgraMat.width() > 1600)
                Imgproc.GaussianBlur(croppedBgra, croppedBgra, new Size(3, 3),
                    0); // apply a gentle blur to high resultion images to remove noise

            var hsvMat = new Mat(croppedBgra.height(), croppedBgra.width(), CvType.CV_8UC3);
            var maskMat1 = new Mat(croppedBgra.height(), croppedBgra.width(), CvType.CV_8UC1);
            var maskMat2 = new Mat(croppedBgra.height(), croppedBgra.width(), CvType.CV_8UC1);

            // Color conversion from BGRA to HSV
            Imgproc.cvtColor(croppedBgra, hsvMat, Imgproc.COLOR_BGRA2BGR);
            Imgproc.cvtColor(hsvMat, hsvMat, Imgproc.COLOR_BGR2HSV);

            // Acquire a mask image of reddish pixels using the inRange method. 
            // Red is separated in two areas in the HSV color space
            var s_min = new Scalar(0, 30, 220);
            var s_max = new Scalar(10, 240, 255);
            Core.inRange(hsvMat, s_min, s_max, maskMat1);
            s_min = new Scalar(170, 30, 220);
            s_max = new Scalar(180, 240, 255);
            Core.inRange(hsvMat, s_min, s_max, maskMat2);

            maskMat1 |= maskMat2;

            Point point = null;
            if (Core.countNonZero(maskMat1) > 0)
            {
                point = fastDetection
                    ? FastFindBrightestPoint(croppedBgra, maskMat1, bgraMat.width())
                    : FindBrightestPoint(croppedBgra, maskMat1, bgraMat.width());
                if (point != null)
                {
                    // correct detection coordinates to original full image
                    point.x += (bgraMat.width() - croppedBgra.width()) / 2;
                    point.y += (bgraMat.height() - croppedBgra.height()) / 2;
                }
            }

            croppedBgra.Dispose();
            hsvMat.Dispose();
            maskMat1.Dispose();
            maskMat2.Dispose();

            return point;
        }

        /// <summary>
        /// Finds the list of features, using the ORB algorithm, for feature visualization
        /// </summary>
        /// <param name="bgraMat">The image from the camera stream</param>
        /// <returns>A list of points denoting the coordinates of each feature</returns>
        private MatOfKeyPoint FindFeatures(Mat bgraMat)
        {
            // Initialize the detector
            var orb = ORB.create();

            // Convert color to grayscale and slightly blur it
            var processed = new Mat();
            Imgproc.cvtColor(bgraMat, processed, Imgproc.COLOR_BGRA2GRAY);

            // Detect the features in the frame
            var matPoints = new MatOfKeyPoint();
            orb.detect(bgraMat, matPoints);

            // Dispose all variables used for calculations
            orb.Dispose();
            processed.Dispose();

            // Return from function
            return matPoints;
        }

        /// <summary>
        /// return frames per second processed by laserpointer detection on average over the last 3 seconds
        /// </summary>
        private float getFPS()
        {
            var now = Time.time;
            if (prevTime < 0) // if this is the first time measurment...
            {
                // ...return 0
                prevTime = now;
                return 0.0f;
            }

            if (timeSpan > 3 &&
                numFrames > 0) // adjust time span or number of frames wrt. to 3 seconds measurement window
                timeSpan *= (numFrames - 1.0f) / numFrames;
            else
                numFrames++;
            timeSpan += now - prevTime;
            prevTime = now;
            return numFrames / timeSpan;
        }

#if ENABLE_WINMD_SUPPORT
        private Vector3 Unproject(Point cameraPoint, Matrix4x4 cameraToWorldMatrix, CameraIntrinsics camIntrinsics)
        {
            // Convert the first point of the detected contour to Unity world coordinates
            //            	Point[] countoursPoint = contours[0].toArray();
            //              Windows.Foundation.Point orgpoint = new Windows.Foundation.Point(countoursPoint[0].x, countoursPoint[0].y);
            var origin = new Windows.Foundation.Point(cameraPoint.x, cameraPoint.y);

            // Unprojects pixel coordinates into a camera space ray from the camera origin, expressed as a X, Y coordinates on a plane one meter from the camera.
            var result = camIntrinsics.UnprojectAtUnitDepth(origin);
            // manual calibration: correct y-axes by 5 cm to get better unprojection accurracy
            var pos = new Vector3(result.X + unprojectionOffset.x, result.Y + unprojectionOffset.y, 1.0f);

            // Convert from camera coordinates to world coordinates and RayCast in that direction.
            // convert right-handed coord-sys to Unity left-handed coord-sys
            var normalized = CameraToUnity(pos, cameraToWorldMatrix);
            Vector3 cameraPos = cameraToWorldMatrix.GetColumn(3);
            return UnprojectNormal(normalized, cameraPos);
        }

        private Vector3 UnprojectNormal(Vector3 normalized, Vector3 cameraPos)
        {
            var hit = new RaycastHit();
            return Physics.Raycast(cameraPos, normalized, out hit, Mathf.Infinity, SpatialAwarnessLayerMask)
                ? hit.point
                : cameraPos + normalized * 5.0f;
        }

        private static Vector3 CameraToUnity(Vector3 cameraPoint, Matrix4x4 cameraToWorldMatrix)
        {
            var rotation = Quaternion.LookRotation(-cameraToWorldMatrix.GetColumn(2), cameraToWorldMatrix.GetColumn(1));
            return Vector3.Normalize(rotation * cameraPoint);
        }

        private static Vector3 UnityToCamera(Vector3 worldPoint, Matrix4x4 projectionMatrix)
        {
            var rotation = Quaternion.LookRotation(projectionMatrix.GetColumn(2), -projectionMatrix.GetColumn(1));
            return rotation * worldPoint;
        }

        private Point Project(Vector3 worldPoint, Matrix4x4 projectionMatrix, CameraIntrinsics camIntrinsics)
        {
            var pt = UnityToCamera(worldPoint, projectionMatrix);
            var cam = UnityToCamera(projectionMatrix.GetColumn(3), projectionMatrix);

            // Project world point onto
            var origin = camIntrinsics.ProjectOntoFrame(new System.Numerics.Vector3(pt.x, pt.y, pt.z));
            var frame = camIntrinsics.ProjectOntoFrame(new System.Numerics.Vector3(cam.x, cam.y, cam.z));
            return new Point(origin.X, origin.Y);
        }

        private Point ProjectEyeGaze(Matrix4x4 projectionMatrix, CameraIntrinsics camIntrinsics)
        {
            Vector3 gaze;
            CoreServices.InputSystem.EyeGazeProvider.IsEyeTrackingEnabled = true;
            if (CoreServices.InputSystem.EyeGazeProvider.IsEyeTrackingEnabledAndValid)
            {
                var eyeGaze = CoreServices.InputSystem.EyeGazeProvider;
                gaze = eyeGaze.GazeDirection + eyeGaze.GazeOrigin;
            }
            else
            {
                var headGaze = CoreServices.InputSystem.GazeProvider;
                gaze = headGaze.GazeDirection + headGaze.GazeOrigin;
            }

            return Project(gaze, projectionMatrix, camIntrinsics);
        }

        public void OnFrameMatAcquired(Mat bgraMat, Matrix4x4 projectionMatrix, Matrix4x4 cameraToWorldMatrix,
            CameraIntrinsics camIntrinsics)
        {
            Enqueue(() =>
            {
                // Implement the HoloLens process here. -->

                /*
                var hitPoint = new Vector3(0, 0, 0);
                var laserPointerPosition = FindLaserPointer(bgraMat);
                if (laserPointerPosition != null)
                    hitPoint = Raycast(laserPointerPosition, cameraToWorldMatrix, camIntrinsics);
                */

                //HololensSquares(bgraMat, projectionMatrix, cameraToWorldMatrix, camIntrinsics);
                //StartCoroutine(SendPlainMat(bgraMat, 0));

                // Implement the HoloLens process here. <--

                /*
                var fps = showFPS ? getFPS() : 0;

                if (laserPointerPosition != null && (redSphere.transform.position - hitPoint).magnitude >= 0.01)
                {
                    redSphere.transform.position = hitPoint;
                    Vector3 cameraPos = cameraToWorldMatrix.GetColumn(3);
                    toolTipText.text = (hitPoint - cameraPos).magnitude.ToString("0.00") + " m" +
                                       (showFPS ? fps.ToString(", 0.0 FPS") : "");
                }
                */
                //Hololens2DPoint(bgraMat, projectionMatrix, camIntrinsics);
                StartCoroutine(SquaresSend(bgraMat));
                CameraStreamDisplay(bgraMat, cameraToWorldMatrix);
                //bgraMat.Dispose();
            });
        }

        private void HololensFeaturePoints(Mat bgraMat, Matrix4x4 cameraToWorldMatrix, CameraIntrinsics camIntrinsics)
        {
            // Get the feature points from the image
            var points = FindFeatures(bgraMat).toList().ConvertAll(point => point.pt);
            if (points.Count == 0) return;

            // Transform the points from image coordinates to world coordinates and draw them
            var transformations = points.ConvertAll(point => Unproject(point, cameraToWorldMatrix, camIntrinsics));
            FastAllocatePointPrefabs(transformations.Count);
            for (var i = 0; i < transformations.Count; i++)
            {
                TransformPointPrefab(transformations[i], _points[i]);
            }
        }

        private void HololensSquares(Mat bgraMat, Matrix4x4 projection, Matrix4x4 cameraToWorld,
            CameraIntrinsics camIntrinsics)
        {
            var center = new Point(bgraMat.width() / 2d, bgraMat.height() / 2d);
            var squares = _squares.FindSquares(bgraMat);
            var possible = _squares.SelectOptimalSquare(squares, bgraMat, center);
            if (possible.Count == 0) return;
            var transformations = new List<Vector3>();
            possible[0].toList().ForEach(point => transformations.Add(Unproject(point, cameraToWorld, camIntrinsics)));
            FastAllocatePointPrefabs(transformations.Count);
            for (var i = 0; i < transformations.Count; i++)
            {
                TransformPointPrefab(transformations[i], _points[i]);
            }
            var distance = Vector3.Distance(_mainCamera.transform.position, transformations[0]);
            toSave = _squares.MeasurePoints(bgraMat, possible[0]);
            //CheckContours(distance);
        }

        private void Hololens2DPoint(Mat bgraMat, Matrix4x4 projection, CameraIntrinsics camIntrinsics)
        {
            var gaze = ProjectEyeGaze(projection, camIntrinsics);
            Imgproc.circle(bgraMat, gaze, 5, new Scalar(0, 255, 0));
        }

        private void Update()
        {
            lock (ExecuteOnMainThread)
            {
                while (ExecuteOnMainThread.Count > 0) ExecuteOnMainThread.Dequeue().Invoke();
            }
        }

        private void Enqueue(Action action)
        {
            lock (ExecuteOnMainThread)
            {
                ExecuteOnMainThread.Enqueue(action);
            }
        }

#else
        // Update is called once per frame
        void Update()
        {
            // Convert the image from RGBA to BGRA as all methods use BGRA
            if (!cameraStream.IsPlaying() || !cameraStream.DidUpdateThisFrame()) return;
            var rgbaMat = cameraStream.GetMat();
            var bgramat = new Mat();
            Imgproc.cvtColor(rgbaMat, bgramat, Imgproc.COLOR_RGBA2BGRA);
            StartCoroutine(SquaresSend(bgramat));
            CameraStreamDisplay(bgramat, _webcamMatrix);
            /*
            Webcam3DSquares(bgraMat);
            
            */
        }
#endif

        IEnumerator SendPlainMat(Mat bgramat, int level)
        {
        
            if (!sendRequest) yield break;
            var matByteStream = new MatOfByte();
            Debug.Log(bgramat.size());
            Imgcodecs.imencode(".jpg", bgramat, matByteStream);
            var data = matByteStream.toArray();
            var start = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();
            using (var www = UnityWebRequest.Put("http://192.168.0.13:8080/frame", data))
            {
                yield return www.SendWebRequest();
                var end = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();
                var response = www.downloadHandler.text;
                if (response.Equals("0")) yield break;
                response = response + "," + start + "," + end + "," + level;
                using (var store =
                       UnityWebRequest.Put("http://192.168.0.13:8080/store", Encoding.UTF8.GetBytes(response)))
                {
                    yield return store.SendWebRequest();
                }
            }
        }
        
        IEnumerator SquaresSend(Mat bgramat)
        {
        
            if (!sendRequest) yield break;
            var matByteStream = new MatOfByte();
            Debug.Log(bgramat.size());
            Imgcodecs.imencode(".jpg", bgramat, matByteStream);
            var data = matByteStream.toArray();
            using (var www = UnityWebRequest.Put("http://192.168.1.35:8080/square", data))
            //using (var www = UnityWebRequest.Put("http://localhost:8080/square", data))
            {
                yield return www.SendWebRequest();
                Debug.Log(www.downloadHandler.text);
            }
        }

        IEnumerator SaveData()
        {
            if (toSave == null) yield return null;
            var timestamp = DateTime.Now.ToString("yyyy-MM-dd-HH-mm-ss");
            var filepath = Application.persistentDataPath + "/" + timestamp + ".txt";
            using (var sw = new StreamWriter(filepath))
            {
                System.Diagnostics.Debug.Assert(toSave != null, nameof(toSave) + " != null");
                sw.WriteLine("Contours: " + toSave[0]);
                sw.WriteLine("Features: " + toSave[1]);
            }

            yield return null;
        }

        IEnumerator CreateDataSection()
        {
            var timestamp = DateTime.Now.ToString("yyyy-MM-dd-HH-mm-ss");
            var filepath = Application.persistentDataPath + "/NEWSECTION-AT-" + timestamp + ".txt";
            using (var sw = new StreamWriter(filepath))
            {
                sw.WriteLine("New Data Section");
            }
            yield return null;
        }

        public void VuforiaOnFound()
        {
            sendRequest = false;
        }

        IEnumerator SetLightBrightness()
        {
            using (var www = UnityWebRequest.Post(URI, new WWWForm()))
            {
                yield return www.SendWebRequest();
            }
        }

        private void CheckContours(float distance)
        {
            var count = toSave[0];
            if (count > 300 && distance < 1f || count > 40 && distance > 1f) StartCoroutine(SetLightBrightness());
        }

        /// <summary>
        /// Finds the features and renders them as green spheres offset from the camera by 1 meter, similar to how the
        /// HoloLens renders the feature points. <see cref="Webcam2DFeaturePoints"/> if desiring to display feature
        /// points directly on the camera stream.
        /// </summary>
        /// <param name="bgraMat">The image to process</param>
        private void Webcam3DFeaturePoints(Mat bgraMat)
        {
            // Get the feature points from the image
            var points = FindFeatures(bgraMat).toList().ConvertAll(point => point.pt);
            if (points.Count == 0) return;

            // Transform the points from image coordinates to world coordinates and draw them
            var transformations = WebcamCalculatePositions(points, bgraMat.width(), bgraMat.height());
            FastAllocatePointPrefabs(transformations.Count);
            for (var i = 0; i < transformations.Count; i++)
            {
                TransformPointPrefab(transformations[i], _points[i]);
            }
        }

        private void Webcam2DSquares(Mat bgraMat)
        {
            // Ensure that display of the camera stream is enabled, else show an error message
            if (!isVisibleImage)
            {
                Debug.LogError("The display of the camera stream is disabled. Enable it in the inspector.");
                return;
            }

            var squares = _squares.FindSquares(bgraMat);
            Imgproc.polylines(bgraMat, squares, true, new Scalar(0, 255, 0));
        }

        private void Webcam3DSquares(Mat bgraMat)
        {
            var squares = _squares.FindSquares(bgraMat);
            if (squares.Count == 0) return;
            var best = _squares.SelectOptimalSquare(squares, bgraMat, Extensions.InvalidPoint());
            var transformations = WebcamCalculatePositions(best[0].toList(), bgraMat.width(), bgraMat.height());
            markerPrefab.DefineNewArea(transformations);
        }


        /// <summary>
        /// Finds the features and renders them as green dots directly on the camera stream as a simple image.
        /// <see cref="Webcam3DFeaturePoints"/> if desiring to render the features as 3D spheres like the HoloLens.
        /// </summary>
        /// <param name="bgraMat">The image to process</param>
        private void Webcam2DFeaturePoints(Mat bgraMat)
        {
            // Ensure that display of the camera stream is enabled, else show an error message
            if (!isVisibleImage)
            {
                Debug.LogError("The display of the camera stream is disabled. Enable it in the inspector.");
                return;
            }

            // Draw the feature points on the camera stream
            var points = FindFeatures(bgraMat);
            Features2d.drawKeypoints(bgraMat, points, bgraMat, new Scalar(0, 255, 0, 255));
            points.Dispose();
        }

        /// <summary>
        /// Transform the point prefab to the correct position based on the vector transformations
        /// </summary>
        /// <param name="toPos">Position to transform to</param>
        /// <param name="prefab">Prefab to transform</param>
        private static void TransformPointPrefab(Vector3 toPos, GameObject prefab)
        {
            if ((prefab.transform.position - toPos).magnitude <= 0.01) return;
            prefab.transform.position = toPos;
        }

        /// <summary>
        /// Allocates the desired amount of point prefabs. If there are more prefabs than points, the excess prefabs are
        /// disabled and are not removed from memory. Fast than destroying objects, but not memory efficient.
        /// <see cref="EfficientAllocatePointPrefabs"/> for a more memory efficient method.
        /// </summary>
        /// <param name="target">Number of prefabs desired</param>
        private void FastAllocatePointPrefabs(int target)
        {
            if (_points.Count < target)
            {
                for (var i = _points.Count; i < target; i++)
                {
                    var point = Instantiate(pointPrefab);
                    _points.Add(point);
                }
            }

            var j = 0;
            _points.ForEach(prefab =>
            {
                prefab.gameObject.SetActive(j < target);
                j++;
            });
        }

        /// <summary>
        /// Allocates the desired amount of point prefabs. If there are more prefabs than points, the excess prefabs are
        /// destroyed and new ones need to be reallocated. Memory efficient, but slower.
        /// <see cref="FastAllocatePointPrefabs"/> for a faster method.
        /// </summary>
        /// <param name="target">Number of prefabs desired</param>
        private void EfficientAllocatePointPrefabs(int target)
        {
            if (_points.Count < target)
            {
                for (var i = _points.Count; i < target; i++)
                {
                    var point = Instantiate(pointPrefab);
                    _points.Add(point);
                }
            }
            else if (_points.Count > target)
            {
                for (var i = target; i < _points.Count; i++)
                {
                    Destroy(_points[i].gameObject);
                    _points.RemoveAt(i);
                }
            }
        }

        /// <summary>
        /// Displays the laser pointer position in the image
        /// </summary>
        /// <param name="bgraMat">The image to process</param>
        private void WebcamLaserPoint(Mat bgraMat)
        {
            // Find the laser pointer and return if null
            InitializeLaserPointer();
            var lpPos = FindLaserPointer(bgraMat);
            if (lpPos == null) return;

            // Transform the red sphere accordingly
            var toPos = WebcamCalculatePositions(new List<Point> {lpPos}, bgraMat.width(), bgraMat.height());
            if ((redSphere.transform.position - toPos[0]).magnitude <= 0.1) return;
            redSphere.transform.position = toPos[0];
            toolTipText.text = (toPos[0] - _mainCamera.transform.position).magnitude.ToString("0.00") + " m";
        }

        /// <summary>
        /// Converts OpenCV points to Unity transformation points to place objects at locations. Unprojects pixel
        /// coordinates into a camera space ray from the camera origin, expressed as a X, Y coordinates on a plane one
        /// meter from the camera.
        /// </summary>
        /// <param name="points">The OpenCV points to transform</param>
        /// <param name="width">The width of the bgraMat</param>
        /// <param name="height">The width of the bgraMat</param>
        /// <returns>List of new positions to transform to</returns>
        private List<Vector3> WebcamCalculatePositions(List<Point> points, int width, int height)
        {
            if (_mainCamera == null) return new List<Vector3>();
            return points.ConvertAll(point => _mainCamera.ScreenToWorldPoint(new Vector3(
                (float) (point.x / width * Screen.currentResolution.width),
                (float) (point.y / height * Screen.currentResolution.height),
                1.0f)));
        }

        /// <summary>
        /// Displays the camera stream on the screen offset form the user
        /// </summary>
        /// <param name="bgraMat">The image to display</param>
        /// <param name="cameraToWorldMatrix">The matrix to use for positioning of the stream</param>
        private void CameraStreamDisplay(Mat bgraMat, Matrix4x4 cameraToWorldMatrix)
        {
            // Check if the camera is playing and if show camera stream is enabled
            if (!cameraStream.IsPlaying() || !isVisibleImage) return;

            // Convert the image to a displayable texture
            Utils.fastMatToTexture2D(bgraMat, texture);

            // Set the matrix to the renderer
            var worldToCameraMatrix = cameraToWorldMatrix.inverse;
            quad_renderer.sharedMaterial.SetMatrix(WorldToCameraMatrix, worldToCameraMatrix);

            // Position the canvas object slightly in front of the real world web camera.
            Vector3 position = cameraToWorldMatrix.GetColumn(3) - cameraToWorldMatrix.GetColumn(2) * 2.2f;

            // Rotate the canvas object so that it faces the user.
            var rotation =
                Quaternion.LookRotation(-cameraToWorldMatrix.GetColumn(2), cameraToWorldMatrix.GetColumn(1));

            // Set the canvas object position and rotation
            var obj = gameObject;
            obj.transform.position = position;
            obj.transform.rotation = rotation;
        }


        /// <summary>
        /// Releases camera whenever the application it put into background (focus == false)
        /// and claims camera when the application moves to foreground (focus == true)
        /// </summary>
        /// <param name="focus">true if put into foreground, false if background</param>
        private void OnApplicationFocus(bool focus)
        {
            if (focus) // get access to the camera
                InitWebCamHelper();
            else // release camera
                OnDestroy();
        }

        /// <summary>
        /// Releases camera on destroy event (application closed).
        /// </summary>
        private void OnDestroy()
        {
#if ENABLE_WINMD_SUPPORT
            cameraStream.FrameMatAcquired -= OnFrameMatAcquired;
#endif
            cameraStream.Dispose();
        }
    }
}