// Modifications by Abist (2020):
// 1) retrieve CameraIntrinsics from VidoCaptureSample and pass it to FameMatAcquiredCallback

#pragma warning disable 0067
using HoloLensCameraStream;
using OpenCVForUnity.CoreModule;
using OpenCVForUnity.UnityUtils;
using OpenCVForUnity.UnityUtils.Helper;
using System;
using System.Collections;
using System.Linq;
using OpenCVForUnity.UtilsModule;
using UnityEngine;

#if ENABLE_WINMD_SUPPORT
using Windows.Media.Devices.Core;
#endif

namespace HoloLensCameraStream
{
#if ENABLE_WINMD_SUPPORT
    /// <summary>
    /// This is called every time there is a new frame image mat available.
    /// The Mat object's type is 'CV_8UC4' (BGRA).
    /// </summary>
    /// <param name="projectionMatrix">projection matrices.</param>
    /// <param name="cameraToWorldMatrix">camera to world matrices.</param>
    /// <param name="camIntrinsics">camera intrinsics for 2D -> 3D unprojection etc.</param>
    public delegate void FrameMatAcquiredCallback(Mat mat, Matrix4x4 projectionMatrix, Matrix4x4 cameraToWorldMatrix,
        CameraIntrinsics camIntrinsics);
#endif

    /// <summary>
    /// Hololens camera stream to mat helper.
    /// v 1.0.4
    /// 
    /// Combination of camera frame size and frame rate that can be acquired on Hololens. (width x height : framerate)
    /// 1280 x 720 : 30
    /// 1280 x 720 : 24
    /// 1280 x 720 : 20
    /// 1280 x 720 : 15
    /// 1280 x 720 : 5
    /// 
    /// 896 x 504 : 29.97003
    /// 896 x 504 : 24
    /// 896 x 504 : 20
    /// 896 x 504 : 15
    /// 896 x 504 : 5
    /// 
    /// 1344 x 756 : 29.97003
    /// 1344 x 756 : 24
    /// 1344 x 756 : 20
    /// 1344 x 756 : 15
    /// 1344 x 756 : 5
    /// 
    /// 1408 x 792 : 29.97003
    /// 1408 x 792 : 24
    /// 1408 x 792 : 20
    /// 1408 x 792 : 15
    /// 1408 x 792 : 5
    /// </summary>
    public sealed class HololensCameraStreamToMatHelper : WebCamTextureToMatHelper
    {
#if ENABLE_WINMD_SUPPORT
        /// <summary>
        /// This will be called whenever a new camera frame image available is converted to Mat.
        /// The Mat object's type is 'CV_8UC4' (BGRA).
        /// You must properly initialize the HololensCameraStreamToMatHelper, 
        /// including calling Play() before this event will begin firing.
        /// </summary>
        public event FrameMatAcquiredCallback FrameMatAcquired;
#endif

#if ENABLE_WINMD_SUPPORT
        public override float requestedFPS
        {
            get => _requestedFPS;
            set
            {
                _requestedFPS = Mathf.Clamp(value, -1f, float.MaxValue);
                if (HasInitDone) Initialize();
            }
        }

        private readonly object _lockObject = new object();
        private readonly object _matrixLockObject = new object();
        private readonly object _latestImageBytesLockObject = new object();

        private VideoCapture _videoCapture;
        private CameraParameters _cameraParams;

        private Matrix4x4 _cameraToWorldMatrix = Matrix4x4.identity;

        private Matrix4x4 CameraToWorldMatrix
        {
            get
            {
                lock (_matrixLockObject)
                {
                    return _cameraToWorldMatrix;
                }
            }
            set
            {
                lock (_matrixLockObject)
                {
                    _cameraToWorldMatrix = value;
                }
            }
        }

        private Matrix4x4 _projectionMatrix = Matrix4x4.identity;

        private Matrix4x4 ProjectionMatrix
        {
            get
            {
                lock (_matrixLockObject)
                {
                    return _projectionMatrix;
                }
            }
            set
            {
                lock (_matrixLockObject)
                {
                    _projectionMatrix = value;
                }
            }
        }

        private bool _didUpdateFrame = false;

        private bool DidUpdateFrame
        {
            get
            {
                lock (_lockObject)
                {
                    return _didUpdateFrame;
                }
            }
            set
            {
                lock (_lockObject)
                {
                    _didUpdateFrame = value;
                }
            }
        }

        private bool _didUpdateImageBufferInCurrentFrame = false;

        private bool DidUpdateImageBufferInCurrentFrame
        {
            get
            {
                lock (_lockObject)
                {
                    return _didUpdateImageBufferInCurrentFrame;
                }
            }
            set
            {
                lock (_lockObject)
                {
                    _didUpdateImageBufferInCurrentFrame = value;
                }
            }
        }

        private byte[] _latestImageBytes;

        private byte[] LatestImageBytes
        {
            get
            {
                lock (_latestImageBytesLockObject)
                {
                    return _latestImageBytes;
                }
            }
            set
            {
                lock (_latestImageBytesLockObject)
                {
                    _latestImageBytes = value;
                }
            }
        }

        private IntPtr _spatialCoordinateSystemPtr;
        private bool _isChangeVideoModeWaiting;

        private bool _hasInitDone;

        private bool HasInitDone
        {
            get
            {
                lock (_lockObject)
                {
                    return _hasInitDone;
                }
            }
            set
            {
                lock (_lockObject)
                {
                    _hasInitDone = value;
                }
            }
        }

        private bool _hasInitEventCompleted;

        private bool HasInitEventCompleted
        {
            get
            {
                lock (_lockObject)
                {
                    return _hasInitEventCompleted;
                }
            }
            set
            {
                lock (_lockObject)
                {
                    _hasInitEventCompleted = value;
                }
            }
        }

        private void LateUpdate()
        {
            if (DidUpdateFrame && !DidUpdateImageBufferInCurrentFrame)
                DidUpdateFrame = false;

            DidUpdateImageBufferInCurrentFrame = false;
        }

        private void OnFrameSampleAcquired(VideoCaptureSample sample)
        {
            lock (_latestImageBytesLockObject)
            {
                //When copying the bytes out of the buffer, you must supply a byte[] that is appropriately sized.
                //You can reuse this byte[] until you need to resize it (for whatever reason).
                if (_latestImageBytes == null || _latestImageBytes.Length < sample.dataLength)
                    _latestImageBytes = new byte[sample.dataLength];
                sample.CopyRawImageDataIntoBuffer(_latestImageBytes);
            }

            if (sample.TryGetCameraToWorldMatrix(out var cameraToWorldMatrixAsFloat) == false)
            {
                sample.Dispose();
                return;
            }

            if (sample.TryGetProjectionMatrix(out var projectionMatrixAsFloat) == false)
            {
                sample.Dispose();
                return;
            }

            var camIntrinsics = sample.GetCameraIntrinsics();
            // Right now we pass things across the pipe as a float array then convert them back into UnityEngine.Matrix using a utility method
            ProjectionMatrix = LocatableCameraUtils.ConvertFloatArrayToMatrix4x4(projectionMatrixAsFloat);
            CameraToWorldMatrix = LocatableCameraUtils.ConvertFloatArrayToMatrix4x4(cameraToWorldMatrixAsFloat);

            sample.Dispose();

            DidUpdateFrame = true;
            DidUpdateImageBufferInCurrentFrame = true;

            if (HasInitEventCompleted && FrameMatAcquired != null)
            {
                var mat = new Mat(_cameraParams.cameraResolutionHeight, _cameraParams.cameraResolutionWidth,
                    CvType.CV_8UC4);
Utils.copyToMat<byte> (LatestImageBytes, frameMat);

                if (_rotate90Degree)
                {
                    var rotatedFrameMat = new Mat(_cameraParams.cameraResolutionWidth,
                        _cameraParams.cameraResolutionHeight, CvType.CV_8UC4);
                    Core.rotate(mat, rotatedFrameMat, Core.ROTATE_90_CLOCKWISE);
                    mat.Dispose();

                    FlipMat(rotatedFrameMat, _flipVertical, _flipHorizontal);
                    FrameMatAcquired?.Invoke(base.rotatedFrameMat, ProjectionMatrix, CameraToWorldMatrix,
                        camIntrinsics);
                }
                else
                {
                    FlipMat(mat, _flipVertical, _flipHorizontal);
                    FrameMatAcquired?.Invoke(mat, ProjectionMatrix, CameraToWorldMatrix, camIntrinsics);
                }
            }
        }

        private CameraParameters CreateCameraParams(VideoCapture videoCapture)
        {
            var min1 = videoCapture.GetSupportedResolutions()
                .Min(r => Mathf.Abs(r.width * r.height - _requestedWidth * _requestedHeight));
            var resolution = videoCapture.GetSupportedResolutions().First(r =>
                Mathf.Abs(r.width * r.height - _requestedWidth * _requestedHeight) == min1);

            var min2 = videoCapture.GetSupportedFrameRatesForResolution(resolution)
                .Min(f => Mathf.Abs(f - _requestedFPS));
            var frameRate = videoCapture.GetSupportedFrameRatesForResolution(resolution)
                .First(f => Math.Abs(Mathf.Abs(f - _requestedFPS) - min2) < 0.05);

            var cameraParams = new CameraParameters
            {
                cameraResolutionHeight = resolution.height,
                cameraResolutionWidth = resolution.width,
                frameRate = Mathf.RoundToInt(frameRate),
                pixelFormat = CapturePixelFormat.BGRA32,
                EnableHolograms = false
            };

            return cameraParams;
        }
#endif

        public void LoadRawTextureData(Texture2D texture)
        {
#if ENABLE_WINMD_SUPPORT
            texture.LoadRawTextureData(LatestImageBytes);
            texture.Apply(false, false);
#else
            return;
#endif
        }

        /// <summary>
        /// Returns the video capture.
        /// </summary>
        /// <returns>The video capture.</returns>
        public VideoCapture GetVideoCapture()
        {
#if ENABLE_WINMD_SUPPORT
            return _videoCapture;
#else
            return null;
#endif
        }

#if ENABLE_WINMD_SUPPORT
        // Update is called once per frame
        protected override void Update()
        {
        }

        /// <summary>
        /// Raises the destroy event.
        /// </summary>
        protected override void OnDestroy()
        {
            Dispose();
            if (_videoCapture == null) return;
            if (_videoCapture.IsStreaming)
            {
                _videoCapture.StopVideoModeAsync(result => { _disposeVideoCapture(); });
            }
            else
            {
                _disposeVideoCapture();
            }
        }

        /// <summary>
        /// Disposes _videoCapture and sets it to null
        /// </summary>
        private void _disposeVideoCapture()
        {
            _videoCapture.Dispose();
            _videoCapture = null;
        }

        /// <summary>
        /// Initializes this instance by coroutine.
        /// </summary>
        protected override IEnumerator _Initialize()
        {
            if (HasInitDone)
            {
                ReleaseResources();

                if (onDisposed != null)
                    onDisposed.Invoke();
            }

            isInitWaiting = true;

            while (_isChangeVideoModeWaiting) yield return null;

            _isChangeVideoModeWaiting = true;
            if (_videoCapture != null)
            {
                _videoCapture.StopVideoModeAsync(result1 =>
                {
                    _cameraParams = CreateCameraParams(_videoCapture);
                    _videoCapture.StartVideoModeAsync(_cameraParams, result2 =>
                    {
                        if (!result2.success)
                        {
                            _isChangeVideoModeWaiting = false;
                            isInitWaiting = false;
                            CancelInitCoroutine();
                            onErrorOccurred?.Invoke(ErrorCode.UNKNOWN);
                        }
                        else
                        {
                            _isChangeVideoModeWaiting = false;
                        }
                    });
                });
            }
            else
            {
                //Fetch a pointer to Unity's spatial coordinate system if you need pixel mapping
#if UNITY_2017_2_OR_NEWER
                _spatialCoordinateSystemPtr = UnityEngine.XR.WSA.WorldManager.GetNativeISpatialCoordinateSystemPtr();
#else
                spatialCoordinateSystemPtr = UnityEngine.VR.WSA.WorldManager.GetNativeISpatialCoordinateSystemPtr ();
#endif
                VideoCapture.CreateAync(videoCapture =>
                {
                    if (initCoroutine == null) return;
                    if (videoCapture == null)
                    {
                        Debug.LogError("Did not find a video capture object. You may not be using the HoloLens.");

                        _isChangeVideoModeWaiting = false;
                        isInitWaiting = false;
                        CancelInitCoroutine();

                        onErrorOccurred?.Invoke(ErrorCode.CAMERA_DEVICE_NOT_EXIST);

                        return;
                    }

                    _videoCapture = videoCapture;

                    //Request the spatial coordinate ptr if you want fetch the camera and set it if you need to 
                    videoCapture.WorldOriginPtr = _spatialCoordinateSystemPtr;

                    _cameraParams = CreateCameraParams(videoCapture);

                    videoCapture.FrameSampleAcquired -= OnFrameSampleAcquired;
                    videoCapture.FrameSampleAcquired += OnFrameSampleAcquired;
                    videoCapture.StartVideoModeAsync(_cameraParams, result =>
                    {
                        if (!result.success)
                        {
                            _isChangeVideoModeWaiting = false;
                            isInitWaiting = false;
                            CancelInitCoroutine();

                            onErrorOccurred?.Invoke(ErrorCode.UNKNOWN);
                        }
                        else
                        {
                            _isChangeVideoModeWaiting = false;
                        }
                    });
                });
            }

            var initFrameCount = 0;
            var isTimeout = false;

            while (true)
                if (initFrameCount > _timeoutFrameCount)
                {
                    isTimeout = true;
                    break;
                }
                else if (DidUpdateFrame)
                {
                    Debug.Log(
                        $"HololensCameraStreamToMatHelper:: name: width:{_cameraParams.cameraResolutionWidth} height:{_cameraParams.cameraResolutionHeight} fps:{_cameraParams.frameRate}");

                    if (colors == null || colors.Length !=
                        _cameraParams.cameraResolutionWidth * _cameraParams.cameraResolutionHeight)
                    {
                        colors = new Color32[_cameraParams.cameraResolutionWidth *
                                             _cameraParams.cameraResolutionHeight];
                    }
                        

                    frameMat = new Mat(_cameraParams.cameraResolutionHeight, _cameraParams.cameraResolutionWidth,
                        CvType.CV_8UC4, new Scalar(0, 0, 0, 255));
                    screenOrientation = Screen.orientation;
                    

                    if (_rotate90Degree)
                        rotatedFrameMat = new Mat(_cameraParams.cameraResolutionWidth,
                            _cameraParams.cameraResolutionHeight, CvType.CV_8UC4, new Scalar(0, 0, 0, 255));

                    isInitWaiting = false;
                    HasInitDone = true;
                    initCoroutine = null;

                    if (onInitialized != null)
                        onInitialized.Invoke();

                    HasInitEventCompleted = true;

                    break;
                }
                else
                {
                    initFrameCount++;
                    yield return null;
                }

            if (isTimeout)
            {
                if (_videoCapture != null)
                {
                    _videoCapture.FrameSampleAcquired -= OnFrameSampleAcquired;

                    _isChangeVideoModeWaiting = true;
                    _videoCapture.StopVideoModeAsync(result =>
                    {
                        _disposeVideoCapture();
                        _isChangeVideoModeWaiting = false;
                    });

                    isInitWaiting = false;
                    initCoroutine = null;

                    if (onErrorOccurred != null)
                        onErrorOccurred.Invoke(ErrorCode.TIMEOUT);
                }
                else
                {
                    isInitWaiting = false;
                    initCoroutine = null;

                    if (onErrorOccurred != null)
                        onErrorOccurred.Invoke(ErrorCode.TIMEOUT);
                }
            }
        }

        /// <summary>
        /// Indicates whether this instance has been initialized.
        /// </summary>
        /// <returns><c>true</c>, if this instance has been initialized, <c>false</c> otherwise.</returns>
        public override bool IsInitialized()
        {
            return HasInitDone;
        }

        /// <summary>
        /// Starts the camera.
        /// </summary>
        public override void Play()
        {
            if (HasInitDone) StartCoroutine(_Play());
        }

        private IEnumerator _Play()
        {
            while (_isChangeVideoModeWaiting) yield return null;

            if (!HasInitDone || _videoCapture.IsStreaming) yield break;

            _isChangeVideoModeWaiting = true;
            _videoCapture.StartVideoModeAsync(_cameraParams, result => { _isChangeVideoModeWaiting = false; });
        }

        /// <summary>
        /// Pauses the active camera.
        /// </summary>
        public override void Pause()
        {
            if (HasInitDone)
                StartCoroutine(_Stop());
        }

        /// <summary>
        /// Stops the active camera.
        /// </summary>
        public override void Stop()
        {
            if (HasInitDone)
                StartCoroutine(_Stop());
        }

        private IEnumerator _Stop()
        {
            while (_isChangeVideoModeWaiting) yield return null;

            if (!HasInitDone || !_videoCapture.IsStreaming) yield break;

            _isChangeVideoModeWaiting = true;
            _videoCapture.StopVideoModeAsync(result => { _isChangeVideoModeWaiting = false; });
        }

        /// <summary>
        /// Indicates whether the active camera is currently playing.
        /// </summary>
        /// <returns><c>true</c>, if the active camera is playing, <c>false</c> otherwise.</returns>
        public override bool IsPlaying()
        {
            if (!HasInitDone)
                return false;

            return _videoCapture.IsStreaming;
        }

        /// <summary>
        /// Indicates whether the active camera device is currently front facng.
        /// </summary>
        /// <returns><c>true</c>, if the active camera device is front facng, <c>false</c> otherwise.</returns>
        public override bool IsFrontFacing()
        {
            return false;
        }

        /// <summary>
        /// Returns the active camera device name.
        /// </summary>
        /// <returns>The active camera device name.</returns>
        public override string GetDeviceName()
        {
            return "";
        }

        /// <summary>
        /// Returns the active camera framerate.
        /// </summary>
        /// <returns>The active camera framerate.</returns>
        public override float GetFPS()
        {
            return HasInitDone ? _cameraParams.frameRate : -1f;
        }

        /// <summary>
        /// Returns the webcam texture.
        /// </summary>
        /// <returns>The webcam texture.</returns>
        public override WebCamTexture GetWebCamTexture()
        {
            return null;
        }

        /// <summary>
        /// Returns the camera to world matrix.
        /// </summary>
        /// <returns>The camera to world matrix.</returns>
        public override Matrix4x4 GetCameraToWorldMatrix()
        {
            return CameraToWorldMatrix;
        }

        /// <summary>
        /// Returns the projection matrix matrix.
        /// </summary>
        /// <returns>The projection matrix.</returns>
        public override Matrix4x4 GetProjectionMatrix()
        {
            return ProjectionMatrix;
        }

        /// <summary>
        /// Indicates whether the video buffer of the frame has been updated.
        /// </summary>
        /// <returns><c>true</c>, if the video buffer has been updated <c>false</c> otherwise.</returns>
        public override bool DidUpdateThisFrame()
        {
            if (!HasInitDone)
                return false;

            return DidUpdateFrame;
        }

        /// <summary>
        /// Gets the mat of the current frame.
        /// The Mat object's type is 'CV_8UC4' (BGRA).
        /// </summary>
        /// <returns>The mat of the current frame.</returns>
        public override Mat GetMat()
        {
            if (!HasInitDone || !_videoCapture.IsStreaming || LatestImageBytes == null)
            {
                if (rotatedFrameMat != null)
                    return rotatedFrameMat;
                else
                    return frameMat;
            }

            Utils.copyToMat<byte>(LatestImageBytes, frameMat);

            if (rotatedFrameMat != null)
            {
                Core.rotate(frameMat, rotatedFrameMat, Core.ROTATE_90_CLOCKWISE);

                FlipMat(rotatedFrameMat, _flipVertical, _flipHorizontal);

                return rotatedFrameMat;
            }
            else
            {
                FlipMat(frameMat, _flipVertical, _flipHorizontal);

                return frameMat;
            }
        }

        /// <summary>
        /// Flips the mat.
        /// </summary>
        /// <param name="mat">Mat.</param>
        protected override void FlipMat(Mat mat, bool flipVertical, bool flipHorizontal)
        {
            var flipCode = int.MinValue;

            if (_flipVertical)
            {
                if (flipCode == int.MinValue)
                    flipCode = 0;
                else if (flipCode == 0)
                    flipCode = int.MinValue;
                else if (flipCode == 1)
                    flipCode = -1;
                else if (flipCode == -1) flipCode = 1;
            }

            if (_flipHorizontal)
            {
                if (flipCode == int.MinValue)
                    flipCode = 1;
                else if (flipCode == 0)
                    flipCode = -1;
                else if (flipCode == 1)
                    flipCode = int.MinValue;
                else if (flipCode == -1) flipCode = 0;
            }

            if (flipCode > int.MinValue) Core.flip(mat, mat, flipCode);
        }

        /// <summary>
        /// To release the resources.
        /// </summary>
        protected override void ReleaseResources()
        {
            isInitWaiting = false;
            HasInitDone = false;
            HasInitEventCompleted = false;

            LatestImageBytes = null;
            DidUpdateFrame = false;
            DidUpdateImageBufferInCurrentFrame = false;

            if (frameMat != null)
            {
                frameMat.Dispose();
                frameMat = null;
            }

            if (rotatedFrameMat != null)
            {
                rotatedFrameMat.Dispose();
                rotatedFrameMat = null;
            }
        }

        /// <summary>
        /// Releases all resource used by the <see cref="WebCamTextureToMatHelper"/> object.
        /// </summary>
        /// <remarks>Call <see cref="Dispose"/> when you are finished using the <see cref="WebCamTextureToMatHelper"/>. The
        /// <see cref="Dispose"/> method leaves the <see cref="WebCamTextureToMatHelper"/> in an unusable state. After
        /// calling <see cref="Dispose"/>, you must release all references to the <see cref="WebCamTextureToMatHelper"/> so
        /// the garbage collector can reclaim the memory that the <see cref="WebCamTextureToMatHelper"/> was occupying.</remarks>
        public override void Dispose()
        {
            if (colors != null)
                colors = null;

            if (isInitWaiting)
            {
                CancelInitCoroutine();
                FrameMatAcquired = null;
                if (_videoCapture != null)
                {
                    _videoCapture.FrameSampleAcquired -= OnFrameSampleAcquired;
                    StartCoroutine(_Dispose());
                }

                ReleaseResources();
            }
            else if (HasInitDone)
            {
                FrameMatAcquired = null;
                _videoCapture.FrameSampleAcquired -= OnFrameSampleAcquired;
                StartCoroutine(_Dispose());
                ReleaseResources();
                if (onDisposed != null)
                    onDisposed.Invoke();
            }
        }

        private IEnumerator _Dispose()
        {
            while (_isChangeVideoModeWaiting) yield return null;

            _isChangeVideoModeWaiting = true;
            _videoCapture.StopVideoModeAsync(result =>
            {
                _disposeVideoCapture();
                _isChangeVideoModeWaiting = false;
            });
        }
#endif
    }
}