/////////////////////////////////////////////////////////////////////////////////////
/// Modified version of OVRCameraRig.
/// Modifications Copyright (c) PsyPhy Consulting 2020 to the 
/// extent allowed by the Oculus Utilities SDK License Version 1.31.
////////////////////////////////////////////////////////////////////////////////////

/************************************************************************************
Copyright : Copyright (c) Facebook Technologies, LLC and its affiliates. All rights reserved.

Licensed under the Oculus Utilities SDK License Version 1.31 (the "License"); you may not use
the Utilities SDK except in compliance with the License, which is provided at the time of installation
or download, or which otherwise accompanies this software in either electronic or hard copy form.

You may obtain a copy of the License at
https://developer.oculus.com/licenses/utilities-1.31

Unless required by applicable law or agreed to in writing, the Utilities SDK distributed
under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF
ANY KIND, either express or implied. See the License for the specific language governing
permissions and limitations under the License.
************************************************************************************/

#if USING_XR_MANAGEMENT && USING_XR_SDK_OCULUS
#define USING_XR_SDK
#endif

using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

#if UNITY_2017_2_OR_NEWER
using InputTracking = UnityEngine.XR.InputTracking;
using Node = UnityEngine.XR.XRNode;
#else
using InputTracking = UnityEngine.VR.InputTracking;
using Node = UnityEngine.VR.VRNode;
#endif

/// <summary>
/// A head-tracked stereoscopic virtual reality camera rig.
/// </summary>
[ExecuteInEditMode]
public class PsyPhy0gCameraRig : MonoBehaviour
{
	/// <summary>
	/// The left eye camera.
	/// </summary>
	public Camera leftEyeCamera { get { return (usePerEyeCameras) ? _leftEyeCamera : _centerEyeCamera; } }
	/// <summary>
	/// The right eye camera.
	/// </summary>
	public Camera rightEyeCamera { get { return (usePerEyeCameras) ? _rightEyeCamera : _centerEyeCamera; } }
	/// <summary>
	/// Provides a root transform for all anchors in tracking space.
	/// </summary>
	public Transform trackingSpace { get; private set; }
	/// <summary>
	/// Always coincides with the pose of the left eye.
	/// </summary>
	public Transform leftEyeAnchor { get; private set; }
	/// <summary>
	/// Always coincides with average of the left and right eye poses.
	/// </summary>
	public Transform centerEyeAnchor { get; private set; }
	/// <summary>
	/// Always coincides with the pose of the right eye.
	/// </summary>
	public Transform rightEyeAnchor { get; private set; }
	/// <summary>
	/// Always coincides with the pose of the left hand.
	/// </summary>
	public Transform leftHandAnchor { get; private set; }
	/// <summary>
	/// Always coincides with the pose of the right hand.
	/// </summary>
	public Transform rightHandAnchor { get; private set; }
	/// <summary>
	/// Anchors controller pose to fix offset issues for the left hand.
	/// </summary>
	public Transform leftControllerAnchor { get; private set; }
	/// <summary>
	/// Anchors controller pose to fix offset issues for the right hand.
	/// </summary>
	public Transform rightControllerAnchor { get; private set; }
	/// <summary>
	/// Always coincides with the pose of the sensor.
	/// </summary>
	public Transform trackerAnchor { get; private set; }
	/// <summary>
	/// Occurs when the eye pose anchors have been set.
	/// </summary>
	public event System.Action<PsyPhy0gCameraRig> UpdatedAnchors;
	/// <summary>
	/// If true, separate cameras will be used for the left and right eyes.
	/// </summary>
	public bool usePerEyeCameras = false;
	/// <summary>
	/// If true, all tracked anchors are updated in FixedUpdate instead of Update to favor physics fidelity.
	/// \note: If the fixed update rate doesn't match the rendering framerate (OVRManager.display.appFramerate), the anchors will visibly judder.
	/// </summary>
	public bool useFixedUpdateForTracking = false;
	/// <summary>
	/// If true, the cameras on the eyeAnchors will be disabled.
	/// \note: The main camera of the game will be used to provide VR rendering. And the tracking space anchors will still be updated to provide reference poses.
	/// </summary>
	public bool disableEyeAnchorCameras = false;

    public enum mode0g { OculusNative, HeadFixed, HeadBob, OculusOneToOne, OculusAmplified, Inertial }
    public mode0g Mode = mode0g.OculusNative;
    public bool ovrInitialized = false;
    public bool hmdPresent = false;
    public bool monoscopic = true;

    public OVRPose tracker;
    public OVRPose inertial;
    public float spin = 0.0f;
    public Vector3 angularVelocity;
    public float deltaT;
    public float previous_instant;

	protected bool _skipUpdate = false;
	protected readonly string trackingSpaceName = "TrackingSpace";
	protected readonly string trackerAnchorName = "TrackerAnchor";
	protected readonly string leftEyeAnchorName = "LeftEyeAnchor";
	protected readonly string centerEyeAnchorName = "CenterEyeAnchor";
	protected readonly string rightEyeAnchorName = "RightEyeAnchor";
	protected readonly string leftHandAnchorName = "LeftHandAnchor";
	protected readonly string rightHandAnchorName = "RightHandAnchor";
	protected readonly string leftControllerAnchorName = "LeftControllerAnchor";
	protected readonly string rightControllerAnchorName = "RightControllerAnchor";
	protected Camera _centerEyeCamera;
	protected Camera _leftEyeCamera;
	protected Camera _rightEyeCamera;

    protected OVRDisplay ovrDisplay;

    protected Vector3 mRate;
    protected Vector3 mVelocity;
    protected float sampleCount;


#region Unity Messages
	protected virtual void Awake()
	{
		_skipUpdate = true;
		EnsureGameObjectIntegrity();
        inertial = OVRPose.identity;
    }

	protected virtual void Start()
	{
		UpdateAnchors(true, true);
		Application.onBeforeRender += OnBeforeRenderCallback;
        spin = 0.0f;
        ovrDisplay = new OVRDisplay();
        previous_instant = Time.time;

        sampleCount = 0.0f;
        mRate = Vector3.zero;
        mVelocity = Vector3.zero;

    }

	protected virtual void FixedUpdate()
	{
		if (useFixedUpdateForTracking)
			UpdateAnchors(true, true);
	}

	protected virtual void Update()
	{
		_skipUpdate = false;

		if (!useFixedUpdateForTracking)
			UpdateAnchors(true, true);
	}

	protected virtual void OnDestroy()
	{
		Application.onBeforeRender -= OnBeforeRenderCallback;
	}
#endregion


	protected virtual void UpdateAnchors(bool updateEyeAnchors, bool updateHandAnchors)
	{
		if (!OVRManager.OVRManagerinitialized) return;
        ovrInitialized = true;

		EnsureGameObjectIntegrity();

		if (!Application.isPlaying) return;

		if (_skipUpdate)
		{
			centerEyeAnchor.FromOVRPose(OVRPose.identity, true);
			leftEyeAnchor.FromOVRPose(OVRPose.identity, true);
			rightEyeAnchor.FromOVRPose(OVRPose.identity, true);

			return;
		}

		monoscopic = OVRManager.instance.monoscopic;
		hmdPresent = OVRNodeStateProperties.IsHmdPresent();

        if (OVRInput.GetDown(OVRInput.RawButton.A) || Input.GetKeyDown(KeyCode.I)) Mode = mode0g.Inertial;
        if (OVRInput.GetDown(OVRInput.RawButton.B) || Input.GetKeyDown(KeyCode.N)) Mode = mode0g.OculusNative;
        if (OVRInput.GetDown(OVRInput.RawButton.X) || Input.GetKeyDown(KeyCode.F)) Mode = mode0g.HeadFixed;
        if (OVRInput.GetDown(OVRInput.RawButton.Y) || Input.GetKeyDown(KeyCode.Alpha1)) Mode = mode0g.OculusOneToOne;
        if (OVRInput.GetDown(OVRInput.RawButton.LIndexTrigger) || OVRInput.GetDown(OVRInput.RawButton.RIndexTrigger) || Input.GetKeyDown(KeyCode.Space))
        {
            spin = 0.0f;
            inertial = OVRPose.identity;
        }

        OVRPose tracker = OVRManager.tracker.GetPose();
        trackerAnchor.localRotation = tracker.orientation;
        Quaternion emulatedRotation = Quaternion.Euler(-OVRManager.instance.headPoseRelativeOffsetRotation.x, -OVRManager.instance.headPoseRelativeOffsetRotation.y, OVRManager.instance.headPoseRelativeOffsetRotation.z);

        // Integrate the velocity 
        tracker = OVRManager.tracker.GetPose();
        trackerAnchor.localRotation = tracker.orientation;

        angularVelocity = ovrDisplay.angularVelocity;
        float current_time = Time.time;
        deltaT = current_time - previous_instant;

        Quaternion qVelocity;
        qVelocity.x = -angularVelocity.x;
        qVelocity.y = -angularVelocity.y;
        qVelocity.z = -angularVelocity.z;
        qVelocity.w = 0.0f;

        Quaternion ovrCenterEyeRotation = Quaternion.identity;
        OVRNodeStateProperties.GetNodeStatePropertyQuaternion(Node.CenterEye, NodeStatePropertyType.Orientation, OVRPlugin.Node.EyeCenter, OVRPlugin.Step.Render, out ovrCenterEyeRotation);
        Quaternion rate = Quaternion.Inverse(ovrCenterEyeRotation) * qVelocity * ovrCenterEyeRotation;

        // Some stuff for debugging
        //mVelocity.x += qVelocity.x;
        //mVelocity.y += qVelocity.y;
        //mVelocity.z += qVelocity.z;
        //mRate.x += rate.x;
        //mRate.y += rate.y;
        //mRate.z += rate.z;

        //sampleCount++;
        //if ( sampleCount >= 40.0f )
        //{
        //    mRate = mRate / sampleCount;
        //    mVelocity = mVelocity / sampleCount;
        //    Debug.Log( mVelocity.ToString() + " " + mRate.ToString() + " " + centerEyeRotation.ToString() ); 
        //    sampleCount = 0.0f;
        //    mRate = Vector3.zero;
        //    mVelocity = Vector3.zero;

        //}

        Quaternion dQ;
        dQ.x = rate.x * deltaT / 2.0f;
        dQ.y = rate.y * deltaT / 2.0f;
        dQ.z = rate.z * deltaT / 2.0f;
        dQ.w = Mathf.Sqrt(1.0f - dQ.x * dQ.x - dQ.y * dQ.y - dQ.z * dQ.z);

        Quaternion newOrientation = inertial.orientation * dQ;
        inertial.orientation = newOrientation;
        previous_instant = current_time;

        if (Mode != mode0g.OculusNative)
        {

            if (Mode == mode0g.OculusAmplified || Mode == mode0g.OculusOneToOne)
            {

                Vector3 centerEyePosition = Vector3.zero;
                Quaternion centerEyeRotation = Quaternion.identity;
                if (OVRNodeStateProperties.GetNodeStatePropertyVector3(Node.CenterEye, NodeStatePropertyType.Position, OVRPlugin.Node.EyeCenter, OVRPlugin.Step.Render, out centerEyePosition)) tracker.position = centerEyePosition;
                if (OVRNodeStateProperties.GetNodeStatePropertyQuaternion(Node.CenterEye, NodeStatePropertyType.Orientation, OVRPlugin.Node.EyeCenter, OVRPlugin.Step.Render, out centerEyeRotation)) tracker.orientation = centerEyeRotation;

                tracker.orientation = centerEyeRotation;
                if (Mode == mode0g.OculusAmplified) tracker.orientation = centerEyeRotation * centerEyeRotation;
                else tracker.orientation = centerEyeRotation;
            }
            else if (Mode == mode0g.HeadBob)
            {
                tracker.orientation = Quaternion.Euler(15.0f * Mathf.Sin(0.20f * spin), 30.0f * Mathf.Sin(0.1f * spin), 0.0f);
                spin += 0.1f;
            }
            else if (Mode == mode0g.Inertial)
            {
 
                tracker.orientation = inertial.orientation;
            }
            else tracker.orientation = Quaternion.Euler(0.0f, 0.0f, 0.0f);

            if (hmdPresent)
            {
                trackerAnchor.localRotation = tracker.orientation;
                trackerAnchor.localPosition = tracker.position;
            }
            else
            {
                trackerAnchor.localRotation = emulatedRotation;
                trackerAnchor.localPosition = tracker.position;
            }

            //Note: in the below code, when using UnityEngine's API, we only update anchor transforms if we have a new, fresh value this frame.
            //If we don't, it could mean that tracking is lost, etc. so the pose should not change in the virtual world.
            //This can be thought of as similar to calling InputTracking GetLocalPosition and Rotation, but only for doing so when the pose is valid.
            //If false is returned for any of these calls, then a new pose is not valid and thus should not be updated.
            if (updateEyeAnchors)
            {
                // PsyPhy - Need to compute the left and right eye poses.
                // For now it is monoscopic.
                centerEyeAnchor.localPosition = trackerAnchor.localPosition;
                centerEyeAnchor.localRotation = trackerAnchor.localRotation;
                leftEyeAnchor.localPosition = trackerAnchor.localPosition;
                leftEyeAnchor.localRotation = trackerAnchor.localRotation;
                rightEyeAnchor.localPosition = trackerAnchor.localPosition;
                rightEyeAnchor.localRotation = trackerAnchor.localRotation;
            }
        }
        else
        {

            //Note: in the below code, when using UnityEngine's API, we only update anchor transforms if we have a new, fresh value this frame.
            //If we don't, it could mean that tracking is lost, etc. so the pose should not change in the virtual world.
            //This can be thought of as similar to calling InputTracking GetLocalPosition and Rotation, but only for doing so when the pose is valid.
            //If false is returned for any of these calls, then a new pose is not valid and thus should not be updated.
            if (updateEyeAnchors)
            {
                if (hmdPresent)
                {
                    Vector3 centerEyePosition = Vector3.zero;
                    Quaternion centerEyeRotation = Quaternion.identity;

                    if (OVRNodeStateProperties.GetNodeStatePropertyVector3(Node.CenterEye, NodeStatePropertyType.Position, OVRPlugin.Node.EyeCenter, OVRPlugin.Step.Render, out centerEyePosition))
                        centerEyeAnchor.localPosition = centerEyePosition;
                    if (OVRNodeStateProperties.GetNodeStatePropertyQuaternion(Node.CenterEye, NodeStatePropertyType.Orientation, OVRPlugin.Node.EyeCenter, OVRPlugin.Step.Render, out centerEyeRotation))
                        centerEyeAnchor.localRotation = centerEyeRotation;
                }
                else
                {
                    centerEyeAnchor.localRotation = emulatedRotation;
                    centerEyeAnchor.localPosition = OVRManager.instance.headPoseRelativeOffsetTranslation;
                }

                if (!hmdPresent || monoscopic)
                {
                    leftEyeAnchor.localPosition = centerEyeAnchor.localPosition;
                    rightEyeAnchor.localPosition = centerEyeAnchor.localPosition;
                    leftEyeAnchor.localRotation = centerEyeAnchor.localRotation;
                    rightEyeAnchor.localRotation = centerEyeAnchor.localRotation;
                }
                else
                {
                    Vector3 leftEyePosition = Vector3.zero;
                    Vector3 rightEyePosition = Vector3.zero;
                    Quaternion leftEyeRotation = Quaternion.identity;
                    Quaternion rightEyeRotation = Quaternion.identity;

                    if (OVRNodeStateProperties.GetNodeStatePropertyVector3(Node.LeftEye, NodeStatePropertyType.Position, OVRPlugin.Node.EyeLeft, OVRPlugin.Step.Render, out leftEyePosition))
                        leftEyeAnchor.localPosition = leftEyePosition;
                    if (OVRNodeStateProperties.GetNodeStatePropertyVector3(Node.RightEye, NodeStatePropertyType.Position, OVRPlugin.Node.EyeRight, OVRPlugin.Step.Render, out rightEyePosition))
                        rightEyeAnchor.localPosition = rightEyePosition;
                    if (OVRNodeStateProperties.GetNodeStatePropertyQuaternion(Node.LeftEye, NodeStatePropertyType.Orientation, OVRPlugin.Node.EyeLeft, OVRPlugin.Step.Render, out leftEyeRotation))
                        leftEyeAnchor.localRotation = leftEyeRotation;
                    if (OVRNodeStateProperties.GetNodeStatePropertyQuaternion(Node.RightEye, NodeStatePropertyType.Orientation, OVRPlugin.Node.EyeRight, OVRPlugin.Step.Render, out rightEyeRotation))
                        rightEyeAnchor.localRotation = rightEyeRotation;
                }
            }

        }

        if (updateHandAnchors)
		{
			//Need this for controller offset because if we're on OpenVR, we want to set the local poses as specified by Unity, but if we're not, OVRInput local position is the right anchor
			if (OVRManager.loadedXRDevice == OVRManager.XRDevice.OpenVR)
			{
				Vector3 leftPos = Vector3.zero;
				Vector3 rightPos = Vector3.zero;
				Quaternion leftQuat = Quaternion.identity;
				Quaternion rightQuat = Quaternion.identity;

				if (OVRNodeStateProperties.GetNodeStatePropertyVector3(Node.LeftHand, NodeStatePropertyType.Position, OVRPlugin.Node.HandLeft, OVRPlugin.Step.Render, out leftPos)) leftHandAnchor.localPosition = leftPos;
				if (OVRNodeStateProperties.GetNodeStatePropertyVector3(Node.RightHand, NodeStatePropertyType.Position, OVRPlugin.Node.HandRight, OVRPlugin.Step.Render, out rightPos)) rightHandAnchor.localPosition = rightPos;
				if (OVRNodeStateProperties.GetNodeStatePropertyQuaternion(Node.LeftHand, NodeStatePropertyType.Orientation, OVRPlugin.Node.HandLeft, OVRPlugin.Step.Render, out leftQuat)) leftHandAnchor.localRotation = leftQuat;
				if (OVRNodeStateProperties.GetNodeStatePropertyQuaternion(Node.RightHand, NodeStatePropertyType.Orientation, OVRPlugin.Node.HandRight, OVRPlugin.Step.Render, out rightQuat)) rightHandAnchor.localRotation = rightQuat;

			}
			else
			{
				leftHandAnchor.localPosition = OVRInput.GetLocalControllerPosition(OVRInput.Controller.LTouch);
				rightHandAnchor.localPosition = OVRInput.GetLocalControllerPosition(OVRInput.Controller.RTouch);
				leftHandAnchor.localRotation = OVRInput.GetLocalControllerRotation(OVRInput.Controller.LTouch);
				rightHandAnchor.localRotation = OVRInput.GetLocalControllerRotation(OVRInput.Controller.RTouch);
			}

			trackerAnchor.localPosition = tracker.position;

			OVRPose leftOffsetPose = OVRPose.identity;
			OVRPose rightOffsetPose = OVRPose.identity;
			if (OVRManager.loadedXRDevice == OVRManager.XRDevice.OpenVR)
			{
				leftOffsetPose = OVRManager.GetOpenVRControllerOffset(Node.LeftHand);
				rightOffsetPose = OVRManager.GetOpenVRControllerOffset(Node.RightHand);

				//Sets poses of left and right nodes, local to the tracking space.
				OVRManager.SetOpenVRLocalPose(trackingSpace.InverseTransformPoint(leftControllerAnchor.position),
					trackingSpace.InverseTransformPoint(rightControllerAnchor.position),
					Quaternion.Inverse(trackingSpace.rotation) * leftControllerAnchor.rotation,
					Quaternion.Inverse(trackingSpace.rotation) * rightControllerAnchor.rotation);
			}
			rightControllerAnchor.localPosition = rightOffsetPose.position;
			rightControllerAnchor.localRotation = rightOffsetPose.orientation;
			leftControllerAnchor.localPosition = leftOffsetPose.position;
			leftControllerAnchor.localRotation = leftOffsetPose.orientation;
		}

		RaiseUpdatedAnchorsEvent();
	}

	protected virtual void OnBeforeRenderCallback()
	{
		if (OVRManager.loadedXRDevice == OVRManager.XRDevice.Oculus)			//Restrict late-update to only Oculus devices
		{
			bool controllersNeedUpdate = OVRManager.instance.LateControllerUpdate;
#if USING_XR_SDK
			//For the XR SDK, we need to late update head pose, not just the controllers, because the functionality
			//is no longer built-in to the Engine. Under legacy, late camera update is done by default. In the XR SDK, you must use
			//Tracked Pose Driver to get this by default, which we do not use. So, we have to manually late update camera poses.
			UpdateAnchors(true, controllersNeedUpdate);
#else
			if (controllersNeedUpdate)
				UpdateAnchors(false, true);
#endif
		}
	}

	protected virtual void RaiseUpdatedAnchorsEvent()
	{
		if (UpdatedAnchors != null)
		{
			UpdatedAnchors(this);
		}
	}

	public virtual void EnsureGameObjectIntegrity()
	{
		bool monoscopic = OVRManager.instance != null ? OVRManager.instance.monoscopic : false;

		if (trackingSpace == null)
			trackingSpace = ConfigureAnchor(null, trackingSpaceName);

		if (leftEyeAnchor == null)
			leftEyeAnchor = ConfigureAnchor(trackingSpace, leftEyeAnchorName);

		if (centerEyeAnchor == null)
			centerEyeAnchor = ConfigureAnchor(trackingSpace, centerEyeAnchorName);

		if (rightEyeAnchor == null)
			rightEyeAnchor = ConfigureAnchor(trackingSpace, rightEyeAnchorName);

		if (leftHandAnchor == null)
			leftHandAnchor = ConfigureAnchor(trackingSpace, leftHandAnchorName);

		if (rightHandAnchor == null)
			rightHandAnchor = ConfigureAnchor(trackingSpace, rightHandAnchorName);

		if (trackerAnchor == null)
			trackerAnchor = ConfigureAnchor(trackingSpace, trackerAnchorName);

		if (leftControllerAnchor == null)
			leftControllerAnchor = ConfigureAnchor(leftHandAnchor, leftControllerAnchorName);

		if (rightControllerAnchor == null)
			rightControllerAnchor = ConfigureAnchor(rightHandAnchor, rightControllerAnchorName);

		if (_centerEyeCamera == null || _leftEyeCamera == null || _rightEyeCamera == null)
		{
			_centerEyeCamera = centerEyeAnchor.GetComponent<Camera>();
			_leftEyeCamera = leftEyeAnchor.GetComponent<Camera>();
			_rightEyeCamera = rightEyeAnchor.GetComponent<Camera>();

			if (_centerEyeCamera == null)
			{
				_centerEyeCamera = centerEyeAnchor.gameObject.AddComponent<Camera>();
				_centerEyeCamera.tag = "MainCamera";
			}

			if (_leftEyeCamera == null)
			{
				_leftEyeCamera = leftEyeAnchor.gameObject.AddComponent<Camera>();
				_leftEyeCamera.tag = "MainCamera";
			}

			if (_rightEyeCamera == null)
			{
				_rightEyeCamera = rightEyeAnchor.gameObject.AddComponent<Camera>();
				_rightEyeCamera.tag = "MainCamera";
			}

			_centerEyeCamera.stereoTargetEye = StereoTargetEyeMask.Both;
			_leftEyeCamera.stereoTargetEye = StereoTargetEyeMask.Left;
			_rightEyeCamera.stereoTargetEye = StereoTargetEyeMask.Right;
		}

		if (monoscopic && !OVRPlugin.EyeTextureArrayEnabled)
		{
			// Output to left eye only when in monoscopic mode
			if (_centerEyeCamera.stereoTargetEye != StereoTargetEyeMask.Left)
			{
				_centerEyeCamera.stereoTargetEye = StereoTargetEyeMask.Left;
			}
		}
		else
		{
			if (_centerEyeCamera.stereoTargetEye != StereoTargetEyeMask.Both)
			{
				_centerEyeCamera.stereoTargetEye = StereoTargetEyeMask.Both;
			}
		}

		if (disableEyeAnchorCameras)
		{
			_centerEyeCamera.enabled = false;
			_leftEyeCamera.enabled = false;
			_rightEyeCamera.enabled = false;
		}
		else
		{
			// disable the right eye camera when in monoscopic mode
			if (_centerEyeCamera.enabled == usePerEyeCameras ||
					_leftEyeCamera.enabled == !usePerEyeCameras ||
					_rightEyeCamera.enabled == !(usePerEyeCameras && (!monoscopic || OVRPlugin.EyeTextureArrayEnabled)))
			{
				_skipUpdate = true;
			}

			_centerEyeCamera.enabled = !usePerEyeCameras;
			_leftEyeCamera.enabled = usePerEyeCameras;
			_rightEyeCamera.enabled = (usePerEyeCameras && (!monoscopic || OVRPlugin.EyeTextureArrayEnabled));

		}
	}

	protected virtual Transform ConfigureAnchor(Transform root, string name)
	{
		Transform anchor = (root != null) ? root.Find(name) : null;

		if (anchor == null)
		{
			anchor = transform.Find(name);
		}

		if (anchor == null)
		{
			anchor = new GameObject(name).transform;
		}

		anchor.name = name;
		anchor.parent = (root != null) ? root : transform;
		anchor.localScale = Vector3.one;
		anchor.localPosition = Vector3.zero;
		anchor.localRotation = Quaternion.identity;

		return anchor;
	}

	public virtual Matrix4x4 ComputeTrackReferenceMatrix()
	{
		if (centerEyeAnchor == null)
		{
			Debug.LogError("centerEyeAnchor is required");
			return Matrix4x4.identity;
		}

		// The ideal approach would be using UnityEngine.VR.VRNode.TrackingReference, then we would not have to depend on the OVRCameraRig. Unfortunately, it is not available in Unity 5.4.3

		OVRPose headPose = OVRPose.identity;

		Vector3 pos;
		Quaternion rot;
		if (OVRNodeStateProperties.GetNodeStatePropertyVector3(Node.Head, NodeStatePropertyType.Position, OVRPlugin.Node.Head, OVRPlugin.Step.Render, out pos))
			headPose.position = pos;
		if (OVRNodeStateProperties.GetNodeStatePropertyQuaternion(Node.Head, NodeStatePropertyType.Orientation, OVRPlugin.Node.Head, OVRPlugin.Step.Render, out rot))
			headPose.orientation = rot;

		OVRPose invHeadPose = headPose.Inverse();
		Matrix4x4 invHeadMatrix = Matrix4x4.TRS(invHeadPose.position, invHeadPose.orientation, Vector3.one);

		Matrix4x4 ret = centerEyeAnchor.localToWorldMatrix * invHeadMatrix;

		return ret;
	}
}
