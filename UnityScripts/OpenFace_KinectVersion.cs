using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using System.Text;
using System;
using System.Runtime.InteropServices;

public class OpenFace_KinectVersion : MonoBehaviour
{
    //***************************************数据传输************************************//
    static IPEndPoint ep = new IPEndPoint(IPAddress.Any, 0);
    static UdpClient udpClient = new UdpClient(8010);
    static byte[] receiveBytes = new byte[1024];
    //static string returnData = "";
    static Thread receiveThread;

    //控制量
    private controlParam[] control_params;
    //控制量数目
    public const int controlParamCount = 25;
    public const int auParamsCount = 14;
    public const int headRotationParamsCount = 4;
    public const int eyeParamsCount = 5;
    public const int faceControlParamsCount = auParamsCount + headRotationParamsCount + eyeParamsCount;
    public const int controlParamsCount = 25;
    //控制方式
    private char runMode = 'w';

    //***************************************表情随动************************************//
    //头部控制节点
    [Tooltip("头部控制节点")]
    public Transform HeadJoint;
    //眼睛瞳孔控制节点
    [Tooltip("左眼瞳孔控制节点")]
    public Transform LeftEyeTransform;
    [Tooltip("右眼瞳孔控制节点")]
    public Transform RightEyeTransform;

    //眼睛控制参数
    private eyeState leftEyeState;
    private eyeState rightEyeState;

    //头部姿态参数
    private Quaternion faceQuaternion;

    //面部表情BlendShape控制节点
    [Tooltip("面部表情BlendShape控制节点")]
    public SkinnedMeshRenderer skinnedMeshRenderer;
    int blendSpaceCount;
    private Mesh skinnedMesh;

    //facial AUs controller
    private float BrowsOuterLowerWeight;
    private float BrowsUpWeight;
    private float BrowsDownWeight;
    private float EyeWideWeight;
    private float SquintWeight;
    private float NoseScrunchWeight;
    private float UpperLipUpWeight;
    private float LipCornerPullerWeight;
    private float DimplerWeight;
    private float LowerLipDownWeight;
    private float ChinRaiserWeight;
    private float LipStretcherWeight;
    private float MouthOpenWeight;
    private float JawDownWeight;

    private float blink;

    private void calcuBlendShapeWeight()
    {
        BrowsOuterLowerWeight = control_params[0].faceControlParam;
        BrowsUpWeight = control_params[1].faceControlParam;
        BrowsDownWeight = control_params[2].faceControlParam;
        EyeWideWeight = control_params[3].faceControlParam;
        SquintWeight = control_params[4].faceControlParam;
        NoseScrunchWeight = control_params[5].faceControlParam;
        UpperLipUpWeight = control_params[6].faceControlParam;
        LipCornerPullerWeight = control_params[7].faceControlParam;
        DimplerWeight = control_params[8].faceControlParam;
        LowerLipDownWeight = control_params[9].faceControlParam;
        ChinRaiserWeight = control_params[10].faceControlParam;
        LipStretcherWeight = control_params[11].faceControlParam;
        MouthOpenWeight = control_params[12].faceControlParam;
        JawDownWeight = control_params[13].faceControlParam;
    }
    //***************************************肢体随动************************************//
    [Tooltip("Bind hand to the ball.")]
    public Transform ballPosition;

    [Tooltip("How high off the ground is the sensor (in meters).")]
    public float sensorHeight = 1.0f;

    [Tooltip("Kinect elevation angle (in degrees). May be positive or negative.")]
    public float sensorAngle = 0f;

    [Tooltip("Whether the avatar is facing the player or not.")]
    public bool mirroredMovement = true;

    [Tooltip("Whether the avatar is allowed to move vertically or not.")]
    public bool verticalMovement = false;

    [Tooltip("Whether to utilize only the tracked joints (and ignore the inferred ones) or not.")]
    public bool ignoreInferredJoints = false;

    [Tooltip("Whether the avatar's root motion is applied by other component or script.")]
    public bool externalRootMotion = false;

    [Tooltip("Whether the finger orientations are allowed or not.")]
    public bool fingerOrientations = false;

    [Tooltip("Rate at which the avatar will move through the scene.")]
    public float moveRate = 1f;

    [Tooltip("Smooth factor used for avatar movements and joint rotations.")]
    public float smoothFactor = 10f;

    [Tooltip("Game object this transform is relative to (optional).")]
    public GameObject offsetNode;

    [Tooltip("If enabled, makes the avatar position relative to this camera to be the same as the player's position to the sensor.")]
    public Camera posRelativeToCamera;

    [Tooltip("Whether the avatar's position should match the color image (in Pos-rel-to-camera mode only).")]
    public bool posRelOverlayColor = false;

    [Tooltip("Whether z-axis movement needs to be inverted (Pos-Relative mode only).")]
    public bool posRelInvertedZ = true;


    public enum AllowedRotations : int { None = 0, Default = 1, All = 2 }
    [Tooltip("Allowed wrist and hand rotations. None - no hand rotations, Default - hand rotations are allowed except the twists, All - all rotations are allowed.")]
    public AllowedRotations allowedHandRotations = AllowedRotations.Default;

    // Kinect to world matrix
    private Matrix4x4 kinectToWorld = Matrix4x4.zero;

    // The body root node
    protected Transform bodyRoot;
    // Variable to hold all them bones. It will initialize the same size as initialRotations.
    protected Transform[] bones;
    // Rotations of the bones when the Kinect tracking starts.
    protected Quaternion[] initialRotations;

    // Initial position and rotation of the transform
    protected Vector3 initialPosition;
    protected Quaternion initialRotation;
    protected Vector3 offsetNodePos;
    protected Quaternion offsetNodeRot;
    protected Vector3 bodyRootPosition;

    // whether the parent transform obeys physics
    protected bool isRigidBody = false;
    // Calibration Offset Variables for Character Position.
    protected bool offsetCalibrated = false;
    protected Vector3 offsetPos = Vector3.zero;

    private KinectJointData[] kinectJoints;
    private JointData[] joints;

    private Vector3 hipsDirection;
    private Vector3 shouldersDirection;
    private float bodyTurnAngle;
    private float turnAroundFactor;
    // transform caching gives performance boost since Unity calls GetComponent<Transform>() each time you call transform 
    private Transform _transformCache;
    public new Transform transform
    {
        get
        {
            if (!_transformCache)
            {
                _transformCache = base.transform;
            }

            return _transformCache;
        }
    }

    protected void SetModelArmsInTpose()
    {
        Vector3 vTposeLeftDir = transform.TransformDirection(Vector3.left);
        Vector3 vTposeRightDir = transform.TransformDirection(Vector3.right);
        Animator animator = GetComponent<Animator>();

        Transform transLeftUarm = animator.GetBoneTransform(HumanBodyBones.LeftUpperArm);
        Transform transLeftLarm = animator.GetBoneTransform(HumanBodyBones.LeftLowerArm);
        Transform transLeftHand = animator.GetBoneTransform(HumanBodyBones.LeftHand);

        if (transLeftUarm != null && transLeftLarm != null)
        {
            Vector3 vUarmLeftDir = transLeftLarm.position - transLeftUarm.position;
            float fUarmLeftAngle = Vector3.Angle(vUarmLeftDir, vTposeLeftDir);

            if (Mathf.Abs(fUarmLeftAngle) >= 5f)
            {
                Quaternion vFixRotation = Quaternion.FromToRotation(vUarmLeftDir, vTposeLeftDir);
                transLeftUarm.rotation = vFixRotation * transLeftUarm.rotation;
            }

            if (transLeftHand != null)
            {
                Vector3 vLarmLeftDir = transLeftHand.position - transLeftLarm.position;
                float fLarmLeftAngle = Vector3.Angle(vLarmLeftDir, vTposeLeftDir);

                if (Mathf.Abs(fLarmLeftAngle) >= 5f)
                {
                    Quaternion vFixRotation = Quaternion.FromToRotation(vLarmLeftDir, vTposeLeftDir);
                    transLeftLarm.rotation = vFixRotation * transLeftLarm.rotation;
                }
            }
        }

        Transform transRightUarm = animator.GetBoneTransform(HumanBodyBones.RightUpperArm);
        Transform transRightLarm = animator.GetBoneTransform(HumanBodyBones.RightLowerArm);
        Transform transRightHand = animator.GetBoneTransform(HumanBodyBones.RightHand);

        if (transRightUarm != null && transRightLarm != null)
        {
            Vector3 vUarmRightDir = transRightLarm.position - transRightUarm.position;
            float fUarmRightAngle = Vector3.Angle(vUarmRightDir, vTposeRightDir);

            if (Mathf.Abs(fUarmRightAngle) >= 5f)
            {
                Quaternion vFixRotation = Quaternion.FromToRotation(vUarmRightDir, vTposeRightDir);
                transRightUarm.rotation = vFixRotation * transRightUarm.rotation;
            }

            if (transRightHand != null)
            {
                Vector3 vLarmRightDir = transRightHand.position - transRightLarm.position;
                float fLarmRightAngle = Vector3.Angle(vLarmRightDir, vTposeRightDir);

                if (Mathf.Abs(fLarmRightAngle) >= 5f)
                {
                    Quaternion vFixRotation = Quaternion.FromToRotation(vLarmRightDir, vTposeRightDir);
                    transRightLarm.rotation = vFixRotation * transRightLarm.rotation;
                }
            }
        }

    }
    // If the bones to be mapped have been declared, map that bone to the model.
    protected virtual void MapBones()
    {
        //		// make OffsetNode as a parent of model transform.
        //		offsetNode = new GameObject(name + "Ctrl") { layer = transform.gameObject.layer, tag = transform.gameObject.tag };
        //		offsetNode.transform.position = transform.position;
        //		offsetNode.transform.rotation = transform.rotation;
        //		offsetNode.transform.parent = transform.parent;

        //		// take model transform as body root
        //		transform.parent = offsetNode.transform;
        //		transform.localPosition = Vector3.zero;
        //		transform.localRotation = Quaternion.identity;

        //bodyRoot = transform;

        // get bone transforms from the animator component
        Animator animatorComponent = GetComponent<Animator>();

        for (int boneIndex = 0; boneIndex < bones.Length; boneIndex++)
        {
            if (!boneIndex2MecanimMap.ContainsKey(boneIndex))
                continue;

            bones[boneIndex] = animatorComponent ? animatorComponent.GetBoneTransform(boneIndex2MecanimMap[boneIndex]) : null;
        }
    }

    // Capture the initial rotations of the bones
    protected void GetInitialRotations()
    {
        // save the initial rotation
        if (offsetNode != null)
        {
            offsetNodePos = offsetNode.transform.position;
            offsetNodeRot = offsetNode.transform.rotation;
        }

        initialPosition = transform.position;
        initialRotation = transform.rotation;

        //		if(offsetNode != null)
        //		{
        //			initialRotation = Quaternion.Inverse(offsetNodeRot) * initialRotation;
        //		}

        transform.rotation = Quaternion.identity;

        // save the body root initial position
        if (bodyRoot != null)
        {
            bodyRootPosition = bodyRoot.position;
        }
        else
        {
            bodyRootPosition = transform.position;
        }

        if (offsetNode != null)
        {
            bodyRootPosition = bodyRootPosition - offsetNodePos;
        }

        // save the initial bone rotations
        for (int i = 0; i < bones.Length; i++)
        {
            if (bones[i] != null)
            {
                initialRotations[i] = bones[i].rotation;
            }
        }

        // Restore the initial rotation
        transform.rotation = initialRotation;
    }
    //calculate bone direction
    private void CalcBodyFrameBoneDirs()
    {
        for (int j = 0; j < (int)JointType.Count; j++)
        {
            if (j == 0)
            {
                joints[j].direction = Vector3.zero;
            }
            else
            {
                int jParent = (int)GetParentJoint((JointType)j);

                if (joints[j].TrackingState != TrackingState.NotTracked &&
                   joints[jParent].TrackingState != TrackingState.NotTracked)
                {
                    joints[j].direction =
                        joints[j].position - joints[jParent].position;
                }
            }
        }

    }
    //process special condition
    private void ProcessBodySpecialData(ref JointData[] joint)
    {
        if (joint[(int)JointType.HipLeft].TrackingState == TrackingState.NotTracked &&
           joint[(int)JointType.SpineBase].TrackingState != TrackingState.NotTracked &&
           joint[(int)JointType.HipRight].TrackingState != TrackingState.NotTracked)
        {
            joint[(int)JointType.HipLeft].TrackingState = TrackingState.Inferred;

            joint[(int)JointType.HipLeft].kinectPos = joint[(int)JointType.SpineBase].kinectPos +
                (joint[(int)JointType.SpineBase].kinectPos - joint[(int)JointType.HipRight].kinectPos);
            joint[(int)JointType.HipLeft].position = joint[(int)JointType.SpineBase].position +
                (joint[(int)JointType.SpineBase].position - joint[(int)JointType.HipRight].position);
            joint[(int)JointType.HipLeft].direction = joint[(int)JointType.HipLeft].position -
                joint[(int)JointType.SpineBase].position;
        }

        if (joint[(int)JointType.HipRight].TrackingState == TrackingState.NotTracked &&
           joint[(int)JointType.SpineBase].TrackingState != TrackingState.NotTracked &&
           joint[(int)JointType.HipLeft].TrackingState != TrackingState.NotTracked)
        {
            joint[(int)JointType.HipRight].TrackingState = TrackingState.Inferred;

            joint[(int)JointType.HipRight].kinectPos = joint[(int)JointType.SpineBase].kinectPos +
                (joint[(int)JointType.SpineBase].kinectPos - joint[(int)JointType.HipLeft].kinectPos);
            joint[(int)JointType.HipRight].position = joint[(int)JointType.SpineBase].position +
                (joint[(int)JointType.SpineBase].position - joint[(int)JointType.HipLeft].position);
            joint[(int)JointType.HipRight].direction = joint[(int)JointType.HipRight].position -
                joint[(int)JointType.SpineBase].position;
        }

        if ((joint[(int)JointType.ShoulderLeft].TrackingState == TrackingState.NotTracked &&
            joint[(int)JointType.SpineShoulder].TrackingState != TrackingState.NotTracked &&
            joint[(int)JointType.ShoulderRight].TrackingState != TrackingState.NotTracked))
        {
            joint[(int)JointType.ShoulderLeft].TrackingState = TrackingState.Inferred;

            joint[(int)JointType.ShoulderLeft].kinectPos = joint[(int)JointType.SpineShoulder].kinectPos +
                (joint[(int)JointType.SpineShoulder].kinectPos - joint[(int)JointType.ShoulderRight].kinectPos);
            joint[(int)JointType.ShoulderLeft].position = joint[(int)JointType.SpineShoulder].position +
                (joint[(int)JointType.SpineShoulder].position - joint[(int)JointType.ShoulderRight].position);
            joint[(int)JointType.ShoulderLeft].direction = joint[(int)JointType.ShoulderLeft].position -
                joint[(int)JointType.SpineShoulder].position;
        }

        if ((joint[(int)JointType.ShoulderRight].TrackingState == TrackingState.NotTracked &&
            joint[(int)JointType.SpineShoulder].TrackingState != TrackingState.NotTracked &&
            joint[(int)JointType.ShoulderLeft].TrackingState != TrackingState.NotTracked))
        {
            joint[(int)JointType.ShoulderRight].TrackingState = TrackingState.Inferred;

            joint[(int)JointType.ShoulderRight].kinectPos = joint[(int)JointType.SpineShoulder].kinectPos +
                (joint[(int)JointType.SpineShoulder].kinectPos - joint[(int)JointType.ShoulderLeft].kinectPos);
            joint[(int)JointType.ShoulderRight].position = joint[(int)JointType.SpineShoulder].position +
                (joint[(int)JointType.SpineShoulder].position - joint[(int)JointType.ShoulderLeft].position);
            joint[(int)JointType.ShoulderRight].direction = joint[(int)JointType.ShoulderRight].position -
                joint[(int)JointType.SpineShoulder].position;
        }

        // calculate special directions
        if (joint[(int)JointType.HipLeft].TrackingState != TrackingState.NotTracked &&
           joint[(int)JointType.HipRight].TrackingState != TrackingState.NotTracked)
        {
            Vector3 posRHip = joint[(int)JointType.HipRight].position;
            Vector3 posLHip = joint[(int)JointType.HipLeft].position;

            hipsDirection = posRHip - posLHip;
            hipsDirection -= Vector3.Project(hipsDirection, Vector3.up);
        }

        if (joint[(int)JointType.ShoulderLeft].TrackingState != TrackingState.NotTracked &&
           joint[(int)JointType.ShoulderRight].TrackingState != TrackingState.NotTracked)
        {
            Vector3 posRShoulder = joint[(int)JointType.ShoulderRight].position;
            Vector3 posLShoulder = joint[(int)JointType.ShoulderLeft].position;

            shouldersDirection = posRShoulder - posLShoulder;
            shouldersDirection -= Vector3.Project(shouldersDirection, Vector3.up);

            Vector3 shouldersDir = shouldersDirection;
            shouldersDir.z = -shouldersDir.z;

            Quaternion turnRot = Quaternion.FromToRotation(Vector3.right, shouldersDir);
            bodyTurnAngle = turnRot.eulerAngles.y;
        }

        //				if(joint[(int)JointType.ElbowLeft].TrackingState != TrackingState.NotTracked &&
        //				   joint[(int)JointType.WristLeft].TrackingState != TrackingState.NotTracked)
        //				{
        //					Vector3 pos1 = joint[(int)JointType.ElbowLeft].position;
        //					Vector3 pos2 = joint[(int)JointType.WristLeft].position;
        //					
        //					leftArmDirection = pos2 - pos1;
        //				}

        //				if(allowHandRotations && leftArmDirection != Vector3.zero &&
        //				   joint[(int)JointType.WristLeft].TrackingState != TrackingState.NotTracked &&
        //				   joint[(int)JointType.ThumbLeft].TrackingState != TrackingState.NotTracked)
        //				{
        //					Vector3 pos1 = joint[(int)JointType.WristLeft].position;
        //					Vector3 pos2 = joint[(int)JointType.ThumbLeft].position;
        //
        //					Vector3 armDir = leftArmDirection;
        //					armDir.z = -armDir.z;
        //					
        //					leftThumbDirection = pos2 - pos1;
        //					leftThumbDirection.z = -leftThumbDirection.z;
        //					leftThumbDirection -= Vector3.Project(leftThumbDirection, armDir);
        //					
        //					leftThumbForward = Quaternion.AngleAxis(bodyTurnAngle, Vector3.up) * Vector3.forward;
        //					leftThumbForward -= Vector3.Project(leftThumbForward, armDir);
        //
        //					if(leftThumbForward.sqrMagnitude < 0.01f)
        //					{
        //						leftThumbForward = Vector3.zero;
        //					}
        //				}
        //				else
        //				{
        //					if(leftThumbDirection != Vector3.zero)
        //					{
        //						leftThumbDirection = Vector3.zero;
        //						leftThumbForward = Vector3.zero;
        //					}
        //				}

        //				if(joint[(int)JointType.ElbowRight].TrackingState != TrackingState.NotTracked &&
        //				   joint[(int)JointType.WristRight].TrackingState != TrackingState.NotTracked)
        //				{
        //					Vector3 pos1 = joint[(int)JointType.ElbowRight].position;
        //					Vector3 pos2 = joint[(int)JointType.WristRight].position;
        //					
        //					rightArmDirection = pos2 - pos1;
        //				}

        //				if(allowHandRotations && rightArmDirection != Vector3.zero &&
        //				   joint[(int)JointType.WristRight].TrackingState != TrackingState.NotTracked &&
        //				   joint[(int)JointType.ThumbRight].TrackingState != TrackingState.NotTracked)
        //				{
        //					Vector3 pos1 = joint[(int)JointType.WristRight].position;
        //					Vector3 pos2 = joint[(int)JointType.ThumbRight].position;
        //
        //					Vector3 armDir = rightArmDirection;
        //					armDir.z = -armDir.z;
        //					
        //					rightThumbDirection = pos2 - pos1;
        //					rightThumbDirection.z = -rightThumbDirection.z;
        //					rightThumbDirection -= Vector3.Project(rightThumbDirection, armDir);
        //
        //					rightThumbForward = Quaternion.AngleAxis(bodyTurnAngle, Vector3.up) * Vector3.forward;
        //					rightThumbForward -= Vector3.Project(rightThumbForward, armDir);
        //
        //					if(rightThumbForward.sqrMagnitude < 0.01f)
        //					{
        //						rightThumbForward = Vector3.zero;
        //					}
        //				}
        //				else
        //				{
        //					if(rightThumbDirection != Vector3.zero)
        //					{
        //						rightThumbDirection = Vector3.zero;
        //						rightThumbForward = Vector3.zero;
        //					}
        //				}

        if (joint[(int)JointType.KneeLeft].TrackingState != TrackingState.NotTracked &&
           joint[(int)JointType.AnkleLeft].TrackingState != TrackingState.NotTracked &&
           joint[(int)JointType.FootLeft].TrackingState != TrackingState.NotTracked)
        {
            Vector3 vFootProjected = Vector3.Project(joint[(int)JointType.FootLeft].direction, joint[(int)JointType.AnkleLeft].direction);

            joint[(int)JointType.AnkleLeft].kinectPos += vFootProjected;
            joint[(int)JointType.AnkleLeft].position += vFootProjected;
            joint[(int)JointType.FootLeft].direction -= vFootProjected;
        }

        if (joint[(int)JointType.KneeRight].TrackingState != TrackingState.NotTracked &&
           joint[(int)JointType.AnkleRight].TrackingState != TrackingState.NotTracked &&
           joint[(int)JointType.FootRight].TrackingState != TrackingState.NotTracked)
        {
            Vector3 vFootProjected = Vector3.Project(joint[(int)JointType.FootRight].direction, joint[(int)JointType.AnkleRight].direction);

            joint[(int)JointType.AnkleRight].kinectPos += vFootProjected;
            joint[(int)JointType.AnkleRight].position += vFootProjected;
            joint[(int)JointType.FootRight].direction -= vFootProjected;
        }
    }

    // calculates orientations of the body joints
    private void CalculateJointOrients(ref JointData[] jointsData)
    {
        int jointCount = (int)JointType.Count;

        for (int j = 0; j < jointCount; j++)
        {
            int joint = j;

            JointData jointData = jointsData[joint];
            bool bJointValid = ignoreInferredJoints ? jointData.TrackingState == TrackingState.Tracked : jointData.TrackingState != TrackingState.NotTracked;

            if (bJointValid)
            {
                int nextJoint = (int)GetNextJoint((JointType)joint);
                if (nextJoint != joint && nextJoint >= 0 && nextJoint < jointCount)
                {
                    JointData nextJointData = jointsData[nextJoint];
                    bool bNextJointValid = ignoreInferredJoints ? nextJointData.TrackingState == TrackingState.Tracked : nextJointData.TrackingState != TrackingState.NotTracked;

                    Vector3 baseDir = JointBaseDir[nextJoint];
                    Vector3 jointDir = nextJointData.direction;
                    jointDir = new Vector3(jointDir.x, jointDir.y, -jointDir.z).normalized;

                    Quaternion jointOrientNormal = jointData.normalRotation;
                    if (bNextJointValid)
                    {
                        jointOrientNormal = Quaternion.FromToRotation(baseDir, jointDir);
                    }

                    if ((joint == (int)JointType.ShoulderLeft) ||
                       (joint == (int)JointType.ShoulderRight))
                    {
                        float angle = -bodyTurnAngle;
                        Vector3 axis = jointDir;
                        Quaternion armTurnRotation = Quaternion.AngleAxis(angle, axis);

                        jointData.normalRotation = armTurnRotation * jointOrientNormal;
                    }
                    else if ((joint == (int)JointType.ElbowLeft) ||
                            (joint == (int)JointType.WristLeft)
                            || (joint == (int)JointType.HandLeft))
                    {
                        //						if(joint == (int)JointType.WristLeft)
                        //						{
                        //							JointData handData = bodyData.joint[(int)JointType.HandLeft];
                        //							JointData handTipData = bodyData.joint[(int)JointType.HandTipLeft];
                        //							
                        //							if(handData.trackingState != TrackingState.NotTracked &&
                        //							   handTipData.trackingState != TrackingState.NotTracked)
                        //							{
                        //								jointDir = handData.direction + handTipData.direction;
                        //								jointDir = new Vector3(jointDir.x, jointDir.y, -jointDir.z).normalized;
                        //							}
                        //						}

                        JointData shCenterData = jointsData[(int)JointType.SpineShoulder];
                        if (shCenterData.TrackingState != TrackingState.NotTracked &&
                           jointDir != Vector3.zero && shCenterData.direction != Vector3.zero &&
                           Mathf.Abs(Vector3.Dot(jointDir, shCenterData.direction.normalized)) < 0.5f)
                        {
                            Vector3 spineDir = shCenterData.direction;
                            spineDir = new Vector3(spineDir.x, spineDir.y, -spineDir.z).normalized;

                            Vector3 fwdDir = Vector3.Cross(-jointDir, spineDir).normalized;
                            Vector3 upDir = Vector3.Cross(fwdDir, -jointDir).normalized;
                            jointOrientNormal = Quaternion.LookRotation(fwdDir, upDir);
                        }
                        //						else
                        //						{
                        //							jointOrientNormal = Quaternion.FromToRotation(baseDir, jointDir);
                        //						}

                        bool bRotated = (allowedHandRotations == AllowedRotations.None) &&
                                        (joint != (int)JointType.ElbowLeft);  // false;
                        if ((allowedHandRotations == AllowedRotations.All)
                           && (joint != (int)JointType.HandLeft))
                        {
                            //							JointData handData = bodyData.joint[(int)JointType.HandLeft];
                            //							JointData handTipData = bodyData.joint[(int)JointType.HandTipLeft];
                            JointData thumbData = jointsData[(int)JointType.ThumbLeft];

                            //							if(handData.trackingState != TrackingState.NotTracked &&
                            //							   handTipData.trackingState != TrackingState.NotTracked &&
                            if (thumbData.TrackingState != TrackingState.NotTracked)
                            {
                                Vector3 rightDir = -nextJointData.direction; // -(handData.direction + handTipData.direction);
                                rightDir = new Vector3(rightDir.x, rightDir.y, -rightDir.z).normalized;

                                Vector3 fwdDir = thumbData.direction;
                                fwdDir = new Vector3(fwdDir.x, fwdDir.y, -fwdDir.z).normalized;

                                if (rightDir != Vector3.zero && fwdDir != Vector3.zero)
                                {
                                    Vector3 upDir = Vector3.Cross(fwdDir, rightDir).normalized;
                                    fwdDir = Vector3.Cross(rightDir, upDir).normalized;

                                    jointData.normalRotation = Quaternion.LookRotation(fwdDir, upDir);
                                    //bRotated = true;

                                    //									// fix invalid wrist rotation
                                    //									JointData elbowData = bodyData.joint[(int)JointType.ElbowLeft];
                                    //									if(elbowData.trackingState != TrackingState.NotTracked)
                                    //									{
                                    //										Quaternion quatLocalRot = Quaternion.Inverse(elbowData.normalRotation) * jointData.normalRotation;
                                    //										float angleY = quatLocalRot.eulerAngles.y;
                                    //										
                                    //										if(angleY >= 90f && angleY < 270f && bodyData.leftHandOrientation != Quaternion.identity)
                                    //										{
                                    //											jointData.normalRotation = bodyData.leftHandOrientation;
                                    //										}
                                    //										
                                    //										bodyData.leftHandOrientation = jointData.normalRotation;
                                    //									}

                                    //bRotated = true;
                                }
                            }

                            bRotated = true;
                        }

                        if (!bRotated)
                        {
                            float angle = -bodyTurnAngle;
                            Vector3 axis = jointDir;
                            Quaternion armTurnRotation = Quaternion.AngleAxis(angle, axis);

                            jointData.normalRotation = //(allowedHandRotations != AllowedRotations.None || joint == (int)JointType.ElbowLeft) ? 
                                armTurnRotation * jointOrientNormal; // : armTurnRotation;
                        }
                    }
                    else if ((joint == (int)JointType.ElbowRight) ||
                            (joint == (int)JointType.WristRight)
                            || (joint == (int)JointType.HandRight))
                    {
                        //						if(joint == (int)JointType.WristRight)
                        //						{
                        //							JointData handData = bodyData.joint[(int)JointType.HandRight];
                        //							JointData handTipData = bodyData.joint[(int)JointType.HandTipRight];
                        //
                        //							if(handData.trackingState != TrackingState.NotTracked &&
                        //							   handTipData.trackingState != TrackingState.NotTracked)
                        //							{
                        //								jointDir = handData.direction + handTipData.direction;
                        //								jointDir = new Vector3(jointDir.x, jointDir.y, -jointDir.z).normalized;
                        //							}
                        //						}

                        JointData shCenterData = jointsData[(int)JointType.SpineShoulder];
                        if (shCenterData.TrackingState != TrackingState.NotTracked &&
                           jointDir != Vector3.zero && shCenterData.direction != Vector3.zero &&
                           Mathf.Abs(Vector3.Dot(jointDir, shCenterData.direction.normalized)) < 0.5f)
                        {
                            Vector3 spineDir = shCenterData.direction;
                            spineDir = new Vector3(spineDir.x, spineDir.y, -spineDir.z).normalized;

                            Vector3 fwdDir = Vector3.Cross(jointDir, spineDir).normalized;
                            Vector3 upDir = Vector3.Cross(fwdDir, jointDir).normalized;
                            jointOrientNormal = Quaternion.LookRotation(fwdDir, upDir);
                        }
                        //						else
                        //						{
                        //							jointOrientNormal = Quaternion.FromToRotation(baseDir, jointDir);
                        //						}

                        bool bRotated = (allowedHandRotations == AllowedRotations.None) &&
                                        (joint != (int)JointType.ElbowRight);  // false;
                        if ((allowedHandRotations == AllowedRotations.All)
                           && (joint != (int)JointType.HandRight))
                        {
                            //							JointData handData = bodyData.joint[(int)JointType.HandRight];
                            //							JointData handTipData = bodyData.joint[(int)JointType.HandTipRight];
                            JointData thumbData = jointsData[(int)JointType.ThumbRight];

                            //							if(handData.trackingState != TrackingState.NotTracked &&
                            //							   handTipData.trackingState != TrackingState.NotTracked &&
                            if (thumbData.TrackingState != TrackingState.NotTracked)
                            {
                                Vector3 rightDir = nextJointData.direction; // handData.direction + handTipData.direction;
                                rightDir = new Vector3(rightDir.x, rightDir.y, -rightDir.z).normalized;

                                Vector3 fwdDir = thumbData.direction;
                                fwdDir = new Vector3(fwdDir.x, fwdDir.y, -fwdDir.z).normalized;

                                if (rightDir != Vector3.zero && fwdDir != Vector3.zero)
                                {
                                    Vector3 upDir = Vector3.Cross(fwdDir, rightDir).normalized;
                                    fwdDir = Vector3.Cross(rightDir, upDir).normalized;

                                    jointData.normalRotation = Quaternion.LookRotation(fwdDir, upDir);
                                    //bRotated = true;

                                    //									// fix invalid wrist rotation
                                    //									JointData elbowData = bodyData.joint[(int)JointType.ElbowRight];
                                    //									if(elbowData.trackingState != TrackingState.NotTracked)
                                    //									{
                                    //										Quaternion quatLocalRot = Quaternion.Inverse(elbowData.normalRotation) * jointData.normalRotation;
                                    //										float angleY = quatLocalRot.eulerAngles.y;
                                    //										
                                    //										if(angleY >= 90f && angleY < 270f && bodyData.rightHandOrientation != Quaternion.identity)
                                    //										{
                                    //											jointData.normalRotation = bodyData.rightHandOrientation;
                                    //										}
                                    //										
                                    //										bodyData.rightHandOrientation = jointData.normalRotation;
                                    //									}

                                    //bRotated = true;
                                }
                            }

                            bRotated = true;
                        }

                        if (!bRotated)
                        {
                            float angle = -bodyTurnAngle;
                            Vector3 axis = jointDir;
                            Quaternion armTurnRotation = Quaternion.AngleAxis(angle, axis);

                            jointData.normalRotation = //(allowedHandRotations != AllowedRotations.None || joint == (int)JointType.ElbowRight) ? 
                                armTurnRotation * jointOrientNormal; // : armTurnRotation;
                        }
                    }
                    else
                    {
                        jointData.normalRotation = jointOrientNormal;
                    }

                    if ((joint == (int)JointType.SpineMid) ||
                       (joint == (int)JointType.SpineShoulder) ||
                       (joint == (int)JointType.Neck))
                    {
                        Vector3 baseDir2 = Vector3.right;
                        Vector3 jointDir2 = Vector3.Lerp(shouldersDirection, -shouldersDirection, turnAroundFactor);
                        jointDir2.z = -jointDir2.z;

                        jointData.normalRotation *= Quaternion.FromToRotation(baseDir2, jointDir2);
                    }
                    else if ((joint == (int)JointType.SpineBase) ||
                       (joint == (int)JointType.HipLeft) || (joint == (int)JointType.HipRight) ||
                       (joint == (int)JointType.KneeLeft) || (joint == (int)JointType.KneeRight) ||
                       (joint == (int)JointType.AnkleLeft) || (joint == (int)JointType.AnkleRight))
                    {
                        Vector3 baseDir2 = Vector3.right;
                        Vector3 jointDir2 = Vector3.Lerp(hipsDirection, -hipsDirection, turnAroundFactor);
                        jointDir2.z = -jointDir2.z;

                        jointData.normalRotation *= Quaternion.FromToRotation(baseDir2, jointDir2);
                    }

                    //if (joint == (int)JointType.Neck)
                    //{

                    //        JointData neckData = jointsData[(int)JointType.Neck];
                    //        JointData headData = jointsData[(int)JointType.Head];

                    //        if (neckData.TrackingState == TrackingState.Tracked &&
                    //           headData.TrackingState == TrackingState.Tracked)
                    //        {
                    //            Quaternion headRotation = Quaternion.identity;
                    //            if (sensorData.sensorInterface.GetHeadRotation(jointData.liTrackingID, ref headRotation))
                    //            {
                    //                Vector3 rotAngles = headRotation.eulerAngles;
                    //                rotAngles.x = -rotAngles.x;
                    //                rotAngles.y = -rotAngles.y;

                    //                jointData.headOrientation = jointData.headOrientation != Quaternion.identity ?
                    //                    Quaternion.Slerp(jointData.headOrientation, Quaternion.Euler(rotAngles), 5f * Time.deltaTime) :
                    //                        Quaternion.Euler(rotAngles);

                    //                jointData.normalRotation = jointData.headOrientation;
                    //            }
                    //        }
                    //    }


                    Vector3 mirroredAngles = jointData.normalRotation.eulerAngles;
                    mirroredAngles.y = -mirroredAngles.y;
                    mirroredAngles.z = -mirroredAngles.z;

                    jointData.mirroredRotation = Quaternion.Euler(mirroredAngles);
                }
                else
                {
                    // get the orientation of the parent joint
                    int prevJoint = (int)GetParentJoint((JointType)joint);
                    if (prevJoint != joint && prevJoint >= 0 && prevJoint < (int)JointType.Count &&
                       joint != (int)JointType.ThumbLeft && joint != (int)JointType.ThumbRight)
                    {
                        //						if((allowedHandRotations == AllowedRotations.All) && 
                        //						   (joint == (int)JointType.ThumbLeft ||
                        //						    joint == (int)JointType.ThumbRight))
                        //						{
                        //							Vector3 jointDir = jointData.direction;
                        //							jointDir = new Vector3(jointDir.x, jointDir.y, -jointDir.z).normalized;
                        //
                        //							Vector3 baseDir = JointBaseDir[joint];
                        //							jointData.normalRotation = Quaternion.FromToRotation(baseDir, jointDir);
                        //
                        //							Vector3 mirroredAngles = jointData.normalRotation.eulerAngles;
                        //							mirroredAngles.y = -mirroredAngles.y;
                        //							mirroredAngles.z = -mirroredAngles.z;
                        //							
                        //							jointData.mirroredRotation = Quaternion.Euler(mirroredAngles);
                        //						}
                        //						else
                        {
                            jointData.normalRotation = jointsData[prevJoint].normalRotation;
                            jointData.mirroredRotation = jointsData[prevJoint].mirroredRotation;
                        }
                    }
                    else
                    {
                        jointData.normalRotation = Quaternion.identity;
                        jointData.mirroredRotation = Quaternion.identity;
                    }
                }
            }

            jointsData[joint] = jointData;

            //if (joint == (int)JointType.SpineBase)
            //{
            //    jointData.normalRotation = jointData.normalRotation;
            //    jointData.mirroredRotation = jointData.mirroredRotation;
            //}
        }
    }

    public JointType GetParentJoint(JointType joint)
    {
        switch (joint)
        {
            case JointType.SpineBase:
                return JointType.SpineBase;

            case JointType.Neck:
                return JointType.SpineShoulder;

            case JointType.SpineShoulder:
                return JointType.SpineMid;

            case JointType.ShoulderLeft:
            case JointType.ShoulderRight:
                return JointType.SpineShoulder;

            case JointType.HipLeft:
            case JointType.HipRight:
                return JointType.SpineBase;

            case JointType.HandTipLeft:
                return JointType.HandLeft;

            case JointType.ThumbLeft:
                return JointType.WristLeft;

            case JointType.HandTipRight:
                return JointType.HandRight;

            case JointType.ThumbRight:
                return JointType.WristRight;
        }

        return (JointType)((int)joint - 1);
    }

    public JointType GetNextJoint(JointType joint)
    {
        switch (joint)
        {
            case JointType.SpineBase:
                return JointType.SpineMid;
            case JointType.SpineMid:
                return JointType.SpineShoulder;
            case JointType.SpineShoulder:
                return JointType.Neck;
            case JointType.Neck:
                return JointType.Head;

            case JointType.ShoulderLeft:
                return JointType.ElbowLeft;
            case JointType.ElbowLeft:
                return JointType.WristLeft;
            case JointType.WristLeft:
                return JointType.HandLeft;
            case JointType.HandLeft:
                return JointType.HandTipLeft;

            case JointType.ShoulderRight:
                return JointType.ElbowRight;
            case JointType.ElbowRight:
                return JointType.WristRight;
            case JointType.WristRight:
                return JointType.HandRight;
            case JointType.HandRight:
                return JointType.HandTipRight;

            case JointType.HipLeft:
                return JointType.KneeLeft;
            case JointType.KneeLeft:
                return JointType.AnkleLeft;
            case JointType.AnkleLeft:
                return JointType.FootLeft;

            case JointType.HipRight:
                return JointType.KneeRight;
            case JointType.KneeRight:
                return JointType.AnkleRight;
            case JointType.AnkleRight:
                return JointType.FootRight;
        }

        return joint;  // in case of end joint - Head, HandTipLeft, HandTipRight, FootLeft, FootRight
    }

    public void SuccessfulCalibration()
    {

        // reset the models position
        if (offsetNode != null)
        {
            offsetNode.transform.position = offsetNodePos;
            offsetNode.transform.rotation = offsetNodeRot;
        }

        transform.position = initialPosition;
        transform.rotation = initialRotation;

        // re-calibrate the position offset
        offsetCalibrated = false;
    }

    /// <summary>
    /// Gets the user position, relative to the sensor, in meters.
    /// </summary>
    /// <returns>The user position.</returns>
    /// <param name="userId">User ID</param>
    public Vector3 GetUserPosition()
    {

        if (joints[(int)JointType.SpineBase].TrackingState != 0)
        {
            return joints[(int)JointType.SpineBase].kinectPos;
        }

        return Vector3.zero;
    }

    // Moves the avatar - gets the tracked position of the user and applies it to avatar.
    protected void MoveAvatar()
    {
        if ((moveRate == 0f) || !IsJointTracked((int)JointType.SpineBase))
        {
            return;
        }

        // get the position of user's spine base
        Vector3 trans = GetUserPosition();

        //// use the color overlay position if needed
        //if (posRelativeToCamera && posRelOverlayColor)
        //{
        //    Rect backgroundRect = posRelativeToCamera.pixelRect;
        //    PortraitBackground portraitBack = PortraitBackground.Instance;

        //    if (portraitBack && portraitBack.enabled)
        //    {
        //        backgroundRect = portraitBack.GetBackgroundRect();
        //    }

        //    trans = kinectManager.GetJointPosColorOverlay(UserID, (int)KinectInterop.JointType.SpineBase, posRelativeToCamera, backgroundRect);
        //}

        // invert the z-coordinate, if needed
        if (posRelativeToCamera && posRelInvertedZ)
        {
            trans.z = -trans.z;
        }

        //		if(posRelativeToCamera && avatarPosOverlaysBackground)
        //		{
        //			// gets the user's spine-base position, matching the color-camera background
        //			Rect backgroundRect = posRelativeToCamera.pixelRect;
        //			PortraitBackground portraitBack = PortraitBackground.Instance;
        //			
        //			if(portraitBack && portraitBack.enabled)
        //			{
        //				backgroundRect = portraitBack.GetBackgroundRect();
        //			}
        //
        //			trans = kinectManager.GetJointPosColorOverlay(UserID, (int)KinectInterop.JointType.SpineBase, posRelativeToCamera, backgroundRect);
        //		}

        if (!offsetCalibrated)
        {
            offsetCalibrated = true;

            offsetPos.x = trans.x;  // !mirroredMovement ? trans.x * moveRate : -trans.x * moveRate;
            offsetPos.y = trans.y;  // trans.y * moveRate;
            offsetPos.z = !mirroredMovement && !posRelativeToCamera ? -trans.z : trans.z;  // -trans.z * moveRate;

            if (posRelativeToCamera)
            {
                Vector3 cameraPos = posRelativeToCamera.transform.position;
                Vector3 bodyRootPos = bodyRoot != null ? bodyRoot.position : transform.position;
                Vector3 hipCenterPos = bodyRoot != null ? bodyRoot.position : bones[0].position;

                float yRelToAvatar = 0f;
                if (verticalMovement)
                {
                    yRelToAvatar = (trans.y - cameraPos.y) - (hipCenterPos - bodyRootPos).magnitude;
                }
                else
                {
                    yRelToAvatar = bodyRootPos.y - cameraPos.y;
                }

                Vector3 relativePos = new Vector3(trans.x, yRelToAvatar, trans.z);
                Vector3 newBodyRootPos = cameraPos + relativePos;

                //				if(offsetNode != null)
                //				{
                //					newBodyRootPos += offsetNode.transform.position;
                //				}

                if (bodyRoot != null)
                {
                    bodyRoot.position = newBodyRootPos;
                }
                else
                {
                    transform.position = newBodyRootPos;
                }

                bodyRootPosition = newBodyRootPos;
            }
        }

        // transition to the new position
        Vector3 targetPos = bodyRootPosition + Kinect2AvatarPos(trans, verticalMovement);

        if (isRigidBody && !verticalMovement)
        {
            // workaround for obeying the physics (e.g. gravity falling)
            targetPos.y = bodyRoot != null ? bodyRoot.position.y : transform.position.y;
        }

        if (bodyRoot != null)
        {
            bodyRoot.position = smoothFactor != 0f ?
                Vector3.Lerp(bodyRoot.position, targetPos, smoothFactor * Time.deltaTime) : targetPos;
        }
        else
        {
            transform.position = smoothFactor != 0f ?
                Vector3.Lerp(transform.position, targetPos, smoothFactor * Time.deltaTime) : targetPos;
        }
    }

    // Converts Kinect position to avatar skeleton position, depending on initial position, mirroring and move rate
    protected Vector3 Kinect2AvatarPos(Vector3 jointPosition, bool bMoveVertically)
    {
        float xPos = (jointPosition.x - offsetPos.x) * moveRate;
        float yPos = (jointPosition.y - offsetPos.y) * moveRate;
        float zPos = !mirroredMovement && !posRelativeToCamera ? (-jointPosition.z - offsetPos.z) * moveRate : (jointPosition.z - offsetPos.z) * moveRate;

        Vector3 newPosition = new Vector3(xPos, bMoveVertically ? yPos : 0f, zPos);

        if (offsetNode != null)
        {
            newPosition += offsetNode.transform.position;
        }

        return newPosition;
    }

    // Apply the rotations tracked by kinect to the joints.
    protected void TransformBone(JointType joint, int boneIndex, bool flip)
    {
        if (joint == JointType.Neck) return;
        Transform boneTransform = bones[boneIndex];
        if (boneTransform == null)
            return;

        int iJoint = (int)joint;
        if (iJoint < 0 || !IsJointTracked(iJoint))
            return;

        // Get Kinect joint orientation
        Quaternion jointRotation = GetJointOrientation(iJoint, flip);
        if (jointRotation == Quaternion.identity)
            return;

        // calculate the new orientation
        Quaternion newRotation = Kinect2AvatarRot(jointRotation, boneIndex);

        if (externalRootMotion)
        {
            newRotation = transform.rotation * newRotation;
        }

        // Smoothly transition to the new rotation
        if (smoothFactor != 0f)
            boneTransform.rotation = Quaternion.Slerp(boneTransform.rotation, newRotation, smoothFactor * Time.deltaTime);
        else
            boneTransform.rotation = newRotation;
    }

    // Apply the rotations tracked by kinect to a special joint
    protected void TransformSpecialBone(JointType joint, JointType jointParent, int boneIndex, Vector3 baseDir, bool flip)
    {
        Transform boneTransform = bones[boneIndex];
        if (boneTransform == null)
            return;

        if (!IsJointTracked((int)joint) ||
           !IsJointTracked((int)jointParent))
        {
            return;
        }

        if (boneIndex >= 27 && boneIndex <= 30)
        {
            //// fingers or thumbs
            //if (fingerOrientations)
            //{
            //    TransformSpecialBoneFingers((int)joint, boneIndex, flip);
            //}

            return;
        }

        Vector3 jointDir = GetJointDirection((int)joint, (int)jointParent, false, true);
        Quaternion jointRotation = jointDir != Vector3.zero ? Quaternion.FromToRotation(baseDir, jointDir) : Quaternion.identity;

        if (!flip)
        {
            Vector3 mirroredAngles = jointRotation.eulerAngles;
            mirroredAngles.y = -mirroredAngles.y;
            mirroredAngles.z = -mirroredAngles.z;

            jointRotation = Quaternion.Euler(mirroredAngles);
        }

        if (jointRotation != Quaternion.identity)
        {
            // Smoothly transition to the new rotation
            Quaternion newRotation = Kinect2AvatarRot(jointRotation, boneIndex);

            if (externalRootMotion)
            {
                newRotation = transform.rotation * newRotation;
            }

            if (smoothFactor != 0f)
                boneTransform.rotation = Quaternion.Slerp(boneTransform.rotation, newRotation, smoothFactor * Time.deltaTime);
            else
                boneTransform.rotation = newRotation;
        }

    }

    public bool IsJointTracked(int joint)
    {

        if (joint >= 0 && joint < (int)JointType.Count)
        {
            return joints[joint].TrackingState == TrackingState.Tracked;
        }


        return false;
    }
    /// <summary>
    /// Gets the joint orientation of the specified user.
    /// </summary>
    /// <returns>The joint rotation.</returns>
    /// <param name="userId">User ID</param>
    /// <param name="joint">Joint index</param>
    /// <param name="flip">If set to <c>true</c>, this means non-mirrored rotation</param>
    public Quaternion GetJointOrientation(int joint, bool flip)
    {
        if (joint >= 0 && joint < (int)JointType.Count)
        {
            if (flip)
            {
                return joints[joint].normalRotation;
            }
            else
            {
                return joints[joint].mirroredRotation;
            }
        }
        return Quaternion.identity;
    }

    public Vector3 GetJointDirection(int joint, int jointParent, bool flipX, bool flipZ)
    {
        if (joint >= 0 && joint < (int)JointType.Count && jointParent >= 0 && jointParent < (int)JointType.Count)
        {
            Vector3 jointDir = joints[joint].direction;

            if (flipX)
                jointDir.x = -jointDir.x;

            if (flipZ)
                jointDir.z = -jointDir.z;

            return jointDir;
        }


        return Vector3.zero;
    }

    protected Quaternion Kinect2AvatarRot(Quaternion jointRotation, int boneIndex)
    {
        Quaternion newRotation = jointRotation * initialRotations[boneIndex];
        //newRotation = initialRotation * newRotation;

        if (offsetNode != null)
        {
            newRotation = offsetNode.transform.rotation * newRotation;
        }
        else
        {
            newRotation = initialRotation * newRotation;
        }

        return newRotation;
    }
    //====================================接收数据=======================================//

    private static void BytesToStruct(byte[] bytes, ref controlParam[] control_params)
    {
        for (int i = 0; i < controlParamCount; ++i)
        {
            if (bytes == null) return;
            if (bytes.Length <= 0) return;
            int objLength = Marshal.SizeOf(typeof(controlParam));
            //Debug.Log(objLength);
            IntPtr bufferPtr = Marshal.AllocHGlobal(objLength);
            try//struct_bytes转换
            {
                Marshal.Copy(bytes, i * objLength, bufferPtr, objLength);
                control_params[i] = (controlParam)Marshal.PtrToStructure(bufferPtr, typeof(controlParam));
            }
            catch (Exception ex)
            {
                throw new Exception("Error in BytesToStruct ! " + ex.Message);
            }
            finally
            {
                Marshal.FreeHGlobal(bufferPtr);
            }
        }
    }
    void ReceiveData()
    {

        while (true)
        {
            // receiveBytes = udpClient.Receive(ref ep);
            // print("message from: " + ep.ToString());
            //returnData = Encoding.ASCII.GetString(receiveBytes);
            // print(returnData);

            receiveBytes = udpClient.Receive(ref ep);
            print("message from: " + ep.ToString());
            BytesToStruct(receiveBytes, ref control_params);
            //runMode = (char)receiveBytes[1023];
            //MouthUp = ChangeFormat(receiveBytes);
            // print(Arr_Pts[36]);
        }
    }
    //====================================更新Unity3D数据=======================================//
    // Use this for initialization
    void Start()
    {
        //绑定面部对象
        skinnedMesh = skinnedMeshRenderer.sharedMesh;
        blendSpaceCount = skinnedMesh.blendShapeCount;

        receiveThread = new Thread(ReceiveData);
        receiveThread.IsBackground = true;
        receiveThread.Start();

        control_params = new controlParam[controlParamCount];

        //create the transform matrix - kinect to world
        Quaternion quatTiltAngle = Quaternion.Euler(-sensorAngle, 0.0f, 0.0f);
        kinectToWorld.SetTRS(new Vector3(0.0f, sensorHeight, 0.0f), quatTiltAngle, Vector3.one);

        joints = new JointData[(int)JointType.Count];

        // check for double start
        if (bones != null)
            return;
        if (!gameObject.activeInHierarchy)
            return;

        // Set model's arms to be in T-pose, if needed
        SetModelArmsInTpose();

        // inits the bones array
        bones = new Transform[31];

        // Initial rotations and directions of the bones.
        initialRotations = new Quaternion[bones.Length];

        // Map bones to the points the Kinect tracks
        MapBones();

        // Get initial bone rotations
        GetInitialRotations();

        // if parent transform uses physics
        isRigidBody = gameObject.GetComponent<Rigidbody>();
    }

    // Update is called once per frame
    void Update()
    {
        ////////////////////////////////////////////////////////////////////////////////////////////////
        //////////////////////////////////////////表情随动//////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////////////////////////////////////
        calcuBlendShapeWeight();

        blink= control_params[auParamsCount + headRotationParamsCount].faceControlParam;

        leftEyeState.horizontalState = control_params[auParamsCount + headRotationParamsCount + 1].faceControlParam;
        leftEyeState.verticalState = control_params[auParamsCount + headRotationParamsCount + 2].faceControlParam;
        rightEyeState.horizontalState = control_params[auParamsCount + headRotationParamsCount + 3].faceControlParam;
        rightEyeState.verticalState = control_params[auParamsCount + headRotationParamsCount + 4].faceControlParam;


        faceQuaternion.w = control_params[auParamsCount].faceControlParam;
        faceQuaternion.x = -control_params[auParamsCount + 1].faceControlParam;
        faceQuaternion.y = control_params[auParamsCount + 2].faceControlParam;
        faceQuaternion.z = -control_params[auParamsCount + 3].faceControlParam;

        if (blendSpaceCount > 0 && runMode == 'w')
        {
            skinnedMeshRenderer.SetBlendShapeWeight((int)FaceBlendShapeIndex.MouthOpen, JawDownWeight);

            //skinnedMeshRenderer.SetBlendShapeWeight((int)FaceBlendShapeIndex.BrowsOuterLowerLeft, BrowsOuterLowerWeight);
            //skinnedMeshRenderer.SetBlendShapeWeight((int)FaceBlendShapeIndex.BrowsOuterLowerRight, BrowsOuterLowerWeight);

            skinnedMeshRenderer.SetBlendShapeWeight((int)FaceBlendShapeIndex.BrowsUpLeft, BrowsUpWeight);
            skinnedMeshRenderer.SetBlendShapeWeight((int)FaceBlendShapeIndex.BrowsUpRight, BrowsUpWeight);

            skinnedMeshRenderer.SetBlendShapeWeight((int)FaceBlendShapeIndex.BrowsDownLeft, BrowsDownWeight);
            skinnedMeshRenderer.SetBlendShapeWeight((int)FaceBlendShapeIndex.BrowsDownRight, BrowsDownWeight);

            skinnedMeshRenderer.SetBlendShapeWeight((int)FaceBlendShapeIndex.EyeWideLeft, EyeWideWeight);
            skinnedMeshRenderer.SetBlendShapeWeight((int)FaceBlendShapeIndex.EyeWideRight, EyeWideWeight);

            skinnedMeshRenderer.SetBlendShapeWeight((int)FaceBlendShapeIndex.SquintLeft, SquintWeight);
            skinnedMeshRenderer.SetBlendShapeWeight((int)FaceBlendShapeIndex.SquintRight, SquintWeight);

            skinnedMeshRenderer.SetBlendShapeWeight((int)FaceBlendShapeIndex.NoseScrunchLeft, NoseScrunchWeight);
            skinnedMeshRenderer.SetBlendShapeWeight((int)FaceBlendShapeIndex.NoseScrunchRight, NoseScrunchWeight);

            //skinnedMeshRenderer.SetBlendShapeWeight((int)FaceBlendShapeIndex.UpperLipUpLeft, UpperLipUpWeight);
            //skinnedMeshRenderer.SetBlendShapeWeight((int)FaceBlendShapeIndex.UpperLipUpRight, UpperLipUpWeight);

            skinnedMeshRenderer.SetBlendShapeWeight((int)FaceBlendShapeIndex.SmileLeft, LipCornerPullerWeight);
            skinnedMeshRenderer.SetBlendShapeWeight((int)FaceBlendShapeIndex.SmileRight, LipCornerPullerWeight);
            //LipCornerPullerWeight;
            //DimplerWeight;

            skinnedMeshRenderer.SetBlendShapeWeight((int)FaceBlendShapeIndex.FrownLeft, LowerLipDownWeight);
            skinnedMeshRenderer.SetBlendShapeWeight((int)FaceBlendShapeIndex.FrownRight, LowerLipDownWeight);

            skinnedMeshRenderer.SetBlendShapeWeight((int)FaceBlendShapeIndex.BlinkLeft, blink);
            skinnedMeshRenderer.SetBlendShapeWeight((int)FaceBlendShapeIndex.BlinkRight, blink);
            //ChinRaiserWeight;
            //LipStretcherWeight;
            //MouthOpenWeight;
            //JawDownWeight;
        }

        //if (LeftEyeTransform)
        //{
        //    LeftEyeTransform.rotation = Quaternion.Euler((float)leftEyeState.verticalState, (float)leftEyeState.horizontalState, 0);
        //}
        //if (RightEyeTransform)
        //{
        //    RightEyeTransform.rotation = Quaternion.Euler((float)rightEyeState.verticalState, (float)rightEyeState.horizontalState, 0);
        //}

        if (HeadJoint && runMode == 'w')
        {
            //if (faceQuaternion.eulerAngles.x * 180 / 3.14 < 30 &&
            //    faceQuaternion.eulerAngles.y * 180 / 3.14 < 30 &&
            //    faceQuaternion.eulerAngles.z * 180 / 3.14 < 30)
            //{
                HeadJoint.rotation = Quaternion.Slerp(HeadJoint.rotation, faceQuaternion, Time.deltaTime * 10);
            //}
        }
        ////////////////////////////////////////////////////////////////////////////////////////////////
        /////////////////////////////////////////肢体随动///////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////////////////////////////////////
        if (runMode == 's')
        {
            for (int i = 0; i < (int)JointType.Count; ++i)
            {
                joints[i].kinectPos = new Vector3(control_params[i].jointData.Position.X, control_params[i].jointData.Position.Y, control_params[i].jointData.Position.Z);
                joints[i].position = kinectToWorld.MultiplyPoint3x4(joints[i].kinectPos);
                joints[i].TrackingState = control_params[i].jointData.TrackingState;
                joints[i].JointType = control_params[i].jointData.JointType;
            }
            //计算骨骼方向
            CalcBodyFrameBoneDirs();
            //处理特定骨骼点丢失情况
            ProcessBodySpecialData(ref joints);
            //计算骨骼节点旋转方向
            CalculateJointOrients(ref joints);

            SuccessfulCalibration();

            if (!externalRootMotion)
            {
                MoveAvatar();
            }

            // rotate the avatar bones
            for (var boneIndex = 0; boneIndex < bones.Length; boneIndex++)
            {
                if (!bones[boneIndex])
                    continue;

                if (boneIndex2JointMap.ContainsKey(boneIndex))
                {
                    JointType joint = !mirroredMovement ? boneIndex2JointMap[boneIndex] : boneIndex2MirrorJointMap[boneIndex];
                    TransformBone(joint, boneIndex, !mirroredMovement);
                }
                else if (specIndex2JointMap.ContainsKey(boneIndex))
                {
                    // special bones (clavicles)
                    List<JointType> alJoints = !mirroredMovement ? specIndex2JointMap[boneIndex] : specIndex2MirrorMap[boneIndex];

                    if (alJoints.Count >= 2)
                    {
                        //Debug.Log(alJoints[0].ToString());
                        Vector3 baseDir = alJoints[0].ToString().EndsWith("Left") ? Vector3.left : Vector3.right;
                        TransformSpecialBone(alJoints[0], alJoints[1], boneIndex, baseDir, !mirroredMovement);
                    }
                }
            }

            //bind hand to the ball
            if (ballPosition)
            {
                ballPosition.position = bones[(int)JointType.HandLeft].position;
            }
        }
    }

    void OnDestroy()
    {
        if (receiveThread != null)
            receiveThread.Abort();

        udpClient.Close();
    }



    //Unity character blendshape species enumerate
    public enum FaceBlendShapeIndex
    {
        MouthOpen = 0,
        MouthUp = 1,
        MouthDown = 2,
        SmileLeft = 3,
        SmileRight = 4,
        FrownLeft = 5,
        FrownRight = 6,
        UpperLipUpLeft = 7,
        UpperLipUpRight = 8,
        LowerLipDownLeft = 9,
        LowerLipDownRight = 10,
        MouthNarrowLeft = 11,
        MouthNarrowRight = 12,
        SquintLeft = 13,
        SquintRight = 14,
        EyeWideLeft = 15,
        EyeWideRight = 16,
        BlinkLeft = 17,
        BlinkRight = 18,
        NoseScrunchLeft = 19,
        NoseScrunchRight = 20,
        BrowsDownLeft = 21,
        BrowsDownRight = 22,
        BrowsUpLeft = 23,
        BrowsUpRight = 24,
        BrowsInLeft = 25,
        BrowsInRight = 26,
        BrowsOuterLowerLeft = 27,
        BrowsOuterLowerRight = 28,
        MidMouthLeft = 29,
        MidNouthRight = 30,
        UpperLipIn = 31,
        UpperLipOut = 32,
        LowerLipIn = 33,
        LowerLipOut = 34,
        CheekPuffLeft = 35,
        CheekPuffRight = 36,
        OFFSETMouthClosedShape = 37,
        Count = 38
    }

    public enum JointType
    {
        SpineBase = 0,
        SpineMid = 1,
        Neck = 2,
        Head = 3,
        ShoulderLeft = 4,
        ElbowLeft = 5,
        WristLeft = 6,
        HandLeft = 7,
        ShoulderRight = 8,
        ElbowRight = 9,
        WristRight = 10,
        HandRight = 11,
        HipLeft = 12,
        KneeLeft = 13,
        AnkleLeft = 14,
        FootLeft = 15,
        HipRight = 16,
        KneeRight = 17,
        AnkleRight = 18,
        FootRight = 19,
        SpineShoulder = 20,
        HandTipLeft = 21,
        ThumbLeft = 22,
        HandTipRight = 23,
        ThumbRight = 24,
        Count = (ThumbRight + 1)
    };
    [StructLayout(LayoutKind.Sequential, Pack = 1)]  //变量在内存中的对齐方式 ，每个结构体都需要
    public struct CameraSpacePoint
    {
        public float X;
        public float Y;
        public float Z;
    }
    [StructLayout(LayoutKind.Sequential, Pack = 1)]  //变量在内存中的对齐方式 ，每个结构体都需要
    public struct Vector4
    {
        public float x;
        public float y;
        public float z;
        public float w;
    }
    public enum TrackingState
    {
        NotTracked = 0,
        Inferred = 1,
        Tracked = 2
    };
    [StructLayout(LayoutKind.Sequential, Pack = 1)]  //变量在内存中的对齐方式 ，每个结构体都需要
    public struct JointData
    {
        public JointType JointType;
        public Vector3 kinectPos;
        public Vector3 position;
        public TrackingState TrackingState;
        public Vector4 orientation;
        public Vector3 direction;
        public Quaternion normalRotation;
        public Quaternion mirroredRotation;
    };
    [StructLayout(LayoutKind.Sequential, Pack = 1)]  //变量在内存中的对齐方式 ，每个结构体都需要
    public struct KinectJointData
    {
        public JointType JointType;
        public CameraSpacePoint Position;
        public TrackingState TrackingState;
        //public Vector4 Orientation;
    }
    [StructLayout(LayoutKind.Sequential, Pack = 1)]  //变量在内存中的对齐方式 ，每个结构体都需要
    public struct eyeState
    {
        public float horizontalState;
        public float verticalState;
    };
    [StructLayout(LayoutKind.Sequential, Pack = 1)]  //变量在内存中的对齐方式 ，每个结构体都需要
    public struct controlParam
    {
        //骨骼节点数据
        public KinectJointData jointData;
        // 控制参数
        public float faceControlParam;
    }

    public static readonly Vector3[] JointBaseDir =
{
        Vector3.zero,
        Vector3.up,
        Vector3.up,
        Vector3.up,
        Vector3.left,
        Vector3.left,
        Vector3.left,
        Vector3.left,
        Vector3.right,
        Vector3.right,
        Vector3.right,
        Vector3.right,
        Vector3.down,
        Vector3.down,
        Vector3.down,
        Vector3.forward,
        Vector3.down,
        Vector3.down,
        Vector3.down,
        Vector3.forward,
        Vector3.up,
        Vector3.left,
        Vector3.forward,
        Vector3.right,
        Vector3.forward
    };

    private readonly Dictionary<int, HumanBodyBones> boneIndex2MecanimMap = new Dictionary<int, HumanBodyBones>
    {
        {0, HumanBodyBones.Hips},
        {1, HumanBodyBones.Spine},
//        {2, HumanBodyBones.Chest},
		{3, HumanBodyBones.Neck},
//		{4, HumanBodyBones.Head},
		
		{5, HumanBodyBones.LeftUpperArm},
        {6, HumanBodyBones.LeftLowerArm},
        {7, HumanBodyBones.LeftHand},
//		{8, HumanBodyBones.LeftIndexProximal},
//		{9, HumanBodyBones.LeftIndexIntermediate},
//		{10, HumanBodyBones.LeftThumbProximal},
		
		{11, HumanBodyBones.RightUpperArm},
        {12, HumanBodyBones.RightLowerArm},
        {13, HumanBodyBones.RightHand},
//		{14, HumanBodyBones.RightIndexProximal},
//		{15, HumanBodyBones.RightIndexIntermediate},
//		{16, HumanBodyBones.RightThumbProximal},
		
		{17, HumanBodyBones.LeftUpperLeg},
        {18, HumanBodyBones.LeftLowerLeg},
        {19, HumanBodyBones.LeftFoot},
//		{20, HumanBodyBones.LeftToes},
		
		{21, HumanBodyBones.RightUpperLeg},
        {22, HumanBodyBones.RightLowerLeg},
        {23, HumanBodyBones.RightFoot},
//		{24, HumanBodyBones.RightToes},
		
		{25, HumanBodyBones.LeftShoulder},
        {26, HumanBodyBones.RightShoulder},
        {27, HumanBodyBones.LeftIndexProximal},
        {28, HumanBodyBones.RightIndexProximal},
        {29, HumanBodyBones.LeftThumbProximal},
        {30, HumanBodyBones.RightThumbProximal},
    };

    protected readonly Dictionary<int, JointType> boneIndex2JointMap = new Dictionary<int, JointType>
    {
        {0, JointType.SpineBase},
        {1, JointType.SpineMid},
        {2, JointType.SpineShoulder},
        {3, JointType.Neck},
        {4, JointType.Head},

        {5, JointType.ShoulderLeft},
        {6, JointType.ElbowLeft},
        {7, JointType.WristLeft},
        {8, JointType.HandLeft},

        {9, JointType.HandTipLeft},
        {10, JointType.ThumbLeft},

        {11, JointType.ShoulderRight},
        {12, JointType.ElbowRight},
        {13, JointType.WristRight},
        {14, JointType.HandRight},

        {15, JointType.HandTipRight},
        {16, JointType.ThumbRight},

        {17, JointType.HipLeft},
        {18, JointType.KneeLeft},
        {19, JointType.AnkleLeft},
        {20, JointType.FootLeft},

        {21, JointType.HipRight},
        {22, JointType.KneeRight},
        {23, JointType.AnkleRight},
        {24, JointType.FootRight},
    };

    protected readonly Dictionary<int, List<JointType>> specIndex2JointMap = new Dictionary<int, List<JointType>>
    {
        {25, new List<JointType> {JointType.ShoulderLeft, JointType.SpineShoulder} },
        {26, new List<JointType> {JointType.ShoulderRight, JointType.SpineShoulder} },
        {27, new List<JointType> {JointType.HandTipLeft, JointType.HandLeft} },
        {28, new List<JointType> {JointType.HandTipRight, JointType.HandRight} },
        {29, new List<JointType> {JointType.ThumbLeft, JointType.HandLeft} },
        {30, new List<JointType> {JointType.ThumbRight, JointType.HandRight} },
    };

    protected readonly Dictionary<int, JointType> boneIndex2MirrorJointMap = new Dictionary<int, JointType>
    {
        {0, JointType.SpineBase},
        {1, JointType.SpineMid},
        {2, JointType.SpineShoulder},
        {3, JointType.Neck},
        {4, JointType.Head},

        {5, JointType.ShoulderRight},
        {6, JointType.ElbowRight},
        {7, JointType.WristRight},
        {8, JointType.HandRight},

        {9, JointType.HandTipRight},
        {10, JointType.ThumbRight},

        {11, JointType.ShoulderLeft},
        {12, JointType.ElbowLeft},
        {13, JointType.WristLeft},
        {14, JointType.HandLeft},

        {15, JointType.HandTipLeft},
        {16, JointType.ThumbLeft},

        {17, JointType.HipRight},
        {18, JointType.KneeRight},
        {19, JointType.AnkleRight},
        {20, JointType.FootRight},

        {21, JointType.HipLeft},
        {22, JointType.KneeLeft},
        {23, JointType.AnkleLeft},
        {24, JointType.FootLeft},
    };

    protected readonly Dictionary<int, List<JointType>> specIndex2MirrorMap = new Dictionary<int, List<JointType>>
    {
        {25, new List<JointType> {JointType.ShoulderRight, JointType.SpineShoulder} },
        {26, new List<JointType> {JointType.ShoulderLeft, JointType.SpineShoulder} },
        {27, new List<JointType> {JointType.HandTipRight, JointType.HandRight} },
        {28, new List<JointType> {JointType.HandTipLeft, JointType.HandLeft} },
        {29, new List<JointType> {JointType.ThumbRight, JointType.HandRight} },
        {30, new List<JointType> {JointType.ThumbLeft, JointType.HandLeft} },
    };

    protected readonly Dictionary<JointType, int> jointMap2boneIndex = new Dictionary<JointType, int>
    {
        {JointType.SpineBase, 0},
        {JointType.SpineMid, 1},
        {JointType.SpineShoulder, 2},
        {JointType.Neck, 3},
        {JointType.Head, 4},

        {JointType.ShoulderLeft, 5},
        {JointType.ElbowLeft, 6},
        {JointType.WristLeft, 7},
        {JointType.HandLeft, 8},

        {JointType.HandTipLeft, 9},
        {JointType.ThumbLeft, 10},

        {JointType.ShoulderRight, 11},
        {JointType.ElbowRight, 12},
        {JointType.WristRight, 13},
        {JointType.HandRight, 14},

        {JointType.HandTipRight, 15},
        {JointType.ThumbRight, 16},

        {JointType.HipLeft, 17},
        {JointType.KneeLeft, 18},
        {JointType.AnkleLeft, 19},
        {JointType.FootLeft, 20},

        {JointType.HipRight, 21},
        {JointType.KneeRight, 22},
        {JointType.AnkleRight, 23},
        {JointType.FootRight, 24},
    };

    protected readonly Dictionary<JointType, int> mirrorJointMap2boneIndex = new Dictionary<JointType, int>
    {
        {JointType.SpineBase, 0},
        {JointType.SpineMid, 1},
        {JointType.SpineShoulder, 2},
        {JointType.Neck, 3},
        {JointType.Head, 4},

        {JointType.ShoulderRight, 5},
        {JointType.ElbowRight, 6},
        {JointType.WristRight, 7},
        {JointType.HandRight, 8},

        {JointType.HandTipRight, 9},
        {JointType.ThumbRight, 10},

        {JointType.ShoulderLeft, 11},
        {JointType.ElbowLeft, 12},
        {JointType.WristLeft, 13},
        {JointType.HandLeft, 14},

        {JointType.HandTipLeft, 15},
        {JointType.ThumbLeft, 16},

        {JointType.HipRight, 17},
        {JointType.KneeRight, 18},
        {JointType.AnkleRight, 19},
        {JointType.FootRight, 20},

        {JointType.HipLeft, 21},
        {JointType.KneeLeft, 22},
        {JointType.AnkleLeft, 23},
        {JointType.FootLeft, 24},
    };


    private readonly Dictionary<int, List<HumanBodyBones>> specialIndex2MultiBoneMap = new Dictionary<int, List<HumanBodyBones>>
    {
        {27, new List<HumanBodyBones> {  // left fingers
				HumanBodyBones.LeftIndexProximal,
                HumanBodyBones.LeftIndexIntermediate,
                HumanBodyBones.LeftIndexDistal,
                HumanBodyBones.LeftMiddleProximal,
                HumanBodyBones.LeftMiddleIntermediate,
                HumanBodyBones.LeftMiddleDistal,
                HumanBodyBones.LeftRingProximal,
                HumanBodyBones.LeftRingIntermediate,
                HumanBodyBones.LeftRingDistal,
                HumanBodyBones.LeftLittleProximal,
                HumanBodyBones.LeftLittleIntermediate,
                HumanBodyBones.LeftLittleDistal,
            }},
        {28, new List<HumanBodyBones> {  // right fingers
				HumanBodyBones.RightIndexProximal,
                HumanBodyBones.RightIndexIntermediate,
                HumanBodyBones.RightIndexDistal,
                HumanBodyBones.RightMiddleProximal,
                HumanBodyBones.RightMiddleIntermediate,
                HumanBodyBones.RightMiddleDistal,
                HumanBodyBones.RightRingProximal,
                HumanBodyBones.RightRingIntermediate,
                HumanBodyBones.RightRingDistal,
                HumanBodyBones.RightLittleProximal,
                HumanBodyBones.RightLittleIntermediate,
                HumanBodyBones.RightLittleDistal,
            }},
        {29, new List<HumanBodyBones> {  // left thumb
				HumanBodyBones.LeftThumbProximal,
                HumanBodyBones.LeftThumbIntermediate,
                HumanBodyBones.LeftThumbDistal,
            }},
        {30, new List<HumanBodyBones> {  // right thumb
				HumanBodyBones.RightThumbProximal,
                HumanBodyBones.RightThumbIntermediate,
                HumanBodyBones.RightThumbDistal,
            }},
    };
}
