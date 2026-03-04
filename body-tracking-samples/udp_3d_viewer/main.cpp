// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include <array>
#include <iostream>
#include <map>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <ctime>

#include <winsock2.h>
#include <ws2tcpip.h>
#include <k4arecord/playback.h>
#include <k4a/k4a.h>
#include <k4abt.h>

#include <BodyTrackingHelpers.h>
#include <Utilities.h>
#include <Window3dWrapper.h>

#include "gui/GuiWindow.h"

void PrintUsage()
{
#ifdef _WIN32
    printf("\nUSAGE: (k4abt_)simple_3d_viewer.exe SensorMode[NFOV_UNBINNED, WFOV_BINNED](optional) RuntimeMode[CPU, CUDA, DIRECTML, TENSORRT](optional) -model MODEL_PATH(optional)\n");
#else
    printf("\nUSAGE: (k4abt_)simple_3d_viewer.exe SensorMode[NFOV_UNBINNED, WFOV_BINNED](optional) RuntimeMode[CPU, CUDA, TENSORRT](optional)\n");
#endif
    printf("  - SensorMode: \n");
    printf("      NFOV_UNBINNED (default) - Narrow Field of View Unbinned Mode [Resolution: 640x576; FOI: 75 degree x 65 degree]\n");
    printf("      WFOV_BINNED             - Wide Field of View Binned Mode [Resolution: 512x512; FOI: 120 degree x 120 degree]\n");
    printf("  - RuntimeMode: \n");
    printf("      CPU - Use the CPU only mode. It runs on machines without a GPU but it will be much slower\n");
    printf("      CUDA - Use CUDA for processing.\n");
#ifdef _WIN32
    printf("      DIRECTML - Use the DirectML processing mode.\n");
#endif
    printf("      TENSORRT - Use the TensorRT processing mode.\n");
    printf("      OFFLINE - Play a specified file. Does not require Kinect device\n");
    printf("e.g.   (k4abt_)simple_3d_viewer.exe WFOV_BINNED CPU\n");
    printf("e.g.   (k4abt_)simple_3d_viewer.exe CPU\n");
    printf("e.g.   (k4abt_)simple_3d_viewer.exe WFOV_BINNED\n");
    printf("e.g.   (k4abt_)simple_3d_viewer.exe OFFLINE MyFile.mkv\n");
}

void PrintAppUsage()
{
    printf("\n");
    printf(" Basic Navigation:\n\n");
    printf(" Rotate: Rotate the camera by moving the mouse while holding mouse left button\n");
    printf(" Pan: Translate the scene by holding Ctrl key and drag the scene with mouse left button\n");
    printf(" Zoom in/out: Move closer/farther away from the scene center by scrolling the mouse scroll wheel\n");
    printf(" Select Center: Center the scene based on a detected joint by right clicking the joint with mouse\n");
    printf("\n");
    printf(" Key Shortcuts\n\n");
    printf(" ESC: quit\n");
    printf(" h: help\n");
    printf(" b: body visualization mode\n");
    printf(" k: 3d window layout\n");
    printf("\n");
}

// Global State and Key Process Function
bool s_isRunning = true;
Visualization::Layout3d s_layoutMode = Visualization::Layout3d::OnlyMainView;
bool s_visualizeJointFrame = false;

namespace
{
    constexpr uint16_t kUdpPort = 7000;

    struct Quaternion
    {
        float w;
        float x;
        float y;
        float z;
    };

    Quaternion ToQuaternion(const k4a_quaternion_t& q)
    {
        return { q.wxyz.w, q.wxyz.x, q.wxyz.y, q.wxyz.z };
    }

    Quaternion Inverse(const Quaternion& q)
    {
        float norm2 = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
        if (norm2 <= 0.f)
        {
            return { 1.f, 0.f, 0.f, 0.f };
        }
        float inv = 1.f / norm2;
        return { q.w * inv, -q.x * inv, -q.y * inv, -q.z * inv };
    }

    Quaternion Multiply(const Quaternion& a, const Quaternion& b)
    {
        return {
            a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
            a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w
        };
    }

    class UdpSender
    {
    public:
        ~UdpSender()
        {
            Stop();
            if (m_wsaStarted)
            {
                WSACleanup();
            }
        }

        bool Start(const std::string& ipAddress, uint16_t port)
        {
            Stop();

            if (!m_wsaStarted)
            {
                WSADATA wsaData = {};
                if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0)
                {
                    return false;
                }
                m_wsaStarted = true;
            }

            m_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
            if (m_socket == INVALID_SOCKET)
            {
                return false;
            }

            sockaddr_in addr = {};
            addr.sin_family = AF_INET;
            addr.sin_port = htons(port);
            if (inet_pton(AF_INET, ipAddress.c_str(), &addr.sin_addr) != 1)
            {
                Stop();
                return false;
            }

            m_destAddr = addr;
            return true;
        }

        void Stop()
        {
            if (m_socket != INVALID_SOCKET)
            {
                closesocket(m_socket);
                m_socket = INVALID_SOCKET;
            }
        }

        bool IsStreaming() const
        {
            return m_socket != INVALID_SOCKET;
        }

        bool Send(const float* data, size_t floatCount)
        {
            if (m_socket == INVALID_SOCKET || data == nullptr || floatCount == 0)
            {
                return false;
            }

            const int byteCount = static_cast<int>(floatCount * sizeof(float));
            int result = sendto(
                m_socket,
                reinterpret_cast<const char*>(data),
                byteCount,
                0,
                reinterpret_cast<const sockaddr*>(&m_destAddr),
                sizeof(m_destAddr));
            return result != SOCKET_ERROR;
        }

    private:
        SOCKET m_socket = INVALID_SOCKET;
        sockaddr_in m_destAddr = {};
        bool m_wsaStarted = false;
    };

    // Global variable to store current body frame for export
    k4abt_body_t s_currentBody = {};
    bool s_hasCurrentBody = false;

    // Get local position relative to parent
    k4a_float3_t GetLocalPosition(const k4a_float3_t& jointPos, const k4a_float3_t& parentPos, k4abt_joint_id_t parentId)
    {
        if (parentId == K4ABT_JOINT_COUNT)
        {
            // Root joint (Pelvis) - local position is same as world position
            return jointPos;
        }
        
        // Local position = joint position - parent position
        k4a_float3_t localPos;
        localPos.xyz.x = jointPos.xyz.x - parentPos.xyz.x;
        localPos.xyz.y = jointPos.xyz.y - parentPos.xyz.y;
        localPos.xyz.z = jointPos.xyz.z - parentPos.xyz.z;
        return localPos;
    }

    // Export hierarchy to CSV
    void ExportHierarchy(const k4abt_body_t& body, const std::map<k4abt_joint_id_t, k4abt_joint_id_t>& parentMap)
    {
        // Generate filename with timestamp
        std::time_t now = std::time(nullptr);
        std::tm timeInfo;
        localtime_s(&timeInfo, &now);
        
        std::ostringstream filename;
        filename << "hierarchy_export_"
                 << std::setfill('0') << std::setw(4) << (timeInfo.tm_year + 1900)
                 << std::setw(2) << (timeInfo.tm_mon + 1)
                 << std::setw(2) << timeInfo.tm_mday << "_"
                 << std::setw(2) << timeInfo.tm_hour
                 << std::setw(2) << timeInfo.tm_min
                 << std::setw(2) << timeInfo.tm_sec
                 << ".csv";
        
        std::ofstream file(filename.str());
        if (!file.is_open())
        {
            std::cerr << "Failed to create export file: " << filename.str() << std::endl;
            return;
        }

        // Write CSV header
        file << "joint_index,joint_id,joint_name,parent_index,parent_name,local_pos_x,local_pos_y,local_pos_z,local_rot_x,local_rot_y,local_rot_z,local_rot_w,tracked\n";

        // Calculate local rotations (relative to pelvis root)
        Quaternion root = ToQuaternion(body.skeleton.joints[K4ABT_JOINT_PELVIS].orientation);
        Quaternion invRoot = Inverse(root);

        // Export each joint
        for (int jointIdx = 0; jointIdx < static_cast<int>(K4ABT_JOINT_COUNT); jointIdx++)
        {
            k4abt_joint_id_t jointId = static_cast<k4abt_joint_id_t>(jointIdx);
            const auto& joint = body.skeleton.joints[jointIdx];
            
            // Get parent info
            k4abt_joint_id_t parentId = parentMap.count(jointId) ? parentMap.at(jointId) : K4ABT_JOINT_COUNT;
            int parentIndex = (parentId == K4ABT_JOINT_COUNT) ? -1 : static_cast<int>(parentId);
            
            // Get joint name
            std::string jointName = g_jointNames.count(jointId) ? g_jointNames.at(jointId) : "UNKNOWN";
            std::string parentName = (parentId == K4ABT_JOINT_COUNT) ? "" : 
                                     (g_jointNames.count(parentId) ? g_jointNames.at(parentId) : "UNKNOWN");
            
            // Calculate local position
            k4a_float3_t localPos;
            if (parentId == K4ABT_JOINT_COUNT)
            {
                // Root joint - local position is world position
                localPos = joint.position;
            }
            else
            {
                const auto& parentPos = body.skeleton.joints[parentId].position;
                localPos = GetLocalPosition(joint.position, parentPos, parentId);
            }
            
            // Calculate local rotation (relative to parent, or pelvis root if no parent)
            Quaternion worldRot = ToQuaternion(joint.orientation);
            Quaternion localRot;
            if (parentId == K4ABT_JOINT_COUNT)
            {
                // Root joint - local rotation is world rotation
                localRot = worldRot;
            }
            else
            {
                // Calculate rotation relative to parent
                Quaternion parentRot = ToQuaternion(body.skeleton.joints[parentId].orientation);
                Quaternion invParentRot = Inverse(parentRot);
                localRot = Multiply(invParentRot, worldRot);
            }
            
            // Check if joint is tracked
            int tracked = (joint.confidence_level >= K4ABT_JOINT_CONFIDENCE_LOW) ? 1 : 0;
            
            // Write CSV row
            file << jointIdx << ","
                 << static_cast<int>(jointId) << ","
                 << jointName << ","
                 << parentIndex << ","
                 << parentName << ","
                 << std::fixed << std::setprecision(6)
                 << localPos.xyz.x << ","
                 << localPos.xyz.y << ","
                 << localPos.xyz.z << ","
                 << localRot.x << ","
                 << localRot.y << ","
                 << localRot.z << ","
                 << localRot.w << ","
                 << tracked << "\n";
        }
        
        file.close();
        std::cout << "Hierarchy exported to: " << filename.str() << std::endl;
    }

    void UpdateGuiAndStreaming(GuiWindow& gui, UdpSender& sender, const std::map<k4abt_joint_id_t, k4abt_joint_id_t>& parentMap)
    {
        gui.PumpMessages();
        if (gui.ConsumeToggleRequest())
        {
            if (sender.IsStreaming())
            {
                sender.Stop();
            }
            else
            {
                sender.Start(gui.GetIpAddress(), kUdpPort);
            }
            gui.SetStreamingState(sender.IsStreaming());
        }
        
        if (gui.ConsumeExportRequest())
        {
            if (s_hasCurrentBody)
            {
                ExportHierarchy(s_currentBody, parentMap);
            }
            else
            {
                std::cout << "No body data available to export. Please wait for body tracking data." << std::endl;
            }
        }
    }

    // Pack 7 floats per joint: Position (X,Y,Z) + Rotation (W,X,Y,Z)
    // Plus 4 floats for pelvis world rotation at the end.
    // Total payload: 32 joints * 7 floats + 4 = 228 floats = 912 bytes
    void PackJointData(const k4abt_body_t& body, std::array<float, K4ABT_JOINT_COUNT * 7 + 4>& out)
    {
        Quaternion root = ToQuaternion(body.skeleton.joints[K4ABT_JOINT_PELVIS].orientation);
        Quaternion invRoot = Inverse(root);

        for (int joint = 0; joint < static_cast<int>(K4ABT_JOINT_COUNT); joint++)
        {
            const k4a_float3_t& pos = body.skeleton.joints[joint].position;
            Quaternion world = ToQuaternion(body.skeleton.joints[joint].orientation);
            Quaternion local = Multiply(invRoot, world);

            size_t base = static_cast<size_t>(joint) * 7;
            // Position
            out[base + 0] = pos.xyz.x;
            out[base + 1] = pos.xyz.y;
            out[base + 2] = pos.xyz.z;
            // Rotation (WXYZ)
            out[base + 3] = local.w;
            out[base + 4] = local.x;
            out[base + 5] = local.y;
            out[base + 6] = local.z;
        }

        // Append pelvis world rotation (WXYZ) after the 32 joints
        size_t pelvisRotBase = static_cast<size_t>(K4ABT_JOINT_COUNT) * 7;
        out[pelvisRotBase + 0] = root.w;
        out[pelvisRotBase + 1] = root.x;
        out[pelvisRotBase + 2] = root.y;
        out[pelvisRotBase + 3] = root.z;
    }

    // Build parent joint map from bone list
    // Note: The bone list connects joints, but we need to determine parent-child relationships
    // Based on Kinect skeleton hierarchy: Pelvis is root, then spine goes up, limbs branch out
    std::map<k4abt_joint_id_t, k4abt_joint_id_t> BuildParentMap()
    {
        std::map<k4abt_joint_id_t, k4abt_joint_id_t> parentMap;
        // Pelvis has no parent
        parentMap[K4ABT_JOINT_PELVIS] = K4ABT_JOINT_COUNT; // Use COUNT as invalid parent
        
        // Build parent map - bone list connects joints, we need to determine which is parent
        // The bone list pairs represent connections, we'll use the standard Kinect hierarchy
        // Pelvis -> SpineNavel -> SpineChest -> Neck -> Head -> Nose
        parentMap[K4ABT_JOINT_SPINE_NAVEL] = K4ABT_JOINT_PELVIS;
        parentMap[K4ABT_JOINT_SPINE_CHEST] = K4ABT_JOINT_SPINE_NAVEL;
        parentMap[K4ABT_JOINT_NECK] = K4ABT_JOINT_SPINE_CHEST;
        parentMap[K4ABT_JOINT_HEAD] = K4ABT_JOINT_NECK;
        parentMap[K4ABT_JOINT_NOSE] = K4ABT_JOINT_HEAD;
        
        // Left arm: SpineChest -> ClavicleLeft -> ShoulderLeft -> ElbowLeft -> WristLeft -> HandLeft
        parentMap[K4ABT_JOINT_CLAVICLE_LEFT] = K4ABT_JOINT_SPINE_CHEST;
        parentMap[K4ABT_JOINT_SHOULDER_LEFT] = K4ABT_JOINT_CLAVICLE_LEFT;
        parentMap[K4ABT_JOINT_ELBOW_LEFT] = K4ABT_JOINT_SHOULDER_LEFT;
        parentMap[K4ABT_JOINT_WRIST_LEFT] = K4ABT_JOINT_ELBOW_LEFT;
        parentMap[K4ABT_JOINT_HAND_LEFT] = K4ABT_JOINT_WRIST_LEFT;
        parentMap[K4ABT_JOINT_HANDTIP_LEFT] = K4ABT_JOINT_HAND_LEFT;
        parentMap[K4ABT_JOINT_THUMB_LEFT] = K4ABT_JOINT_WRIST_LEFT;
        
        // Right arm: SpineChest -> ClavicleRight -> ShoulderRight -> ElbowRight -> WristRight -> HandRight
        parentMap[K4ABT_JOINT_CLAVICLE_RIGHT] = K4ABT_JOINT_SPINE_CHEST;
        parentMap[K4ABT_JOINT_SHOULDER_RIGHT] = K4ABT_JOINT_CLAVICLE_RIGHT;
        parentMap[K4ABT_JOINT_ELBOW_RIGHT] = K4ABT_JOINT_SHOULDER_RIGHT;
        parentMap[K4ABT_JOINT_WRIST_RIGHT] = K4ABT_JOINT_ELBOW_RIGHT;
        parentMap[K4ABT_JOINT_HAND_RIGHT] = K4ABT_JOINT_WRIST_RIGHT;
        parentMap[K4ABT_JOINT_HANDTIP_RIGHT] = K4ABT_JOINT_HAND_RIGHT;
        parentMap[K4ABT_JOINT_THUMB_RIGHT] = K4ABT_JOINT_WRIST_RIGHT;
        
        // Left leg: Pelvis -> HipLeft -> KneeLeft -> AnkleLeft -> FootLeft
        parentMap[K4ABT_JOINT_HIP_LEFT] = K4ABT_JOINT_PELVIS;
        parentMap[K4ABT_JOINT_KNEE_LEFT] = K4ABT_JOINT_HIP_LEFT;
        parentMap[K4ABT_JOINT_ANKLE_LEFT] = K4ABT_JOINT_KNEE_LEFT;
        parentMap[K4ABT_JOINT_FOOT_LEFT] = K4ABT_JOINT_ANKLE_LEFT;
        
        // Right leg: Pelvis -> HipRight -> KneeRight -> AnkleRight -> FootRight
        parentMap[K4ABT_JOINT_HIP_RIGHT] = K4ABT_JOINT_PELVIS;
        parentMap[K4ABT_JOINT_KNEE_RIGHT] = K4ABT_JOINT_HIP_RIGHT;
        parentMap[K4ABT_JOINT_ANKLE_RIGHT] = K4ABT_JOINT_KNEE_RIGHT;
        parentMap[K4ABT_JOINT_FOOT_RIGHT] = K4ABT_JOINT_ANKLE_RIGHT;
        
        // Face: Nose -> EyeLeft/EyeRight -> EarLeft/EarRight
        parentMap[K4ABT_JOINT_EYE_LEFT] = K4ABT_JOINT_NOSE;
        parentMap[K4ABT_JOINT_EAR_LEFT] = K4ABT_JOINT_EYE_LEFT;
        parentMap[K4ABT_JOINT_EYE_RIGHT] = K4ABT_JOINT_NOSE;
        parentMap[K4ABT_JOINT_EAR_RIGHT] = K4ABT_JOINT_EYE_RIGHT;
        
        return parentMap;
    }

    void MaybeSendBodyFrame(k4abt_frame_t bodyFrame, UdpSender& sender)
    {
        if (!sender.IsStreaming())
        {
            return;
        }

        uint32_t numBodies = k4abt_frame_get_num_bodies(bodyFrame);
        if (numBodies == 0)
        {
            return;
        }

        k4abt_body_t body;
        if (k4abt_frame_get_body_skeleton(bodyFrame, 0, &body.skeleton) != K4A_RESULT_SUCCEEDED)
        {
            return;
        }

        std::array<float, K4ABT_JOINT_COUNT * 7 + 4> payload;
        PackJointData(body, payload);
        sender.Send(payload.data(), payload.size());
    }
}


int64_t ProcessKey(void* /*context*/, int key)
{
    // https://www.glfw.org/docs/latest/group__keys.html
    switch (key)
    {
        // Quit
    case GLFW_KEY_ESCAPE:
        s_isRunning = false;
        break;
    case GLFW_KEY_K:
        s_layoutMode = (Visualization::Layout3d)(((int)s_layoutMode + 1) % (int)Visualization::Layout3d::Count);
        break;
    case GLFW_KEY_B:
        s_visualizeJointFrame = !s_visualizeJointFrame;
        break;
    case GLFW_KEY_H:
        PrintAppUsage();
        break;
    }
    return 1;
}

int64_t CloseCallback(void* /*context*/)
{
    s_isRunning = false;
    return 1;
}

struct InputSettings
{
    k4a_depth_mode_t DepthCameraMode = K4A_DEPTH_MODE_NFOV_UNBINNED;
#ifdef _WIN32
    k4abt_tracker_processing_mode_t processingMode = K4ABT_TRACKER_PROCESSING_MODE_GPU_DIRECTML;
#else
    k4abt_tracker_processing_mode_t processingMode = K4ABT_TRACKER_PROCESSING_MODE_GPU_CUDA;
#endif
    bool Offline = false;
    std::string FileName;
    std::string ModelPath;
};

bool ParseInputSettingsFromArg(int argc, char** argv, InputSettings& inputSettings)
{
    for (int i = 1; i < argc; i++)
    {
        std::string inputArg(argv[i]);
        if (inputArg == std::string("NFOV_UNBINNED"))
        {
            inputSettings.DepthCameraMode = K4A_DEPTH_MODE_NFOV_UNBINNED;
        }
        else if (inputArg == std::string("WFOV_BINNED"))
        {
            inputSettings.DepthCameraMode = K4A_DEPTH_MODE_WFOV_2X2BINNED;
        }
        else if (inputArg == std::string("CPU"))
        {
            inputSettings.processingMode = K4ABT_TRACKER_PROCESSING_MODE_CPU;
        }
        else if (inputArg == std::string("TENSORRT"))
        {
            inputSettings.processingMode = K4ABT_TRACKER_PROCESSING_MODE_GPU_TENSORRT;
        }
        else if (inputArg == std::string("CUDA"))
        {
            inputSettings.processingMode = K4ABT_TRACKER_PROCESSING_MODE_GPU_CUDA;
        }
#ifdef _WIN32
        else if (inputArg == std::string("DIRECTML"))
        {
            inputSettings.processingMode = K4ABT_TRACKER_PROCESSING_MODE_GPU_DIRECTML;
        }
#endif
        else if (inputArg == std::string("OFFLINE"))
        {
            inputSettings.Offline = true;
            if (i < argc - 1) {
                // Take the next argument after OFFLINE as file name
                inputSettings.FileName = argv[i + 1];
                i++;
            }
            else {
                return false;
            }
        }
        else if (inputArg == std::string("-model"))
        {
            if (i < argc - 1)
                inputSettings.ModelPath = argv[++i];
            else
            {
                printf("Error: model path missing\n");
                return false;
            }
        }
        else
        {
            printf("Error: command not understood: %s\n", inputArg.c_str());
            return false;
        }
    }
    return true;
}

void VisualizeResult(k4abt_frame_t bodyFrame, Window3dWrapper& window3d, int depthWidth, int depthHeight) {

    // Obtain original capture that generates the body tracking result
    k4a_capture_t originalCapture = k4abt_frame_get_capture(bodyFrame);
    k4a_image_t depthImage = k4a_capture_get_depth_image(originalCapture);

    std::vector<Color> pointCloudColors(depthWidth * depthHeight, { 1.f, 1.f, 1.f, 1.f });

    // Read body index map and assign colors
    k4a_image_t bodyIndexMap = k4abt_frame_get_body_index_map(bodyFrame);
    const uint8_t* bodyIndexMapBuffer = k4a_image_get_buffer(bodyIndexMap);
    for (int i = 0; i < depthWidth * depthHeight; i++)
    {
        uint8_t bodyIndex = bodyIndexMapBuffer[i];
        if (bodyIndex != K4ABT_BODY_INDEX_MAP_BACKGROUND)
        {
            uint32_t bodyId = k4abt_frame_get_body_id(bodyFrame, bodyIndex);
            pointCloudColors[i] = g_bodyColors[bodyId % g_bodyColors.size()];
        }
    }
    k4a_image_release(bodyIndexMap);

    // Visualize point cloud
    window3d.UpdatePointClouds(depthImage, pointCloudColors);

    // Visualize the skeleton data
    window3d.CleanJointsAndBones();
    uint32_t numBodies = k4abt_frame_get_num_bodies(bodyFrame);
    for (uint32_t i = 0; i < numBodies; i++)
    {
        k4abt_body_t body;
        VERIFY(k4abt_frame_get_body_skeleton(bodyFrame, i, &body.skeleton), "Get skeleton from body frame failed!");
        body.id = k4abt_frame_get_body_id(bodyFrame, i);

        // Assign the correct color based on the body id
        Color color = g_bodyColors[body.id % g_bodyColors.size()];
        color.a = 0.4f;
        Color lowConfidenceColor = color;
        lowConfidenceColor.a = 0.1f;

        // Visualize joints
        for (int joint = 0; joint < static_cast<int>(K4ABT_JOINT_COUNT); joint++)
        {
            if (body.skeleton.joints[joint].confidence_level >= K4ABT_JOINT_CONFIDENCE_LOW)
            {
                const k4a_float3_t& jointPosition = body.skeleton.joints[joint].position;
                const k4a_quaternion_t& jointOrientation = body.skeleton.joints[joint].orientation;

                window3d.AddJoint(
                    jointPosition,
                    jointOrientation,
                    body.skeleton.joints[joint].confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM ? color : lowConfidenceColor);
            }
        }

        // Visualize bones
        for (size_t boneIdx = 0; boneIdx < g_boneList.size(); boneIdx++)
        {
            k4abt_joint_id_t joint1 = g_boneList[boneIdx].first;
            k4abt_joint_id_t joint2 = g_boneList[boneIdx].second;

            if (body.skeleton.joints[joint1].confidence_level >= K4ABT_JOINT_CONFIDENCE_LOW &&
                body.skeleton.joints[joint2].confidence_level >= K4ABT_JOINT_CONFIDENCE_LOW)
            {
                bool confidentBone = body.skeleton.joints[joint1].confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM &&
                    body.skeleton.joints[joint2].confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM;
                const k4a_float3_t& joint1Position = body.skeleton.joints[joint1].position;
                const k4a_float3_t& joint2Position = body.skeleton.joints[joint2].position;

                window3d.AddBone(joint1Position, joint2Position, confidentBone ? color : lowConfidenceColor);
            }
        }
    }

    k4a_capture_release(originalCapture);
    k4a_image_release(depthImage);

}

void PlayFile(InputSettings inputSettings, GuiWindow& gui, UdpSender& sender, const std::map<k4abt_joint_id_t, k4abt_joint_id_t>& parentMap)
{
    // Initialize the 3d window controller
    Window3dWrapper window3d;

    //create the tracker and playback handle
    k4a_calibration_t sensorCalibration;
    k4abt_tracker_t tracker = nullptr;
    k4a_playback_t playbackHandle = nullptr;

    const char* file = inputSettings.FileName.c_str();
    if (k4a_playback_open(file, &playbackHandle) != K4A_RESULT_SUCCEEDED)
    {
        printf("Failed to open recording: %s\n", file);
        return;
    }

    if (k4a_playback_get_calibration(playbackHandle, &sensorCalibration) != K4A_RESULT_SUCCEEDED)
    {
        printf("Failed to get calibration\n");
        return;
    }

    k4a_capture_t capture = nullptr;
    k4a_stream_result_t playbackResult = K4A_STREAM_RESULT_SUCCEEDED;

    k4abt_tracker_configuration_t trackerConfig = K4ABT_TRACKER_CONFIG_DEFAULT;
    trackerConfig.processing_mode = inputSettings.processingMode;
    trackerConfig.model_path = inputSettings.ModelPath.c_str();
    VERIFY(k4abt_tracker_create(&sensorCalibration, trackerConfig, &tracker), "Body tracker initialization failed!");

    int depthWidth = sensorCalibration.depth_camera_calibration.resolution_width;
    int depthHeight = sensorCalibration.depth_camera_calibration.resolution_height;

    window3d.Create("3D Visualization", sensorCalibration);
    window3d.SetCloseCallback(CloseCallback);
    window3d.SetKeyCallback(ProcessKey);

    while (playbackResult == K4A_STREAM_RESULT_SUCCEEDED && s_isRunning)
    {
        UpdateGuiAndStreaming(gui, sender, parentMap);
        playbackResult = k4a_playback_get_next_capture(playbackHandle, &capture);
        if (playbackResult == K4A_STREAM_RESULT_EOF)
        {
            // End of file reached
            break;
        }

        if (playbackResult == K4A_STREAM_RESULT_SUCCEEDED)
        {
            // check to make sure we have a depth image
            k4a_image_t depthImage = k4a_capture_get_depth_image(capture);
            if (depthImage == nullptr) {
                //If no depth image, print a warning and skip to next frame
                std::cout << "Warning: No depth image, skipping frame!" << std::endl;
                k4a_capture_release(capture);
                continue;
            }
            // Release the Depth image
            k4a_image_release(depthImage);

            //enque capture and pop results - synchronous
            k4a_wait_result_t queueCaptureResult = k4abt_tracker_enqueue_capture(tracker, capture, K4A_WAIT_INFINITE);

            // Release the sensor capture once it is no longer needed.
            k4a_capture_release(capture);

            if (queueCaptureResult == K4A_WAIT_RESULT_FAILED)
            {
                std::cout << "Error! Add capture to tracker process queue failed!" << std::endl;
                break;
            }

            k4abt_frame_t bodyFrame = nullptr;
            k4a_wait_result_t popFrameResult = k4abt_tracker_pop_result(tracker, &bodyFrame, K4A_WAIT_INFINITE);
            if (popFrameResult == K4A_WAIT_RESULT_SUCCEEDED)
            {
                /************* Successfully get a body tracking result, process the result here ***************/
                // Store current body for export
                uint32_t numBodies = k4abt_frame_get_num_bodies(bodyFrame);
                if (numBodies > 0)
                {
                    if (k4abt_frame_get_body_skeleton(bodyFrame, 0, &s_currentBody.skeleton) == K4A_RESULT_SUCCEEDED)
                    {
                        s_currentBody.id = k4abt_frame_get_body_id(bodyFrame, 0);
                        s_hasCurrentBody = true;
                    }
                }
                
                VisualizeResult(bodyFrame, window3d, depthWidth, depthHeight);
                MaybeSendBodyFrame(bodyFrame, sender);
                //Release the bodyFrame
                k4abt_frame_release(bodyFrame);
            }
            else
            {
                std::cout << "Pop body frame result failed!" << std::endl;
                break;
            }
        }

        window3d.SetLayout3d(s_layoutMode);
        window3d.SetJointFrameVisualization(s_visualizeJointFrame);
        window3d.Render();
    }

    sender.Stop();
    gui.SetStreamingState(false);
    k4abt_tracker_shutdown(tracker);
    k4abt_tracker_destroy(tracker);
    window3d.Delete();
    printf("Finished body tracking processing!\n");
    k4a_playback_close(playbackHandle);
}

void PlayFromDevice(InputSettings inputSettings, GuiWindow& gui, UdpSender& sender, const std::map<k4abt_joint_id_t, k4abt_joint_id_t>& parentMap)
{
    k4a_device_t device = nullptr;
    VERIFY(k4a_device_open(0, &device), "Open K4A Device failed!");

    // Start camera. Make sure depth camera is enabled.
    k4a_device_configuration_t deviceConfig = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    deviceConfig.depth_mode = inputSettings.DepthCameraMode;
    deviceConfig.color_resolution = K4A_COLOR_RESOLUTION_OFF;
    VERIFY(k4a_device_start_cameras(device, &deviceConfig), "Start K4A cameras failed!");

    // Get calibration information
    k4a_calibration_t sensorCalibration;
    VERIFY(k4a_device_get_calibration(device, deviceConfig.depth_mode, deviceConfig.color_resolution, &sensorCalibration),
        "Get depth camera calibration failed!");
    int depthWidth = sensorCalibration.depth_camera_calibration.resolution_width;
    int depthHeight = sensorCalibration.depth_camera_calibration.resolution_height;

    // Create Body Tracker
    k4abt_tracker_t tracker = nullptr;
    k4abt_tracker_configuration_t trackerConfig = K4ABT_TRACKER_CONFIG_DEFAULT;
    trackerConfig.processing_mode = inputSettings.processingMode;
    trackerConfig.model_path = inputSettings.ModelPath.c_str();
    VERIFY(k4abt_tracker_create(&sensorCalibration, trackerConfig, &tracker), "Body tracker initialization failed!");

    // Initialize the 3d window controller
    Window3dWrapper window3d;
    window3d.Create("3D Visualization", sensorCalibration);
    window3d.SetCloseCallback(CloseCallback);
    window3d.SetKeyCallback(ProcessKey);

    while (s_isRunning)
    {
        UpdateGuiAndStreaming(gui, sender, parentMap);
        k4a_capture_t sensorCapture = nullptr;
        k4a_wait_result_t getCaptureResult = k4a_device_get_capture(device, &sensorCapture, 0); // timeout_in_ms is set to 0

        if (getCaptureResult == K4A_WAIT_RESULT_SUCCEEDED)
        {
            // timeout_in_ms is set to 0. Return immediately no matter whether the sensorCapture is successfully added
            // to the queue or not.
            k4a_wait_result_t queueCaptureResult = k4abt_tracker_enqueue_capture(tracker, sensorCapture, 0);

            // Release the sensor capture once it is no longer needed.
            k4a_capture_release(sensorCapture);

            if (queueCaptureResult == K4A_WAIT_RESULT_FAILED)
            {
                std::cout << "Error! Add capture to tracker process queue failed!" << std::endl;
                break;
            }
        }
        else if (getCaptureResult != K4A_WAIT_RESULT_TIMEOUT)
        {
            std::cout << "Get depth capture returned error: " << getCaptureResult << std::endl;
            break;
        }

        // Pop Result from Body Tracker
        k4abt_frame_t bodyFrame = nullptr;
        k4a_wait_result_t popFrameResult = k4abt_tracker_pop_result(tracker, &bodyFrame, 0); // timeout_in_ms is set to 0
        if (popFrameResult == K4A_WAIT_RESULT_SUCCEEDED)
        {
            /************* Successfully get a body tracking result, process the result here ***************/
            // Store current body for export
            uint32_t numBodies = k4abt_frame_get_num_bodies(bodyFrame);
            if (numBodies > 0)
            {
                if (k4abt_frame_get_body_skeleton(bodyFrame, 0, &s_currentBody.skeleton) == K4A_RESULT_SUCCEEDED)
                {
                    s_currentBody.id = k4abt_frame_get_body_id(bodyFrame, 0);
                    s_hasCurrentBody = true;
                }
            }
            
            VisualizeResult(bodyFrame, window3d, depthWidth, depthHeight);
            MaybeSendBodyFrame(bodyFrame, sender);
            //Release the bodyFrame
            k4abt_frame_release(bodyFrame);
        }
       
        window3d.SetLayout3d(s_layoutMode);
        window3d.SetJointFrameVisualization(s_visualizeJointFrame);
        window3d.Render();
    }

    std::cout << "Finished body tracking processing!" << std::endl;

    sender.Stop();
    gui.SetStreamingState(false);
    window3d.Delete();
    k4abt_tracker_shutdown(tracker);
    k4abt_tracker_destroy(tracker);

    k4a_device_stop_cameras(device);
    k4a_device_close(device);
}

int main(int argc, char** argv)
{
    InputSettings inputSettings;
   
    if (!ParseInputSettingsFromArg(argc, argv, inputSettings))
    {
        // Print app usage if user entered incorrect arguments.
        PrintUsage();
        return -1;
    }

    // Build parent joint map
    std::map<k4abt_joint_id_t, k4abt_joint_id_t> parentMap = BuildParentMap();

    GuiWindow gui;
    gui.Create("UDP Streaming Controls", 100, 100, 340, 200);
    gui.SetStreamingState(false);
    UdpSender sender;

    // Either play the offline file or play from the device
    if (inputSettings.Offline == true)
    {
        PlayFile(inputSettings, gui, sender, parentMap);
    }
    else
    {
        PlayFromDevice(inputSettings, gui, sender, parentMap);
    }

    return 0;
}
