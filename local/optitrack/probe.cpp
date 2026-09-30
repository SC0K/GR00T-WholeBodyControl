// Receive-only diagnostic: requests descriptions and observes frames; no robot output.
#include <NatNetClient.h>
#include <NatNetCAPI.h>
#include <atomic>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <thread>

struct Stats {
    std::atomic<int> frames{0}, skeleton_frames{0}, max_skeletons{0};
};
void NATNET_CALLCONV on_frame(sFrameOfMocapData* frame, void* context) {
    auto& stats = *static_cast<Stats*>(context);
    if (stats.frames.fetch_add(1) == 0) {
        std::cerr << "FIRST_FRAME id=" << frame->iFrame << " markersets=" << frame->nMarkerSets
                  << " rigidbodies=" << frame->nRigidBodies << " skeletons=" << frame->nSkeletons
                  << " timestamp=" << frame->fTimestamp << '\n';
    }
    if (frame->nSkeletons > 0) ++stats.skeleton_frames;
    int previous = stats.max_skeletons.load();
    while (previous < frame->nSkeletons &&
           !stats.max_skeletons.compare_exchange_weak(previous, frame->nSkeletons)) {}
}
int main(int argc, char** argv) {
    if (argc != 3) {
        std::cerr << "Usage: probe SERVER_IP CLIENT_INTERFACE_IP\n";
        return 2;
    }
    unsigned char version[4];
    NatNet_GetVersion(version);
    std::cout << "Local NatNet SDK: " << int(version[0]) << '.' << int(version[1])
              << '.' << int(version[2]) << '.' << int(version[3]) << '\n';
    Stats stats;
    NatNetClient client;
    client.SetFrameReceivedCallback(on_frame, &stats);
    sNatNetClientConnectParams params;
    params.connectionType = ConnectionType_Unicast;
    params.serverAddress = argv[1];
    params.localAddress = argv[2];
    params.serverCommandPort = 1510;
    params.serverDataPort = 1511;
    params.BitstreamVersion[0] = 4;
    params.BitstreamVersion[1] = 0;
    int result = client.Connect(params);
    if (result != ErrorCode_OK) {
        std::cerr << "NatNet connection failed: " << result << '\n';
        return 1;
    }
    sDataDescriptions* descriptions = nullptr;
    result = client.GetDataDescriptionList(&descriptions);
    int skeletons = 0;
    void* response = nullptr;
    int response_size = 0;
    for (const char* query : {"FrameRate", "GetProperty,Skeleton 001,Enable"}) {
        if (client.SendMessageAndWait(query, 1, 250, &response, &response_size) == ErrorCode_OK) {
            std::cout << "QUERY " << query << " bytes=" << response_size << " hex=";
            for (int i = 0; i < response_size && i < 32; ++i)
                std::cout << std::hex << std::setw(2) << std::setfill('0')
                          << int(static_cast<unsigned char*>(response)[i]);
            std::cout << std::dec << '\n';
        }
    }
    if (result == ErrorCode_OK && descriptions) {
        for (int i = 0; i < descriptions->nDataDescriptions; ++i) {
            const auto& description = descriptions->arrDataDescriptions[i];
            if (description.type == Descriptor_RigidBody) {
                const auto& body = *description.Data.RigidBodyDescription;
                std::cout << "RIGID_BODY " << body.ID << ' ' << std::quoted(body.szName) << '\n';
            } else if (description.type == Descriptor_Skeleton) {
                ++skeletons;
                const auto& skeleton = *description.Data.SkeletonDescription;
                std::cout << "SKELETON " << skeleton.skeletonID << ' '
                          << std::quoted(skeleton.szName) << " bones=" << skeleton.nRigidBodies << '\n';
                for (int j = 0; j < skeleton.nRigidBodies; ++j) {
                    const auto& bone = skeleton.RigidBodies[j];
                    std::cout << "  BONE " << bone.ID << " parent=" << bone.parentID
                              << " name=" << std::quoted(bone.szName) << " offset="
                              << bone.offsetx << ',' << bone.offsety << ',' << bone.offsetz << '\n';
                }
            }
        }
        NatNet_FreeDescriptions(descriptions);
    } else {
        std::cerr << "Data description request failed: " << result << '\n';
    }
    std::this_thread::sleep_for(std::chrono::seconds(5));
    client.Disconnect();
    std::cout << "SUMMARY skeleton_descriptions=" << skeletons
              << " frames=" << stats.frames << " skeleton_frames=" << stats.skeleton_frames
              << " max_skeletons_per_frame=" << stats.max_skeletons << '\n';
    return (skeletons > 0 && stats.skeleton_frames > 0) ? 0 : 3;
}
