// Native receive-only NatNet bridge. Python owns conversion and publication.
#include <NatNetClient.h>
#include <NatNetCAPI.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <array>
#include <chrono>
#include <mutex>
#include <string>
#include <vector>
namespace py = pybind11;
struct Bone {
    int id;
    std::array<float, 3> position;
    std::array<float, 4> quaternion;
    int flags;
};
struct Frame {
    int number = -1;
    double received = 0, timestamp = 0;
    std::vector<Bone> bones;
};
class Receiver {
    std::mutex mutex_;
    Frame latest_;
    int skeleton_id_;
    std::string server_, local_;
    NatNetClient client_;
    py::list description_;
    static void NATNET_CALLCONV callback(sFrameOfMocapData* data, void* context) {
        auto* self = static_cast<Receiver*>(context);
        Frame frame;
        frame.number = data->iFrame;
        frame.timestamp = data->fTimestamp;
        frame.received = std::chrono::duration<double>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        for (int i = 0; i < data->nSkeletons; ++i) {
            const auto& skeleton = data->Skeletons[i];
            if (skeleton.skeletonID != self->skeleton_id_) continue;
            for (int j = 0; j < skeleton.nRigidBodies; ++j) {
                const auto& bone = skeleton.RigidBodyData[j];
                frame.bones.push_back({bone.ID & 0xffff, {bone.x, bone.y, bone.z},
                                      {bone.qx, bone.qy, bone.qz, bone.qw}, bone.params});
            }
        }
        std::lock_guard<std::mutex> guard(self->mutex_);
        self->latest_ = std::move(frame);
    }
public:
    Receiver(std::string server, std::string local, int skeleton_id)
        : skeleton_id_(skeleton_id), server_(std::move(server)), local_(std::move(local)) {
        sNatNetClientConnectParams params;
        params.connectionType = ConnectionType_Unicast;
        params.serverAddress = server_.c_str();
        params.localAddress = local_.c_str();
        params.serverCommandPort = 1510;
        params.serverDataPort = 1511;
        params.BitstreamVersion[0] = 4;
        client_.SetFrameReceivedCallback(callback, this);
        if (client_.Connect(params) != ErrorCode_OK) {
            client_.Disconnect();
            throw std::runtime_error("Cannot connect to NatNet server");
        }
        sDataDescriptions* descriptions = nullptr;
        if (client_.GetDataDescriptionList(&descriptions) != ErrorCode_OK || !descriptions) {
            client_.Disconnect();
            throw std::runtime_error("Cannot retrieve NatNet descriptions");
        }
        for (int i = 0; i < descriptions->nDataDescriptions; ++i) {
            const auto& d = descriptions->arrDataDescriptions[i];
            if (d.type != Descriptor_Skeleton || d.Data.SkeletonDescription->skeletonID != skeleton_id_)
                continue;
            const auto& skeleton = *d.Data.SkeletonDescription;
            for (int j = 0; j < skeleton.nRigidBodies; ++j) {
                const auto& b = skeleton.RigidBodies[j];
                py::dict bone;
                bone["id"] = b.ID;
                bone["parent_id"] = b.parentID;
                bone["name"] = b.szName;
                bone["offset"] = std::array<float, 3>{b.offsetx, b.offsety, b.offsetz};
                description_.append(bone);
            }
        }
        NatNet_FreeDescriptions(descriptions);
        if (description_.empty()) {
            client_.Disconnect();
            throw std::runtime_error("Configured skeleton not found in NatNet descriptions");
        }
    }
    ~Receiver() { client_.Disconnect(); }
    void close() { client_.Disconnect(); }
    py::list description() const { return description_; }
    py::dict latest() {
        Frame frame;
        { std::lock_guard<std::mutex> guard(mutex_); frame = latest_; }
        py::dict result;
        result["frame"] = frame.number;
        result["timestamp"] = frame.timestamp;
        result["received"] = frame.received;
        py::dict bones;
        for (const auto& bone : frame.bones) {
            py::dict b;
            b["position"] = bone.position;
            b["quaternion"] = bone.quaternion;
            b["flags"] = bone.flags;
            bones[py::int_(bone.id)] = b;
        }
        result["bones"] = bones;
        return result;
    }
};
PYBIND11_MODULE(_natnet, module) {
    py::class_<Receiver>(module, "Receiver")
        .def(py::init<std::string, std::string, int>())
        .def("latest", &Receiver::latest)
        .def("description", &Receiver::description)
        .def("close", &Receiver::close);
}
