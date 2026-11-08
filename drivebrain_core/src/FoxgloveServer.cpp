// sends measurements to Foxglove so they can be viewed live
#include <FoxgloveServer.hpp>
#include <spdlog/spdlog.h>

static uint64_t nanosecondsSinceEpoch() {
    return uint64_t(std::chrono::duration_cast<std::chrono::nanoseconds>(
                        std::chrono::system_clock::now().time_since_epoch())
                        .count());
}

// finds the Protobuf definitions used by the program
static std::vector<const google::protobuf::FileDescriptor *> get_pb_descriptors(const std::vector<std::string> &filenames) {
    std::vector<const google::protobuf::FileDescriptor *> descriptors;

    for (const auto &name : filenames) {
        const google::protobuf::FileDescriptor *file_descriptor = google::protobuf::DescriptorPool::generated_pool()->FindFileByName(name);

        if (!file_descriptor) {
            spdlog::error("File descriptor not found {}", name);
        }
        else {
            descriptors.push_back(file_descriptor);
        }
    }

    return descriptors;
}

// packages the message definitions so Foxglove can decode the data
static std::string SerializeFdSet(const google::protobuf::Descriptor *toplevelDescriptor) {
    google::protobuf::FileDescriptorSet fdSet;
    std::queue<const google::protobuf::FileDescriptor *> toAdd;
    toAdd.push(toplevelDescriptor->file());
    std::unordered_set<std::string> seenDependencies;

    while (!toAdd.empty()) {
        const google::protobuf::FileDescriptor *next = toAdd.front();
        toAdd.pop();
        next->CopyTo(fdSet.add_file());

        for (int i = 0; i < next->dependency_count(); ++i) {
            const auto &dep = next->dependency(i);

            if (seenDependencies.find(dep->name()) == seenDependencies.end()) {
                seenDependencies.insert(dep->name());
                toAdd.push(dep);
            }
        }
    }

    return fdSet.SerializeAsString();
}

// creates one shared server instance
void core::FoxgloveServer::create() {
    FoxgloveServer* expected = nullptr;
    FoxgloveServer* local = new FoxgloveServer();

    if (!_s_instance.compare_exchange_strong(expected, local, std::memory_order_release, std::memory_order_relaxed)) {
        delete local;
    }
}

core::FoxgloveServer& core::FoxgloveServer::instance() {
    FoxgloveServer* instance = _s_instance.load(std::memory_order_acquire);
    assert(instance != nullptr && "Foxglove server has not been initialized");
    return *instance;
}

// destroys the server when the program stops
void core::FoxgloveServer::destroy() {
    FoxgloveServer* instance = _s_instance.exchange(nullptr, std::memory_order_acq_rel);

    if (instance) {
        delete instance;
    }
}

core::FoxgloveServer::FoxgloveServer() {
    // creates the websocket server
    const auto logHandler = [](foxglove::WebSocketLogLevel, char const *msg) {
        spdlog::info("{}", msg);
    };

    _server = foxglove::ServerFactory::createServer<websocketpp::connection_hdl>(
        "mini-drivebrain", logHandler, _server_options
    );

    foxglove::ServerHandlers<foxglove::ConnHandle> hdlrs;

    hdlrs.subscribeHandler = [&](foxglove::ChannelId chanId, foxglove::ConnHandle clientHandle) {
        const auto clientStr = _server->remoteEndpointString(clientHandle);
        spdlog::info("Client {} subscribed to {}", clientStr, chanId);
    };

    hdlrs.unsubscribeHandler = [&](foxglove::ChannelId chanId, foxglove::ConnHandle clientHandle) {
        const auto clientStr = _server->remoteEndpointString(clientHandle);
        spdlog::info("Client {} unsubscribed from {}", clientStr, chanId);
    };

    // creates channels for the Protobuf messages
    std::vector<std::string> proto_names = {"drivebrain_msgs.proto"};

    auto descriptors = get_pb_descriptors(proto_names);
    std::vector<foxglove::ChannelWithoutId> channels;

    int running_index = 1;

    for (const auto &file_descriptor : descriptors) {
        for (int i = 0; i < file_descriptor->message_type_count(); ++i) {
            const google::protobuf::Descriptor *message_descriptor = file_descriptor->message_type(i);

            foxglove::ChannelWithoutId server_channel;
            server_channel.topic = message_descriptor->name();
            server_channel.encoding = "protobuf";
            server_channel.schemaName = message_descriptor->full_name();
            server_channel.schema = foxglove::base64Encode(SerializeFdSet(message_descriptor));

            _name_to_id_map[server_channel.topic] = running_index;
            channels.push_back(server_channel);
            running_index++;
        }
    }

    auto res_ids = _server->addChannels(channels);

    // starts listening for Foxglove connections
    _server->setHandlers(std::move(hdlrs));
    _server->start("0.0.0.0", 5555);
}

// sends a message on its Foxglove channel
void core::FoxgloveServer::send_live_telem_msg(
    std::shared_ptr<google::protobuf::Message> msg,
    const std::string &topic
) {
    const std::string &name = topic.empty() ? msg->GetDescriptor()->name() : topic;
    foxglove::ChannelId msg_chan_id;

    {
        std::unique_lock lock(_channel_mutex);
        auto channel = _name_to_id_map.find(name);

        if (channel == _name_to_id_map.end()) {
            foxglove::ChannelWithoutId server_channel;
            server_channel.topic = name;
            server_channel.encoding = "protobuf";
            server_channel.schemaName = msg->GetDescriptor()->full_name();
            server_channel.schema = foxglove::base64Encode(
                SerializeFdSet(msg->GetDescriptor())
            );

            channel = _name_to_id_map.emplace(
                name, _server->addChannels({server_channel})[0]
            ).first;
        }

        msg_chan_id = channel->second;
    }

    const auto serialized_msg = msg->SerializeAsString();
    const auto now = nanosecondsSinceEpoch();

    _server->broadcastMessage(
        msg_chan_id,
        now,
        reinterpret_cast<const uint8_t *>(serialized_msg.data()),
        serialized_msg.size()
    );
}