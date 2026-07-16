#pragma once

#include <localization/interface/i_slam_odometry.hpp>

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace localization {

template <typename Base>
class PluginFactory {
public:
    using Creator = std::function<std::shared_ptr<Base>()>;

    static PluginFactory& instance() {
        static PluginFactory factory;
        return factory;
    }

    void registerCreator(const std::string& name, Creator creator) {
        creators_[name] = std::move(creator);
    }

    std::shared_ptr<Base> create(const std::string& name) const {
        const auto it = creators_.find(name);
        if (it == creators_.end()) {
            return nullptr;
        }
        return it->second();
    }

    std::vector<std::string> registeredNames() const {
        std::vector<std::string> names;
        for (const auto& kv : creators_) {
            names.push_back(kv.first);
        }
        return names;
    }

private:
    std::map<std::string, Creator> creators_;
};

using SlamFactory = PluginFactory<ISlamOdometry>;

void registerDefaultPlugins();

}  // namespace localization
