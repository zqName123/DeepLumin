#include "localization/factory/factory_registry.hpp"

#include "localization/slam/faster_lio_impl.hpp"

namespace localization {

void registerDefaultPlugins() {
    SlamFactory::instance().registerCreator(
        "faster_lio", []() { return std::make_shared<FasterLIOImpl>(); });
}

}  // namespace localization
