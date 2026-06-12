#ifndef __HELPERS_H__
#define __HELPERS_H__

#include <ros_tools/logging.h>

/** Logging Macros */
#define ROADMAP_INFO(msg)               \
    if (config_->debug_output_)         \
    {                                   \
        LOG_INFO("[Roadmap]: " << msg); \
    }

#define ROADMAP_WARN(msg)               \
    if (config_->debug_output_)         \
    {                                   \
        LOG_WARN("[Roadmap]: " << msg); \
    }

#define ROADMAP_ERROR(msg) LOG_ERROR("[Roadmap]: " << msg)

#define ROADMAP_INFO_STREAM(msg)        \
    if (config_->debug_output_)         \
    {                                   \
        LOG_INFO("[Roadmap]: " << msg); \
    }

#define ROADMAP_WARN_STREAM(msg)        \
    if (config_->debug_output_)         \
    {                                   \
        LOG_WARN("[Roadmap]: " << msg); \
    }

#define ROADMAP_ERROR_STREAM(msg) LOG_ERROR("[Roadmap]: " << msg)

#define ROADMAP_INFO_ALWAYS(msg) LOG_INFO("[Roadmap]: " << msg)
#define ROADMAP_WARN_ALWAYS(msg) LOG_WARN("[Roadmap]: " << msg)

#endif // __HELPERS_H__