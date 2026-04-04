// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#ifndef CUBBYFLOW_LOGGER_HPP
#define CUBBYFLOW_LOGGER_HPP

#include <cstdint>
#include <sstream>

namespace CubbyFlow
{
//! Level of the log.
//! All < Debug < Info < Warn < Error < Off.
enum class LogLevel : uint8_t
{
    All = 0,
    Debug = 1,
    Info = 2,
    Warn = 3,
    Error = 4,
    Off = 5
};

//!
//! \brief Super simple logger implementation.
//!
//! This is a super simple logger implementation that has minimal logging
//! capability. Currently, the class doesn't support multi-thread logging.
//!
class Logger final
{
 public:
    //! Constructs a logger with logging level.
    explicit Logger(LogLevel level);

    //! Deleted copy constructor.
    Logger(const Logger&) = delete;

    //! Deleted move constructor.
    Logger(Logger&&) noexcept = delete;

    //! Destructor.
    ~Logger();

    //! Deleted copy assignment operator.
    Logger& operator=(const Logger&) = delete;

    //! Deleted move assignment operator.
    Logger& operator=(Logger&&) noexcept = delete;

    //! Writes a value to the buffer stream.
    template <typename T>
    const Logger& operator<<(const T& x) const
    {
        m_buffer << x;
        return *this;
    }

 private:
    LogLevel m_level;
    mutable std::stringstream m_buffer{};
};

//! Helper class for logging.
class Logging
{
 public:
    //! Sets the output stream for the info level logs.
    static void SetInfoStream(std::ostream* stream);

    //! Sets the output stream for the warning level logs.
    static void SetWarnStream(std::ostream* stream);

    //! Sets the output stream for the error level logs.
    static void SetErrorStream(std::ostream* stream);

    //! Sets the output stream for the debug level logs.
    static void SetDebugStream(std::ostream* stream);

    //! Sets the output stream for all the log levels.
    static void SetAllStream(std::ostream* stream);

    //! Returns the header string.
    static std::string GetHeader(LogLevel level);

    //! Sets the log level.
    static void SetLevel(LogLevel level);

    //! Mutes the logger.
    static void Mute();

    //! Un-mutes the logger.
    static void Unmute();
};

//! Info-level logger.
extern Logger infoLogger;

//! Warn-level logger.
extern Logger warnLogger;

//! Error-level logger.
extern Logger errorLogger;

//! Debug-level logger.
extern Logger debugLogger;

#define CUBBYFLOW_INFO                                               \
    (Logger(LogLevel::Info)                                          \
     << Logging::GetHeader(LogLevel::Info) << "[" << __FILE__ << ":" \
     << __LINE__ << " (" << __func__ << ")] ")
#define CUBBYFLOW_WARN                                               \
    (Logger(LogLevel::Warn)                                          \
     << Logging::GetHeader(LogLevel::Warn) << "[" << __FILE__ << ":" \
     << __LINE__ << " (" << __func__ << ")] ")
#define CUBBYFLOW_ERROR                                               \
    (Logger(LogLevel::Error)                                          \
     << Logging::GetHeader(LogLevel::Error) << "[" << __FILE__ << ":" \
     << __LINE__ << " (" << __func__ << ")] ")
#define CUBBYFLOW_DEBUG                                               \
    (Logger(LogLevel::Debug)                                          \
     << Logging::GetHeader(LogLevel::Debug) << "[" << __FILE__ << ":" \
     << __LINE__ << " (" << __func__ << ")] ")
}  // namespace CubbyFlow

#endif