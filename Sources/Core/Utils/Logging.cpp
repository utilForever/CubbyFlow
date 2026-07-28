// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#include <Core/Utils/Logging.hpp>
#include <Core/Utils/Macros.hpp>

#include <chrono>
#include <iostream>
#include <mutex>

namespace CubbyFlow
{
namespace
{
struct LoggingState
{
    std::mutex critical;
    std::ostream* infoOutStream = &std::cout;
    std::ostream* warnOutStream = &std::cout;
    std::ostream* errorOutStream = &std::cerr;
    std::ostream* debugOutStream = &std::cout;
    LogLevel logLevel = LogLevel::All;
};

LoggingState& GetLoggingState()
{
    static LoggingState state;
    return state;
}
}  // namespace

inline std::ostream* LevelToStream(LogLevel level)
{
    const auto& state = GetLoggingState();

    switch (level)
    {
        case LogLevel::All:
        case LogLevel::Info:
            return state.infoOutStream;
        case LogLevel::Warn:
            return state.warnOutStream;
        case LogLevel::Error:
            return state.errorOutStream;
        case LogLevel::Debug:
            return state.debugOutStream;
        case LogLevel::Off:
            return nullptr;
    }

    return nullptr;
}

inline std::string LevelToString(LogLevel level)
{
    switch (level)
    {
        case LogLevel::All:
            return "";
        case LogLevel::Info:
            return "INFO";
        case LogLevel::Warn:
            return "WARN";
        case LogLevel::Error:
            return "ERROR";
        case LogLevel::Debug:
            return "DEBUG";
        case LogLevel::Off:
            return "";
    }

    return "";
}

inline bool IsLeq(LogLevel a, LogLevel b)
{
    return static_cast<uint8_t>(a) <= static_cast<uint8_t>(b);
}

Logger::Logger(LogLevel level) : m_level{ level }
{
    // Do nothing
}

Logger::~Logger()
{
    auto& state = GetLoggingState();
    std::lock_guard<std::mutex> lock(state.critical);

    if (IsLeq(state.logLevel, m_level))
    {
        std::ostream* stream = LevelToStream(m_level);
        *stream << m_buffer.str() << std::endl;
        stream->flush();
    }
}

void Logging::SetInfoStream(std::ostream* stream)
{
    auto& state = GetLoggingState();
    std::lock_guard<std::mutex> lock(state.critical);
    state.infoOutStream = stream;
}

void Logging::SetWarnStream(std::ostream* stream)
{
    auto& state = GetLoggingState();
    std::lock_guard<std::mutex> lock(state.critical);
    state.warnOutStream = stream;
}

void Logging::SetErrorStream(std::ostream* stream)
{
    auto& state = GetLoggingState();
    std::lock_guard<std::mutex> lock(state.critical);
    state.errorOutStream = stream;
}

void Logging::SetDebugStream(std::ostream* stream)
{
    auto& state = GetLoggingState();
    std::lock_guard<std::mutex> lock(state.critical);
    state.debugOutStream = stream;
}

void Logging::SetAllStream(std::ostream* stream)
{
    SetInfoStream(stream);
    SetWarnStream(stream);
    SetErrorStream(stream);
    SetDebugStream(stream);
}

std::string Logging::GetHeader(LogLevel level)
{
    auto now =
        std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    char timeStr[20];
#ifdef CUBBYFLOW_WINDOWS
    tm time{};
    localtime_s(&time, &now);
#ifdef _MSC_VER
    strftime(timeStr, sizeof(timeStr), "%F %T", &time);
#else
    // Such as MinGW - https://sourceforge.net/p/mingw-w64/bugs/793/
    strftime(timeStr, sizeof(timeStr), "%Y-%m-%d %H:%M:%S", &time);
#endif
#else
    tm time;
    localtime_r(&now, &time);
    strftime(timeStr, sizeof(timeStr), "%F %T", &time);
#endif
    char header[256];
    snprintf(header, sizeof(header), "[%s] %s ", LevelToString(level).c_str(),
             timeStr);

    return header;
}

void Logging::SetLevel(LogLevel level)
{
    auto& state = GetLoggingState();
    std::lock_guard<std::mutex> lock(state.critical);
    state.logLevel = level;
}

void Logging::Mute()
{
    SetLevel(LogLevel::Off);
}

void Logging::Unmute()
{
    SetLevel(LogLevel::All);
}
}  // namespace CubbyFlow
