#include "MemPerfTestsUtils.hpp"

#include <iostream>
#include <string>
#include <utility>

void PrintMemReport(double memUsage, const std::string& memMessage)
{
    std::cout << "Mem usage: " << memUsage << ' ' << memMessage << '\n';
}

std::pair<double, std::string> MakeReadableByteSize(size_t bytes)
{
    double s = static_cast<double>(bytes);
    const char* unit = "B";

    if (s > 1024)
    {
        s /= 1024;
        unit = "kB";
    }

    if (s > 1024)
    {
        s /= 1024;
        unit = "MB";
    }

    if (s > 1024)
    {
        s /= 1024;
        unit = "GB";
    }

    if (s > 1024)
    {
        s /= 1024;
        unit = "TB";
    }

    return std::make_pair(s, unit);
}
