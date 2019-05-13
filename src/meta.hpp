#pragma once

inline const std::string getSrcDir()
{
    return std::string(__FILE__).erase(std::string(__FILE__).rfind('/')) + "/";
};

inline const std::string getResDir()
{
    return getSrcDir() + "../res/";
};
