#pragma once

#include <string>
#include <cstdio>

namespace dai
{
namespace bootloader
{

struct Version {
    explicit Version(const std::string& v){
        // Parse string
        if(std::sscanf(v.c_str(), "%u.%u.%u", &major, &minor, &patch) != 3) throw std::runtime_error("Cannot parse version: " + v);
    }

    Version(unsigned major, unsigned minor, unsigned patch){
        this->major = major;
        this->minor = minor;
        this->patch = patch;
    }

    bool operator==(const Version& other){
        if(major == other.major && minor == other.minor && patch == other.patch) return true;
        return false;
    }

    bool operator>(const Version &other){
        if(major > other.major){
            return true;
        } else {
            if(minor > other.minor){
                return true;
            } else {
                if(patch > other.patch){
                    return true;
                }
            }
        }
        return false;
    }

    bool operator<(const Version &other){
        if(major < other.major){
            return true;
        } else {
            if(minor < other.minor){
                return true;
            } else {
                if(patch < other.patch){
                    return true;
                }
            }
        }
        return false;
    } 

    std::string toString(){
        return std::to_string(major) + "." + std::to_string(minor) + "." + std::to_string(patch);
    }    

private:
    unsigned major, minor, patch;
};



} // namespace bootloader
} // namespace dai
