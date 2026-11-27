#pragma once

#include <signal.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iterator>
#include <map>
#include <mutex>
#include <stdexcept>
#include <string>
#include <system_error>
#include <vector>

#include "SNPE/SNPEFactory.hpp"
#include "SNPE/SNPEUtil.h"
#include "depthai/properties/NeuralNetworkProperties.hpp"

namespace dai::gate {

/**
 * Registers SNPE UDO packages for the DSP runtime.
 *
 * The DSP finds an implementation library by the file name stored in the
 * registration library, searching the directories that fastrpc derives from
 * ADSP_LIBRARY_PATH (see the constructor). NNArchive libraries are staged in one
 * private directory at the front of that path. A staged library may not differ
 * from a library with the same name in any other searched directory, so it
 * cannot replace a firmware, SNPE or OS library. SNPE cannot unregister a
 * package, so packages stay registered until the process exits.
 *
 * A DLC without an offline-prepared HTP graph is prepared on the CPU. SNPE then
 * loads the AArch64 build of each DSP library, named "<DSP library>_AltPrep.so",
 * through the dynamic loader. The loader reads LD_LIBRARY_PATH only at process
 * start, so the gate's on_start (scripts/custom_scripts.py) creates
 * PREPARE_DIRECTORY and adds it to LD_LIBRARY_PATH; those libraries are staged
 * there.
 *
 * Call get() in main() once ADSP_LIBRARY_PATH is final and before any
 * pipeline is built: the constructor changes the environment, which SNPE reads
 * when it opens its DSP session.
 */
class UdoRuntime {
   public:
    /** Reads a library from its pipeline asset URI. */
    using Loader = std::function<std::vector<uint8_t>(const std::string& uri)>;

    static UdoRuntime& get() {
        static UdoRuntime instance;
        return instance;
    }

    /**
     * Validates, stages and registers the UDO packages of an NNArchive.
     *
     * @param packages Packages to register.
     * @param uris Asset URI of each library path in packages.
     * @param load Reads a library from its asset URI.
     */
    void registerArchivePackages(const std::vector<NeuralNetworkProperties::UdoPackage>& packages,
                                 const std::map<std::string, std::string>& uris,
                                 const Loader& load) {
        // Read and check every library before staging or registering anything.
        std::vector<Package> stagedPackages;
        for(const auto& udoPackage : packages) {
            if(udoPackage.dspImplementations.empty()) {
                throw std::runtime_error("NNArchive UDO package " + udoPackage.registration + " has no DSP implementation");
            }
            Package package;
            const auto read = [&](const std::string& path, bool registration) {
                const auto name = libraryName(path);
                const auto uri = uris.find(path);
                if(uri == uris.end()) throw std::runtime_error("Missing NNArchive UDO asset URI: " + path);
                auto bytes = load(uri->second);
                auto* files = &package.files;
                const auto machine = elfMachine(bytes);
                if(machine != Machine::OTHER && !containsProgramSegments(bytes)) {
                    throw std::runtime_error("NNArchive UDO library " + path + " is truncated or damaged: the file ends before its ELF program segments");
                }
                if(registration) {
                    if(machine != Machine::AARCH64) throw std::runtime_error("NNArchive UDO library is not a compatible AArch64 ELF shared library: " + path);
                } else if(machine == Machine::AARCH64) {
                    if(!isPrepareLibraryName(name)) {
                        throw std::runtime_error("NNArchive UDO DSP library " + path
                                                 + " is an AArch64 library. SNPE loads the AArch64 build of a DSP library only to prepare the "
                                                   "network, under the name <DSP library>_AltPrep.so");
                    }
                    files = &package.prepareFiles;
                } else if(machine != Machine::HEXAGON) {
                    throw std::runtime_error("NNArchive UDO DSP library is not a compatible Hexagon DSP or AArch64 _AltPrep ELF shared library: " + path);
                }
                const auto existing = files->find(name);
                if(existing != files->end() && existing->second != bytes) throw std::runtime_error("Conflicting NNArchive UDO library name: " + name);
                files->emplace(name, std::move(bytes));
                return name;
            };
            package.registration = read(udoPackage.registration, true);
            for(const auto& path : udoPackage.dspImplementations) read(path, false);
            if(package.files.size() < 2) {
                throw std::runtime_error("NNArchive UDO package " + udoPackage.registration + " has no Hexagon DSP implementation");
            }
            stagedPackages.push_back(std::move(package));
        }

        std::lock_guard<std::mutex> lock(mutex);
        if(directory.empty()) throw std::runtime_error("Could not create the NNArchive UDO directory: " + directoryError);
        for(const auto& package : stagedPackages) stageAndRegister(package);
    }

    /**
     * Registers a UDO package installed with the OS. Its DSP implementation
     * must be in a directory on ADSP_LIBRARY_PATH.
     *
     * @param registration Path of the registration library.
     */
    void registerSystemPackage(const std::filesystem::path& registration) {
        auto bytes = readFile(registration);
        const auto name = registration.filename().string();
        std::lock_guard<std::mutex> lock(mutex);
        // An archive may have registered the same package from its staged copy.
        if(isRegistered(name, bytes)) return;
        addOpPackage(registration);
        registered.emplace(name, std::move(bytes));
    }

    ~UdoRuntime() {
        std::error_code error;
        // The prepare directory belongs to the gate; remove only what was staged.
        for(const auto& path : preparedFiles) std::filesystem::remove(path, error);
        if(!directory.empty()) std::filesystem::remove_all(directory, error);
    }
    UdoRuntime(const UdoRuntime&) = delete;
    UdoRuntime& operator=(const UdoRuntime&) = delete;
    UdoRuntime(UdoRuntime&&) = delete;
    UdoRuntime& operator=(UdoRuntime&&) = delete;

   private:
    static constexpr const char* DIRECTORY_PREFIX = ".depthai-udo-";
    // Created by the gate's on_start next to the executable; see the class comment.
    static constexpr const char* PREPARE_DIRECTORY = ".depthai-udo-prepare";
    static constexpr const char* PREPARE_SUFFIX = "_AltPrep.so";

    enum class Machine { OTHER, HEXAGON, AARCH64 };

    struct Package {
        std::string registration;
        // File name -> bytes of the registration and Hexagon DSP libraries.
        std::map<std::string, std::vector<uint8_t>> files;
        // File name -> bytes of the AArch64 _AltPrep libraries.
        std::map<std::string, std::vector<uint8_t>> prepareFiles;
    };

    std::filesystem::path directory;
    std::string directoryError;
    // Directories that fastrpc searches for a CDSP library, other than the staging directory.
    std::vector<std::filesystem::path> searchPath;
    std::filesystem::path prepareDirectory;
    std::string prepareError;
    // LD_LIBRARY_PATH directories other than the prepare directory.
    std::vector<std::filesystem::path> loaderPath;
    std::vector<std::filesystem::path> preparedFiles;
    std::mutex mutex;
    // Registration library file name -> its bytes.
    std::map<std::string, std::vector<uint8_t>> registered;

    UdoRuntime() {
        // fastrpc searches the ADSP_LIBRARY_PATH entries followed by its own
        // default entries. For a CDSP library (SNPE and this firmware use only the
        // CDSP) it tries <entry>/cdsp before each entry, and for an empty entry it
        // opens the bare file name in the working directory.
        static constexpr const char* FASTRPC_DEFAULT_PATH = ";/usr/lib/rfsa/adsp;/dsp;";
        static constexpr const char* DOMAIN_DIRECTORY = "cdsp";
        const auto previous = std::getenv("ADSP_LIBRARY_PATH");
        const auto allPaths = std::string(previous ? previous : "") + ";" + FASTRPC_DEFAULT_PATH;
        const auto addSearchDirectory = [this](const std::filesystem::path& searchDirectory) {
            if(std::find(searchPath.begin(), searchPath.end(), searchDirectory) == searchPath.end()) searchPath.push_back(searchDirectory);
        };
        for(std::size_t start = 0; start <= allPaths.size();) {
            const auto end = std::min(allPaths.find(';', start), allPaths.size());
            const auto entry = allPaths.substr(start, end - start);
            start = end + 1;
            if(entry.empty()) {
                std::error_code error;
                const auto workingDirectory = std::filesystem::current_path(error);
                if(!error) addSearchDirectory(workingDirectory);
            } else {
                addSearchDirectory(std::filesystem::path(entry) / DOMAIN_DIRECTORY);
                addSearchDirectory(entry);
            }
        }

        // Stage next to the executable: the gate's on_destroy.sh removes the
        // directory when it kills the process before static destructors run.
        std::error_code error;
        const auto base = std::filesystem::read_symlink("/proc/self/exe", error).parent_path();
        if(error) {
            directoryError = "could not find the firmware directory: " + error.message();
            prepareError = directoryError;
            return;
        }
        findPrepareDirectory(base / PREPARE_DIRECTORY);
        removeStaleDirectories(base);
        auto pattern = (base / (DIRECTORY_PREFIX + std::to_string(getpid()) + "-XXXXXX")).string();
        if(mkdtemp(pattern.data()) == nullptr) {
            directoryError = std::strerror(errno);
            return;
        }
        // fastrpc still appends its default entries.
        const auto libraryPath = previous ? pattern + ";" + previous : pattern;
        if(setenv("ADSP_LIBRARY_PATH", libraryPath.c_str(), 1) != 0) {
            directoryError = std::string("could not add it to ADSP_LIBRARY_PATH: ") + std::strerror(errno);
            std::filesystem::remove(pattern, error);
            return;
        }
        directory = pattern;
    }

    // Uses the prepare directory only if it existed when the process started and
    // is on LD_LIBRARY_PATH; the loader caches directories it found missing.
    // Collects the other directories the loader searches, for the conflict check.
    void findPrepareDirectory(const std::filesystem::path& candidate) {
        const auto value = std::getenv("LD_LIBRARY_PATH");
        // Like glibc: ':' and ';' separate entries, an empty entry is the working
        // directory, and the default directories follow LD_LIBRARY_PATH.
        const std::string libraryPath = std::string(value ? value : "") + ":/lib:/usr/lib";
        bool listed = false;
        for(std::size_t start = 0; start <= libraryPath.size();) {
            const auto end = std::min(libraryPath.find_first_of(":;", start), libraryPath.size());
            const auto entry = libraryPath.substr(start, end - start);
            start = end + 1;
            std::error_code error;
            const auto directoryPath = entry.empty() ? std::filesystem::current_path(error) : std::filesystem::path(entry).lexically_normal();
            if(directoryPath.empty()) continue;
            // Compare file identity: the entry may name the directory through a symlink.
            if(std::filesystem::equivalent(directoryPath, candidate, error)) {
                listed = true;
            } else if(std::find(loaderPath.begin(), loaderPath.end(), directoryPath) == loaderPath.end()) {
                loaderPath.push_back(directoryPath);
            }
        }
        std::error_code error;
        if(!listed) {
            prepareError = candidate.string()
                           + " is not an existing directory on LD_LIBRARY_PATH. The gate's on_start (scripts/custom_scripts.py) creates it; "
                             "check the gate log";
        } else if(!std::filesystem::is_directory(candidate, error)) {
            prepareError = candidate.string() + " is not a directory";
        } else {
            prepareDirectory = candidate;
        }
    }

    // Removes directories of earlier processes that ended without cleanup.
    // Never throws: it runs at firmware startup.
    static void removeStaleDirectories(const std::filesystem::path& base) {
        std::error_code error;
        for(std::filesystem::directory_iterator entry(base, error), end; !error && entry != end; entry.increment(error)) {
            const auto name = entry->path().filename().string();
            if(name.rfind(DIRECTORY_PREFIX, 0) != 0) continue;
            const auto pid = std::strtol(name.c_str() + std::strlen(DIRECTORY_PREFIX), nullptr, 10);
            if(pid <= 0 || pid == getpid() || kill(static_cast<pid_t>(pid), 0) == 0 || errno != ESRCH) continue;
            std::error_code removeError;
            std::filesystem::remove_all(entry->path(), removeError);
        }
    }

    static std::vector<uint8_t> readFile(const std::filesystem::path& path) {
        std::ifstream input(path, std::ios::binary);
        if(!input) throw std::runtime_error("Could not read UDO library: " + path.string());
        std::vector<uint8_t> bytes{std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
        if(input.bad()) throw std::runtime_error("Could not read UDO library: " + path.string());
        return bytes;
    }

    // SNPE and the DSP use only the file name of a library.
    static std::string libraryName(const std::string& path) {
        const auto name = std::filesystem::path(path).filename().string();
        if(name.empty() || name == "." || name == ".." || path.find('\0') != std::string::npos) {
            throw std::runtime_error("Invalid NNArchive UDO library path: " + path);
        }
        return name;
    }

    // A little-endian ELF shared library: 32-bit Hexagon or 64-bit AArch64.
    static Machine elfMachine(const std::vector<uint8_t>& bytes) {
        // 52 and 64 bytes are the ELF header sizes of the two classes.
        if(bytes.size() < 52 || bytes[0] != 0x7f || bytes[1] != 'E' || bytes[2] != 'L' || bytes[3] != 'F' || bytes[5] != 1 || bytes[6] != 1 || bytes[16] != 3
           || bytes[17] != 0 || bytes[19] != 0) {
            return Machine::OTHER;
        }
        if(bytes[4] == 1 && bytes[18] == 164) return Machine::HEXAGON;                        // ELFCLASS32, EM_HEXAGON
        if(bytes[4] == 2 && bytes[18] == 183 && bytes.size() >= 64) return Machine::AARCH64;  // ELFCLASS64, EM_AARCH64
        return Machine::OTHER;
    }

    // Reads a little-endian unsigned integer. The caller checks the bounds.
    static uint64_t readUnsigned(const std::vector<uint8_t>& bytes, std::size_t offset, std::size_t size) {
        uint64_t value = 0;
        for(std::size_t i = size; i > 0; --i) value = (value << 8U) | bytes[offset + i - 1];
        return value;
    }

    // The dynamic loader maps the program segments of a library from its file. If
    // the file ends before a segment does, dlopen crashes the process with SIGBUS
    // instead of failing, so check that the file holds the program header table
    // and every segment. Call only for a library that elfMachine() accepted.
    static bool containsProgramSegments(const std::vector<uint8_t>& bytes) {
        const bool elf64 = bytes[4] == 2;
        const std::size_t wordSize = elf64 ? 8 : 4;
        const auto tableOffset = readUnsigned(bytes, elf64 ? 32 : 28, wordSize);  // e_phoff
        const auto entrySize = readUnsigned(bytes, elf64 ? 54 : 42, 2);           // e_phentsize
        const auto count = readUnsigned(bytes, elf64 ? 56 : 44, 2);               // e_phnum
        // The loader rejects a library without program headers with an error.
        if(count == 0) return true;
        const uint64_t size = bytes.size();
        if(entrySize != (elf64 ? 56U : 32U) || tableOffset > size || count * entrySize > size - tableOffset) return false;
        for(uint64_t index = 0; index < count; ++index) {
            const auto header = static_cast<std::size_t>(tableOffset + index * entrySize);
            const auto offset = readUnsigned(bytes, header + (elf64 ? 8 : 4), wordSize);      // p_offset
            const auto fileSize = readUnsigned(bytes, header + (elf64 ? 32 : 16), wordSize);  // p_filesz
            if(offset > size || fileSize > size - offset) return false;
        }
        return true;
    }

    static bool isPrepareLibraryName(const std::string& name) {
        const std::string suffix = PREPARE_SUFFIX;
        return name.size() > suffix.size() && name.compare(name.size() - suffix.size(), suffix.size(), suffix) == 0;
    }

    static void addOpPackage(const std::filesystem::path& registration) {
        if(!Snpe_Util_AddOpPackage(registration.c_str())) {
            const auto lastError = Snpe_Util_GetLastError();
            // SNPE often reports no reason, so name the usual ones.
            const std::string error = lastError && *lastError ? lastError
                                                              : "SNPE gave no reason. Check that the package was built for this SNPE version and that no "
                                                                "other registered package declares the same UDO package name";
            throw std::runtime_error("Could not register UDO package " + registration.filename().string() + " with SNPE "
                                     + zdl::SNPE::SNPEFactory::getLibraryVersion().toString() + ": " + error);
        }
    }

    // Call with mutex held.
    bool isRegistered(const std::string& name, const std::vector<uint8_t>& bytes) const {
        const auto package = registered.find(name);
        if(package == registered.end()) return false;
        if(package->second != bytes) throw std::runtime_error("Conflicting UDO registration library name: " + name);
        return true;
    }

    // Writes a library into a staging directory unless an identical copy is there.
    // Libraries with the same name in the other directories of the search path
    // must be identical, so staging never shadows a device library.
    static void stage(const std::filesystem::path& stagingDirectory,
                      const std::vector<std::filesystem::path>& otherDirectories,
                      const std::string& name,
                      const std::vector<uint8_t>& bytes,
                      std::vector<std::filesystem::path>& created) {
        for(const auto& otherDirectory : otherDirectories) {
            const auto other = otherDirectory / name;
            std::error_code error;
            if(std::filesystem::is_regular_file(other, error) && readFile(other) != bytes) {
                throw std::runtime_error("NNArchive UDO library " + name + " conflicts with the device library " + other.string()
                                         + ": give it a distinct file name");
            }
        }
        const auto path = stagingDirectory / name;
        if(std::filesystem::exists(path)) {
            if(readFile(path) != bytes) throw std::runtime_error("Conflicting NNArchive UDO library name: " + name);
            return;
        }
        created.push_back(path);
        std::ofstream output(path, std::ios::binary);
        // Streams take char bytes; write the library unchanged.
        output.write(reinterpret_cast<const char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
        output.close();
        if(!output) throw std::runtime_error("Could not write NNArchive UDO library: " + path.string());
    }

    // Call with mutex held.
    void stageAndRegister(const Package& package) {
        std::vector<std::filesystem::path> created;
        std::vector<std::filesystem::path> createdPrepare;
        try {
            for(const auto& file : package.files) stage(directory, searchPath, file.first, file.second, created);
            if(!package.prepareFiles.empty() && prepareDirectory.empty()) {
                throw std::runtime_error("NNArchive UDO library " + package.prepareFiles.begin()->first
                                         + " needs the directory for libraries that prepare the network: " + prepareError);
            }
            for(const auto& file : package.prepareFiles) stage(prepareDirectory, loaderPath, file.first, file.second, createdPrepare);
            const auto& bytes = package.files.at(package.registration);
            if(!isRegistered(package.registration, bytes)) {
                addOpPackage(directory / package.registration);
                registered.emplace(package.registration, bytes);
            }
        } catch(...) {
            // Keep files of packages that registered earlier; remove only new ones.
            for(const auto& path : created) {
                std::error_code error;
                std::filesystem::remove(path, error);
            }
            for(const auto& path : createdPrepare) {
                std::error_code error;
                std::filesystem::remove(path, error);
            }
            throw;
        }
        preparedFiles.insert(preparedFiles.end(), createdPrepare.begin(), createdPrepare.end());
    }
};

}  // namespace dai::gate
