#include "depthai/utility/AudioHelpers.hpp"

#include <alsa/asoundlib.h>

#include <stdexcept>

#include <iostream>

namespace dai {
namespace audio {

std::vector<AudioDevice> ListAlsaDevices() {
    int status;
    char **hints;

    // Get a list of all sound cards
    status = snd_device_name_hint(-1, "pcm", (void***)&hints);
    if (status < 0) {
        //logger->warn("AudioHelpers {}: Error getting device hints: {}", __func__, snd_strerror(status));
        return std::vector<AudioDevice>();
    }

    std::vector<AudioDevice> vec;

    char **n = hints;
    while (*n != NULL) {
	AudioDevice dev;

        char *name = snd_device_name_get_hint(*n, "NAME");
	if (name) {
	    dev.name = std::string(name);
            free(name);
	}

        char *desc = snd_device_name_get_hint(*n, "DESC");
	if (desc) {
		dev.desc = std::string(desc);
            free(desc);
	}
	
        char *ioid = snd_device_name_get_hint(*n, "IOID");
	if (ioid) {
		dev.ioid = std::string(ioid);
            free(ioid);
	}

	vec.push_back(dev);

        n++;
    }

    snd_device_name_free_hint((void**)hints);

    return vec;
}

AudioFile::AudioFile(const char *path, int mode) {
    file = sf_open(path, mode, &fileInfo);
    if (!file) {
	throw std::runtime_error("Unable to open audio file from path");
    }
}

AudioFile::AudioFile(int fd, int mode, bool closeOnDestruct) {
    file = sf_open_fd(fd, mode, &fileInfo, (int)closeOnDestruct);
    if (!file) {
	throw std::runtime_error("Unable to open audio file from fd");
    }
}

AudioFile::AudioFile(SF_VIRTUAL_IO *virtualFile, int mode, void *userData) {
    file = sf_open_virtual(virtualFile, mode, &fileInfo, userData);
    if (!file) {
	throw std::runtime_error("Unable to open virtual audio file");
    }
}
		
AudioFile::AudioFile(SF_VIRTUAL_IO *virtualFile, int mode, SF_INFO fileInfo, void *userData) {
    this->fileInfo = fileInfo;

    AudioFile(virtualFile, mode, userData);
}

AudioFile::~AudioFile() {
    sf_close(file);
}

sf_count_t AudioFile::readItem(int *ptr, sf_count_t items) {
	return sf_read_int(file, ptr, items);
}

sf_count_t AudioFile::readFrame(short *ptr, sf_count_t frames) {
	return sf_readf_short(file, ptr, frames);
}

sf_count_t AudioFile::readFrame(int *ptr, sf_count_t frames) {
	return sf_readf_int(file, ptr, frames);
}

sf_count_t AudioFile::readFrame(float *ptr, sf_count_t frames) {
	return sf_readf_float (file, ptr, frames);
}

sf_count_t AudioFile::readFrame(double *ptr, sf_count_t frames) {
	return sf_readf_double(file, ptr, frames);
}

sf_count_t AudioFile::readRaw(void *ptr, sf_count_t bytes) {
	return sf_read_raw(file, ptr, bytes) ;
}

sf_count_t AudioFile::writeItem(int *ptr, sf_count_t items) {
	return sf_write_int(file, ptr, items);
}

sf_count_t AudioFile::writeFrame(int *ptr, sf_count_t frames) {
	return sf_writef_int(file, ptr, frames);
}

sf_count_t AudioFile::writeRaw(void *ptr, sf_count_t bytes) {
	return sf_write_raw(file, ptr, bytes) ;
}

sf_count_t AudioFile::seek(sf_count_t frames, int whence) {
	return sf_seek(file, frames, whence);
}

int AudioFile::getFormat() const {
	if(fileInfo.format & SF_FORMAT_PCM_S8) {
		return SF_FORMAT_PCM_S8;
	} else 	if(fileInfo.format & SF_FORMAT_PCM_16) {
		return SF_FORMAT_PCM_16;
	} else 	if(fileInfo.format & SF_FORMAT_PCM_24) {
		return SF_FORMAT_PCM_24;
	} else 	if(fileInfo.format & SF_FORMAT_PCM_32) {
		return SF_FORMAT_PCM_32;
	} else 	if(fileInfo.format & SF_FORMAT_FLOAT) {
		return SF_FORMAT_FLOAT;
	} else 	if(fileInfo.format & SF_FORMAT_DOUBLE) {
		return SF_FORMAT_DOUBLE;
	} else {
		return 0;
	}
}

int AudioFile::getSampleSize() const {
	switch(getFormat()) {
		case SF_FORMAT_PCM_S8:
		case SF_FORMAT_PCM_16:
			return 16;
		case SF_FORMAT_PCM_24:
		case SF_FORMAT_PCM_32:
		case SF_FORMAT_FLOAT:
			return 32;
		case SF_FORMAT_DOUBLE:
			return 64;
		default:
			return 0;
	}
}

int AudioFile::getError() {
	return sf_error(file);
}

}
}
