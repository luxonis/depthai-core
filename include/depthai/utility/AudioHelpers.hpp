#pragma once
#include <vector>
#include <string>

extern "C" {
#include <sndfile.h>
#include <samplerate.h>
}

namespace dai {
namespace audio {
	class AudioDevice {
	public:
		std::string name = "";
		std::string desc = "";
		std::string ioid = "";
	};

	class AudioFile {
	public:
		AudioFile() {}
		AudioFile(const char *path, int mode);
		AudioFile(int fd, int mode, bool closeOnDestruct);
		AudioFile(SF_VIRTUAL_IO *virtualFile, int mode, void *userData);
		AudioFile(SF_VIRTUAL_IO *virtualFile, int mode, SF_INFO fileInfo, void *userData);

		~AudioFile();

		sf_count_t readItem(int *ptr, sf_count_t items);

		sf_count_t readFrame(short *ptr, sf_count_t frames);
		sf_count_t readFrame(int *ptr, sf_count_t frames);
		sf_count_t readFrame(float *ptr, sf_count_t frames);
		sf_count_t readFrame(double *ptr, sf_count_t frames);

		sf_count_t readRaw(void *ptr, sf_count_t bytes);

		sf_count_t writeItem(int *ptr, sf_count_t items);
		sf_count_t writeFrame(int *ptr, sf_count_t frames);
		sf_count_t writeRaw(void *ptr, sf_count_t bytes);

		sf_count_t seek(sf_count_t frames, int whence);

		SF_INFO getInfo() const { return fileInfo; }
		int getFormat() const;
		int getSampleSize() const;
		int getError();
	protected:
		SNDFILE *file;
		SF_INFO fileInfo;
	};

	std::vector<AudioDevice> ListAlsaDevices();

}
}
