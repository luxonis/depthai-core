#include "depthai/pipeline/datatype/AudioFrame.hpp"

namespace dai {

sf_count_t virtualGetSize(void *userData) {
    AudioFrame* audioFrame = static_cast<AudioFrame*>(userData);
    return audioFrame->getSize();
}

sf_count_t virtualRead(void* ptr, sf_count_t count, void *userData) {
    AudioFrame* audioFrame = static_cast<AudioFrame*>(userData);
    return audioFrame->read(static_cast<float*>(ptr), count);
}

sf_count_t virtualWrite(const void* ptr, sf_count_t count, void *userData) {
    AudioFrame* audioFrame = static_cast<AudioFrame*>(userData);
    return audioFrame->write(static_cast<const float*>(ptr), count);
}

sf_count_t virtualSeek(sf_count_t offset, int whence, void *userData) {
    AudioFrame* audioFrame = static_cast<AudioFrame*>(userData);
    return audioFrame->seek(offset, whence);
}

sf_count_t virtualTell(void *userData) {
    AudioFrame* audioFrame = static_cast<AudioFrame*>(userData);
    return audioFrame->tell();
}

AudioFrame::AudioFrame(SF_INFO info) {
//    sf_info.samplerate = AUDIO_SAMPLE_RATE;
//    sf_info.frames = 0;
//    sf_info.channels = AUDIO_CHANNELS;
//    sf_info.format = SF_FORMAT_WAV | SF_FORMAT_PCM_16;
//

    auto mem = std::make_shared<VectorMemory>();
    mem->resize(0);
    data = mem;

    // Open the virtual file for reading
    position = 0;
    dataReady = false;

    SF_VIRTUAL_IO virtualIo = {
        &virtualGetSize,
        &virtualSeek,
        &virtualRead,
        &virtualWrite,
        &virtualTell
    };

    AudioFile(&virtualIo, SFM_RDWR, info, (void*)this);
}

sf_count_t AudioFrame::getSize() {
    std::lock_guard<std::mutex> lock(this->mtx);
    return this->data->getData().size();
}

sf_count_t AudioFrame::seek(sf_count_t offset, int whence) {
    std::lock_guard<std::mutex> lock(this->mtx);
    sf_count_t newPosition = 0;

    switch (whence) {
    case SEEK_SET:
        newPosition = offset;
        break;
    case SEEK_CUR:
        newPosition = this->position + offset;
        break;
    case SEEK_END:
        newPosition = this->data->getData().size() + offset;
        break;
    default:
        return -1;
    }

    if (newPosition < 0 || newPosition > static_cast<sf_count_t>(this->data->getData().size())) {
        return -1;
    }

    this->position = newPosition;
    return newPosition;
}

sf_count_t AudioFrame::read(void* ptr, sf_count_t count) {
    std::unique_lock<std::mutex> lock(this->mtx);

    // Wait until data is ready
    this->cv.wait(lock, [this] { return this->dataReady; });

    if (this->position + count > static_cast<sf_count_t>(this->data->getData().size())) {
        count = this->data->getData().size() - this->position;
    }

    memcpy(ptr, this->data->getData().data() + this->position, count);
    this->position += count;

    // Clear the read section
    memset(this->data->getData().data(), 0, this->position);
    this->position = 0;
    this->dataReady = false;
    return count;
}

sf_count_t AudioFrame::write(const void* ptr, sf_count_t count) {
(void)ptr; (void) count;
    // TODO
    return 0;
}

sf_count_t AudioFrame::tell() {
    std::lock_guard<std::mutex> lock(this->mtx);
    return this->position;
}
/*
extern "C" {
#include <sndfile.h>
}

void audio_capture_loop(VirtualSoundFile* this, snd_pcm_t* capture_handle, char* audio_buffer, snd_pcm_uframes_t frames) {
    while (true) {
        int ret = snd_pcm_readi(capture_handle, audio_buffer, frames);
        if (ret == -EPIPE) {
            std::cerr << "Overrun occurred" << std::endl;
            snd_pcm_prepare(capture_handle);
            continue;
        } else if (ret < 0) {
            std::cerr << "Error from read: " << snd_strerror(ret) << std::endl;
            break;
        } else if (ret != static_cast<int>(frames)) {
            std::cerr << "Short read, read " << ret << " frames" << std::endl;
            continue;
        }

        {
            std::lock_guard<std::mutex> lock(this->mtx);
            this->data.insert(this->data.end(), audio_buffer, audio_buffer + ret * AUDIO_CHANNELS * 2);
            this->dataReady = true;
        }
        this->cv.notify_one();
    }
}

void capture_audio_to_virtual_file() {
    // ALSA setup
    snd_pcm_t* capture_handle;
    snd_pcm_hw_params_t* hw_params;
    unsigned int rate = AUDIO_SAMPLE_RATE;
    int dir;
    snd_pcm_uframes_t frames;
    char* audio_buffer;

    snd_pcm_open(&capture_handle, "default", SND_PCM_STREAM_CAPTURE, 0);
    snd_pcm_hw_params_alloca(&hw_params);
    snd_pcm_hw_params_any(capture_handle, hw_params);
    snd_pcm_hw_params_set_access(capture_handle, hw_params, SND_PCM_ACCESS_RW_INTERLEAVED);
    snd_pcm_hw_params_set_format(capture_handle, hw_params, SND_PCM_FORMAT_S16_LE);
    snd_pcm_hw_params_set_channels(capture_handle, hw_params, AUDIO_CHANNELS);
    snd_pcm_hw_params_set_rate_near(capture_handle, hw_params, &rate, &dir);
    snd_pcm_hw_params_set_period_size_near(capture_handle, hw_params, &frames, &dir);
    snd_pcm_hw_params(capture_handle, hw_params);

    snd_pcm_hw_params_get_period_size(hw_params, &frames, &dir);
    size_t audio_buffer_size = frames * AUDIO_CHANNELS * 2; // 2 bytes/sample
    audio_buffer = new char[audio_buffer_size];

    // Virtual sound file setup
    VirtualSoundFile this = { std::vector<char>(), 0, std::mutex(), std::condition_variable(), false };
    SF_VIRTUAL_IO virtual_io = {
        &virtual_get_filelen,
        &virtual_seek,
        &virtual_read,
        &virtual_write,
        &virtual_tell
    };

    SF_INFO sf_info;
    sf_info.samplerate = AUDIO_SAMPLE_RATE;
    sf_info.frames = 0;
    sf_info.channels = AUDIO_CHANNELS;
    sf_info.format = SF_FORMAT_WAV | SF_FORMAT_PCM_16;

    SNDFILE* sndfile = sf_open_virtual(&virtual_io, SFM_READ, &sf_info, &this);
    if (!sndfile) {
        std::cerr << "Error opening virtual sound file" << std::endl;
        delete[] audio_buffer;
        snd_pcm_close(capture_handle);
        return;
    }

    // Start audio capture in a separate thread
    std::thread capture_thread(audio_capture_loop, &this, capture_handle, audio_buffer, frames);
    capture_thread.detach();
}

int main(int argc, char* argv[]) {
    capture_audio_to_virtual_file();

    // Simulate reading from the virtual file
    char read_buffer[AUDIO_BUFFER_SIZE];


    if (!sndfile) {
        std::cerr << "Error opening virtual sound file" << std::endl;
        return 1;
    }

    while (true) {
        sf_count_t read_count = sf_read_short(sndfile, reinterpret_cast<short*>(read_buffer), AUDIO_BUFFER_SIZE / 2); // 2 bytes per sample
        if (read_count > 0) {
            // Process read_buffer data
            std::cout << "Read " << read_count << " samples" << std::endl;
        }
    }

    sf_close(sndfile);
    return 0;
}*/

}  // namespace dai
