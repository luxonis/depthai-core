#include "depthai/utility/AudioCodecs.hpp"
/*
// WAV file header structure
struct WAVHeader {
    char riff[4];                  // "RIFF"
    uint32_t overallSize;          // file size minus 8 bytes
    char wave[4];                  // "WAVE"
    char fmtChunkMarker[4];        // "fmt " (note the space after 't')
    uint32_t lengthOfFmt;          // length of the format data
    uint16_t formatType;           // format type (1 is PCM)
    uint16_t channels;             // number of channels
    uint32_t sampleRate;           // sampling rate (blocks per second)
    uint32_t byterate;             // (Sample Rate * BitsPerSample * Channels) / 8
    uint16_t blockAlign;           // (BitsPerSample * Channels) / 8
    uint16_t bitsPerSample;        // bits per sample, 8- 8bits, 16- 16 bits, etc.
    char dataChunkHeader[4];       // "data"
    uint32_t dataSize;             // number of bytes in the data
};

void writeWAVHeader(uint8_t *buffer, uint32_t sampleRate, uint16_t channels, uint16_t bitsPerSample, uint32_t dataSize) {
    WAVHeader header;
    
    // RIFF header
    memcpy(header.riff, "RIFF", 4);
    header.overalSize = dataSize + 36;
    memcpy(header.wave, "WAVE", 4);
    
    // fmt subchunk
    memcpy(header.fmtChunkMarker, "fmt ", 4);
    header.lengthOfFmt = 16;
    header.formatType = 1; // PCM
    header.channels = channels;
    header.sampleRate = sampleRate;
    header.byterate = sampleRate * channels * bitsPerSample / 8;
    header.blockAlign = channels * bitsPerSample / 8;
    header.bitsPerSample = bitsPerSample;
    
    // data subchunk
    memcpy(header.dataChunkHeader, "data", 4);
    header.dataSize = data_size;
    
    *(WAVHeader*)buffer = header;
}

void extractAudioSegment(const char *input_file, const char *output_file, double offset_ms, double duration_ms) {
    SNDFILE *infile, *outfile;
    SF_INFO sfinfo;
    SF_INFO outsfinfo;
    sf_count_t offset_frames, duration_frames;
    int *buffer;
    int readcount;

    // Open the input file
    if (!(infile = sf_open(input_file, SFM_READ, &sfinfo))) {
        std::cerr << "Not able to open input file " << input_file << std::endl;
        std::cerr << sf_strerror(NULL) << std::endl;
        return;
    }

    // Calculate offset and duration in frames
    offset_frames = static_cast<sf_count_t>((offset_ms / 1000.0) * sfinfo.samplerate);
    duration_frames = static_cast<sf_count_t>((duration_ms / 1000.0) * sfinfo.samplerate);

    // Seek to the offset
    if (sf_seek(infile, offset_frames, SEEK_SET) == -1) {
        std::cerr << "Error seeking to offset in input file" << std::endl;
        sf_close(infile);
        return;
    }

    // Allocate buffer
    buffer = new int[duration_frames * sfinfo.channels];

    // Read frames into buffer
    readcount = sf_readf_int(infile, buffer, duration_frames);
    if (readcount != duration_frames) {
        std::cerr << "Error reading frames from input file" << std::endl;
        sf_close(infile);
        delete[] buffer;
        return;
    }

    // Prepare output file info
    outsfinfo = sfinfo;

    // Open the output file
    if (!(outfile = sf_open(output_file, SFM_WRITE, &outsfinfo))) {
        std::cerr << "Not able to open output file " << output_file << std::endl;
        std::cerr << sf_strerror(NULL) << std::endl;
        sf_close(infile);
        delete[] buffer;
        return;
    }

    // Write the buffer to the output file
    if (sf_writef_int(outfile, buffer, duration_frames) != duration_frames) {
        std::cerr << "Error writing frames to output file" << std::endl;
        sf_close(infile);
        sf_close(outfile);
        delete[] buffer;
        return;
    }

    // Clean up
    sf_close(infile);
    sf_close(outfile);
    delete[] buffer;
}

*/
namespace dai {

}
