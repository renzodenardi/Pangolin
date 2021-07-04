/* This file is part of the Pangolin Project.
 * http://github.com/stevenlovegrove/Pangolin
 *
 * Copyright (c) 2011 Steven Lovegrove
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

#pragma once

#include <pangolin/pangolin.h>
#include <pangolin/video/video.h>
#include <pangolin/utils/timer.h>

#ifdef _MSC_VER
// Define missing timeval struct
typedef struct timeval {
    long tv_sec;
    long tv_usec;
} timeval;
#endif // _MSC_VER

#include <ArduCamLib.h>
#include <arducam_config_parser.h>

namespace pangolin
{

class PANGOLIN_EXPORT ArducamVideo : public VideoInterface, public VideoPropertiesInterface
{
public:
    ArducamVideo(const std::string sn, const std::string config_file_name);
    ~ArducamVideo();

    void InitDevice(const char* sn, const char* config_file_name);
    void DeinitDevice();

    //! Implement VideoInput::Start()
    void Start();

    //! Implement VideoInput::Stop()
    void Stop();

    //! Implement VideoInput::SizeBytes()
    size_t SizeBytes() const;

    //! Implement VideoInput::Streams()
    const std::vector<StreamInfo>& Streams() const;

    //! Implement VideoInput::GrabNext()
    bool GrabNext( unsigned char* image, bool wait = true );

    //! Implement VideoInput::GrabNewest()
    bool GrabNewest( unsigned char* image, bool wait = true );

    //! Implement VideoUvcInterface::GetExposure()
    bool GetExposure(int& exp_us);

    //! Implement VideoUvcInterface::SetExposure()
    bool SetExposure(int exp_us);

    //! Implement VideoUvcInterface::GetGain()
    bool GetGain(float& gain);

    //! Implement VideoUvcInterface::SetGain()
    bool SetGain(float gain);

    //! Access JSON properties of device
    const picojson::value& DeviceProperties() const;

    //! Access JSON properties of most recently captured frame
    const picojson::value& FrameProperties() const;

protected:
    void InitPangoDeviceProperties();

    void ConfigBoard(Config config);

    bool FindDevice(const char *sn, int& id);

    std::vector<StreamInfo> streams;
    size_t size_bytes;

    ArduCamCfg* cameraCfg;
    ArduCamOutData* frameData;
    ArduCamHandle cameraHandle;
    int cameraId;

    picojson::value device_properties;
    picojson::value frame_properties;
    bool is_streaming;

};

}
