/* This file is part of the Pangolin Project.
 * http://github.com/stevenlovegrove/Pangolin
 *
 * Copyright (c) 2015 Steven Lovegrove
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


#include <pangolin/factory/factory_registry.h>
#include <pangolin/video/drivers/ximea.h>
#include <pangolin/video/iostream_operators.h>

namespace pangolin
{

void HandleResult(XI_RETURN res,std::string place) {
    if (res!=XI_OK) {
        pango_print_error("Error after %s\n", place.c_str());
    }
}

XimeaVideo::XimeaVideo(const Params& p)
{
    memset(&x_image,0,sizeof(x_image));
    x_image.size = sizeof(XI_IMG);

    XI_RETURN stat = xiOpenDevice(0, &xiH);
	HandleResult(stat,"xiOpenDevice");

    // Setting "exposure" parameter (10ms=10000us)
	stat = xiSetParamInt(xiH, XI_PRM_EXPOSURE, 10000);
	HandleResult(stat,"xiSetParam (exposure set)");

    InitPangoDeviceProperties();
}

XimeaVideo::~XimeaVideo()
{
    xiCloseDevice(xiH);
}

bool XimeaVideo::GetParameter(const std::string& name, std::string& result)
{
    if (name.compare("Gain")==0) {
        float g;
        xiGetParamFloat(xiH, XI_PRM_GAIN, &g);
        result.assign(std::to_string(g));
        return true;
    } else if (name.compare("ExposureTime")==0) {
        float e;
        xiGetParamFloat(xiH, XI_PRM_EXPOSURE, &e);
        result.assign(std::to_string(e));
        return true;
    } else {
        pango_print_error("Parameter %s not recognized\n", name.c_str());
        return false;
    }
}

bool XimeaVideo::SetParameter(const std::string& name, const std::string& value)
{
    if (name.compare("Gain")==0) {
        xiSetParamFloat(xiH, XI_PRM_GAIN, stof(value));
        return true;
    } else if (name.compare("ExposureTime")==0) {
        xiSetParamInt(xiH, XI_PRM_EXPOSURE, stoi(value));
        return true;
    } else {
        pango_print_error("Parameter %s not recognized\n", name.c_str());
        return false;
    }
}

void XimeaVideo::InitPangoDeviceProperties()
{

    // Teli::CAM_INFO info;
    // Teli::Cam_GetInformation(cam, 0, &info);

    // Store camera details in device properties
    // device_properties["SerialNumber"] = std::string(info.szSerialNumber);
    // device_properties["VendorName"] = std::string(info.szManufacturer);
    // device_properties["ModelName"] = std::string(info.szModelName);
    // device_properties["ManufacturerInfo"] = std::string(info.sU3vCamInfo.szManufacturerInfo);
    // device_properties["Version"] = std::string(info.sU3vCamInfo.szDeviceVersion);
    device_properties[PANGO_HAS_TIMING_DATA] = true;

    // TODO: Enumerate other settings.
}

//! Implement VideoInput::Start()
void XimeaVideo::Start()
{
    XI_RETURN stat = xiStartAcquisition(xiH);
	HandleResult(stat,"xiStartAcquisition");
}

//! Implement VideoInput::Stop()
void XimeaVideo::Stop()
{
    xiStopAcquisition(xiH);
}

//! Implement VideoInput::SizeBytes()
size_t XimeaVideo::SizeBytes() const
{
    return size_bytes;
}

//! Implement VideoInput::Streams()
const std::vector<StreamInfo>& XimeaVideo::Streams() const
{
    return streams;
}

bool XimeaVideo::GrabNext(unsigned char* image, bool /*wait*/)
{
    // getting image from camera
    XI_RETURN stat = xiGetImage(xiH, 5000, &x_image);
    HandleResult(stat,"xiGetImage");
    unsigned char pixel = *(unsigned char*)x_image.bp;
    printf("Image %dx%d received from camera. First pixel value: %d\n", (int)x_image.width, (int)x_image.height, pixel);

    return false;
}

//! Implement VideoInput::GrabNewest()
bool XimeaVideo::GrabNewest(unsigned char* image, bool wait)
{
    return GrabNext(image,wait);
}

//! Access JSON properties of device
const picojson::value& XimeaVideo::DeviceProperties() const
{
    return device_properties;
}

//! Access JSON properties of most recently captured frame
const picojson::value& XimeaVideo::FrameProperties() const
{
    return frame_properties;
}

PANGOLIN_REGISTER_FACTORY(XimeaVideo)
{
    struct XimeaVideoFactory final : public FactoryInterface<VideoInterface> {
        std::unique_ptr<VideoInterface> Open(const Uri& uri) override {
            return std::unique_ptr<VideoInterface>(new XimeaVideo(uri));
        }
    };

    auto factory = std::make_shared<XimeaVideoFactory>();
    FactoryRegistry<VideoInterface>::I().RegisterFactory(factory, 10, "ximea");
}

}
