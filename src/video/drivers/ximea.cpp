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

XimeaVideo::XimeaVideo(const Params& p): sn(""), streaming(false)
{
    XI_RETURN stat;
    memset(&x_image,0,sizeof(x_image));
    x_image.size = sizeof(XI_IMG);

    for(Params::ParamMap::const_iterator it = p.params.begin(); it != p.params.end(); it++) {
        if(it->first == "sn"){
            sn = it->second;
        }
    }

    if(sn=="") {
        stat = xiOpenDevice(0, &xiH);
        if (stat != XI_OK)
            throw pangolin::VideoException("XimeaVideo: Unable to open first Ximea camera.");
        char c_sn[100]="";
        xiGetParamString(xiH, XI_PRM_DEVICE_SN, c_sn, sizeof(sn));
        sn.assign(c_sn);
	    pango_print_info("XimeaVideo: camera sn:%s\n",c_sn);
    } else {
        stat = xiOpenDeviceBy(XI_OPEN_BY_SN, sn.c_str(), &xiH);
        if (stat != XI_OK)
            throw pangolin::VideoException("XimeaVideo: Unable to open Ximea camera with sn:" + sn);
    }

    // Read pixel format
    PixelFormat pfmt;
    int x_fmt = 0;
    stat = xiGetParamInt(xiH, XI_PRM_IMAGE_DATA_FORMAT, &x_fmt);
    if (stat != XI_OK)
        throw pangolin::VideoException("XimeaVideo: Error getting image format.");

    switch(x_fmt) {
        case XI_MONO8:
        case XI_RAW8:
            pfmt = pangolin::PixelFormatFromString("GRAY8");
            break;
        case XI_MONO16:
        case XI_RAW16:
            pfmt = pangolin::PixelFormatFromString("GRAY16LE");
            break;
        case XI_RGB24:
            pfmt = pangolin::PixelFormatFromString("RGB24");
            break;
        case XI_RGB32:
            pfmt = pangolin::PixelFormatFromString("RGB32");
            break;
        default:
            throw pangolin::VideoException("XimeaVideo: Unknown pixel format: " + std::to_string(XI_RGB24) );
    }

    for(Params::ParamMap::const_iterator it = p.params.begin(); it != p.params.end(); it++) {
        if(it->first == "size") {
            const ImageDim dim = p.Get<ImageDim>("size", ImageDim(0,0) );
            SetParameter("Width"  , std::to_string(dim.x));
            SetParameter("Height" , std::to_string(dim.y));
        } else if(it->first == "pos") {
            const ImageDim pos = p.Get<ImageDim>("pos", ImageDim(0,0) );
            SetParameter("OffsetX"  , std::to_string(pos.x));
            SetParameter("OffsetY" , std::to_string(pos.y));
        } else if(it->first == "roi") {
            const ImageRoi roi = p.Get<ImageRoi>("roi", ImageRoi(0,0,0,0) );
            SetParameter("Width"  , std::to_string(roi.w));
            SetParameter("Height" , std::to_string(roi.h));
            SetParameter("OffsetX", std::to_string(roi.x));
            SetParameter("OffsetY", std::to_string(roi.y));
        } else {
            SetParameter(it->first, it->second);
        }
    }


    int h = 0;
    stat = xiGetParamInt(xiH, XI_PRM_HEIGHT, &h);
    if (stat != XI_OK)
        throw pangolin::VideoException("XimeaVideo: Error getting image height.");
    int w = 0;
    stat = xiGetParamInt(xiH, XI_PRM_WIDTH, &w);
    if (stat != XI_OK)
        throw pangolin::VideoException("XimeaVideo: Error getting image width.");

    const StreamInfo stream_info(pfmt, w, h, (w*pfmt.bpp) / 8, 0);
    streams.push_back(stream_info);
    size_bytes = w*h*pfmt.bpp/8;

    InitPangoDeviceProperties();
}

XimeaVideo::~XimeaVideo()
{
    XI_RETURN stat;
    stat = xiCloseDevice(xiH);
    if (stat != XI_OK)
        pango_print_error("XimeaVideo: Error closing device.");

}

bool XimeaVideo::GetParameter(const std::string& name, std::string& result)
{
    XI_RETURN stat;
    if (name.compare("Gain")==0) {
        float g;
        stat = xiGetParamFloat(xiH, XI_PRM_GAIN, &g);
        if(stat == XI_OK) {
            result.assign(std::to_string(g));
            return true;
        } else {
            pango_print_error("XimeaVideo: error getting Gain\n");
            return false;
        }
    } else if (name.compare("ExposureTime")==0) {
        float e;
        stat = xiGetParamFloat(xiH, XI_PRM_EXPOSURE, &e);
        if(stat == XI_OK) {
            result.assign(std::to_string(e));
            return true;
        } else {
            pango_print_error("XimeaVideo: error getting ExposureTime\n");
            return false;
        }
    } else {
        pango_print_error("XimeaVideo: GetParameter, param %s not recognized\n", name.c_str());
        return false;
    }
}

bool XimeaVideo::SetParameter(const std::string& name, const std::string& value)
{
    XI_RETURN stat;
    if (name.compare("Gain")==0) {
        stat = xiSetParamFloat(xiH, XI_PRM_GAIN, stof(value));
        if(stat == XI_OK) {
            return true;
        } else {
            pango_print_error("XimeaVideo: error setting Gain\n");
            return false;
        }
    } else if (name.compare("ExposureTime")==0) {
        stat = xiSetParamInt(xiH, XI_PRM_EXPOSURE, stoi(value));
        if(stat == XI_OK) {
            return true;
        } else {
            pango_print_error("XimeaVideo: error setting ExposureTime\n");
            return false;
        }
    } else if (name.compare("Width")==0) {
        stat = xiSetParamInt(xiH, XI_PRM_WIDTH, stoi(value));
        if(stat == XI_OK) {
            return true;
        } else {
            pango_print_error("XimeaVideo: error setting Width\n");
            return false;
        }
    } else if (name.compare("Height")==0) {
        stat = xiSetParamInt(xiH, XI_PRM_HEIGHT, stoi(value));
        if(stat == XI_OK) {
            return true;
        } else {
            pango_print_error("XimeaVideo: error setting Height\n");
            return false;
        }
    } else if (name.compare("OffsetX")==0) {
        stat = xiSetParamInt(xiH, XI_PRM_OFFSET_X, stoi(value));
        if(stat == XI_OK) {
            return true;
        } else {
            pango_print_error("XimeaVideo: error setting OffsetX\n");
            return false;
        }
    } else if (name.compare("OffsetY")==0) {
        stat = xiSetParamInt(xiH, XI_PRM_OFFSET_Y, stoi(value));
        if(stat == XI_OK) {
            return true;
        } else {
            pango_print_error("XimeaVideo: error setting OffsetY\n");
            return false;
        }
    } else if ((name.compare("downsampling")==0)||(name.compare("binning")==0)) {
        int v;
        if((value.compare("1")==0)||(value.compare("1x1")==0)){
            v = XI_DWN_1x1;
        } else if((value.compare("2")==0)||(value.compare("2x2")==0)){
            v = XI_DWN_2x2;
        } else if((value.compare("3")==0)||(value.compare("3x3")==0)){
            v = XI_DWN_3x3;
        } else if((value.compare("4")==0)||(value.compare("4x4")==0)){
            v = XI_DWN_4x4;
        } else if((value.compare("5")==0)||(value.compare("5x5")==0)){
            v = XI_DWN_5x5;
        } else if((value.compare("6")==0)||(value.compare("6x6")==0)){
            v = XI_DWN_6x6;
        } else if((value.compare("7")==0)||(value.compare("7x7")==0)){
            v = XI_DWN_7x7;
        } else if((value.compare("8")==0)||(value.compare("8x8")==0)){
            v = XI_DWN_8x8;
        } else if((value.compare("9")==0)||(value.compare("9x9")==0)){
            v = XI_DWN_9x9;
        } else if((value.compare("10")==0)||(value.compare("10x10")==0)){
            v = XI_DWN_10x10;
        } else if((value.compare("16")==0)||(value.compare("16x16")==0)){
            v = XI_DWN_16x16;
        } else {
            pango_print_error("XimeaVideo: unrecognized downsampling factor %s\n",value.c_str());
            return false;
        }
        stat = xiSetParamInt(xiH, XI_PRM_DOWNSAMPLING, v);
        if(stat == XI_OK) {
            return true;
        } else {
            pango_print_error("XimeaVideo: error setting downsampling\n");
            return false;
        }
    } else if (name.compare("downsampling_type")==0) {
        int v;
        if(value.compare("binning")==0){
            v = XI_BINNING;
        } else if(value.compare("skipping")==0){
            v = XI_SKIPPING;
        } else {
            pango_print_error("XimeaVideo: unrecognized downsampling_type %s\n",value.c_str());
            return false;
        }
        stat = xiSetParamInt(xiH, XI_PRM_DOWNSAMPLING_TYPE, v);
        if(stat == XI_OK) {
            return true;
        } else {
            pango_print_error("XimeaVideo: error setting downsampling_type\n");
            return false;
        }
    } else {
        pango_print_error("XimeaVideo: SetParameter, param %s not recognized\n", name.c_str());
        return false;
    }
}

void XimeaVideo::InitPangoDeviceProperties()
{
    XI_RETURN stat;
    // Store camera details in device properties
    device_properties["VendorName"] = "Ximea";
    device_properties["SerialNumber"] = sn;
    int id = 0;
    stat = xiGetParamInt(xiH, XI_PRM_DEVICE_MODEL_ID, &id);
    if(stat != XI_OK) {
        pango_print_error("XimeaVideo: error getting DeviceModelID\n");
    } else {
        device_properties["ModelID"] = std::to_string(id);
    }
    int sid = 0;
    stat = xiGetParamInt(xiH, XI_PRM_SENSOR_MODEL_ID, &sid);
    if(stat != XI_OK) {
        pango_print_error("XimeaVideo: error getting SensorModelID\n");
    } else {
        device_properties["SensorModelID"] = std::to_string(sid);
    }
    device_properties[PANGO_HAS_TIMING_DATA] = true;
}

//! Implement VideoInput::Start()
void XimeaVideo::Start()
{
    XI_RETURN stat = xiStartAcquisition(xiH);
	if (stat != XI_OK) {
        throw pangolin::VideoException("XimeaVideo: Error starting stream.");
    } else {
        streaming = true;
    }
}

//! Implement VideoInput::Stop()
void XimeaVideo::Stop()
{
    if(streaming) {
        XI_RETURN stat = xiStopAcquisition(xiH);
        if (stat != XI_OK)
            throw pangolin::VideoException("XimeaVideo: Error stopping stream.");
    }
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

void XimeaVideo::PopulateEstimatedCenterCaptureTime(basetime host_reception_time)
{
    //if(transfer_bandwidth_gbps) {
        const float transfer_time_us = 0;//size_bytes / int64_t((transfer_bandwidth_gbps * 1E3) / 8.0);
        frame_properties[PANGO_ESTIMATED_CENTER_CAPTURE_TIME_US] = picojson::value(int64_t(pangolin::Time_us(host_reception_time) -  (exposure_us/2.0) - transfer_time_us));
    //}
}

bool XimeaVideo::GrabNext(unsigned char* image, bool /*wait*/)
{
    // getting image from camera
    XI_RETURN stat = xiGetImage(xiH, 5000, &x_image);
    if(stat == XI_OK) {
        memcpy(image,x_image.bp,x_image.bp_size);
        frame_properties[PANGO_EXPOSURE_US] = picojson::value(exposure_us);
        frame_properties[PANGO_CAPTURE_TIME_US] = picojson::value(uint64_t(x_image.tsUSec+x_image.tsSec*1e6));
        basetime now = pangolin::TimeNow();
        frame_properties[PANGO_HOST_RECEPTION_TIME_US] = picojson::value(pangolin::Time_us(now));
        frame_properties["frame_number"] = picojson::value(x_image.nframe);
        //pango_print_info("frame: %5d : %10lu\n",x_image.nframe,uint64_t(x_image.tsUSec+x_image.tsSec*1e6));
        if(x_image.padding_x!=0) {
            throw pangolin::VideoException("XimeaVideo: image has non zero padding, current code does not handle this!");
        }
        PopulateEstimatedCenterCaptureTime(now);
        return true;
    } else {
        return false;
    }
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
