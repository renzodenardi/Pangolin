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

#include <pangolin/factory/factory_registry.h>
#include <pangolin/video/drivers/arducam.h>
#include <pangolin/video/iostream_operators.h>
#include <unistd.h>

namespace pangolin
{

ArducamVideo::ArducamVideo(const std::string sn, const std::string config_file)
    : is_streaming(false)
{
    cameraCfg = (ArduCamCfg*) malloc (sizeof (ArduCamCfg));
    InitDevice(sn.c_str(), config_file.c_str());
    InitPangoDeviceProperties();
}

ArducamVideo::~ArducamVideo()
{
    DeinitDevice();
    free(cameraCfg);
}

bool ArducamVideo::FindDevice(const char *sn, int& id) {

    ArduCamIndexinfo pUsbIdxArray[16];
	int num_cams = ArduCam_scan(pUsbIdxArray);

    if (num_cams > 0) {
        pango_print_info("ArducamVideo: available devices:\n");
	    for (int c = 0; c < num_cams; c++) {
            pango_print_info("ArducamVideo: index:%4d\tserial:%s\n", pUsbIdxArray[c].u8UsbIndex, pUsbIdxArray[c].u8SerialNum);
	    }
        if(strcmp(sn,"") == 0) {
            pango_print_info("ArducamVideo: no serial specified opening first device found\n");
            id = pUsbIdxArray[0].u8UsbIndex;
            return true;
        } else {
	        for (int c = 0; c < num_cams; c++) {
                if(strcmp(sn,(char*)pUsbIdxArray[c].u8SerialNum)==0) {
                    pango_print_info("ArducamVideo: requested device with sn %s, found!\n",sn);
                    id = c;
                    return true;
                }
	        }
        }
	    pango_print_error("ArducamVideo: requested device with sn %s, not found!\n",sn);
        return false;
    } else {
        pango_print_error("ArducamVideo: no cameras found\n");
        return false;
    }
}

void ArducamVideo::ConfigBoard(Config config) {
	uint8_t u8Buf[10];
	for (uint32_t n = 0; n < config.params[3]; n++) {
		u8Buf[n] = config.params[4 + n];
	}
	ArduCam_setboardConfig(cameraHandle, config.params[0], config.params[1],
		config.params[2], config.params[3], u8Buf);
}

void ArducamVideo::InitDevice(const char* sn, const char* config_file_name)
{
    if(!FindDevice(sn,cameraId)) {
        throw VideoException("Unable to find ArduCam Device");
    }

	CameraConfigs cam_cfgs;
	memset(&cam_cfgs, 0x00, sizeof(CameraConfigs));
	if (arducam_parse_config(config_file_name, &cam_cfgs)) {
		pango_print_error("ArducamVideo: cannot find configuration file %s\n",config_file_name);
		throw VideoException("Unable to open ArduCam Device");
	}

	CameraParam *cam_param = &cam_cfgs.camera_param;
	Config *configs = cam_cfgs.configs;
	int configs_length = cam_cfgs.configs_length;

	switch (cam_param->i2c_mode) {
	    case 0: cameraCfg->emI2cMode = I2C_MODE_8_8; break;
	    case 1: cameraCfg->emI2cMode = I2C_MODE_8_16; break;
	    case 2: cameraCfg->emI2cMode = I2C_MODE_16_8; break;
	    case 3: cameraCfg->emI2cMode = I2C_MODE_16_16; break;
	    default: break;
	}

	if (cameraCfg->u8PixelBits <= 8) {
		cameraCfg->u8PixelBytes = 1;
	} else {
		cameraCfg->u8PixelBytes = 2;
	}

    // Default to greyscale.
    PixelFormat pfmt = PixelFormatFromString("GRAY8");

	switch (cam_param->format >> 8) {
	    case 0:
            cameraCfg->emImageFmtMode = FORMAT_MODE_RAW;
            if(cameraCfg->u8PixelBits == 8) pfmt = PixelFormatFromString("GRAY8");
            if(cameraCfg->u8PixelBits == 10) pfmt = PixelFormatFromString("GRAY10");
            if(cameraCfg->u8PixelBits == 12) pfmt = PixelFormatFromString("GRAY12");
            if(cameraCfg->u8PixelBits == 16) pfmt = PixelFormatFromString("GRAY16LE");
            break;
	    case 1:
            // untested
            pango_print_info("ArducamVideo:: FORMAT_MODE_RGB untested image format\n");
            cameraCfg->emImageFmtMode = FORMAT_MODE_RGB;
            pfmt = PixelFormatFromString("RGB24");
            break;
	    case 2:
            // untested
            pango_print_info("ArducamVideo:: FORMAT_MODE_YUV untested image format\n");
            cameraCfg->emImageFmtMode = FORMAT_MODE_YUV;
            pfmt = PixelFormatFromString("YUYU422");
            break;
	    default:
            cameraCfg->emImageFmtMode = FORMAT_MODE_JPG;
            pango_print_info("ArducamVideo:: detected image format %d\n", (cam_param->format >> 8));
            throw VideoException("Image format unsupported by in Pangolin");
            break;
	}

	cameraCfg->u32Width = cam_param->width;
	cameraCfg->u32Height = cam_param->height;

	cameraCfg->u32I2cAddr = cam_param->i2c_addr;
	cameraCfg->u8PixelBits = cam_param->bit_width;
	cameraCfg->u32TransLvl = cam_param->trans_lvl;

	int ret_val = ArduCam_open(cameraHandle, cameraCfg, cameraId);
	if (ret_val == USB_CAMERA_NO_ERROR) {
		//ArduCam_enableForceRead(cameraHandle);	//Force display image
		for (int i = 0; i < configs_length; i++) {
			uint32_t type = configs[i].type;
			if (((type >> 16) & 0xFF) && ((type >> 16) & 0xFF) != cameraCfg->usbType)
				continue;
			switch (type & 0xFFFF) {
			case CONFIG_TYPE_REG:
				ArduCam_writeSensorReg(cameraHandle, configs[i].params[0], configs[i].params[1]);
				break;
			case CONFIG_TYPE_DELAY:
				usleep(1000 * configs[i].params[0]);
				break;
			case CONFIG_TYPE_VRCMD:
				ConfigBoard(configs[i]);
				break;
			}
		}
		ArduCam_registerCtrls(cameraHandle, cam_cfgs.controls, cam_cfgs.controls_length);

        const StreamInfo stream_info(pfmt, cameraCfg->u32Width, cameraCfg->u32Height, cameraCfg->u32Width*cameraCfg->u8PixelBytes, 0);
        streams.push_back(stream_info);
        size_bytes = cameraCfg->u32Width*cameraCfg->u32Height*cameraCfg->u8PixelBytes;

	} else {
        pango_print_info("ArducamVideo: open retuned %04x\n", ret_val);
        throw VideoException("Unable to open ArduCam Device");
    }
}

void ArducamVideo::InitPangoDeviceProperties()
{
    // Store camera details in device properties
    //device_properties["BusNumber"] = std::to_string(uvc_get_bus_number(dev_));
    //device_properties["DeviceAddress"] = std::to_string(uvc_get_device_address(dev_));
    device_properties[PANGO_HAS_TIMING_DATA] = true;
}

void ArducamVideo::DeinitDevice()
{
    Stop();
    ArduCam_close(cameraHandle);
}

void ArducamVideo::Start()
{
    if(!is_streaming) {
        ArduCam_setMode(cameraHandle, CONTINUOUS_MODE);
        uint32_t rtn_val = ArduCam_beginCaptureImage(cameraHandle);

        if (rtn_val == USB_CAMERA_USB_TASK_ERROR) {
            throw VideoException("Unable to start streaming.");
        } else {
            pango_print_debug("ArducamVideo: streaming started %d\n",rtn_val);
            is_streaming = true;
        }
    }
}

void ArducamVideo::Stop()
{
    if(is_streaming) {
        ArduCam_endCaptureImage(cameraHandle);
    }
    is_streaming = false;
}

size_t ArducamVideo::SizeBytes() const
{
    return size_bytes;
}

const std::vector<StreamInfo>& ArducamVideo::Streams() const
{
    return streams;
}

bool ArducamVideo::GrabNext( unsigned char* image, bool /*wait*/ )
{
    uint32_t rtn_val = ArduCam_captureImage(cameraHandle);

	if ((rtn_val == USB_CAMERA_USB_TASK_ERROR) || (rtn_val > 0xFF)) {
		pango_print_debug("ArducamVideo: Error capture image %04x\n",rtn_val);
		return false;
	}

    if (ArduCam_availableImage(cameraHandle) > 0) {
	    rtn_val = ArduCam_readImage(cameraHandle, frameData);
        if (rtn_val == USB_CAMERA_NO_ERROR) {
            memcpy(image, frameData->pu8ImageData, size_bytes);
            // This is a hack, this ts should come from the device, unfortunately frameData->uTime64 is always zero.
            frame_properties[PANGO_HOST_RECEPTION_TIME_US] = picojson::value(pangolin::Time_us(pangolin::TimeNow()));
            ArduCam_del(cameraHandle);
            return true;
        } else {
            pango_print_error("ArducamVideo: Error read image %04x\n", rtn_val);
            return false;
        }
    }
    return false;
}

bool ArducamVideo::GrabNewest( unsigned char* image, bool wait )
{
    return GrabNext(image, wait);
}

bool ArducamVideo::SetExposure(int /*exp_us*/)
{
    //uint32_t e = uint32_t(exp_us);

    //if (uvc_set_exposure_abs(devh_, e) < 0) {
        pango_print_warn("ArducamVideo::setExposure() not implemented\n");
        return false;
    //} else {
    //    return true;
    //}
}

bool ArducamVideo::GetExposure(int& /*exp_us*/)
{
    //uint32_t e;
    //if (uvc_get_exposure_abs(devh_, &e, uvc_req_code::UVC_GET_CUR) < 0) {
        pango_print_warn("ArducamVideo::GetExposure() not implemented\n");
        return false;
    //} else {
    //    exp_us = e;
    //   return true;
    //}
}

bool ArducamVideo::SetGain(float /*gain*/)
{
    //uint16_t g = uint16_t(gain);

    //if (uvc_set_gain(devh_, g) < 0) {
        pango_print_warn("ArducamVideo::setGain() not implemented\n");
        return false;
    //} else {
    //    return true;
    //}
}

bool ArducamVideo::GetGain(float& /*gain*/)
{
    //uint16_t g;
    //if (uvc_get_gain(devh_, &g, uvc_req_code::UVC_GET_CUR) < 0) {
        pango_print_warn("ArducamVideo::GetGain() not implemented\n");
        return false;
    //} else {
    //   gain = g;
    //    return true;
    //}
}

//! Access JSON properties of device
const picojson::value& ArducamVideo::DeviceProperties() const
{
    return device_properties;
}

//! Access JSON properties of most recently captured frame
const picojson::value& ArducamVideo::FrameProperties() const
{
    return frame_properties;
}

PANGOLIN_REGISTER_FACTORY(ArducamVideo)
{
    struct ArducamVideoFactory final : public FactoryInterface<VideoInterface> {
        std::unique_ptr<VideoInterface> Open(const Uri& uri) override {
            std::cout<<"*>"<<uri.url<<"<*"<<std::endl;
            const std::string sn = uri.Get<std::string>("sn","");
            const std::string cfgFile = uri.Get<std::string>("cfgFile","");
            //const ImageDim dim = uri.Get<ImageDim>("size", ImageDim(640,480));
            return std::unique_ptr<VideoInterface>( new ArducamVideo(sn,cfgFile) );
        }
    };

    FactoryRegistry<VideoInterface>::I().RegisterFactory(std::make_shared<ArducamVideoFactory>(), 10, "arducam");
}

}
