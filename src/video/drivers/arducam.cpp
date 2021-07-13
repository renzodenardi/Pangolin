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

ArducamVideo::ArducamVideo(const std::string sn, const std::string config_file, bool externalTrigger)
    : is_streaming(false), externalTrigger(externalTrigger)
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
                    id = pUsbIdxArray[c].u8UsbIndex;
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

	int rtn_val = ArduCam_open(cameraHandle, cameraCfg, cameraId);
	if (rtn_val == USB_CAMERA_NO_ERROR) {
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

        ArduCam_readSensorReg(cameraHandle, 0x300C, &line_length_pck);
        pango_print_info("ArducamVideo: line_length_pck %04x\n", line_length_pck);
        vt_pix_clk_Mhz = 427; // empirically determined

        const StreamInfo stream_info(pfmt, cameraCfg->u32Width, cameraCfg->u32Height, cameraCfg->u32Width*cameraCfg->u8PixelBytes, 0);
        streams.push_back(stream_info);
        size_bytes = cameraCfg->u32Width*cameraCfg->u32Height*cameraCfg->u8PixelBytes;

	} else {
        pango_print_info("ArducamVideo: open retuned %04x\n", rtn_val);
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
    uint32_t rtn_val;

    if(!is_streaming) {
        if(externalTrigger) {
            rtn_val =  ArduCam_setMode(cameraHandle,EXTERNAL_TRIGGER_MODE);
            if(rtn_val == USB_BOARD_FW_VERSION_NOT_SUPPORT_ERROR){
                throw VideoException("Usb board firmware version does not support single mode.\n");
            }
            is_streaming = true;
        } else {
            ArduCam_setMode(cameraHandle, CONTINUOUS_MODE);
            rtn_val = ArduCam_beginCaptureImage(cameraHandle);
            if (rtn_val == USB_CAMERA_USB_TASK_ERROR) {
                throw VideoException("Unable to start streaming.");
            } else {
                pango_print_debug("ArducamVideo: streaming started %d\n",rtn_val);
                is_streaming = true;
            }
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
    uint32_t rtn_val;

    if(externalTrigger) {
        rtn_val = ArduCam_isFrameReady(cameraHandle);
        if(rtn_val != 1){
		    pango_print_debug("ArducamVideo: FrameReady Error capture image %04x\n",rtn_val);
		    return false;
        }

        rtn_val = ArduCam_getSingleFrame(cameraHandle, frameData);
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
    } else {
        rtn_val = ArduCam_captureImage(cameraHandle);

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
}

bool ArducamVideo::GrabNewest( unsigned char* image, bool wait )
{
    return GrabNext(image, wait);
}

bool ArducamVideo::SetExposure(int exp_us)
{
    if(!cameraHandle) {
        return false;
    }

    uint32_t coarse_int_time = (exp_us * 4 * vt_pix_clk_Mhz) / line_length_pck;
    uint32_t rtn_val = ArduCam_writeSensorReg(cameraHandle, 0x3012, coarse_int_time);

    if(rtn_val == USB_CAMERA_NO_ERROR) {
        return true;
    } else {
        pango_print_error("ArducamVideo::SetExposure() error %d\n",rtn_val);
        return false;
    }
}

bool ArducamVideo::GetExposure(int& exp_us)
{
    if(!cameraHandle) {
        return false;
    }

    uint32_t coarse_int_time;
    uint32_t rtn_val = ArduCam_readSensorReg(cameraHandle, 0x3012, &coarse_int_time);

    if(rtn_val == USB_CAMERA_NO_ERROR) {
        exp_us = (coarse_int_time * line_length_pck)/(vt_pix_clk_Mhz * 4.0);
        return true;
    } else {
        pango_print_error("ArducamVideo::GetExposure() error %d\n",rtn_val);
        return false;
    }
}

bool ArducamVideo::SetPeriod(int period_us)
{
    if(!cameraHandle) {
        return false;
    }

    uint16_t frame_length_lines = (period_us * vt_pix_clk_Mhz) / line_length_pck;
    uint32_t rtn_val = ArduCam_writeSensorReg(cameraHandle, 0x300A, frame_length_lines);
    if(rtn_val == USB_CAMERA_NO_ERROR) {
        return true;
    } else {
        pango_print_error("ArducamVideo::SetPeriod() error %d\n",rtn_val);
        return false;
    }
}

bool ArducamVideo::GetPeriod(int& period_us)
{
    if(!cameraHandle) {
        return false;
    }

    uint32_t frame_length_lines;
    uint32_t rtn_val = ArduCam_readSensorReg(cameraHandle, 0x300A, &frame_length_lines);
    if(rtn_val == USB_CAMERA_NO_ERROR) {
        period_us = (line_length_pck * frame_length_lines)/vt_pix_clk_Mhz;
        return true;
    } else {
        pango_print_error("ArducamVideo::GetPeriod() error %d\n",rtn_val);
        return false;
    }

}

bool ArducamVideo::SetGain(float gain)
{
    if(!cameraHandle) {
        return false;
    }

    //see datasheet Table 10.
    //note this is correct only for full res.
    float g = (gain < 1) ? 1 : gain;
    g = (gain >= 16) ? 15.95 : g;

    const uint16_t r305E_2_0 = int(log2f(g))+1;
    g = g/pow(2,r305E_2_0-1);

    uint16_t r305E_6_3 = 0;
    if(gain < 8) {
        r305E_6_3 = round(32-32/g-0.4);   // 0.4 empirically added to match datasheet
        g = g*(1-r305E_6_3/32.0);
    }
    uint16_t r305E_15_7 = round(g*64);

    uint32_t rtn_val = ArduCam_writeSensorReg(cameraHandle, 0x305E, (r305E_15_7<<7)|(r305E_6_3<<3)|r305E_2_0);
    if(rtn_val != USB_CAMERA_NO_ERROR) {
        pango_print_error("ArducamVideo::SetGain() error %d\n",rtn_val);
        return false;
    } else {
        return true;
    }
}

bool ArducamVideo::GetGain(float& gain)
{
    //see datasheet Table 10.
    if(!cameraHandle) {
        return false;
    }

    float g = 0;
    uint32_t rtn_val;

    uint32_t r305E;
    uint32_t r3040;
    //uint32_t r3EE0;
    //uint32_t r3ED2;
    // rtn_val = ArduCam_readSensorReg(cameraHandle, 0x3ED2, &r3ED2);
    // if(rtn_val != USB_CAMERA_NO_ERROR) {
    //     pango_print_error("ArducamVideo::GetGain() error %d\n",rtn_val);
    //     return false;
    // }
    rtn_val = ArduCam_readSensorReg(cameraHandle, 0x305E, &r305E);
    if(rtn_val != USB_CAMERA_NO_ERROR) {
        pango_print_error("ArducamVideo::GetGain() error %d\n",rtn_val);
        return false;
    }
    rtn_val = ArduCam_readSensorReg(cameraHandle, 0x3040, &r3040);
    if(rtn_val != USB_CAMERA_NO_ERROR) {
        pango_print_error("ArducamVideo::GetGain() error %d\n",rtn_val);
        return false;
    }
    // rtn_val = ArduCam_readSensorReg(cameraHandle, 0x3ED2, &r3EE0);
    // if(rtn_val != USB_CAMERA_NO_ERROR) {
    //     pango_print_error("ArducamVideo::GetGain() error %d\n",rtn_val);
    //     return false;
    // }

    const uint16_t r305E_2_0 = (r305E & 0x00000007);
    const uint16_t r305E_6_3 = (r305E & 0x00000078) >> 3;
    const uint16_t r305E_15_7 = (r305E & 0x0000FF80) >> 7;
    const uint16_t r3040_13 = (r3040 & 0x00002000) >> 13;
    //const uint16_t r3EE0_0 = (r3EE0 & 0x00000001);
    //const uint16_t r3ED2_15_12 = (r3ED2 & 0x0000F000) >> 12;

    g = (r305E_15_7/64.0);
    g*= pow(2,(r3040_13));//+r3EE0_0));
 //   if(r3ED2_15_12 == 0x4) {
    if(r305E_2_0 < 4) {
        g*= pow(2,(r305E_2_0-1));
        g*= 1.0/(1-(r305E_6_3/32.0));
    } else if(r305E_2_0 == 4) {
        g*= pow(2,(r305E_2_0-1));
    } else if(r305E_2_0 < 7) {
        g*= 1.5*(r305E_2_0-6);
        g*= 1.0/(1-(r305E_6_3)/32.0);
    } else {
        pango_print_warn("ArducamVideo:: invalid 0x305E[2:0] register value %d\n",r305E_2_0);
    }
    // } else if(r3ED2_15_12 == 14){
    //     if(r305E_2_0 < 4) {
    //         g*= 0.64*pow(2,(r305E_2_0-1));
    //         g*= 1.0/(1-(r305E_6_3/32.0));
    //     } else if(r305E_2_0 == 4) {
    //         g*= 0.64*pow(2,(r305E_2_0-1));
    //     } else if(r305E_2_0 < 7) {
    //         g*= 0.64*1.5*(r305E_2_0-6);
    //         g*= 1.0/(1-(r305E_6_3/32.0));
    //     } else {
    //         pango_print_warn("ArducamVideo:: invalid 0x305E[2:0] register value %d\n",r305E_2_0);
    //     }
    // } else {
    //     pango_print_warn("ArducamVideo:: invalid 0x3ED2[15:12] register value %d\n",r3ED2);
    // }

    //pango_print_warn("ArducamVideo:: GetGain() value: %f\n",g);

    gain = g;
    return true;
}

bool ArducamVideo::GetParameter(const std::string& name, std::string& result) {

    if(name.compare("Gain")==0) {
        float g;
        GetGain(g);
        result.assign(std::to_string(g));
        return true;
    } else if(name.compare("ExposureTime")==0) {
        int e;
        GetExposure(e);
        result.assign(std::to_string(e));
        return true;
    } else if(name.compare("Period")==0) {
        int p;
        GetPeriod(p);
        result.assign(std::to_string(p));
        return true;
    } else {
        pango_print_error("Parameter %s not recognized\n", name.c_str());
        return false;
    }
}

bool ArducamVideo::SetParameter(const std::string& name, const std::string& value) {

    if(name.compare("Gain")==0) {
        return SetGain(std::stof(value));
    } else if(name.compare("ExposureTime")==0) {
        return SetExposure(std::stoi(value));
    } else if(name.compare("Period")==0) {
        return SetPeriod(std::stoi(value));
    } else {
        pango_print_error("Parameter %s not recognized\n", name.c_str());
        return false;
    }
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

            const std::string sn = uri.Get<std::string>("sn","");
            const std::string cfgFile = uri.Get<std::string>("cfgFile","");
            const bool externalTrigger = uri.Get<bool>("ExternalTrigger", false);

            ArducamVideo* video_raw = new ArducamVideo(sn,cfgFile,externalTrigger);
            if(video_raw  && uri.Contains("ExposureTime")) {
                static_cast<ArducamVideo*>(video_raw)->SetExposure(uri.Get<int>("ExposureTime", 10000));
            }
            if(video_raw  && uri.Contains("Gain")) {
                static_cast<ArducamVideo*>(video_raw)->SetGain(uri.Get<int>("Gain", 1));
            }
            if(video_raw  && uri.Contains("period")) {
                static_cast<ArducamVideo*>(video_raw)->SetPeriod(uri.Get<uint64_t>("period", 10000));
            }
            return std::unique_ptr<VideoInterface>(video_raw);
        }
    };

    FactoryRegistry<VideoInterface>::I().RegisterFactory(std::make_shared<ArducamVideoFactory>(), 10, "arducam");
}

}
