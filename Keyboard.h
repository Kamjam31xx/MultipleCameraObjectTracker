#pragma once

#include "ImgProc.h"
#include "AppSettings.h"

void HandleKeyboard(DeltaTime& _timer, cv::VideoCapture& _cap, AppSettings& appSettings) {

	CameraSettings& _camera = *appSettings.cameras[0];
	ImageProcessSettings& _process = *appSettings.imgProcessing;
	TrackingSettings& _tracking = *appSettings.tracking;
	CaptureSettings& _capturing = *appSettings.capturing;
	RenderSettings& _rendering = *appSettings.rendering;

	if (_kbhit())
	{
		char key = _getch();
		switch (key) {
		case '0': 
		case 'q':
			_camera.exposure += float(_timer.getDeltaMilliSeconds() / 100.0);
			_cap.set(cv::CAP_PROP_EXPOSURE, _camera.exposure);
			logSetting("exposure", _camera.exposure);
			break;
		case 'a':
			_camera.exposure -= float(_timer.getDeltaMilliSeconds() / 100.0);
			_cap.set(cv::CAP_PROP_EXPOSURE, _camera.exposure);
			logSetting("exposure", _camera.exposure);
			break;
		case 'w':
			_process.threshold += float(_timer.getDeltaMilliSeconds() / 50.0);
			logSetting("threshold", _process.threshold);
			break;
		case 's':
			_process.threshold -= float(_timer.getDeltaMilliSeconds() / 50.0);
			logSetting("threshold", _process.threshold);
			break;
		case 'e':
			_tracking.areaMinSize += float(_timer.getDeltaMilliSeconds() / 10.0);
			logSetting("min size", _tracking.areaMinSize);
			break;
		case 'd':
			_tracking.areaMinSize -= float(_timer.getDeltaMilliSeconds() / 10.0);
			logSetting("min size", _tracking.areaMinSize);
			break;
		case 'r':
			_tracking.distMod += float(_timer.getDeltaMilliSeconds() / 10000.0);
			logSetting("distMod", _tracking.distMod);
			break;
		case 'f':
			_tracking.distMod -= float(_timer.getDeltaMilliSeconds() / 10000.0);
			logSetting("distMod", _tracking.distMod);
			break;
		case 't':
			_tracking.areaMod += float(_timer.getDeltaMilliSeconds() / 10000.0);
			logSetting("areaMod", _tracking.areaMod);
			break;
		case 'g':
			_tracking.areaMod -= float(_timer.getDeltaMilliSeconds() / 10000.0);
			logSetting("areaMod", _tracking.areaMod);
			break;
		case 'y':
			_tracking.rectMod += float(_timer.getDeltaMilliSeconds() / 10000.0);
			logSetting("rectMod", _tracking.rectMod);
			break;
		case 'h':
			_tracking.rectMod -= float(_timer.getDeltaMilliSeconds() / 10000.0);
			logSetting("rectMod", _tracking.rectMod);
			break;
		case 'u':
			_tracking.rectAreaRatioMod += float(_timer.getDeltaMilliSeconds() / 10000.0);
			logSetting("rect : area mod", _tracking.rectAreaRatioMod);
			break;
		case 'j':
			_tracking.rectAreaRatioMod -= float(_timer.getDeltaMilliSeconds() / 10000.0);
			logSetting("rect : area mod", _tracking.rectAreaRatioMod);
			break;
		case 'i':
			_tracking.discardThreshold += float(_timer.getDeltaMilliSeconds() / 10000.0);
			logSetting("discard threshold", _tracking.discardThreshold);
			break;
		case 'k':
			_tracking.discardThreshold -= float(_timer.getDeltaMilliSeconds() / 10000.0);
			logSetting("discard threshold", _tracking.discardThreshold);
			break;
		case 'o':
			_tracking.acceptThreshold += float(_timer.getDeltaMilliSeconds() / 10000.0);
			logSetting("accept threshold", _tracking.acceptThreshold);
			break;
		case 'l':
			_tracking.acceptThreshold -= float(_timer.getDeltaMilliSeconds() / 10000.0);
			logSetting("accept threshold", _tracking.acceptThreshold);
			break;
		case 'p':
			_tracking.distScale += float(_timer.getDeltaMilliSeconds() / 10000.0);
			logSetting("dist scale", _tracking.distScale);
			break;
		case ';':
			_tracking.distScale -= float(_timer.getDeltaMilliSeconds() / 10000.0);
			logSetting("dist scale", _tracking.distScale);
			break;
		case '[':
			_tracking.postMod += float(_timer.getDeltaMilliSeconds() / 10000.0);
			logSetting("post modPAR", _tracking.postMod);
			break;
		case '\'':
			_tracking.postMod -= float(_timer.getDeltaMilliSeconds() / 10000.0);
			logSetting("poast modPAR", _tracking.postMod);
			break;
		case 'b':
			_capturing.wait = !_capturing.wait;
			logSetting("wait", _capturing.wait);
			break;
		case 'n':
			_capturing.advanceFrame = true;
			logSetting("advanced frame");
			break;
		case 'z':
			_process.erode = !_process.erode;
			logSetting("erode", _process.erode);
			break;
		case 'x':
			_process.dilate = !_process.dilate;
			logSetting("dilate", _process.dilate);
			break;
		case 'c':
			_process.blur = !_process.blur;
			logSetting("blur", _process.blur);
			break;
		case 'v':
			_rendering.color = !_rendering.color;
			logSetting("color", _rendering.color);
			break;
		case '<':
			_capturing.cvwaitSlider += float(_timer.getDeltaMilliSeconds() / 500.0);
			_capturing.cvwait = 1 + abs(floor(_capturing.cvwaitSlider));
			log("cv wait", _capturing.cvwait);
			break;
		case '>':
			_capturing.cvwaitSlider -= float(_timer.getDeltaMilliSeconds() / 500.0);
			_capturing.cvwait = 1 + abs(floor(_capturing.cvwaitSlider));
			log("cv wait", _capturing.cvwait);
			break;
		case 'm':
			_capturing.lag = !_capturing.lag;
			logSetting("lag" + _capturing.lag);
			break;
		case '/':
			_process.sobel = !_process.sobel;
			logSetting("pre-process | sobel", _process.sobel);
			break;
		case ']':
			log("  ");
			log("  __SETTINGS___________________________________________");
			log("  ");
			log("                exposure |  " + std::to_string(_camera.exposure));
			log("               threshold |  " + std::to_string(_process.threshold));
			log("                          ");
			log("                min size |  " + std::to_string(_tracking.areaMinSize));
			log("                          ");
			log("                 distMod |  " + std::to_string(_tracking.distMod));
			log("                 areaMod |  " + std::to_string(_tracking.areaMod));
			log("                 rectMod |  " + std::to_string(_tracking.rectMod));
			log("         rect : area mod |  " + std::to_string(_tracking.rectAreaRatioMod));
			log("            post mod PAR |  " + std::to_string(_tracking.postMod));
			log("                          ");
			log("       discard threshold |  " + std::to_string(_tracking.discardThreshold));
			log("        accept threshold |  " + std::to_string(_tracking.acceptThreshold));
			log("                          ");
			log("              dist scale |  " + std::to_string(_tracking.distScale));
			log("                          ");
			log("                    wait |  " + std::to_string(_capturing.wait));
			log("           advance frame |  " + std::to_string(_capturing.advanceFrame));
			log("                          ");
			log("                   erode |  " + std::to_string(_process.erode));
			log("                  dilate |  " + std::to_string(_process.dilate));
			log("                    blur |  " + std::to_string(_process.blur));
			log("                   color |  " + std::to_string(_rendering.color));
			log("                          ");
			log("                 cv wait |  " + std::to_string(_capturing.cvwait));
			log("  ");
			break;
		}

		if (_camera.exposure < -15) { _camera.exposure = -15; }
		else if (_camera.exposure > 15) { _camera.exposure = 15; }
		if (_process.threshold < 0) { _process.threshold = 0; }
		else if (_process.threshold > 255) { _process.threshold = 255; }
		_tracking.areaMinSize = _tracking.areaMinSize < 0 ? 0 : _tracking.areaMinSize;
		_tracking.distMod = _tracking.distMod < 0 ? 0 : _tracking.distMod;
		_tracking.areaMod = _tracking.areaMod < 0 ? 0 : _tracking.areaMod;
		_tracking.rectMod = _tracking.rectMod < 0 ? 0 : _tracking.rectMod;
		_tracking.rectAreaRatioMod = _tracking.rectAreaRatioMod < 0 ? 0 : _tracking.rectAreaRatioMod;
		_tracking.discardThreshold = _tracking.discardThreshold < 0 ? 0 : _tracking.discardThreshold;
		_tracking.acceptThreshold = _tracking.acceptThreshold < 0 ? 0 : _tracking.acceptThreshold;
	}
}