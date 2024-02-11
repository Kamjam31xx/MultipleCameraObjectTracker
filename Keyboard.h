#pragma once

#include "ImgProc.h"

void HandleKeyboard(ProcessSettings& _process, CameraSettings& _camera, DeltaTime& _timer, cv::VideoCapture& _cap) {


	if (_kbhit())
	{
		char key = _getch();
		switch (key) {
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
			_process.areaMinSize += float(_timer.getDeltaMilliSeconds() / 10.0);
			logSetting("min size", _process.areaMinSize);
			break;
		case 'd':
			_process.areaMinSize -= float(_timer.getDeltaMilliSeconds() / 10.0);
			logSetting("min size", _process.areaMinSize);
			break;
		case 'r':
			_process.distMod += float(_timer.getDeltaMilliSeconds() / 10000.0);
			logSetting("distMod", _process.distMod);
			break;
		case 'f':
			_process.distMod -= float(_timer.getDeltaMilliSeconds() / 10000.0);
			logSetting("distMod", _process.distMod);
			break;
		case 't':
			_process.areaMod += float(_timer.getDeltaMilliSeconds() / 10000.0);
			logSetting("areaMod", _process.areaMod);
			break;
		case 'g':
			_process.areaMod -= float(_timer.getDeltaMilliSeconds() / 10000.0);
			logSetting("areaMod", _process.areaMod);
			break;
		case 'y':
			_process.rectMod += float(_timer.getDeltaMilliSeconds() / 10000.0);
			logSetting("rectMod", _process.rectMod);
			break;
		case 'h':
			_process.rectMod -= float(_timer.getDeltaMilliSeconds() / 10000.0);
			logSetting("rectMod", _process.rectMod);
			break;
		case 'u':
			_process.rectAreaRatioMod += float(_timer.getDeltaMilliSeconds() / 10000.0);
			logSetting("rect : area mod", _process.rectAreaRatioMod);
			break;
		case 'j':
			_process.rectAreaRatioMod -= float(_timer.getDeltaMilliSeconds() / 10000.0);
			logSetting("rect : area mod", _process.rectAreaRatioMod);
			break;
		case 'i':
			_process.discardThreshold += float(_timer.getDeltaMilliSeconds() / 10000.0);
			logSetting("discard threshold", _process.discardThreshold);
			break;
		case 'k':
			_process.discardThreshold -= float(_timer.getDeltaMilliSeconds() / 10000.0);
			logSetting("discard threshold", _process.discardThreshold);
			break;
		case 'o':
			_process.acceptThreshold += float(_timer.getDeltaMilliSeconds() / 10000.0);
			logSetting("accept threshold", _process.acceptThreshold);
			break;
		case 'l':
			_process.acceptThreshold -= float(_timer.getDeltaMilliSeconds() / 10000.0);
			logSetting("accept threshold", _process.acceptThreshold);
			break;
		case 'p':
			_process.distScale += float(_timer.getDeltaMilliSeconds() / 10000.0);
			logSetting("dist scale", _process.distScale);
			break;
		case ';':
			_process.distScale -= float(_timer.getDeltaMilliSeconds() / 10000.0);
			logSetting("dist scale", _process.distScale);
			break;
		case '[':
			_process.postMod += float(_timer.getDeltaMilliSeconds() / 10000.0);
			logSetting("post modPAR", _process.postMod);
			break;
		case '\'':
			_process.postMod -= float(_timer.getDeltaMilliSeconds() / 10000.0);
			logSetting("poast modPAR", _process.postMod);
			break;
		case 'b':
			_process.wait = !_process.wait;
			logSetting("wait", _process.wait);
			break;
		case 'n':
			_process.advanceFrame = true;
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
			_process.color = !_process.color;
			logSetting("color", _process.color);
			break;
		case '<':
			_process.cvwaitSlider += float(_timer.getDeltaMilliSeconds() / 500.0);
			_process.cvwait = 1 + abs(floor(_process.cvwaitSlider));
			log("cv wait", _process.cvwait);
			break;
		case '>':
			_process.cvwaitSlider -= float(_timer.getDeltaMilliSeconds() / 500.0);
			_process.cvwait = 1 + abs(floor(_process.cvwaitSlider));
			log("cv wait", _process.cvwait);
			break;
		case 'm':
			_process.lag = !_process.lag;
			logSetting("lag" + _process.lag);
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
			log("                min size |  " + std::to_string(_process.areaMinSize));
			log("                          ");
			log("                 distMod |  " + std::to_string(_process.distMod));
			log("                 areaMod |  " + std::to_string(_process.areaMod));
			log("                 rectMod |  " + std::to_string(_process.rectMod));
			log("         rect : area mod |  " + std::to_string(_process.rectAreaRatioMod));
			log("            post mod PAR |  " + std::to_string(_process.postMod));
			log("                          ");
			log("       discard threshold |  " + std::to_string(_process.discardThreshold));
			log("        accept threshold |  " + std::to_string(_process.acceptThreshold));
			log("                          ");
			log("              dist scale |  " + std::to_string(_process.distScale));
			log("                          ");
			log("                    wait |  " + std::to_string(_process.wait));
			log("           advance frame |  " + std::to_string(_process.advanceFrame));
			log("                          ");
			log("                   erode |  " + std::to_string(_process.erode));
			log("                  dilate |  " + std::to_string(_process.dilate));
			log("                    blur |  " + std::to_string(_process.blur));
			log("                   color |  " + std::to_string(_process.color));
			log("                          ");
			log("                 cv wait |  " + std::to_string(_process.cvwait));
			log("  ");
			break;
		}

		if (_camera.exposure < -15) { _camera.exposure = -15; }
		else if (_camera.exposure > 15) { _camera.exposure = 15; }
		if (_process.threshold < 0) { _process.threshold = 0; }
		else if (_process.threshold > 255) { _process.threshold = 255; }
		_process.areaMinSize = _process.areaMinSize < 0 ? 0 : _process.areaMinSize;
		_process.distMod = _process.distMod < 0 ? 0 : _process.distMod;
		_process.areaMod = _process.areaMod < 0 ? 0 : _process.areaMod;
		_process.rectMod = _process.rectMod < 0 ? 0 : _process.rectMod;
		_process.rectAreaRatioMod = _process.rectAreaRatioMod < 0 ? 0 : _process.rectAreaRatioMod;
		_process.discardThreshold = _process.discardThreshold < 0 ? 0 : _process.discardThreshold;
		_process.acceptThreshold = _process.acceptThreshold < 0 ? 0 : _process.acceptThreshold;
	}
}