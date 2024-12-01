#pragma once

#include "ImgProc.h"
#include "AppSettings.h"

void HandleKeyboard(bool* showSettings) {

	if (_kbhit())
	{
		char key = _getch();
		if (key == 'q') {
			*showSettings = !*showSettings;
		}
	}
}