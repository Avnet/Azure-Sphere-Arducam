/* Copyright (c) Microsoft Corporation. All rights reserved.
   Licensed under the MIT License. */

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include "arducam_driver\ArduCAM.h"
#include "Log_Debug.h"
#include "delay.h"
#include "VectorTable.h"

#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <time.h>
#include <signal.h>
#include <applibs/log.h>
#include <applibs/networking.h>
#include <curl\curl.h>
#include <curl\easy.h>

#define CFG_MODE_JPEG

//uint8_t ImageBuffer[64 * 1024];

struct image_buffer {
	uint8_t* p_data;
	uint32_t size;
};

const char* FileURI = "https://owenstorageaccount.blob.core.windows.net/img/test.jpg";
const char* SASToken = "?sv=2019-10-10&st=2020-11-03T11%3A56%3A11Z&se=2021-11-04T11%3A56%3A00Z&sr=c&sp=rcwl&sig=%2BTrpLpN3c7WY2v12Le3rRQSZ7EPsfUPYTy3RPUsxCTs%3D";

static void LogCurlError(const char* message, int curlErrCode)
{
	Log_Debug(message);
	Log_Debug(" (curl err=%d, '%s')\n", curlErrCode, curl_easy_strerror(curlErrCode));
}

static size_t read_callback(char* buffer, size_t size, size_t nitems, void* userdata)
{
	struct image_buffer* p_image_buffer = (struct image_buffer*)userdata;

	size_t total_available_size = size * nitems;
	size_t copy_size = 0;

	if (p_image_buffer->size > total_available_size) {
		copy_size = total_available_size;
		p_image_buffer->size -= total_available_size;
	}
	else {
		copy_size = p_image_buffer->size;
		p_image_buffer->size = 0;
	}

	for (size_t i = 0; i < copy_size; i++) {
		buffer[i] = *p_image_buffer->p_data++;
	}

	return copy_size;
}

static void UploadFileToAzureBlob(uint8_t* p_data, uint32_t size)
{
	static struct image_buffer userdata;
	userdata.p_data = p_data;
	userdata.size = size;

	CURL* curlHandle = NULL;
	CURLcode res = CURLE_OK;
	struct curl_slist* list = NULL;

	if ((res = curl_global_init(CURL_GLOBAL_ALL)) != CURLE_OK) {
		LogCurlError("curl_global_init", res);
		goto exitLabel;
	}

	char* sasurl = calloc(strlen(FileURI) + strlen(SASToken) + sizeof('\0'), sizeof(char));
	(void)strcat(strcat(sasurl, FileURI), SASToken);

	if ((curlHandle = curl_easy_init()) == NULL) {
		Log_Debug("curl_easy_init() failed\r\n");
		goto cleanupLabel;
	}

	if ((res = curl_easy_setopt(curlHandle, CURLOPT_URL, sasurl)) != CURLE_OK) {
		LogCurlError("curl_easy_setopt CURLOPT_URL", res);
		goto cleanupLabel;
	}

	list = curl_slist_append(list, "x-ms-blob-type:BlockBlob");
	if ((res = curl_easy_setopt(curlHandle, CURLOPT_HTTPHEADER, list)) != CURLE_OK) {
		LogCurlError("curl_easy_setopt CURLOPT_HTTPHEADER", res);
		goto cleanupLabel;
	}

	if ((res = curl_easy_setopt(curlHandle, CURLOPT_UPLOAD, 1)) != CURLE_OK) {
		LogCurlError("curl_easy_setopt CURLOPT_UPLOAD", res);
		goto cleanupLabel;
	}
	if ((res = curl_easy_setopt(curlHandle, CURLOPT_INFILESIZE, size)) != CURLE_OK) {
		LogCurlError("curl_easy_setopt CURLOPT_INFILESIZE", res);
		goto cleanupLabel;
	}

	if ((res = curl_easy_setopt(curlHandle, CURLOPT_READFUNCTION, read_callback)) != CURLE_OK) {
		LogCurlError("curl_easy_setopt CURLOPT_READFUNCTION", res);
		goto cleanupLabel;
	}

	if ((res = curl_easy_setopt(curlHandle, CURLOPT_READDATA, &userdata)) != CURLE_OK) {
		LogCurlError("curl_easy_setopt CURLOPT_READFUNCTION", res);
		goto cleanupLabel;
	}

	// Set output level to verbose.
	if ((res = curl_easy_setopt(curlHandle, CURLOPT_VERBOSE, 1L)) != CURLE_OK) {
		LogCurlError("curl_easy_setopt CURLOPT_VERBOSE", res);
		goto cleanupLabel;
	}

	// Perform the opeartion
	if ((res = curl_easy_perform(curlHandle)) != CURLE_OK) {
		LogCurlError("curl_easy_perform", res);
	}

cleanupLabel:
	free(sasurl);
	// Clean up sample's cURL resources.
	curl_easy_cleanup(curlHandle);
	// Clean up cURL library's resources.
	curl_global_cleanup();

exitLabel:
	return;
}

_Noreturn void RTCoreMain(void)
{
	VectorTableInit();
	//CPUFreq_Set(197600000);

	//initialises UART
	DebugUARTInit();

	Log_Debug("Exmaple to capture a JPEG image from ArduCAM mini 2MP Plus\r\n");

	// init hardware and probe camera
	arducam_ll_init();
	arducam_reset();
	if (arducam_test() == 0) {
		Log_Debug("ArduCAM mini 2MP Plus is found\r\n");
	}
	else {
		Log_Debug("ArduCAM mini 2MP Plus is not found\r\n");
		while (1);
		goto END;
	}

	// config Camera
	arducam_set_format(JPEG);
	arducam_InitCAM();
	arducam_OV2640_set_JPEG_size(OV2640_160x120);
	delay_ms(1000);
	arducam_clear_fifo_flag();
	arducam_flush_fifo();

	// Trigger a capture and wait for data ready in DRAM
	arducam_start_capture();
	while (!arducam_check_fifo_done());

	uint32_t img_len = arducam_read_fifo_length();
	if (img_len > MAX_FIFO_SIZE) {
		Log_Debug("ERROR: FIFO overflow\r\n");
		goto END;
	}

	Log_Debug("len = %d\r\n", img_len);

	uint8_t* p_imgBuffer = malloc(img_len);

	arducam_CS_LOW();
	arducam_set_fifo_burst();
	arducam_read_fifo_burst(p_imgBuffer, img_len);
	arducam_CS_HIGH();

	arducam_clear_fifo_flag();

	// OV2640 pad 0x00 bytes at the end of JPG image
	while (p_imgBuffer[img_len - 1] != 0xD9) {
		--img_len;
	}

	uint8_t* p_file;
	uint32_t file_size;

	p_file = p_imgBuffer;
	file_size = img_len;

	bool isNetworkingReady = false;
	while ((Networking_IsNetworkingReady(&isNetworkingReady) < 0) || !isNetworkingReady) {
		Log_Debug("\nNot doing download because network is not up, try again\r\n");
	}

	UploadFileToAzureBlob(p_file, file_size);

	free(p_file);

	Log_Debug("App Exit\r\n");

END:
	while (1);
}