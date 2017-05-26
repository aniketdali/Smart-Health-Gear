/*
 *     SocialLedge.com - Copyright (C) 2013
 *
 *     This file is part of free software framework for embedded processors.
 *     You can use it and/or distribute it as long as this copyright header
 *     remains unmodified.  The code is free for personal use and requires
 *     permission to use in a commercial product.
 *
 *      THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 *      OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 *      MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 *      I SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
 *      CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 *     You can reach the author of this software at :
 *          p r e e t . w i k i @ g m a i l . c o m
 */

#include <disk/spi_flash.h>
#include <stdio.h>              // printf()
#include <string.h>
#include <time.h>
#include "printf_lib.h"
#include "FreeRTOS.h"
#include "task.h"               // uxTaskGetSystemState()

#include "command_handler.hpp"  // CMD_HANDLER_FUNC()
#include "rtc.h"                // Set and Get System Time
#include "sys_config.h"         // TERMINAL_END_CHARS
#include "lpc_sys.h"
#include "eint.h"

#include "utilities.h"          // printMemoryInfo()
#include "storage.hpp"          // Get Storage Device instances
#include "spi_sem.h"
#include "file_logger.h"

#include "uart0.hpp"
#include "uart2.hpp"
#include "wireless.h"
#include "nrf_stream.hpp"

#include "io.hpp"
#include "shared_handles.h"
#include "scheduler_task.hpp"

#include "c_tlm_stream.h"
#include "c_tlm_var.h"
#include "tasks.hpp"

#include "singleton_template.hpp"

#include "gpio.hpp"
#include "i2c1.hpp"
#include "task.h"
#include "lpc_sys.h"
#include "soft_timer.hpp"
#include "algorithm.hpp"
#include "Thermistor.hpp"

#include "examples/examples.hpp"
#include "handlers.hpp"
#include "printf_lib.h"
#include "switches.hpp"
#include "stdio.h"
#include "LPC17xx.h"
#include "utilities.h"
using namespace std;


CMD_HANDLER_FUNC(taskListHandler)
{
#if (1 == configUSE_TRACE_FACILITY)
    const int delayInMs = (int)cmdParams;  // cast parameter str to integer

    if(delayInMs > 0) {
        vTaskResetRunTimeStats();
        vTaskDelayMs(delayInMs);
    }

    // Enum to char : eRunning, eReady, eBlocked, eSuspended, eDeleted
    const char * const taskStatusTbl[] = { "RUN", "RDY", "BLK", "SUS", "DEL" };

    // Limit the tasks to avoid heap allocation.
    const unsigned portBASE_TYPE maxTasks = 16;
    TaskStatus_t status[maxTasks];
    uint32_t totalRunTime = 0;
    uint32_t tasksRunTime = 0;
    const unsigned portBASE_TYPE uxArraySize =
            uxTaskGetSystemState(&status[0], maxTasks, &totalRunTime);

    output.printf("%10s Sta Pr Stack CPU%%          Time\n", "Name");
    for(unsigned priorityNum = 0; priorityNum < configMAX_PRIORITIES; priorityNum++)
    {
        /* Print in sorted priority order */
        for (unsigned i = 0; i < uxArraySize; i++) {
            TaskStatus_t *e = &status[i];
            if (e->uxBasePriority == priorityNum) {
                tasksRunTime += e->ulRunTimeCounter;

                const uint32_t cpuPercent = (0 == totalRunTime) ? 0 : e->ulRunTimeCounter / (totalRunTime/100);
                const uint32_t timeUs = e->ulRunTimeCounter;
                const uint32_t stackInBytes = (4 * e->usStackHighWaterMark);

                output.printf("%10s %s %2u %5u %4u %10u us\n",
                              e->pcTaskName, taskStatusTbl[e->eCurrentState], e->uxBasePriority,
                              stackInBytes, cpuPercent, timeUs);
            }
        }
    }

    /* Overhead is the time not accounted towards any of the tasks.
     * For example, when an ISR happens, that is not part of a task's CPU usage.
     */
    const uint32_t overheadUs = (totalRunTime - tasksRunTime);
    const uint32_t overheadPercent = overheadUs / (totalRunTime / 100);
    output.printf("%10s --- -- ----- %4u %10u uS\n",
                  "(overhead)", overheadPercent, overheadUs);

    if (uxTaskGetNumberOfTasks() > maxTasks) {
        output.printf("** WARNING: Only reported first %u tasks\n", maxTasks);
    }
#else
    output.printf("OOPS, I can't do this for you.  Please set configUSE_TRACE_FACILITY to 1 at FreeRTOSConfig.h\n");
#endif

    return true;
}

CMD_HANDLER_FUNC(memInfoHandler)
{
#if 0 /* This was for memory test */
    int mem = (int) cmdParams;
    if (mem > 0) {
        char *p = (char*) malloc(mem);
        printf("Pointer = %p\n", p);
    }
#endif

    char buffer[512];
    sys_get_mem_info_str(buffer);
    output.putline(buffer);
    return true;
}

CMD_HANDLER_FUNC(healthHandler)
{
    Uart0 &u0 = Uart0::getInstance();

    unsigned int total = 0;
    unsigned int available = 0;
    Storage::getFlashDrive().getDriveInfo(&total, &available);

    float floatTemp = TS.getFarenheit();
    int floatSig1 = (int) floatTemp;
    int floatDec1 = ((floatTemp - floatSig1) * 10);
    rtc_t bt = sys_get_boot_time();

    unsigned int highestWrCnt = 0;
    unsigned int highestPageWrCnt = 0;
    if (flash_supports_metadata())
    {
        spi1_lock();
        /* Determine the page that has been written the most */
        const unsigned int pages = flash_get_page_count();

        for (unsigned int i = 0; i < pages; i++) {
            const unsigned int wrCount = flash_get_page_write_count(i);
            if (wrCount > highestWrCnt) {
                highestWrCnt = wrCount;
                highestPageWrCnt = i;
            }
        }
        spi1_unlock();

        const int max_writes = 100 * 1000;
        int life = 100 - (100 * highestWrCnt / max_writes);
        output.printf("Flash: %u/%u Life: %i%% (page %u written %u times)\n",
                        available, total, life, highestPageWrCnt, highestWrCnt);
    }
    else {
        output.printf("Flash: %u/%u\n", available, total);
    }

    output.printf( "Temp : %u.%u\n"
                   "Light: %u\n"
                   "Time : %s"
                   "Boot Time: %02u/%02u/%4u,%02u:%02u:%02u\n"
                   "Uart0 Watermarks: %u/%u (rx/tx)\n",
                    floatSig1, floatDec1,
                    LS.getRawValue(),
                    rtc_get_date_time_str(),
                    bt.month, bt.day, bt.year, bt.hour, bt.min, bt.sec,
                    u0.getRxQueueWatermark(), u0.getTxQueueWatermark()
    );

    // TODO: Print U2/U3 and CAN statistics if it is initialized

    return true;
}

CMD_HANDLER_FUNC(timeHandler)
{
    rtc_t time;

    /**
     * If cmdParam contains "set" with six spaces, we can parse the time
     * Example: set 11 30 2014 8 25 0 1
     */
    if(cmdParams.beginsWith("set"))
    {
        unsigned m, d, y, hr, mn, sc, w;
        if( 7 != cmdParams.scanf("%*s %u %u %u %u %u %u %u", &m, &d, &y, &hr, &mn, &sc, &w))
        {
            return false;
        }

        time.month = m; time.day = d; time.year = y;
        time.hour = hr, time.min = mn; time.sec = sc;
        time.dow = w;
        rtc_settime(&time);
    }

    output.printf("%s", rtc_get_date_time_str());
    return true;
}

CMD_HANDLER_FUNC(logHandler)
{
    bool enablePrintf = false;

    if (cmdParams == "flush") {
        LOG_FLUSH();
        output.putline("Log(s) have been flushed");
    }
    else if (cmdParams == "status") {
        output.printf("Blocked calls  : %u\n", logger_get_blocked_call_count());
        output.printf("Queue watermark: %u\n", logger_get_num_buffers_watermark());
        output.printf("Highest file write time: %ums\n", logger_get_highest_file_write_time_ms());
        output.printf("Call counts    : %u dgb %u info %u warn %u err\n",
                      logger_get_logged_call_count(log_debug),
                      logger_get_logged_call_count(log_info),
                      logger_get_logged_call_count(log_warn),
                      logger_get_logged_call_count(log_error));
    }
    else if (cmdParams.beginsWith("raw")) {
        cmdParams.eraseFirstWords(1);
        logger_log_raw(cmdParams());
    }
    else if ( (enablePrintf = cmdParams.beginsWith("enable ")) || cmdParams.beginsWith("disable ")) {
        // command is: 'enable print info/warning/error'
        logger_msg_t type = cmdParams.containsIgnoreCase("warn")  ? log_warn  :
                            cmdParams.containsIgnoreCase("error") ? log_error :
                            cmdParams.containsIgnoreCase("info")  ? log_info  : log_debug;

        logger_set_printf(type, enablePrintf);
        output.printf("%s logger printf for %s\n",
                      enablePrintf ? "Enabled" : "Disabled",
                      type == log_debug ? "debug" : type == log_info ? "info" : type == log_warn ? "warn" : "error");
    }
    else {
        // This loop was the test code used while testing the logger such that the user
        // can type "log 1000 foobar" to log a message 1000 times.
        // for (int i = 0; i < ((int) cmdParams) + 1; i++)
        {
            LOG_INFO(cmdParams());
        }
        output.printf("Logged: |%s|\n", cmdParams());
    }
    return true;
}

CMD_HANDLER_FUNC(cpHandler)
{
    char *srcFile = NULL;
    char *dstFile = NULL;
    if(2 != cmdParams.tokenize(" ", 2, &srcFile, &dstFile)) {
        return false;
    }

    unsigned int readTimeMs = 0;
    unsigned int writeTimeMs = 0;
    unsigned int bytesTransferred = 0;
    FRESULT copyStatus = Storage::copy(srcFile, dstFile,
                                       &readTimeMs, &writeTimeMs, &bytesTransferred);

    if(FR_OK != copyStatus) {
        output.printf("Error %u copying |%s| -> |%s|\n", copyStatus, srcFile, dstFile);
    }
    else {
        output.printf("Finished!  Read: %u Kb/sec, Write: %u Kb/sec\n",
                      bytesTransferred/(0 == readTimeMs  ? 1 : readTimeMs),
                      bytesTransferred/(0 == writeTimeMs ? 1 : writeTimeMs));
    }
    return true;
}

CMD_HANDLER_FUNC(catHandler)
{
    // If -print was present, we will print to console
    const bool printToScreen = !cmdParams.erase("-noprint");
    cmdParams.trimStart(" ");
    cmdParams.trimEnd(" ");

    char c = 0;
    //output.printf("Press a key to print one buffer at a time or enter 'x' to quit...\n");
    //output.getChar(&c, portMAX_DELAY);
    if ('x' == c) {
        return true;
    }

    FIL file;
    if(FR_OK != f_open(&file, cmdParams(), FA_OPEN_EXISTING | FA_READ))
    {
        output.printf("Failed to open: %s\n", cmdParams());
    }
    else
    {
        // Extra char for null terminator
        char buffer[512] = { 0 };
        UINT bytesRead = 0;
        UINT totalBytesRead = 0;

        const unsigned int startTime = sys_get_uptime_ms();
        while(FR_OK == f_read(&file, buffer, sizeof(buffer), &bytesRead) && bytesRead > 0)
        {
            totalBytesRead += bytesRead;

            if(printToScreen) {
                for (UINT i = 0; i < bytesRead; i++) {
                    output.putChar(buffer[i]);
                }

                //output.getChar(&c, portMAX_DELAY);
                if ('x' == c) {
                    break;
                }
            }
        }
        f_close(&file);

        if(!printToScreen) {
            const unsigned int timeTaken = sys_get_uptime_ms() - startTime;
            output.printf("\nRead %d bytes @ %d Kb/sec", totalBytesRead,
                          totalBytesRead/(0 == timeTaken  ? 1 : timeTaken));
        }
        output.putline("");
    }
    return true;
}

CMD_HANDLER_FUNC(lsHandler)
{
    DIR Dir;
    FILINFO Finfo;
    FATFS *fs;
    FRESULT returnCode = FR_OK;

    unsigned int fileBytesTotal = 0, numFiles = 0, numDirs = 0;
    #if _USE_LFN
        char Lfname[_MAX_LFN];
    #endif

    const char *dirPath = cmdParams == "" ? "0:" : cmdParams();
    if (FR_OK != (returnCode = f_opendir(&Dir, dirPath))) {
        output.printf("Invalid directory: |%s| (Error %i)\n", dirPath, returnCode);
        return true;
    }

#if 0
    // Offset the listing
    while(lsOffset-- > 0) {
        #if _USE_LFN
            Finfo.lfname = Lfname;
            Finfo.lfsize = sizeof(Lfname);
        #endif
        if (FR_OK != f_readdir(&Dir, &Finfo)) {
            break;
        }
    }
#endif

    output.printf("Directory listing of: %s\n\n", dirPath);
    for (;;)
    {
        #if _USE_LFN
            Finfo.lfname = Lfname;
            Finfo.lfsize = sizeof(Lfname);
        #endif

        returnCode = f_readdir(&Dir, &Finfo);
        if ( (FR_OK != returnCode) || !Finfo.fname[0]) {
            break;
        }

        if (Finfo.fattrib & AM_DIR){
            numDirs++;
        }
        else{
            numFiles++;
            fileBytesTotal += Finfo.fsize;
        }
        output.printf("%c%c%c%c%c %u/%02u/%02u %02u:%02u %10lu %13s",
                (Finfo.fattrib & AM_DIR) ? 'D' : '-',
                (Finfo.fattrib & AM_RDO) ? 'R' : '-',
                (Finfo.fattrib & AM_HID) ? 'H' : '-',
                (Finfo.fattrib & AM_SYS) ? 'S' : '-',
                (Finfo.fattrib & AM_ARC) ? 'A' : '-',
                (Finfo.fdate >> 9) + 1980, (Finfo.fdate >> 5) & 15, Finfo.fdate & 31,
                (Finfo.ftime >> 11), (Finfo.ftime >> 5) & 63,
                Finfo.fsize, &(Finfo.fname[0]));

        // LFN names tend to increase memory requirements for output str, enable with caution
        #if (_USE_LFN)
        output.put(" - ");
        output.put(Lfname);
        #endif
        output.putline("");
    }
    output.printf("\n%4u File(s), %10d bytes total\n%4d Dir(s)", numFiles, fileBytesTotal, numDirs);

    if (f_getfree(dirPath, (DWORD*) &fileBytesTotal, &fs) == FR_OK)
    {
        output.printf(", %10dK bytes free\n", fileBytesTotal * fs->csize / 2);
        output.printf("sectors per cluster:%d",fs->csize);
        output.printf("Bytes per sector:%d",fs->csize * 512);
        output.printf("Total number of sectors:%d",fs->fsize);

    }
    return true;
}

CMD_HANDLER_FUNC(fatHandler)
{
    DIR Dir;
    FILINFO Finfo;
    FATFS *fs;
    FRESULT returnCode = FR_OK;

    unsigned int fileBytesTotal = 0, numFiles = 0, numDirs = 0;
    #if _USE_LFN
        char Lfname[_MAX_LFN];
    #endif

    const char *dirPath = cmdParams == "" ? "0:" : cmdParams();
    if (FR_OK != (returnCode = f_opendir(&Dir, dirPath))) {
        return true;
    }

    for (;;)
    {
        #if _USE_LFN
            Finfo.lfname = Lfname;
            Finfo.lfsize = sizeof(Lfname);
        #endif

        returnCode = f_readdir(&Dir, &Finfo);
        if ( (FR_OK != returnCode) || !Finfo.fname[0]) {
            break;
        }

        if (Finfo.fattrib & AM_DIR){
            numDirs++;
        }
        else{
            numFiles++;
            fileBytesTotal += Finfo.fsize;
        }

    }

    if (f_getfree(dirPath, (DWORD*) &fileBytesTotal, &fs) == FR_OK)
    {
        output.printf("Sectors per cluster:%d\n",fs->csize);
        output.printf("Bytes per sector:%d\n",fs->csize * _MIN_SS);
        output.printf("Total number of sectors:%d\n",fs->fsize);

    }
    return true;
}

CMD_HANDLER_FUNC(mkdirHandler)
{
    output.printf("Create directory '%s' : %s\n",
                   cmdParams(), (FR_OK == f_mkdir(cmdParams())) ? "OK" : "ERROR");
    return true;
}

CMD_HANDLER_FUNC(rmHandler)
{
    output.printf("Delete '%s' : %s\n",
                  cmdParams(), (FR_OK == f_unlink(cmdParams())) ? "OK" : "ERROR");
    return true;
}

CMD_HANDLER_FUNC(i2cIoHandler)
{
    bool read = cmdParams.beginsWithIgnoreCase("read");
    bool write = cmdParams.beginsWithIgnoreCase("write");
    bool discover = cmdParams.beginsWithIgnoreCase("discover");

    int addr = 0;
    int reg = 0;
    int data = 0;
    unsigned int count = 0;

    if (read) {
        if (cmdParams.scanf("%*s %0x %0x %u", &addr, &reg, &count) < 2) {
            output.putline("Need device and register address");
            return false;
        }

        uint8_t buffer[256] = { 0 };
        if (count <= 0) {
            count = 1;
        }
        else if (count > sizeof(buffer)) {
            count = sizeof(buffer);
        }

        bool ok = I2C2::getInstance().readRegisters(addr, reg, &buffer[0], count);
        output.printf("Read status from device %#2X: %s: \n", addr, ok ? "OK" : "ERROR");
        for (unsigned int i = 0; i < count; i++) {
            output.printf("    %#2X: %#2X\n", (reg + i), (buffer[i] & 0xFF));
        }
    }
    else if (write) {
        if (3 != cmdParams.scanf("%*s %0x %0x %0x", &addr, &reg, &data)) {
            output.putline("Need device, register address and data");
            return false;
        }

        if (I2C2::getInstance().writeReg(addr, reg, data)) {
            output.printf("Wrote %#2X to %#2X::%#2X\n", data, addr, reg);
        }
        else {
            output.printf("Error writing to device %#x\n", addr);
        }
    }
    else if (discover) {
        for (addr = 2; addr <= 254; addr += 2) {
            if (I2C2::getInstance().checkDeviceResponse(addr)) {
                output.printf("I2C device responded to address %#4x\n", addr);
            }
        }
    }

    return (read || write || discover);
}

CMD_HANDLER_FUNC(mvHandler)
{
    char *srcFile = NULL;
    char *dstFile = NULL;
    if(2 != cmdParams.tokenize(" ", 2, &srcFile, &dstFile)) {
        return false;
    }
    else {
        output.printf("Move '%s' -> '%s' : %s\n",
                      srcFile, dstFile,
                      (FR_OK == f_rename(srcFile, dstFile))  ? "OK" : "ERROR");
    }
    return true;
}

CMD_HANDLER_FUNC(newFileHandler)
{
    const char end_file = '~';
    int timeout_ms = OS_MS(10 * 1000);

    FIL file;
    if (FR_OK != f_open(&file, cmdParams(), FA_WRITE | FA_CREATE_ALWAYS)) {
        output.printf("Unable to open '%s' to write the file\n", cmdParams());
        return true;
    }

    char c = 0;
    UINT bw = 0;
    output.printf("End the file by using %c character.  %i is the timeout\n", end_file, timeout_ms);
    output.printf("Sorry, no backspace support :(\n");

    while (output.getChar(&c, timeout_ms) && c != end_file) {
        if (FR_OK != f_write(&file, &c, 1, &bw) || 1 != bw) {
            output.printf("Error occurred while writing the file\n");
        }
        else {
            output.putChar(c);
        }
    }

    f_close(&file);
    return true;
}

CMD_HANDLER_FUNC(dcpHandler)
{
    DIR Dir;
    FILINFO Finfo;
    FRESULT returnCode = FR_OK;
    #if _USE_LFN
        char Lfname[_MAX_LFN];
    #endif

    char *srcDir = NULL;
    char *dstDir = NULL;
    if(2 != cmdParams.tokenize(" ", 2, &srcDir, &dstDir)) {
        output.putline("ERROR: Give me source and destination directory separated by a space");
        return true;
    }

    /* Check destination directory */
    if (FR_OK != (returnCode = f_opendir(&Dir, dstDir))) {
        output.printf("Invalid destination directory: |%s|\n", dstDir);
        return true;
    }
    /* Check source directory */
    if (FR_OK != (returnCode = f_opendir(&Dir, srcDir))) {
        output.printf("Invalid source directory: |%s|\n", srcDir);
        return true;
    }

    STR_ON_STACK(src, 256);
    STR_ON_STACK(dst, 256);
    for (;;)
    {
        #if _USE_LFN
            Finfo.lfname = Lfname;
            Finfo.lfsize = sizeof(Lfname);
        #endif

        /* If no more files */
        if ( (FR_OK != f_readdir(&Dir, &Finfo)) || !Finfo.fname[0]) {
            break;
        }

        /* If not a directory */
        if (!(Finfo.fattrib & AM_DIR))
        {
            src.printf("%s/%s", srcDir, Finfo.fname);
            dst.printf("%s/%s", dstDir, Finfo.fname);

            output.printf("Copy %s -> %s : %d Bytes : %s\n",
                           src(), dst(), Finfo.fsize,
                           (FR_OK == Storage::copy(src(), dst())) ? "OK" : "ERROR");
        }
    }
    return true;
}

CMD_HANDLER_FUNC(storageHandler)
{
    if(cmdParams == "format sd") {
        output.putline((FR_OK == Storage::getSDDrive().format()) ? "Format OK" : "Format ERROR");
    }
    else if(cmdParams == "format flash") {
        output.putline((FR_OK == Storage::getFlashDrive().format()) ? "Format OK" : "Format ERROR");
    }
    else if(cmdParams == "mount sd") {
        output.putline(FR_OK == Storage::getSDDrive().mount() ? "SD Card mounted" : "Error mounting SD Card");
    }
    else if(cmdParams == "mount flash") {
        output.putline(FR_OK == Storage::getFlashDrive().mount() ? "Flash mounted" : "Error mounting Flash Memory");
    }
    else {
        return false;
    }
    return true;
}

CMD_HANDLER_FUNC(rebootHandler)
{
    output.putline("Rebooting System");

    // Flush out everything before we reboot
    LOG_FLUSH();

    vTaskDelayMs(2000);
    sys_reboot();

    return true;
}

#if (SYS_CFG_ENABLE_TLM)
static void stream_tlm(const char *s, void *arg)
{
    CharDev *out = (CharDev*) arg;
    while (*s) {
        out->putChar(*s);
        s++;
    }
}

CMD_HANDLER_FUNC(telemetryHandler)
{
    if(cmdParams.getLen() == 0)
    {
        tlm_stream_all(stream_tlm, &output, false);
    }
    else if (cmdParams.beginsWithIgnoreCase("ascii"))
    {
        tlm_stream_all(stream_tlm, &output, true);
    }
    else if(cmdParams == "save") {
        FILE *fd = fopen(SYS_CFG_DISK_TLM_NAME, "w");
        tlm_stream_one_file(tlm_component_get_by_name(SYS_CFG_DISK_TLM_NAME), fd);
        fclose(fd);
        output.putline("Telemetry was saved to disk");
    }
    else if(cmdParams.beginsWithIgnoreCase("get")) {
        char *compName = NULL;
        char *varName = NULL;
        if (3 == cmdParams.tokenize(" ", 3, NULL, &compName, &varName)) {
            char buffer[256] = { 0 };
            if (tlm_variable_get_value(compName, varName, &buffer[0], sizeof(buffer))) {
                output.putline(buffer);
            }
            else {
                output.putline("Error locating or printing variable value");
            }
        }
        else {
            output.putline("Required parameters: 'get <comp name> <var name>");
        }
    }
    else {
        char *compName = NULL;
        char *varName = NULL;
        char *varVal = NULL;
        if (3 != cmdParams.tokenize(" ", 3, &compName, &varName, &varVal)) {
            output.putline("ERROR: See 'help telemetry'");
        }
        else {
            if (tlm_variable_set_value(compName, varName, varVal)) {
                output.printf("%s:%s set to %s\n", compName, varName, varVal);
            } else {
                output.printf("Failed to set %s:%s to %s\n", compName, varName, varVal);
            }
        }
    }
    return true;
}
#endif

CMD_HANDLER_FUNC(learnIrHandler)
{
    SemaphoreHandle_t learn_sem = scheduler_task::getSharedObject(shared_learnSemaphore);

    if (learn_sem)
    {
        xSemaphoreGive(learn_sem);
        output.putline("Learning mode enabled");
    }
    else
    {
        output.putline("ERROR: Semaphore was NULL, is the 'remote' task running?");
    }

    return true;
}

CMD_HANDLER_FUNC(spi0Handler)
{
	SPI0_task spi0_obj(1);

	//spi0_obj.run(&spi0_obj);
	//while(1)
	//{
		spi0_obj.display_MID_DID();
		spi0_obj.read_status_display();
		spi0_obj.signature_read();
		//spi0_obj.sector_details();

	//}

    return true;
}
uint32_t checkBit =0;
CMD_HANDLER_FUNC(conProHandler)
{

	   checkBit = 1;

	   return true;

}


SemaphoreHandle_t semaphore_signal = 0;
SoftTimer myTimer(50);
void functionOne()
{
	printf("entered ISR Handler\n");


	if (myTimer.expired()) {
		/* Do something */
		xSemaphoreGiveFromISR(semaphore_signal, NULL);
		myTimer.restart();
	}

}

void SemaphoreTakeTask( void* p )
{
	while(1) {
		   // printf("I got the semaphore\n");
	        if(xSemaphoreTake(semaphore_signal, portMAX_DELAY)) {
	            // Process the interrupt
	        	printf("I got the semaphore\n");

	        }
	    }
}



CMD_HANDLER_FUNC(binSemaHandler)
{
	vSemaphoreCreateBinary( semaphore_signal ); // Create the semaphore
	uint32_t pin_num_1;
	//uint32_t func_1 = 1;
	GPIO_CUSTOM gpioObj;
	void_func_t func1;

	printf("Enter the pin to connect switch 1\n");
	cin>> pin_num_1;
	cout<< "First pin Val "<<pin_num_1<<endl;

	func1 = functionOne;

	gpioObj.setInputDir(2, pin_num_1);


	delay_ms(2000);
//	enable_port2(pin_num_1, rising_edge, func1);
	xTaskCreate(SemaphoreTakeTask, "t1", 2048, NULL, 1, NULL);
	xSemaphoreTake(semaphore_signal, 0);
	return true;

}

int set =0;
extern "C"
{
    void UART2_IRQHandler()
    {
        uint32_t ready = LPC_UART2->IIR >> 2;
        char c;
        if(ready)
           c =receive_Byte();
        printf("Received: %c\n",c);

    }
}

CMD_HANDLER_FUNC(uart2Handler)
{

	UART2_task UART2_obj(1);
	//while(1)
	UART2_obj.transmit_Byte('Y');


	//printf("Received: %c\n",UART2_obj.receive_Byte());


    return true;
}
I2C1& i2c = I2C1::getInstance();
// heart rate measurement
bool maxim_max30102_read_fifo(uint32_t *pun_red_led, uint32_t *pun_ir_led)
/**
* \brief        Read a set of samples from the MAX30102 FIFO register
* \par          Details
*               This function reads a set of samples from the MAX30102 FIFO register
*
* \param[out]   *pun_red_led   - pointer that stores the red LED reading data
* \param[out]   *pun_ir_led    - pointer that stores the IR LED reading data
*
* \retval       true on success
*/
{
  uint32_t un_temp;
 // unsigned char uch_temp;
  uint8_t uch_temp;

  *pun_red_led=0;
  *pun_ir_led=0;
  uint8_t ach_i2c_data[6] = {0};
  //uint8_t*
  i2c.readRegisters(0xAE ,0x00, &uch_temp, 1);
  i2c.readRegisters(0xAE ,0x01, &uch_temp, 1);

  i2c.readRegisters(0xAE ,0x07, ach_i2c_data, 6);

  un_temp=(unsigned char) ach_i2c_data[0];
  un_temp<<=16;
  *pun_red_led+=un_temp;
  un_temp=(unsigned char) ach_i2c_data[1];
  un_temp<<=8;
  *pun_red_led+=un_temp;
  un_temp=(unsigned char) ach_i2c_data[2];
  *pun_red_led+=un_temp;

  un_temp=(unsigned char) ach_i2c_data[3];
  un_temp<<=16;
  *pun_ir_led+=un_temp;
  un_temp=(unsigned char) ach_i2c_data[4];
  un_temp<<=8;
  *pun_ir_led+=un_temp;
  un_temp=(unsigned char) ach_i2c_data[5];
  *pun_ir_led+=un_temp;
  *pun_red_led&=0x03FFFF;  //Mask MSB [23:18]
  *pun_ir_led&=0x03FFFF;  //Mask MSB [23:18]

  return true;
}
volatile bool flag = false;
extern bool flag1;
// irq
/// ISR callback function upon NRF IRQ rising edge interrupt
 void heartrate_irq_callback(void)
{

	LPC_GPIOINT->IO2IntClr = 0xFFFFFFFF;
	flag1 = true;
	return;

}
//
CMD_HANDLER_FUNC(i2cTaskHandlerTrial)
{
	while(1)
	{
       // i2c.readReg(I2CAddr_TemperatureSensor, 0);
		float resistance;
		float temp = ((LS.getRawValue() * 3.3)/4096);
		temp = 3.3/temp;
		resistance = 10/(temp - 1);
		float temperature, T1,T2,R1,R2;
		for(int i =0; i<240; i++)
		{
			//cout<<resistance<<","<<resistance_array[i] <<endl;
			if((resistance > resistance_array[i] )  )
			{
				if(resistance == resistance_array[i - 1])
				{
					temperature = i - 41;
					break;
				}
				if(resistance < resistance_array[i - 1] )
				{
					R1 = resistance_array[i - 1];
					R2 = resistance_array[i];
					T1 = i - 41;
					T2 = i - 40;

					//cout<<R1<<","<<R2<<","<<T1<<","<<T2 <<endl;
					temperature = ((resistance - R1) * (R2 - R1))/(T2-T1);
					temperature = temperature + T1;
					break;
				}

			}

		}

	    cout<<"temp:"<<temperature<<endl;
		cout<<"resistance:"<<resistance<<endl;
		LD.setNumber(temperature);

	    delay_ms(1000);
	}
}

CMD_HANDLER_FUNC(i2cTaskHandler)
{
	GPIO_CUSTOM gpioObj;

    gpioObj.setInputDir(2, 0);
	// definitions
	#define MAX_BRIGHTNESS 255

	uint32_t aun_ir_buffer[500]; //IR LED sensor data
	int32_t n_ir_buffer_length;    //data length
	uint32_t aun_red_buffer[500];    //Red LED sensor data
	int32_t n_sp02; //SPO2 value
	int8_t ch_spo2_valid;   //indicator to show if the SP02 calculation is valid
	int32_t n_heart_rate;   //heart rate value
	int8_t  ch_hr_valid;    //indicator to show if the heart rate calculation is valid
	uint8_t uch_dummy;
	uint32_t un_min, un_max, un_prev_data;  //variables to calculate the on-board LED brightness that reflects the heartbeats
	int i;
	int32_t n_brightness;
	float f_temp;
	i2c.readRegisters(0xAE ,0x00, &uch_dummy, 1);
	n_brightness=0;
	un_min = 0x3FFFF;
	un_max=0;
	// def
	uint32_t pun_red_led;
	uint32_t pun_ir_led;

	Board_I2C_Device_AddressesI2C1 deviceAdd;
	deviceAdd = I2CAddr_HeartRateSensor;
	uint8_t data[4] = {5, 1, 0, 0};
	int a;
	cout<<"Press 1 to start\n";
	cin>>a;
	// init
	i2c.writeReg(deviceAdd, 0x09, 0x03); // HR mode
	i2c.writeReg(deviceAdd, 0x02, 0xC0); // intr 1 enable
	i2c.writeReg(deviceAdd, 0x03, 0x00); // intr 2 enable
	i2c.writeReg(deviceAdd, 0x04, 0x00); // fifo write ptr
	i2c.writeReg(deviceAdd, 0x05, 0x00); // fifo ovf ptr
	i2c.writeReg(deviceAdd, 0x06, 0x00); // fifo read ptr
	i2c.writeReg(deviceAdd, 0x08, 0x0F); // fifo config
	i2c.writeReg(deviceAdd, 0x0C, 0x24); // led 1
	i2c.writeReg(deviceAdd, 0x0D, 0x24); // led 2
	i2c.writeReg(deviceAdd, 0x10, 0x7F); // pilot led
	i2c.writeReg(deviceAdd, 0x0A, 0x27); // so2 config
	// done init
    cout<<"out";
	//maxim_max30102_read_fifo(&pun_red_led,&pun_ir_led);
    // main function


    n_ir_buffer_length=500; //buffer length of 100 stores 5 seconds of samples running at 100sps

    eint3_enable_port2(0, eint_falling_edge, heartrate_irq_callback);
   // NVIC_EnableIRQ(EINT3_IRQn);
    //read the first 500 samples, and determine the signal range
	for(i=0;i<n_ir_buffer_length;i++)
	{
		//while(((LPC_GPIOINT->IO2IntStatF & 0x01))!=1);   //wait until the interrupt pin asserts
        while(!flag);
        flag = false;
		maxim_max30102_read_fifo((aun_red_buffer+i), (aun_ir_buffer+i));  //read from MAX30102 FIFO

		if(un_min>aun_red_buffer[i])
			un_min=aun_red_buffer[i];    //update signal min
		if(un_max<aun_red_buffer[i])
			un_max=aun_red_buffer[i];    //update signal max
	}


        //calculate heart rate and SpO2 after first 500 samples (first 5 seconds of samples)
	maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, n_ir_buffer_length, aun_red_buffer, &n_sp02, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid);

        //Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every 1 second
        while(1)
        {
            i=0;
            un_min=0x3FFFF;
            un_max=0;

            //dumping the first 100 sets of samples in the memory and shift the last 400 sets of samples to the top
            for(i=100;i<500;i++)
            {
                aun_red_buffer[i-100]=aun_red_buffer[i];
                aun_ir_buffer[i-100]=aun_ir_buffer[i];

                //update the signal min and max
                if(un_min>aun_red_buffer[i])
                un_min=aun_red_buffer[i];
                if(un_max<aun_red_buffer[i])
                un_max=aun_red_buffer[i];
            }
        	//u0_dbg_printf("min=");
        		/*	u0_dbg_printf("%i\t", un_min);
        			u0_dbg_printf(", max=");
        			u0_dbg_printf("%i\n\r", un_max);*/
            //take 100 sets of samples before calculating the heart rate.
            for(i=400;i<500;i++)
            {
                un_prev_data=aun_red_buffer[i-1];
              //  while(((LPC_GPIOINT->IO2IntStatF & 0x01))!=1);
                while(!flag);
                flag = false;
                maxim_max30102_read_fifo((aun_red_buffer+i), (aun_ir_buffer+i));

                if(aun_red_buffer[i]>un_prev_data)
                {
                    f_temp=aun_red_buffer[i]-un_prev_data;
                    f_temp/=(un_max-un_min);
                    f_temp*=MAX_BRIGHTNESS;
                    n_brightness-=(int)f_temp;
                    if(n_brightness<0)
                        n_brightness=0;
                }
                else
                {
                    f_temp=un_prev_data-aun_red_buffer[i];
                    f_temp/=(un_max-un_min);
                    f_temp*=MAX_BRIGHTNESS;
                    n_brightness+=(int)f_temp;
                    if(n_brightness>MAX_BRIGHTNESS)
                        n_brightness=MAX_BRIGHTNESS;
                }

            }
            maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, n_ir_buffer_length, aun_red_buffer, &n_sp02, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid);
            // temp measurement
           // i2c.readReg(I2CAddr_TemperatureSensor, 0);

            delay_ms(1000);

        }


    //

    return true;
}

CMD_HANDLER_FUNC(orientationCmd)
{
    // Our parameter was the orientation tasks' pointer, but you may want to check for NULL pointer first.
    scheduler_task *compute = scheduler_task::getTaskPtrByName("compute");

    // You can use FreeRTOS API or the wrapper resume() or suspend() methods
    if (cmdParams == "on") {
        vTaskResume(compute->getTaskHandle());  // Can also use: compute->resume();
    }
    else {
        vTaskSuspend(compute->getTaskHandle()); // Can also use: compute->suspend();
    }

    return true;
}
volatile bool start = false;
// 240 task
const uint32_t switch_pin = 9;
//LPC_GPIO2->FIODIR |= (0 << switch_pin);
Switches sw = Switches::getInstance();
CMD_HANDLER_FUNC(smartHealthHandler)
{
	while(1)
	{
		if(sw.getSwitch(1))
		{
			//cout<< " i am in";

			start = true;
			delay_ms(500);
			//cout<<"start"<<start<<endl;
		}

		if(sw.getSwitch(2))
		{
			//cout<< " i am in";

			start = false;
			delay_ms(500);
			//cout<<"start"<<start<<endl;
		}
		delay_ms(100);
	}
	return true;
}

#if TERMINAL_USE_CAN_BUS_HANDLER
#include "can.h"
#include "printf_lib.h"
void can_BusOffCallback(uint32_t ibits)
{
    const uint32_t errbit = (ibits >> 16) & 0x1F;
    const char * rxtx = ((ibits >> 21) & 1) ? "RX" : "TX";
    const uint32_t errc = (ibits >> 22) & 3;

    u0_dbg_put("\n\n ***** CAN BUS ENTERED ERROR STATE!\n");
    u0_dbg_printf("ERRC = %#x, ERRBIT = %#x while %s\n", errc, errbit, rxtx);
}

extern "C" bool CAN_test(void);
CMD_HANDLER_FUNC(canBusHandler)
{
    can_t can = can1;
    const uint32_t baudrate = 100;

#if 0 /* Need to turn on CAN_TESTING at can.c if you want the test option */
    if (cmdParams == "test")
    {
        CAN_test();
    }
    else
#endif
    if (cmdParams == "init")
    {
        bool ok = CAN_init(can, baudrate, 3, 3, can_BusOffCallback, NULL);
        output.printf("CAN init: %s\n", ok ? "OK" : "ERROR");

        CAN_reset_bus(can);
        CAN_bypass_filter_ack_all_msgs();
    }
    else if (cmdParams.beginsWithIgnoreCase("filter"))
    {
        uint32_t id = 0;
        if (cmdParams.scanf("%*s %x", &id))
        {
            can_std_id_t *nosid = NULL;
            can_std_grp_id_t *nosgp = NULL;
            can_ext_grp_id_t *noeid = NULL;
            can_ext_id_t eid = CAN_gen_eid(can, id);

            CAN_setup_filter(nosid, 0,
                             nosgp, 0,
                             &eid, 1,
                             noeid, 0);
        }
        else {
            output.printf("Please specify the ID to filter: 'filter 0x100'\n");
        }
    }
    else if (cmdParams.beginsWithIgnoreCase("tx"))
    {
        int length = 0;
        int message_id = 0;
        can_msg_t msg = { 0 };

        /* Get length and message id */
        if (2 != cmdParams.scanf("%*s %x %i", &message_id, &length)) {
            output.printf("Need <message id> <length> <bytes>\n");
            return true;
        }

        /* Remove first three words, then scan for data bytes of can message */
        cmdParams.eraseFirstWords(3);
        cmdParams.scanf("%x %x %x %x %x %x %x %x",
                        &msg.data.bytes[0], &msg.data.bytes[1], &msg.data.bytes[2], &msg.data.bytes[3],
                        &msg.data.bytes[4], &msg.data.bytes[5], &msg.data.bytes[6], &msg.data.bytes[7]);

        msg.frame_fields.data_len = length;
        msg.frame_fields.is_29bit = 1;
        msg.msg_id = message_id;

        output.printf("Send CAN message with length: %u, ID: %#4X\n    ", length, message_id);
        for (int i = 0; i < msg.frame_fields.data_len; i++) {
            printf("%#2X, ", msg.data.bytes[i]);
        }
        output.printf("\n");
        output.flush();

        if (CAN_tx(can, &msg, 100)) {
            output.printf("  CAN message was possibly sent!\n");
        }
        else {
            output.printf("  ERROR sending CAN message\n");
        }
    }
    else if (cmdParams.beginsWithIgnoreCase("rx"))
    {
        int timeout = 0;
        cmdParams.scanf("%*s %i", &timeout);

        bool rx = false;
        can_msg_t msg;
        while (CAN_rx(can, &msg, timeout)) {
            rx = true;
            output.printf("Received a can frame with ID: %#4X\n", msg.msg_id);
            for (int i = 0; i < msg.frame_fields.data_len; i++) {
                output.printf("%#2X, ", msg.data.bytes[i]);
            }
            output.printf("\n");
        }

        if (!rx) {
            output.printf("Failed to receive data with %i timeout\n", timeout);
        }
    }
    else if (cmdParams == "registers")
    {
        /* Read CAN registers for debugging */
        output.printf("CANBus Status: %s\n", CAN_is_bus_off(can) ? "OFF" : "OK");
        output.printf("MOD : %#8X\n", LPC_CAN1->MOD);
        output.printf("IER : %#8X\n", LPC_CAN1->IER);
        output.printf("ICR : %#8X\n", LPC_CAN1->ICR);
        output.printf("GSR : %#8X\n", LPC_CAN1->GSR);
        output.printf("AMFR: %#8X\n", LPC_CANAF->AFMR);

        output.printf("\n");
        output.printf(" SFF SA: %#8X\n", LPC_CANAF->SFF_sa);
        output.printf("SFFG SA: %#8X\n", LPC_CANAF->SFF_GRP_sa);
        output.printf("EFFG SA: %#8X\n", LPC_CANAF->EFF_GRP_sa);
        output.printf("END PTR: %#8X\n", LPC_CANAF->ENDofTable);

        for (int i = 0; i < 4; i++) {
            output.printf("%2i: s8X\n", i, (uint32_t) LPC_CANAF_RAM->mask[i]);
        }
    }
    else {
        return false;
    }

    return true;
}



#endif
