/*
 * Copyright (c) 2017 ARM Limited. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "mbed.h"
#include "common_functions.h"
#include "CellularNonIPSocket.h"
#include "CellularDevice.h"
#include "UDPSocket.h"
#include "CellularLog.h"

#define UDP 0
#define TCP 1
#define NONIP 2

// Number of retries /
#define RETRY_COUNT 3

NetworkInterface *iface;

// Echo server hostname
const char *host_name = MBED_CONF_APP_ECHO_SERVER_HOSTNAME;

// Echo server port (same for TCP and UDP)
const int port = MBED_CONF_APP_ECHO_SERVER_PORT;

// IoT Technologies
static const int IOT_TECHNOLOGY_CATM1           = 0;
static const int IOT_TECHNOLOGY_NBIOT           = 1;
static const int IOT_TECHNOLOGY_CATM1_PREFERRED = 2;
static const int IOT_TECHNOLOGY_NBIOT_PREFERRED = 3;

// Desired IoT technology
static const int DESIRED_IOT_TECHNOLOGY = IOT_TECHNOLOGY_NBIOT_PREFERRED;

// WDS-Side Stacks
static const int WDS_SIDE_STACK_GERAN_ONLY          = 12;
static const int WDS_SIDE_STACK_EUTRAN_ONLY         = 28;
static const int WDS_SIDE_STACK_GERAN_AND_EUTRAN    = 30;

// Desired WDS-Side Stack
static const int DESIRED_WDS_SIDE_STACK = WDS_SIDE_STACK_EUTRAN_ONLY;

// Desired LTE bands bitmask
static const int DESIRED_LTE_BANDS_BITMASK  = 134742021;

static rtos::Mutex trace_mutex;

#if MBED_CONF_MBED_TRACE_ENABLE
static void trace_wait()
{
    trace_mutex.lock();
}

static void trace_release()
{
    trace_mutex.unlock();
}

static char time_st[50];

static char* trace_time(size_t ss)
{
    snprintf(time_st, 49, "[%08llums]", Kernel::get_ms_count());
    return time_st;
}

static void trace_open()
{
    mbed_trace_init();
    mbed_trace_prefix_function_set( &trace_time );

    mbed_trace_mutex_wait_function_set(trace_wait);
    mbed_trace_mutex_release_function_set(trace_release);

    mbed_cellular_trace::mutex_wait_function_set(trace_wait);
    mbed_cellular_trace::mutex_release_function_set(trace_release);
}

static void trace_close()
{
    mbed_cellular_trace::mutex_wait_function_set(NULL);
    mbed_cellular_trace::mutex_release_function_set(NULL);

    mbed_trace_free();
}
#endif // #if MBED_CONF_MBED_TRACE_ENABLE

Thread dot_thread(osPriorityNormal, 512);

void print_function(const char *format, ...)
{
    trace_mutex.lock();
    va_list arglist;
    va_start( arglist, format );
    vprintf(format, arglist);
    va_end( arglist );
    trace_mutex.unlock();
}

void set_iot_technology(CellularDevice *dev, int desired_iot_technology)
{
    int iot_technology = -1;

    // Check the current IoT technology first
    ATHandler *at_handler = dev->get_at_handler();
    at_handler->lock();
    at_handler->cmd_start_stop("#WS46", "?");
    at_handler->resp_start("#WS46:");

    // Read the current IoT technology
    iot_technology = at_handler->read_int();
    at_handler->resp_stop();

    // If already configured as desired, return
    if (iot_technology == desired_iot_technology) {
        at_handler->unlock();
        return;
    }

    // Set the desired IoT technology
    at_handler->at_cmd_discard("#WS46", "=", "%d", desired_iot_technology);
    if (at_handler->get_last_error() != NSAPI_ERROR_OK) {
        printf("ERROR: Unable to set IoT technology!\n");
        at_handler->unlock();
        return;
    }

    at_handler->unlock();

    // Power cycle the module
    dev->hard_power_off();
    dev->hard_power_on();
    dev->soft_power_on();

    return;
}

void set_wds_side_stack(CellularDevice *dev, int desired_side_stack)
{
    int side_stack = -1;

    // Check the current WDS side stack first
    ATHandler *at_handler = dev->get_at_handler();
    at_handler->lock();
    at_handler->cmd_start_stop("+WS46", "?");
    at_handler->resp_start("+WS46:");

    // Read the current WDS side stack
    side_stack = at_handler->read_int();
    at_handler->resp_stop();

    // If already configured as desired, return
    if (side_stack == desired_side_stack) {
        at_handler->unlock();
        return;
    }

    // Set the desired WDS side stack
    at_handler->at_cmd_discard("+WS46", "=", "%d", desired_side_stack);
    if (at_handler->get_last_error() != NSAPI_ERROR_OK) {
        printf("ERROR: Unable to set WDS-Side Stack!\n");
        at_handler->unlock();
        return;
    }

    at_handler->unlock();

    // Power cycle the module
    dev->hard_power_off();
    dev->hard_power_on();
    dev->soft_power_on();
}

void set_desired_lte_bands(CellularDevice *dev)
{
    int gsm_bands_bitmask = -1;
    int umts_bands_bitmask = -1;
    int lte_bands_bitmask = -1;

    // Check the LTE bands first
    ATHandler *at_handler = dev->get_at_handler();
    at_handler->lock();
    at_handler->cmd_start_stop("#BND", "?");
    at_handler->resp_start("#BND:");

    // Read the current bands
    gsm_bands_bitmask = at_handler->read_int();
    umts_bands_bitmask = at_handler->read_int();
    lte_bands_bitmask = at_handler->read_int();
    at_handler->resp_stop();
    tr_info("GSM Bands Bitmask:     %d", gsm_bands_bitmask);
    tr_info("UMTS Bands Bitmask:    %d", umts_bands_bitmask);
    tr_info("LTE Bands Bitmask:     %d", lte_bands_bitmask);

    // If already configured as desired, return
    if (lte_bands_bitmask == DESIRED_LTE_BANDS_BITMASK) {
        at_handler->unlock();
        return;
    }

    // Set the desired LTE bands
    at_handler->at_cmd_discard("#BND", "=", "%d%d%d", 5, 0, DESIRED_LTE_BANDS_BITMASK);
    if (at_handler->get_last_error() != NSAPI_ERROR_OK) {
        printf("ERROR: Unable to set LTE bands!\n");
        at_handler->unlock();
        return;
    }

    at_handler->unlock();

    // Power cycle the module
    dev->hard_power_off();
    dev->hard_power_on();
    dev->soft_power_on();
}

void dot_event()
{
    while (true) {
        ThisThread::sleep_for(4000);
        if (iface && iface->get_connection_status() == NSAPI_STATUS_GLOBAL_UP) {
            break;
        } else {
            trace_mutex.lock();
            printf(".");
            fflush(stdout);
            trace_mutex.unlock();
        }
    }
}

/**
 * Connects to the Cellular Network
 */
nsapi_error_t do_connect()
{
    nsapi_error_t retcode = NSAPI_ERROR_OK;
    uint8_t retry_counter = 0;

    CellularDevice *dev = CellularDevice::get_target_default_instance();
    dev->hard_power_on();
    dev->soft_power_on();

    set_desired_lte_bands(dev);
    set_iot_technology(dev, DESIRED_IOT_TECHNOLOGY);
    set_wds_side_stack(dev, DESIRED_WDS_SIDE_STACK);

    while (iface->get_connection_status() != NSAPI_STATUS_GLOBAL_UP) {
        retcode = iface->connect();
        if (retcode == NSAPI_ERROR_AUTH_FAILURE) {
            print_function("\n\nAuthentication Failure. Exiting application\n");
        } else if (retcode == NSAPI_ERROR_OK) {
            print_function("\n\nConnection Established.\n");
        } else if (retry_counter > RETRY_COUNT) {
            print_function("\n\nFatal connection failure: %d\n", retcode);
        } else {
            print_function("\n\nCouldn't connect: %d, will retry\n", retcode);
            retry_counter++;
            continue;
        }
        break;
    }
    return retcode;
}

/**
 * Opens:
 * - UDP or TCP socket with the given echo server and performs an echo
 *   transaction retrieving current.
 * - Cellular Non-IP socket for which the data delivery path is decided
 *   by network's control plane CIoT optimisation setup, for the given APN.
 */
nsapi_error_t test_send_recv()
{
    nsapi_size_or_error_t retcode;
#if MBED_CONF_APP_SOCK_TYPE == TCP
    TCPSocket sock;
#elif MBED_CONF_APP_SOCK_TYPE == UDP
    UDPSocket sock;
#elif MBED_CONF_APP_SOCK_TYPE == NONIP
    CellularNonIPSocket sock;
#endif

#if MBED_CONF_APP_SOCK_TYPE == NONIP
    retcode = sock.open((CellularContext*)iface);
#else
    retcode = sock.open(iface);
#endif

    if (retcode != NSAPI_ERROR_OK) {
#if MBED_CONF_APP_SOCK_TYPE == TCP
        print_function("TCPSocket.open() fails, code: %d\n", retcode);
#elif MBED_CONF_APP_SOCK_TYPE == UDP
        print_function("UDPSocket.open() fails, code: %d\n", retcode);
#elif MBED_CONF_APP_SOCK_TYPE == NONIP
        print_function("CellularNonIPSocket.open() fails, code: %d\n", retcode);
#endif
        return -1;
    }

    int n = 0;
    const char *echo_string = "TEST";
    char recv_buf[4];

    sock.set_timeout(15000);

#if MBED_CONF_APP_SOCK_TYPE == NONIP
    retcode = sock.send((void*) echo_string, strlen(echo_string));
    if (retcode < 0) {
        print_function("CellularNonIPSocket.send() fails, code: %d\n", retcode);
        return -1;
    } else {
        print_function("CellularNonIPSocket: Sent %d Bytes\n", retcode);
    }

    n = sock.recv((void*) recv_buf, sizeof(recv_buf));

#else

    SocketAddress sock_addr;
    retcode = iface->gethostbyname(host_name, &sock_addr);
    if (retcode != NSAPI_ERROR_OK) {
        print_function("Couldn't resolve remote host: %s, code: %d\n", host_name, retcode);
        return -1;
    }

    sock_addr.set_port(port);

#if MBED_CONF_APP_SOCK_TYPE == TCP
    retcode = sock.connect(sock_addr);
    if (retcode < 0) {
        print_function("TCPSocket.connect() fails, code: %d\n", retcode);
        return -1;
    } else {
        print_function("TCP: connected with %s server\n", host_name);
    }
    retcode = sock.send((void*) echo_string, strlen(echo_string));
    if (retcode < 0) {
        print_function("TCPSocket.send() fails, code: %d\n", retcode);
        return -1;
    } else {
        print_function("TCP: Sent %d Bytes to %s\n", retcode, host_name);
    }

    n = sock.recv((void*) recv_buf, sizeof(recv_buf));
#else

    retcode = sock.sendto(sock_addr, (void*) echo_string, strlen(echo_string));
    if (retcode < 0) {
        print_function("UDPSocket.sendto() fails, code: %d\n", retcode);
        return -1;
    } else {
        print_function("UDP: Sent %d Bytes to %s\n", retcode, host_name);
    }

    n = sock.recvfrom(&sock_addr, (void*) recv_buf, sizeof(recv_buf));
#endif
#endif

    sock.close();

    if (n > 0) {
        print_function("Received from echo server %d Bytes\n", n);
        return 0;
    }

    return -1;
}

int main()
{
    print_function("\n\nmbed-os-example-cellular\n");
    print_function("\n\nBuilt: %s, %s\n", __DATE__, __TIME__);
#ifdef MBED_CONF_NSAPI_DEFAULT_CELLULAR_PLMN
    print_function("\n\n[MAIN], plmn: %s\n", (MBED_CONF_NSAPI_DEFAULT_CELLULAR_PLMN ? MBED_CONF_NSAPI_DEFAULT_CELLULAR_PLMN : "NULL"));
#endif

    print_function("Establishing connection\n");
#if MBED_CONF_MBED_TRACE_ENABLE
    trace_open();
#else
    dot_thread.start(dot_event);
#endif // #if MBED_CONF_MBED_TRACE_ENABLE

#if MBED_CONF_APP_SOCK_TYPE == NONIP
    iface = CellularContext::get_default_nonip_instance();
#else
    iface = CellularContext::get_default_instance();
#endif

    MBED_ASSERT(iface);

    // sim pin, apn, credentials and possible plmn are taken automatically from json when using NetworkInterface::set_default_parameters()
    iface->set_default_parameters();

    nsapi_error_t retcode = NSAPI_ERROR_NO_CONNECTION;

    /* Attempt to connect to a cellular network */
    if (do_connect() == NSAPI_ERROR_OK) {
        retcode = test_send_recv();
    }

    if (iface->disconnect() != NSAPI_ERROR_OK) {
        print_function("\n\n disconnect failed.\n\n");
    }

    if (retcode == NSAPI_ERROR_OK) {
        print_function("\n\nSuccess. Exiting \n\n");
    } else {
        print_function("\n\nFailure. Exiting \n\n");
    }

#if MBED_CONF_MBED_TRACE_ENABLE
    trace_close();
#else
    dot_thread.terminate();
#endif // #if MBED_CONF_MBED_TRACE_ENABLE

    return 0;
}
// EOF
